import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class BackendTakeoffTest(Node):
    def __init__(self, drone_ids=("cf3", "cf6")):
        super().__init__("backend_takeoff_test_parallel")

        self.drone_ids = list(drone_ids)
        self.poses = {did: None for did in self.drone_ids}
        self.pose_counts = {did: 0 for did in self.drone_ids}

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.subs = {}
        self.ekf_cli = {}
        self.takeoff_cli = {}
        self.land_cli = {}

        for did in self.drone_ids:
            ns = f"/{did}"
            mocap_topic = f"/mocap/{did}/pose"

            self.subs[did] = self.create_subscription(
                PoseStamped,
                mocap_topic,
                lambda msg, drone_id=did: self.pose_cb(drone_id, msg),
                qos,
            )

            self.ekf_cli[did] = self.create_client(Trigger, f"{ns}/ekf_reset")
            self.takeoff_cli[did] = self.create_client(Trigger, f"{ns}/takeoff")
            self.land_cli[did] = self.create_client(Trigger, f"{ns}/land")

    def pose_cb(self, drone_id, msg):
        self.poses[drone_id] = msg
        self.pose_counts[drone_id] += 1

    def wait_all_poses(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(self.poses[did] is not None for did in self.drone_ids):
                return True
        return False

    def get_xyz(self, drone_id):
        p = self.poses[drone_id].pose.position
        return float(p.x), float(p.y), float(p.z)

    def call_trigger(self, client, name, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service not available: {name}")

        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)

        if not fut.done():
            raise TimeoutError(f"Timeout calling {name}")

        res = fut.result()
        print(f"{name}: success={res.success}, message='{res.message}'")
        return res.success

    def call_all(self, clients, srv_name, timeout=5.0):
        futures = {}

        for did in self.drone_ids:
            name = f"/{did}/{srv_name}"
            client = clients[did]

            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Service not available: {name}")

            futures[did] = client.call_async(Trigger.Request())

        t0 = time.time()
        pending = set(self.drone_ids)

        while pending and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            for did in list(pending):
                fut = futures[did]
                if fut.done():
                    res = fut.result()
                    print(
                        f"/{did}/{srv_name}: success={res.success}, "
                        f"message='{res.message}'"
                    )
                    pending.remove(did)

        if pending:
            raise TimeoutError(f"Timeout calling {srv_name} for: {sorted(pending)}")

        return True

    def check_mocap_stability(self, duration=3.0):
        print("\nChecking mocap stability...")
        samples = {did: [] for did in self.drone_ids}

        t0 = time.time()
        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.05)

            for did in self.drone_ids:
                if self.poses[did] is not None:
                    samples[did].append(self.get_xyz(did))

        all_stable = True

        for did in self.drone_ids:
            if len(samples[did]) < 10:
                raise RuntimeError(f"Not enough mocap samples for {did}")

            xs = [s[0] for s in samples[did]]
            ys = [s[1] for s in samples[did]]
            zs = [s[2] for s in samples[did]]

            dx = max(xs) - min(xs)
            dy = max(ys) - min(ys)
            dz = max(zs) - min(zs)

            print(f"{did} mocap range over {duration:.1f}s:")
            print(f"  dx={dx:.4f} m")
            print(f"  dy={dy:.4f} m")
            print(f"  dz={dz:.4f} m")

            if dx > 0.03 or dy > 0.03 or dz > 0.03:
                print(f"WARNING: {did} mocap is not very stable.")
                all_stable = False

        return all_stable

    def monitor_pose(self, duration=5.0):
        print("\nMonitoring poses...")
        t0 = time.time()

        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

            parts = []
            for did in self.drone_ids:
                if self.poses[did] is None:
                    continue
                x, y, z = self.get_xyz(did)
                parts.append(f"{did}: x={x:.3f} y={y:.3f} z={z:.3f}")

            print(" | ".join(parts))

    def run(self):
        print(f"Testing drones in parallel: {self.drone_ids}")

        print("\nWaiting for all mocap poses...")
        if not self.wait_all_poses(timeout=5.0):
            missing = [did for did in self.drone_ids if self.poses[did] is None]
            raise RuntimeError(f"No mocap pose received for: {missing}")

        for did in self.drone_ids:
            x, y, z = self.get_xyz(did)
            print(f"{did} initial mocap: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        stable = self.check_mocap_stability(duration=3.0)
        if not stable:
            print("STOP: fix mocap stability before flying.")
            return

        print("\nResetting EKF for all drones...")
        self.call_all(self.ekf_cli, "ekf_reset", timeout=5.0)

        print("Waiting 5 seconds after EKF reset...")
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        print("\nTakeoff all...")
        self.call_all(self.takeoff_cli, "takeoff", timeout=5.0)

        self.monitor_pose(duration=5.0)

        print("\nLanding all...")
        self.call_all(self.land_cli, "land", timeout=5.0)

        print("Done.")



def main():
    rclpy.init()
    node = BackendTakeoffTest(("cf3", "cf6"))

    try:
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted. Trying to land all...")
        try:
            node.call_all(node.land_cli, "land", timeout=3.0)
        except Exception as e:
            print(f"Landing failed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()