#!/usr/bin/env python3
#
#
#chmod +x test_manual_backend_takeoff.py

#source /opt/ros/jazzy/setup.bash
#source backend_ros2/install/setup.bash

#python3 test_manual_takeoff.py
#
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class BackendTakeoffTest(Node):
    def __init__(self):
        super().__init__("backend_takeoff_test")

        self.pose = None
        self.pose_count = 0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(
            PoseStamped,
            "/mocap/cf1/pose",
            self.pose_cb,
            qos,
        )

        self.ekf_cli = self.create_client(Trigger, "/cf1/ekf_reset")
        self.takeoff_cli = self.create_client(Trigger, "/cf1/takeoff")
        self.land_cli = self.create_client(Trigger, "/cf1/land")

    def pose_cb(self, msg):
        self.pose = msg
        self.pose_count += 1

    def wait_pose(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None:
                return True
        return False

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

    def get_xyz(self):
        p = self.pose.pose.position
        return float(p.x), float(p.y), float(p.z)

    def check_mocap_stability(self, duration=3.0):
        print("\nChecking mocap stability...")
        samples = []
        t0 = time.time()

        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None:
                samples.append(self.get_xyz())

        if len(samples) < 10:
            raise RuntimeError("Not enough mocap samples")

        xs = [s[0] for s in samples]
        ys = [s[1] for s in samples]
        zs = [s[2] for s in samples]

        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        dz = max(zs) - min(zs)

        print(f"Mocap range over {duration:.1f}s:")
        print(f"  dx={dx:.4f} m")
        print(f"  dy={dy:.4f} m")
        print(f"  dz={dz:.4f} m")

        if dx > 0.03 or dy > 0.03 or dz > 0.03:
            print("WARNING: mocap is not very stable. Takeoff may be unsafe.")
            return False

        print("Mocap looks stable.")
        return True

    def monitor_pose(self, duration=5.0):
        print("\nMonitoring pose...")
        t0 = time.time()

        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is None:
                continue

            x, y, z = self.get_xyz()
            print(f"mocap: x={x: .3f} y={y: .3f} z={z: .3f}")

    def run(self):
        print("Waiting for mocap pose...")
        if not self.wait_pose(timeout=5.0):
            raise RuntimeError("No mocap pose received")

        x, y, z = self.get_xyz()
        print(f"Initial mocap: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        stable = self.check_mocap_stability(duration=3.0)
        if not stable:
            print("STOP: fix mocap stability before flying.")
            return

        print("\nResetting EKF...")
        if not self.call_trigger(self.ekf_cli, "/cf1/ekf_reset", timeout=5.0):
            print("STOP: EKF reset failed.")
            return

        print("Waiting 5 seconds after EKF reset...")
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        print("\nTakeoff...")
        if not self.call_trigger(self.takeoff_cli, "/cf1/takeoff", timeout=5.0):
            print("STOP: takeoff failed.")
            return

        self.monitor_pose(duration=5.0)

        print("\nLanding...")
        self.call_trigger(self.land_cli, "/cf1/land", timeout=5.0)

        print("Done.")


def main():
    rclpy.init()
    node = BackendTakeoffTest()

    try:
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted. Trying to land...")
        try:
            node.call_trigger(node.land_cli, "/cf1/land", timeout=3.0)
        except Exception as e:
            print(f"Landing failed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()