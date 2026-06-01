#!/usr/bin/env python3
import time
import threading

from optiswarmcf import (
    RosContext,
    ContextConfig,
    OptiTrack,
    OptiTrackConfig,
    CrazyflieAgent,
    AgentConfig,
)


DRONE_IDS = ("cf3", "cf6")


def get_xyz(mocap: OptiTrack, drone_id: str):
    p = mocap.get_pose(drone_id)
    if p is None:
        return None
    return float(p.x), float(p.y), float(p.z)


def call_all(agents, method_name: str, timeout_sec: float = 5.0):
    errors = {}

    def worker(did, agent):
        try:
            method = getattr(agent, method_name)
            method(timeout_sec=timeout_sec)
            print(f"/{did}/{method_name}: success=True")
        except Exception as e:
            errors[did] = e
            print(f"/{did}/{method_name}: success=False, error='{e}'")

    threads = [
        threading.Thread(target=worker, args=(did, agent))
        for did, agent in agents.items()
    ]

    for t in threads:
        t.start()

    for t in threads:
        t.join()

    if errors:
        raise RuntimeError(f"{method_name} failed for: {errors}")


def wait_all_poses(mocap: OptiTrack, drone_ids, timeout=5.0):
    return mocap.wait_all_ready(drone_ids, tmax=timeout)


def check_mocap_stability(mocap: OptiTrack, drone_ids, duration=3.0):
    print("\nChecking mocap stability...")
    samples = {did: [] for did in drone_ids}

    t0 = time.monotonic()
    while time.monotonic() - t0 < duration:
        for did in drone_ids:
            xyz = get_xyz(mocap, did)
            if xyz is not None:
                samples[did].append(xyz)
        time.sleep(0.05)

    all_stable = True

    for did in drone_ids:
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


def monitor_pose(mocap: OptiTrack, drone_ids, duration=5.0):
    print("\nMonitoring poses...")
    t0 = time.monotonic()

    while time.monotonic() - t0 < duration:
        parts = []

        for did in drone_ids:
            xyz = get_xyz(mocap, did)
            if xyz is None:
                continue

            x, y, z = xyz
            parts.append(f"{did}: x={x:.3f} y={y:.3f} z={z:.3f}")

        print(" | ".join(parts))
        time.sleep(0.1)


def main():
    with RosContext(ContextConfig(node_name="sdk_takeoff_test_parallel")) as ctx:
        node = ctx.node

        mocap = OptiTrack(
            node,
            OptiTrackConfig(
                pose_topics={
                    did: f"/mocap/{did}/pose"
                    for did in DRONE_IDS
                }
            ),
        )

        agents = {
            did: CrazyflieAgent(
                ctx,
                AgentConfig.from_drone_id(did),
            )
            for did in DRONE_IDS
        }

        print(f"Testing drones in parallel with SDK: {list(DRONE_IDS)}")

        print("\nWaiting for all mocap poses...")
        if not wait_all_poses(mocap, DRONE_IDS, timeout=5.0):
            missing = [did for did in DRONE_IDS if mocap.get_pose(did) is None]
            raise RuntimeError(f"No mocap pose received for: {missing}")

        for did in DRONE_IDS:
            xyz = get_xyz(mocap, did)
            if xyz is None:
                raise RuntimeError(f"No mocap pose received for {did}")
            x, y, z = xyz
            print(f"{did} initial mocap: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        stable = check_mocap_stability(mocap, DRONE_IDS, duration=3.0)
        if not stable:
            print("STOP: fix mocap stability before flying.")
            return

        flying = False

        try:
            print("\nResetting EKF for all drones...")
            call_all(agents, "ekf_reset", timeout_sec=5.0)

            print("Waiting 5 seconds after EKF reset...")
            time.sleep(5.0)

            print("\nTakeoff all...")
            call_all(agents, "takeoff", timeout_sec=5.0)
            flying = True

            monitor_pose(mocap, DRONE_IDS, duration=5.0)

            print("\nLanding all...")
            call_all(agents, "land", timeout_sec=5.0)
            flying = False

            print("Done.")

        except KeyboardInterrupt:
            print("\nInterrupted.")

        finally:
            if flying:
                print("\nTrying to land all...")
                try:
                    call_all(agents, "land", timeout_sec=3.0)
                except Exception as e:
                    print(f"Landing failed: {e}")


if __name__ == "__main__":
    main()