import time

from optiswarmcf.context import RosContext
from optiswarmcf.optitrack import OptiTrack, OptiTrackConfig
from optiswarmcf.crazyflie import AgentConfig, CrazyflieAgent



DRONE_ID = "cf1"
MOCAP_TOPIC = f"/mocap/{DRONE_ID}/pose"


def main() -> None:
    context = RosContext()

    try:
        context.start()

        agent_cfg = AgentConfig.from_drone_id(DRONE_ID)
        agent = CrazyflieAgent(context.node, agent_cfg)

        opti = OptiTrack(
            context.node,
            OptiTrackConfig(
                pose_topics={
                    DRONE_ID: MOCAP_TOPIC,
                }
            ),
        )

        print(f"Waiting for {DRONE_ID} pose...")
        if not opti.wait_pose(DRONE_ID, tmax=10.0):
            raise RuntimeError(f"{DRONE_ID} pose not received within timeout")

        initial_pose = opti.get_pose(DRONE_ID)
        if initial_pose is None:
            raise RuntimeError(f"{DRONE_ID} pose is None after wait_pose")

        print(f"Initial pose: {initial_pose}")

        print("Resetting EKF...")
        agent.ekf_reset(timeout_sec=5.0)

        print("Waiting for estimator stabilization...")
        time.sleep(2.0)

        print("Sending takeoff...")
        agent.takeoff(timeout_sec=5.0)

        print("Waiting after takeoff...")
        time.sleep(5.0)

        pose_after_takeoff = opti.get_pose(DRONE_ID)
        if pose_after_takeoff is None:
            raise RuntimeError(f"{DRONE_ID} pose disappeared after takeoff")

        print(f"Pose after takeoff: {pose_after_takeoff}")

        dz_takeoff = pose_after_takeoff.z - initial_pose.z
        print(f"Estimated takeoff dz: {dz_takeoff:.3f} m")

        if dz_takeoff < 0.20:
            raise RuntimeError(
                f"Takeoff probably failed: z increased only by {dz_takeoff:.3f} m"
            )

        print("Sending land...")
        agent.land(timeout_sec=5.0)

        print("Waiting after land...")
        time.sleep(5.0)

        pose_after_land = opti.get_pose(DRONE_ID)
        if pose_after_land is None:
            raise RuntimeError(f"{DRONE_ID} pose disappeared after land")

        print(f"Pose after land: {pose_after_land}")

        dz_land = pose_after_land.z - initial_pose.z
        print(f"Estimated final dz from initial pose: {dz_land:.3f} m")

        if dz_land > 0.15:
            raise RuntimeError(
                f"Land probably failed: final z is still {dz_land:.3f} m above initial pose"
            )

        print("Takeoff/land test completed successfully.")

    finally:
        context.shutdown()


if __name__ == "__main__":
    main()