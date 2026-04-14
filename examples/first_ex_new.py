import time

from optiswarmcf.context import RosContext
from optiswarmcf.optitrack import OptiTrack, OptiTrackConfig
from optiswarmcf.crazyflie import AgentConfig, CrazyflieAgent


def main() -> None:
    context = RosContext()

    # --- CONFIGURATION ---
    agent_cfg = AgentConfig.from_drone_id("cf1")
    mocap_topic = "/mocap/cf1/pose"

    # --- START CONTEXT ---
    context.start()

    # --- CREATE OBJECTS ---
    agent = CrazyflieAgent(context.node, agent_cfg)

    opti = OptiTrack(
        context.node,
        OptiTrackConfig(
            pose_topics={
                "cf1": mocap_topic,
            }
        ),
    )

    try:
        print("Waiting for cf1 pose...")
        ok = opti.wait_pose("cf1", tmax=10.0)

        if not ok:
            raise RuntimeError("cf1 pose not received within timeout")

        pose = opti.get_pose("cf1")
        if pose is None:
            raise RuntimeError("cf1 pose is None after successful wait_pose")

        print("cf1 pose received:", pose)
        print("Current snapshot:", opti.snapshot())
        print("cf1 agent created successfully.")

        # ---------------------------------------------------------
        # TAKEOFF
        # ---------------------------------------------------------
        print("Sending takeoff...")
        agent.takeoff(timeout_sec=3.0)
        time.sleep(3.0)

        # ---------------------------------------------------------
        # GO TO ABS:
        # keep current x,y and increase only z
        # ---------------------------------------------------------
        current_pose = opti.get_pose("cf1")
        if current_pose is None:
            raise RuntimeError("cf1 pose disappeared before go_to_abs")

        target_x = current_pose.x
        target_y = current_pose.y
        target_z = current_pose.z + 0.2

        print(
            f"Sending go_to_abs to x={target_x:.3f}, "
            f"y={target_y:.3f}, z={target_z:.3f}"
        )
        agent.go_to_abs(target_x, target_y, target_z)

        time.sleep(3.0)

        # ---------------------------------------------------------
        # LAND
        # ---------------------------------------------------------
        print("Sending land...")
        agent.land(timeout_sec=3.0)
        time.sleep(3.0)

        print("Minimal controller completed successfully.")

    finally:
        context.shutdown()


if __name__ == "__main__":
    main()