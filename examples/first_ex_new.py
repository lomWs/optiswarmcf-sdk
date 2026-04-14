import time

from optiswarmcf.context import RosContext
from optiswarmcf.optitrack import OptiTrack, OptiTrackConfig
from optiswarmcf.crazyflie import AgentConfig, CrazyflieAgent


DRONE_ID = "cf3"
MOCAP_TOPIC = f"/mocap/{DRONE_ID}/pose"


def main() -> None:
    context = RosContext()

    # --- CONFIGURATION ---
    agent_cfg = AgentConfig.from_drone_id(DRONE_ID)

    # --- START CONTEXT ---
    context.start()

    # --- CREATE OBJECTS ---
    agent = CrazyflieAgent(context.node, agent_cfg)

    opti = OptiTrack(
        context.node,
        OptiTrackConfig(
            pose_topics={
                DRONE_ID: MOCAP_TOPIC,
            }
        ),
    )

    try:
        print(f"Waiting for {DRONE_ID} pose...")
        ok = opti.wait_pose(DRONE_ID, tmax=10.0)

        if not ok:
            raise RuntimeError(f"{DRONE_ID} pose not received within timeout")

        pose = opti.get_pose(DRONE_ID)
        if pose is None:
            raise RuntimeError(f"{DRONE_ID} pose is None after successful wait_pose")

        print(f"{DRONE_ID} pose received: {pose}")
        print("Current snapshot:", opti.snapshot())
        print(f"{DRONE_ID} agent created successfully.")

        # Give the estimator some time to stabilize
        print("Waiting a bit for pose stabilization...")
        time.sleep(2.0)

        

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
        current_pose = opti.get_pose(DRONE_ID)
        if current_pose is None:
            raise RuntimeError(f"{DRONE_ID} pose disappeared before go_to_abs")

        target_x = current_pose.x
        target_y = current_pose.y
        target_z = min(current_pose.z + 0.10, 0.50)

        print(
            f"Sending go_to_abs to "
            f"x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}"
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