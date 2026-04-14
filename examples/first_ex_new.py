import time

from optiswarmcf.context import RosContext
from optiswarmcf.optitrack import OptiTrack, OptiTrackConfig
from optiswarmcf.crazyflie import CrazyflieAgent, AgentConfig


def main():
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
        print("cf1 pose received:", pose)

        print("Current snapshot:", opti.snapshot())

        print("cf1 agent created successfully.")
        print("No movement command will be sent.")

        # Keep the program alive briefly so you can inspect logs/output
        time.sleep(2.0)

    finally:
        context.shutdown()


if __name__ == "__main__":
    main()