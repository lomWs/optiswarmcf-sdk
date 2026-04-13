
import time

from sdk.src.optiswarmcf.optitrack import OptiTrack, OptiTrackConfig
from sdk.src.optiswarmcf.context import RosContext
from sdk.src.optiswarmcf.crazyflie import AgentConfig
from sdk.src.optiswarmcf.swarm import Swarm, SwarmConfig


context = RosContext()


# --- CONFIGURATIONS ---

agent_cfg1 = AgentConfig.from_drone_id("cf1")

agent_cfg3 = AgentConfig.from_drone_id("cf3")

swarm_cfg = SwarmConfig(
    mocap_pose_topics={
        "cf1": "/mocap/cf1/pose",
        "cf3": "/mocap/cf3/pose",
    },
    agents={
        "cf1": agent_cfg1,
        "cf3": agent_cfg3,
    },
)

# --- START CONTEXT ---
context.start()


# group helper
swarm = Swarm(context.node, swarm_cfg)

# observation-side object
opti = OptiTrack(
    context.node,
    OptiTrackConfig(pose_topics=swarm_cfg.mocap_pose_topics),
)

try:
    # -----------------------------------------------------------------
    # EXAMPLE 1: wait for one specific drone pose
    # -----------------------------------------------------------------
    print("Waiting for cf1 pose...")
    ok_cf1 = opti.wait_pose("cf1", tmax=10.0)

    if not ok_cf1:
        raise RuntimeError("cf1 pose not received within timeout")

    print("cf1 pose received:", opti.get_pose("cf1"))

    # -----------------------------------------------------------------
    # EXAMPLE 2: wait for all swarm poses
    # -----------------------------------------------------------------
    print("Waiting for all swarm poses...")
    ok_all = swarm.wait_all_ready(opti, tmax=10.0)

    if not ok_all:
        raise RuntimeError("Not all swarm drones received pose within timeout")

    print("All swarm drones are ready")

    # snapshot after readiness
    obs = opti.snapshot()
    print("Current snapshot:", obs)

    # -----------------------------------------------------------------
    # optional: EKF reset
    # -----------------------------------------------------------------
    swarm["cf1"].ekf_reset(3.0)
    swarm["cf3"].ekf_reset(3.0)

    time.sleep(1.0)

    # -----------------------------------------------------------------
    # TAKEOFF
    # -----------------------------------------------------------------
    swarm.takeoff_all(timeout_sec=3.0)
    time.sleep(3.0)

    # -----------------------------------------------------------------
    # SIMPLE COMMAND USING CURRENT POSE
    # keep x,y and go to z=0.5
    # -----------------------------------------------------------------
    pose_cf1 = opti.get_pose("cf1")
    pose_cf3 = opti.get_pose("cf3")

    if pose_cf1 is None or pose_cf3 is None:
        raise RuntimeError("Pose disappeared unexpectedly")

    swarm["cf1"].go_to_abs(pose_cf1.x, pose_cf1.y, 0.5)
    swarm["cf3"].go_to_abs(pose_cf3.x, pose_cf3.y, 0.5)

    time.sleep(5.0)

    # -----------------------------------------------------------------
    # LAND
    # -----------------------------------------------------------------
    swarm.land_all(timeout_sec=3.0)

finally:
    context.shutdown()