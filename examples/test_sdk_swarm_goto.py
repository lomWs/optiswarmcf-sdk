#==============================================================================
#   Example of using goto_abs to command absolute positions in the OptiTrack frame.
#   The example assumes that the drones are initially on the ground, 
#   and commands takeoff followed by goto_abs to a target position above the takeoff point. 
#   The target positions are computed as an offset from the initial positions, 
#   so the drones should move to a position directly above their initial location.
#   Note: The example does not include any safety checks or collision avoidance,
#==============================================================================



import time

from optiswarmcf.context import RosContext
from optiswarmcf.optitrack import OptiTrack, OptiTrackConfig
from optiswarmcf.crazyflie import CrazyflieConfig, CrazyflieAgent


DRONES = ["cf3", "cf6"]

OFFSETS = {
    "cf3": (0.0, 0.0, 0.6),
    "cf6": (0.1, 0.0, 0.6),
}


def print_pose(mocap, did: str) -> None:
    pose = mocap.get_pose(did)

    if pose is None:
        print(f"{did}: no pose available")
        return

    print(
        f"{did}: "
        f"x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}, "
        f"t={pose.stamp_sec:.3f}"
    )


def get_target(mocap, did: str) -> tuple[float, float, float]:
    pose = mocap.get_pose(did)

    if pose is None:
        raise RuntimeError(f"No pose available for {did}")

    dx, dy, dz = OFFSETS[did]

    return (
        pose.x + dx,
        pose.y + dy,
        pose.z + dz,
    )


with RosContext() as ctx:
    mocap = OptiTrack(
        ctx,
        OptiTrackConfig.from_drone_ids(DRONES),
    )

    agents = {
        did: CrazyflieAgent(
            ctx,
            CrazyflieConfig.from_drone_id(did),
        )
        for did in DRONES
    }

    print("Waiting for mocap...")
    if not mocap.wait_all_ready(DRONES, tmax=10.0):
        raise RuntimeError("Not all drones received mocap pose")

    print("Waiting for Crazyflie services...")
    for did, agent in agents.items():
        if not agent.wait_for_services(timeout_sec=5.0):
            raise RuntimeError(f"Services not available for {did}")

    print("Current mocap poses:")
    for did in DRONES:
        print_pose(mocap, did)

    print("Takeoff...")
    for agent in agents.values():
        agent.takeoff(timeout_sec=5.0)

    time.sleep(3.0)

    print("Poses after takeoff:")
    for did in DRONES:
        print_pose(mocap, did)

    print("Computing absolute targets from current poses...")
    targets = {
        did: get_target(mocap, did)
        for did in DRONES
    }

    print("Targets:")
    for did, target in targets.items():
        x, y, z = target
        print(f"{did}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

    print("Goto absolute targets...")
    for did, agent in agents.items():
        x, y, z = targets[did]
        agent.go_to_abs(x, y, z)

    time.sleep(5.0)

    print("Poses after goto:")
    for did in DRONES:
        print_pose(mocap, did)

    print("Land...")
    for agent in agents.values():
        agent.land(timeout_sec=5.0)

    time.sleep(2.0)