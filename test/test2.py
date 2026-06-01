from optiswarmcf.context import RosContext
from optiswarmcf.optitrack import OptiTrack, OptiTrackConfig

def main():
    with RosContext() as ctx:
        mocap = OptiTrack(
            ctx,
            OptiTrackConfig(
                pose_topics={
                    "cf1": "/optitrack/cf1/pose",
                    "cf2": "/optitrack/cf2/pose",
                    "cf3": "/optitrack/cf3/pose",
                }
            ),
        )

        ok = mocap.wait_all_ready(tmax=5.0)
        print("ready:", ok)

        obs = mocap.snapshot()
        for drone_id, pose in obs.poses.items():
            print(drone_id, pose)

if __name__ == "__main__":
    main()