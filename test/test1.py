from optiswarmcf import RosContext, CrazyflieAgentConfig, Swarm, SwarmConfig

def main():
    with RosContext() as ctx:
        agents = {
            "cf1": CrazyflieAgentConfig.from_drone_id("cf1"),
            "cf2": CrazyflieAgentConfig.from_drone_id("cf2"),
            "cf3": CrazyflieAgentConfig.from_drone_id("cf3"),
        }

        cfg = SwarmConfig(
            mocap_pose_topics={
                "cf1": "/optitrack/cf1/pose",
                "cf2": "/optitrack/cf2/pose",
                "cf3": "/optitrack/cf3/pose",
            },
            agents=agents,
        )

        swarm = Swarm(ctx, cfg)

        swarm.takeoff_all(timeout_sec=2.0)

        swarm.go_to_abs("cf1", 0.0, 0.0, 0.5)
        swarm.go_to_abs("cf2", 0.5, 0.0, 0.5)
        swarm.go_to_abs("cf3", 0.0, 0.5, 0.5)

        swarm.land_all(timeout_sec=2.0)

if __name__ == "__main__":
    main()