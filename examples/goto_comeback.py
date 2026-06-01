import time

from optiswarmcf import (
    RosContext,
    OptiTrack,
    OptiTrackConfig,
    ContextConfig,
    CrazyflieAgentConfig,
    Swarm,
    SwarmConfig,
)





class GotoComebackTest:
    def __init__(self, ros_context, optitrack_config, swarm_config, G_x=0.0, G_y=0.0, G_z=0.0):
        self.ros_context = ros_context
        self.optitrack = OptiTrack(self.ros_context, optitrack_config)
        self.swarm = Swarm(self.ros_context, swarm_config)

        self.G = [G_x, G_y, G_z]  # Goal point -> center of the globule


    def get_next_position(self, drone_id, poses):
        
        '''
            Compute only next position based on go to, actual position + relative goal position
        '''


        next_x = poses[drone_id].x + self.G[0]
        next_y = poses[drone_id].y + self.G[1]
        next_z = poses[drone_id].z + self.G[2]

        return next_x, next_y, next_z

    def get_prev_position(self, drone_id, poses):
        
        '''
            Compute only next position based on go to, actual position - relative goal position
        '''


        next_x = poses[drone_id].x - self.G[0]
        next_y = poses[drone_id].y - self.G[1]
        next_z = poses[drone_id].z - self.G[2]

        return next_x, next_y, next_z


    def run(self, do_takeoff=True, do_land=True):


        self.swarm.reset_estimator_all(timeout_sec=5.0)
        time.sleep(3.0)

        print("[DEBUG] Starting GotoComebackTest...")
        print("[DEBUG] Waiting for OptiTrack to be ready...")
        time.sleep(0.5)
        obs0 = self.optitrack.snapshot()
        poses0 = obs0.poses
        print(f"[DEBUG] Current poses from initial snapshot: {poses0}")


        if do_takeoff:
            self.swarm.takeoff_all(timeout_sec=2.0)
            time.sleep(3.0)
        
        print("[DEBUG] TAKEOFF COMPLETE...")
        try:
            obs = self.optitrack.snapshot()
            poses = obs.poses
            print(f"[DEBUG] Current poses after takeoff: {poses}")

            for drone_id in self.swarm.ids():
                if drone_id not in poses:
                    continue


                x_ref, y_ref, z_ref = self.get_next_position(
                    drone_id=drone_id,
                    poses=poses,
                )

                # Move the drone by a small offset
                self.swarm.go_to_abs(
                    drone_id=drone_id,
                    x=x_ref,
                    y=y_ref,
                    z=z_ref,
                )

            time.sleep(5.0)

            obs2 = self.optitrack.snapshot()
            poses2 = obs2.poses
            print(f"[DEBUG] Current poses: {poses2}")
            # Return to the initial flying position
            for drone_id in self.swarm.ids():
                if drone_id not in poses2:
                    continue

                x_ref, y_ref, z_ref = self.get_prev_position(
                    drone_id=drone_id,
                    poses=poses2,
                )

                self.swarm.go_to_abs(
                    drone_id=drone_id,
                    x=x_ref,
                    y=y_ref,
                    z=z_ref,
                )

            time.sleep(5.0)

        finally:
            if do_land:
                self.swarm.land_all(timeout_sec=2.0)




def main():
    ctx = RosContext(
        ContextConfig(
            node_name="goto_comeback_controller"
        )
    )

    try:
        ctx.start()

        drone_ids = ["cf2"]

        mocap_pose_topics = {
            "cf2": "/optitrack/cf2/pose"
        }

        optitrack_config = OptiTrackConfig(
            pose_topics=mocap_pose_topics
        )

        swarm_config = SwarmConfig(
            mocap_pose_topics=mocap_pose_topics,
            agents={
                drone_id: CrazyflieAgentConfig.from_drone_id(drone_id)
                for drone_id in drone_ids
            },
        )


        controller = GotoComebackTest(
            ros_context=ctx,
            optitrack_config=optitrack_config,
            swarm_config=swarm_config,
            G_x=0.0,
            G_y=0.2,
            G_z=0.2,
        )

        controller.run(
            do_takeoff=True,
            do_land=True,
        )

    finally:
        ctx.shutdown()


if __name__ == "__main__":
    main()