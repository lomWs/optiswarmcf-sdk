# simple go to test
# This algorithm is a simple potential field swarm algorithm that uses the potential field to guide the swarm. 
# Each agent in the swarm is attracted toward a common reference point while being repelled by other agents in the swarm.
# The attractive term keeps the swarm bounded around the desired center, whereas the repulsive term prevents inter-agent 
# collisions and promotes spatial separation. 
# The balance between these two effects leads the drones to arrange themselves around the reference point, 
# forming a bounded spatial configuration around it.
# 

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





class GotoTest:
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



    def run(self, do_takeoff=True, do_land=True):

        self.swarm.reset_estimator_all(timeout_sec=5.0)
        time.sleep(3.0)
        if do_takeoff:
            self.swarm.takeoff_all(timeout_sec=2.0)
            time.sleep(3.0)



        try:

            obs = self.optitrack.snapshot()
            poses = obs.poses

            for drone_id in self.swarm.ids():
                if drone_id not in poses:
                    continue

                x_ref, y_ref, z_ref = self.get_next_position(
                    drone_id=drone_id,
                    poses=poses,
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
            node_name="potential_field_controller"
        )
    )

    try:
        ctx.start()

        drone_ids = ["cf2"]

        mocap_pose_topics = {
            "cf2": "/optitrack/cf2/pose",
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


        controller = GotoTest(
            ros_context=ctx,
            optitrack_config=optitrack_config,
            swarm_config=swarm_config,
            G_x=0.0,
            G_y=0.1,
            G_z=0.1,
        )

        controller.run(
            do_takeoff=True,
            do_land=True,
        )

    finally:
        ctx.shutdown()


if __name__ == "__main__":
    main()