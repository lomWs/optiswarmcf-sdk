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





class PotentialFieldSwarm:
    def __init__(self, ros_context, optitrack_config, swarm_config, G_x=0.0, G_y=0.0, G_z=0.0, RHO=1.0, BETA=1.0,Ts=0.5):
        self.ros_context = ros_context
        self.optitrack = OptiTrack(self.ros_context, optitrack_config)
        self.swarm = Swarm(self.ros_context, swarm_config)

        # Potential field parameters
        self.G = [G_x, G_y, G_z]  # Goal point -> center of the globule
        self.RHO = RHO  # attraction gain
        self.BETA = BETA  # repulsion gain
        self.Ts = Ts  # sampling time


    def potential_field(self, drone_id, poses):
            """
            Compute f_i for drone i.

            f_i = -RHO * (x_i - G)+ BETA * sum_{j != i} (x_i - x_j) / ||x_i - x_j||^2
            """

            if drone_id not in poses:
                raise RuntimeError(f"Missing pose for drone {drone_id}")

            pos_i = poses[drone_id]

            pos_x = pos_i.x
            pos_y = pos_i.y
            pos_z = pos_i.z

            G_x, G_y, G_z = self.G

            # Attractive term toward G
            f_x = -self.RHO * (pos_x - G_x)
            f_y = -self.RHO * (pos_y - G_y)
            f_z = -self.RHO * (pos_z - G_z)

            # Repulsive term from all other drones
            for other_id, pos_j in poses.items():
                if other_id == drone_id: #skip self j!=i
                    continue

                dx = pos_x - pos_j.x
                dy = pos_y - pos_j.y
                dz = pos_z - pos_j.z

                dist_sq = dx * dx + dy * dy + dz * dz #distance squared between drone i and drone j

                if dist_sq < 1e-9: # Avoid division by zero and collision
                    continue

                f_x += self.BETA * dx / dist_sq
                f_y += self.BETA * dy / dist_sq
                f_z += self.BETA * dz / dist_sq

            return f_x, f_y, f_z
    


    def get_next_position(self, drone_id, poses):
        
        '''
            Compute the next position using :
            x_{i, next} = x_i + f_i * Ts
        '''

        f_x, f_y, f_z = self.potential_field(drone_id, poses)
        next_x = poses[drone_id].x + f_x * self.Ts
        next_y = poses[drone_id].y + f_y * self.Ts
        next_z = poses[drone_id].z + f_z * self.Ts

        return next_x, next_y, next_z



    def run(self, duration=30.0, do_takeoff=True, do_land=True):


        if do_takeoff:
            self.swarm.takeoff_all(timeout_sec=2.0)
            time.sleep(3.0)

        t0 = time.monotonic()

        try:
            while time.monotonic() - t0 < duration:
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


                time.sleep(self.Ts)

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

        drone_ids = ["cf3", "cf6"]

        mocap_pose_topics = {
            "cf3": "/mocap/cf3/pose",
            "cf6": "/mocap/cf6/pose",
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

        #construct the potential field swarm controller with the desired parameters
        controller = PotentialFieldSwarm(
            ros_context=ctx,
            optitrack_config=optitrack_config,
            swarm_config=swarm_config,
            G_x=0.0,
            G_y=0.0,
            G_z=0.9,
            RHO=1,
            BETA=1,
            Ts=0.1,
        )

        #run simulation for 35 seconds with takeoff and landing
        controller.run(
            duration=35.0,
            do_takeoff=True,
            do_land=True,
        )

    finally:
        ctx.shutdown()


if __name__ == "__main__":
    main()