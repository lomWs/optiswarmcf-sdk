#HOW RUN : -> Scrivania/backendProvas/sdk $PYTHONPATH=src:$PYTHONPATH python3 src/cfswarm_sdk/examples/first_ex_cf.py


from optiswarmcf.optitrack import OptiTrack, OptiTrackConfig
from optiswarmcf.context import RosContext
from optiswarmcf.crazyflie import CrazyflieAgent, AgentConfig
from optiswarmcf.swarm import Swarm, SwarmConfig
import time



context = RosContext()


#---CONFIGURATONs--
agent_cfg1 = AgentConfig(
    drone_id="cf1",
    cmd_pos_topic="/cf1/cmd_pos",
    cmd_pos_rel_topic="/cf1/cmd_pos_relative",
    takeoff_srv="/cf1/takeoff",
    land_srv="/cf1/land",
    ekf_reset_srv="/cf1/ekf_reset"
)

agent_cfg3 = AgentConfig(
    drone_id="cf3",
    cmd_pos_topic="/cf3/cmd_pos",
    cmd_pos_rel_topic="/cf3/cmd_pos_relative",
    takeoff_srv="/cf3/takeoff",
    land_srv="/cf3/land",
    ekf_reset_srv="/cf3/ekf_reset" 
)


swarm_cfg = SwarmConfig(
    mocap_pose_topics={

        #"cf1": "/mocap/cf1/pose",
        "cf3": "/mocap/cf3/pose",
        #"cf6": "/mocap/cf6/pose",
    },
    agents={
        "cf1": agent_cfg1,
        "cf3": agent_cfg3,
        
    }
)


#---CONFIGURATONs--


context.start()

#example without swarm, just two agents and no optitrack (so no pose feedback, just to test the connection and topics)
cf1 = CrazyflieAgent(context.node, agent_cfg1)
cf3 = CrazyflieAgent(context.node, agent_cfg3)


#try using swarm with the same two agents and no optitrack (so no pose feedback, just to test the connection and topics)
opti= OptiTrack(context.node, OptiTrackConfig(pose_topics=swarm_cfg.mocap_pose_topics))
#swarm = Swarm(context.node, swarm_cfg)


time.sleep(5)

    

#swarm.agent("cf1").go_to_abs(0.0, 0.0, 0.1)
cf1.ekf_reset(3.0)


#cf6.takeoff()
i=0
while i<10:
    print(opti.get_pose("cf3"))
    print(opti.get_pose("cf1"))
    i+=1
print("Agent ready:" , cf3)
print("Agent ready:" , cf1)

#cf3.takeoff(3.0)
#cf3.go_to_abs(0.012, 0.007, 0.5)
#cf1.go_to_abs(0.031, 0.54, 0.5)
time.sleep(5)
cf1.land()
cf3.land()
#cf3.set_vel_body(vx=1.0, vy=0.0, vz=0.1, yaw_rate=0.0)
#
#print("Agent ready:" , swarm.agent("cf1"))
#print("Agent ready:" , swarm.agent("cf3"))
context.shutdown()