from __future__ import annotations

from dataclasses import dataclass
from typing import Optional,Any

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
import time
from .models import VelocityCmd


def make_cmd_qos() -> QoSProfile:
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


@dataclass(frozen=True)
class AgentConfig:
    """
    Wraps the cf-bridge API (topics/services) exposed by your backend.

    Typical topics/services (per drone namespace /cf1):
      - /cf1/cmd_pos            PoseStamped
      - /cf1/cmd_pos_relative   Twist
      - /cf1/takeoff            Trigger
      - /cf1/land               Trigger
      - /cf1/ekf_reset          Trigger
    """
    drone_id: str
    cmd_pos_topic: str
    cmd_pos_rel_topic: str
    takeoff_srv: str
    land_srv: str
    ekf_reset_srv: str
    cmd_vel_world_topic: Optional[str] = None  
    cmd_vel_body_topic: Optional[str] = None  

    def __post_init__(self):
        if not self.drone_id.strip():
            raise ValueError("drone_id must be non-empty")

        required = {
            "cmd_pos_topic": self.cmd_pos_topic,
            "cmd_pos_rel_topic": self.cmd_pos_rel_topic,
            "takeoff_srv": self.takeoff_srv,
            "land_srv": self.land_srv,
            "ekf_reset_srv": self.ekf_reset_srv,
        }

        for name, value in required.items():
            if not value or not value.strip():
                raise ValueError(f"{name} must be non-empty")

    @staticmethod
    def from_drone_id(drone_id: str) -> "AgentConfig":
        if not drone_id or not drone_id.strip():
            raise ValueError("drone_id must be non-empty")

        drone_id = drone_id.strip()

        # enforce naming convention
        if "/" in drone_id:
            raise ValueError("drone_id must not contain '/'")

        ns = f"/{drone_id}"

        return AgentConfig(
            drone_id=drone_id,
            cmd_pos_topic=f"{ns}/cmd_pos",
            cmd_pos_rel_topic=f"{ns}/cmd_pos_relative",
            takeoff_srv=f"{ns}/takeoff",
            land_srv=f"{ns}/land",
            ekf_reset_srv=f"{ns}/ekf_reset",
            cmd_vel_world_topic=f"{ns}/cmd_vel_world",
            cmd_vel_body_topic=f"{ns}/cmd_vel_body",
        )


class CrazyflieAgent:
    """Client wrapper around cf-bridge topics/services"""

    def __init__(self, node: Node, cfg: AgentConfig) -> None:
        self.id = cfg.drone_id
        self._node = node
        self._cfg = cfg

        qos_cmd = make_cmd_qos()
        self._pub_abs = node.create_publisher(PoseStamped, cfg.cmd_pos_topic, qos_cmd)
        self._pub_rel = node.create_publisher(Twist, cfg.cmd_pos_rel_topic, qos_cmd)
        self._pub_vel_body = node.create_publisher(Twist, cfg.cmd_vel_body_topic, qos_cmd) if cfg.cmd_vel_body_topic else None
        self._pub_vel_world = node.create_publisher(Twist, cfg.cmd_vel_world_topic, qos_cmd) if cfg.cmd_vel_world_topic else None

        self._takeoff = node.create_client(Trigger, cfg.takeoff_srv)
        self._land = node.create_client(Trigger, cfg.land_srv)
        self._ekf = node.create_client(Trigger, cfg.ekf_reset_srv)

    def go_to_abs(self, x: float, y: float, z: float, frame_id: str = "map") -> None:
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self._pub_abs.publish(msg)

    def go_to_rel(self, dx: float, dy: float, dz: float) -> None:
        msg = Twist()
        msg.linear.x = float(dx)
        msg.linear.y = float(dy)
        msg.linear.z = float(dz)
        self._pub_rel.publish(msg)

    def set_velocity_world(self, cmd: VelocityCmd) -> None:
        if self._pub_vel_world is None:
            raise RuntimeError("cmd_vel_world_topic not configured for this agent")
        msg = Twist()
        msg.linear.x = float(cmd.vx)
        msg.linear.y = float(cmd.vy)
        msg.linear.z = float(cmd.vz)
        msg.angular.z = float(cmd.yaw_rate)  # keep ROS convention rad/s
        self._pub_vel_world.publish(msg)

    def set_velocity_body(self, cmd: VelocityCmd) -> None:
        if self._pub_vel_body is None:
            raise RuntimeError("cmd_vel_body_topic not configured for this agent")
        msg = Twist()
        msg.linear.x = float(cmd.vx)
        msg.linear.y = float(cmd.vy)
        msg.linear.z = float(cmd.vz)
        msg.angular.z = float(cmd.yaw_rate)  # rad/s
        self._pub_vel_body.publish(msg)
    
    #--- shortcut method for velocity whitout needing to create VelocityCmd object
    def set_vel_body(self, vx: float, vy: float, vz: float, yaw_rate: float=0.0) -> None:
        self.set_velocity_body(VelocityCmd(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate))
    
    def set_vel_world(self, vx: float, vy: float, vz: float, yaw_rate: float=0.0) -> None:
        self.set_velocity_world(VelocityCmd(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate))

    #-- others commands (takeoff, land, ekf reset) that call the corresponding services with a timeout
    def takeoff(self, timeout_sec: float = 1.0) -> None:
        self._call_trigger(self._takeoff, timeout_sec)

    def land(self, timeout_sec: float = 1.0) -> None:
        self._call_trigger(self._land, timeout_sec)

    def ekf_reset(self, timeout_sec: float = 1.0) -> None:
        self._call_trigger(self._ekf, timeout_sec)
    
    def _call_trigger(self, client: Any, timeout_sec: float) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be > 0")

        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service not available: {client.srv_name}")

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec

        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timeout calling service: {client.srv_name}")
            time.sleep(0.01)

        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"Service call failed: {client.srv_name}: {exc}")

        response = future.result()
        if response is None:
            raise RuntimeError(f"No response from service: {client.srv_name}")

        if not response.success:
            msg = response.message.strip() if response.message else "unknown error"
            raise RuntimeError(
                f"Service returned failure: {client.srv_name}: {msg}"
            )