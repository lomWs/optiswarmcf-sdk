from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from geometry_msgs.msg import PoseStamped, Twist

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger

from .context import RosContext


def make_cmd_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.history = HistoryPolicy.KEEP_LAST
    #qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


@dataclass(frozen=True)
class CrazyflieAgentConfig:
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
    def from_drone_id(drone_id: str) -> "CrazyflieAgentConfig":
        if not drone_id or not drone_id.strip():
            raise ValueError("drone_id must be non-empty")

        drone_id = drone_id.strip()

        # enforce naming convention
        if "/" in drone_id:
            raise ValueError("drone_id must not contain '/'")

        ns = f"/{drone_id}"

        return CrazyflieAgentConfig(
            drone_id=drone_id,
            cmd_pos_topic=f"{ns}/cmd_pos",
            cmd_pos_rel_topic=f"{ns}/cmd_pos_relative",
            takeoff_srv=f"{ns}/takeoff",
            land_srv=f"{ns}/land",
            ekf_reset_srv=f"{ns}/ekf_reset",
        )

    @property
    def namespace(self) -> str:
        return f"/{self.drone_id}"


class CrazyflieAgent:
    """Client wrapper around cf-bridge topics/services"""

    def __init__(self, ctx: RosContext, cfg: CrazyflieAgentConfig) -> None:
        self._ctx = ctx
        self._node = ctx.node
        self.id = cfg.drone_id
        self._cfg = cfg

        qos_cmd = make_cmd_qos()

        self._pub_abs = self._node.create_publisher(PoseStamped, cfg.cmd_pos_topic, qos_cmd)
        self._pub_rel = self._node.create_publisher(Twist, cfg.cmd_pos_rel_topic, qos_cmd)

        self._takeoff = self._node.create_client(Trigger, cfg.takeoff_srv)
        self._land = self._node.create_client(Trigger, cfg.land_srv)
        self._ekf = self._node.create_client(Trigger, cfg.ekf_reset_srv)

    def go_to_abs(self, x: float, y: float, z: float,yaw: float=0.0, frame_id: str = "map") -> None:
        
        if not all(math.isfinite(v) for v in (x, y, z, yaw)):
            raise ValueError("x, y, z, yaw must be finite")

        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        # convert yaw (in radians) to quaternion (assuming roll=pitch=0) for orientation
        half_yaw = 0.5 * float(yaw)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)

        self._pub_abs.publish(msg)

    def go_to_rel(self, dx: float, dy: float, dz: float, yaw: float = 0.0) -> None:

        if not all(math.isfinite(v) for v in (dx, dy, dz, yaw)):
            raise ValueError("dx, dy, dz, yaw must be finite")
        msg = Twist()
        msg.linear.x = float(dx)
        msg.linear.y = float(dy)
        msg.linear.z = float(dz)
        msg.angular.z = float(yaw)
        self._pub_rel.publish(msg)


    #-- others commands (takeoff, land, ekf reset) that call the corresponding services with a timeout
    def takeoff(self, timeout_sec: float = 5.0) -> None:
        self._call_trigger(self._takeoff, timeout_sec)

    def land(self, timeout_sec: float = 5.0) -> None:
        self._call_trigger(self._land, timeout_sec)

    def ekf_reset(self, timeout_sec: float = 3.0) -> None:
        self._call_trigger(self._ekf, timeout_sec)
    
    def _call_trigger(self, client: Any, timeout_sec: float) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be > 0")

        self._ctx.check_ok()

        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service not available: {client.srv_name}")

        future = client.call_async(Trigger.Request())

        try:
            self._ctx.wait_future(future, timeout_sec)
        except TimeoutError as e:
            raise TimeoutError(f"Timeout calling service: {client.srv_name}") from e

        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"Service call failed: {client.srv_name}: {exc}")

        response = future.result()
        if response is None:
            raise RuntimeError(f"No response from service: {client.srv_name}")

        if not response.success:
            msg = response.message.strip() if response.message else "unknown error"
            raise RuntimeError(f"Service returned failure: {client.srv_name}: {msg}")