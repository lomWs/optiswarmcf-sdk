from __future__ import annotations

import os
from typing import Any, Callable, Dict

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry

from .config import MocapNormalizerConfig, SourceConfig, load_config_from_dict
from .normalize import normalize_any_to_poseparts, poseparts_to_posestamped


def make_sensor_qos() -> QoSProfile:
    """
    Low-latency QoS for high-rate sensor streams.
    Prefer freshest sample over perfect delivery.
    """
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


_TYPE_TO_MSG = {
    "pose_stamped": PoseStamped,
    "transform_stamped": TransformStamped,
    "odom": Odometry,
    "pose": Pose,
    "twist_xyz": Twist,
}


class MocapBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mocap_bridge_node")

        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            raise RuntimeError("Parameter 'config_path' is required")

        config_path = os.path.realpath(os.path.expanduser(config_path))
        self._cfg = self._load_cfg(config_path)

        sensor_qos = make_sensor_qos()

        self._pubs: Dict[str, Any] = {}
        self._subs: list[Any] = []

        self.get_logger().info(f"Loaded config: {config_path}")
        self.get_logger().info(
            f"Starting mocap normalizer with output_prefix={self._cfg.output_topic_prefix}, "
            f"frame_id={self._cfg.output_frame_id}, axis_mode={self._cfg.axis_mode}, "
            f"sources={len(self._cfg.sources)}"
        )

        for drone_id, scfg in self._cfg.sources.items():
            out_topic = f"{self._cfg.output_topic_prefix}/{drone_id}/pose"
            self._pubs[drone_id] = self.create_publisher(PoseStamped, out_topic, sensor_qos)

            msg_type = _TYPE_TO_MSG.get(scfg.input_type)
            if msg_type is None:
                raise ValueError(f"Unknown input type for {drone_id}: {scfg.input_type}")

            sub = self.create_subscription(
                msg_type,
                scfg.topic,
                self._make_cb(drone_id, scfg),
                sensor_qos,
            )
            self._subs.append(sub)

            self.get_logger().info(
                f"Mapping input topic '{scfg.topic}' ({scfg.input_type}) "
                f"to output topic '{out_topic}'"
            )

    def _load_cfg(self, path: str) -> MocapNormalizerConfig:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Config file not found: {path}")

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            raise RuntimeError(f"Invalid YAML in config file '{path}': {e}") from e
        except OSError as e:
            raise RuntimeError(f"Failed to read config file '{path}': {e}") from e

        try:
            return load_config_from_dict(data)
        except Exception as e:
            raise RuntimeError(f"Invalid mocap bridge configuration in '{path}': {e}") from e

    def _make_cb(self, drone_id: str, scfg: SourceConfig) -> Callable:
        def cb(msg) -> None:
            try:
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                parts = normalize_any_to_poseparts(msg, scfg.input_type)
                frame_id = scfg.frame_id or self._cfg.output_frame_id
                out = poseparts_to_posestamped(
                    drone_id=drone_id,
                    parts=parts,
                    frame_id=frame_id,
                    axis_mode=self._cfg.axis_mode,
                    now_sec=now_sec,
                )
                self._pubs[drone_id].publish(out)
            except Exception as e:
                self.get_logger().error(
                    f"[{drone_id}] Failed to process message from '{scfg.topic}' "
                    f"({scfg.input_type}): {e}"
                )
        return cb


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = MocapBridgeNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()