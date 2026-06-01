from __future__ import annotations

import os
from typing import Any, Callable, Dict

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped

from .config import OptiBridgeConfig, SourceConfig, load_config_from_dict
from .normalize import pose_stamped_to_normalized


def make_sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=5)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


class OptiBridgeNode(Node):
    def __init__(self, cfg: OptiBridgeConfig) -> None:
        super().__init__("opti_bridge_node")

        self._cfg = cfg
        self._pubs: Dict[str, Any] = {}
        self._subs: list[Any] = []

        sensor_qos = make_sensor_qos()

        self.get_logger().info(
            f"Starting opti_bridge with topic_prefix={self._cfg.topic_prefix}, "
            f"frame_id={self._cfg.frame_id}, axis_mode={self._cfg.axis_mode}, "
            f"sources={len(self._cfg.sources)}"
        )

        for id, scfg in self._cfg.sources.items():
            out_topic = f"{self._cfg.topic_prefix}/{id}/pose"

            self._pubs[id] = self.create_publisher(
                PoseStamped,
                out_topic,
                sensor_qos,
            )

            sub = self.create_subscription(
                PoseStamped,
                scfg.topic,
                self._make_cb(id, scfg),
                sensor_qos,
            )

            self._subs.append(sub)

            self.get_logger().info(
                f"Mapping source topic '{scfg.topic}' "
                f"to normalized topic '{out_topic}' "
                f"(frame_id={self._cfg.frame_id}, axis_mode={self._cfg.axis_mode})"
            )

    def _make_cb(self, id: str, scfg: SourceConfig) -> Callable[[PoseStamped], None]:
        def cb(msg: PoseStamped) -> None:
            try:
                out = pose_stamped_to_normalized(
                    msg=msg,
                    id=id,
                    frame_id=self._cfg.frame_id,
                    axis_mode=self._cfg.axis_mode,
                    invert_y=self._cfg.invert_y,
                )

                self._pubs[id].publish(out)

            except Exception as e:
                self.get_logger().error(
                    f"[{id}] Failed to process OptiTrack pose "
                    f"from '{scfg.topic}': {e}"
                )

        return cb


def main() -> None:
    rclpy.init()
    node = None

    try:
        loader = rclpy.create_node("opti_bridge_loader")
        loader.declare_parameter("config_path", "")
        config_path = loader.get_parameter("config_path").get_parameter_value().string_value
        loader.destroy_node()

        if not config_path:
            raise RuntimeError("Parameter 'config_path' is required")

        config_path = os.path.realpath(os.path.expanduser(config_path))
        if not os.path.isfile(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            raise RuntimeError(f"Invalid YAML in config file '{config_path}': {e}") from e
        except OSError as e:
            raise RuntimeError(f"Failed to read config file '{config_path}': {e}") from e

        try:
            cfg = load_config_from_dict(data)
        except Exception as e:
            raise RuntimeError(
                f"Invalid opti_bridge configuration in '{config_path}': {e}"
            ) from e

        node = OptiBridgeNode(cfg)
        node.get_logger().info(f"Loaded config: {config_path}")
        rclpy.spin(node)

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()