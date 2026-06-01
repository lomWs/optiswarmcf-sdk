from __future__ import annotations

import os

import yaml
import rclpy
from rclpy.node import Node

import cflib.crtp

from .client import CfClient
from .config import CfBridgeConfig, load_config_from_dict


class CfBridgeNode(Node):
    def __init__(self, cfg: CfBridgeConfig) -> None:
        super().__init__("cf_bridge_node")
        self._cf_clients: list[CfClient] = []

        try:
            for d in cfg.drones:
                self.get_logger().info(
                    f"Starting client for {d.drone_id} uri={d.uri} mocap={d.mocap_topic}"
                )
                self._cf_clients.append(CfClient(self, d, cfg))

            self.get_logger().info(
                f"cf_bridge_node up: {len(self._cf_clients)} drones"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start cf_bridge_node: {e}")
            self.shutdown()
            raise

    def shutdown(self) -> None:
        for c in reversed(self._cf_clients):
            try:
                c.close()
            except Exception as e:
                self.get_logger().warning(f"Failed to close CF client: {e}")

        self._cf_clients.clear()


def main() -> None:
    rclpy.init()
    node = None

    try:
        cflib.crtp.init_drivers()

        tmp = rclpy.create_node("cf_bridge_loader")
        tmp.declare_parameter("config_path", "")
        config_path = tmp.get_parameter("config_path").get_parameter_value().string_value
        tmp.destroy_node()

        if not config_path:
            raise RuntimeError("Parameter 'config_path' is required")

        config_path = os.path.realpath(os.path.expanduser(config_path))
        if not os.path.isfile(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                d = yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            raise RuntimeError(f"Invalid YAML in config file '{config_path}': {e}") from e

        cfg = load_config_from_dict(d)

        node = CfBridgeNode(cfg)
        rclpy.spin(node)

    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()