from __future__ import annotations

import os

import yaml
import rclpy
from rclpy.node import Node

from .client import CfClient


import cflib.crtp

from .config import CfBridgeConfig, load_config_from_dict




class CfBridgeNode(Node):
    def __init__(self, cfg: CfBridgeConfig) -> None:
        super().__init__("cf_bridge")

        # Initialize CRTP drivers ONCE so USB Crazyradio is shared
        cflib.crtp.init_drivers()

        self._cf_clients: list[CfClient] = []
        for d in cfg.drones:
            self._cf_clients.append(CfClient(self, d, cfg))

        self.get_logger().info(f"cf_bridge up: {len(self._cf_clients)} drones")

    def shutdown(self) -> None:
        for c in self._cf_clients:
            c.close()




def main() -> None:
    rclpy.init()
    node = None
    try:
        tmp = rclpy.create_node("cf_bridge_loader")
        tmp.declare_parameter("config_path", "")
        config_path = tmp.get_parameter("config_path").get_parameter_value().string_value
        tmp.destroy_node()

        if not config_path:
            raise RuntimeError("Parameter 'config_path' is required")
        config_path = os.path.realpath(os.path.expanduser(config_path))

        with open(config_path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f) or {}
        cfg = load_config_from_dict(d)

        node = CfBridgeNode(cfg)
        rclpy.spin(node)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()