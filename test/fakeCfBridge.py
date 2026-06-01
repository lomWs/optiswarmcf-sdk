#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger


class FakeCfBridge(Node):
    def __init__(self):
        super().__init__("fake_cf_bridge_node")

        self.ids = ["cf1", "cf2", "cf3"]
        self.last_abs_cmd = {}
        self.last_rel_cmd = {}
        self.flying = {drone_id: False for drone_id in self.ids}

        for drone_id in self.ids:
            ns = f"/{drone_id}"

            self.create_subscription(
                PoseStamped,
                f"{ns}/cmd_pos",
                lambda msg, did=drone_id: self.on_cmd_abs(did, msg),
                10,
            )

            self.create_subscription(
                Twist,
                f"{ns}/cmd_pos_relative",
                lambda msg, did=drone_id: self.on_cmd_rel(did, msg),
                10,
            )

            self.create_service(
                Trigger,
                f"{ns}/takeoff",
                lambda req, resp, did=drone_id: self.on_takeoff(did, req, resp),
            )

            self.create_service(
                Trigger,
                f"{ns}/land",
                lambda req, resp, did=drone_id: self.on_land(did, req, resp),
            )

            self.create_service(
                Trigger,
                f"{ns}/ekf_reset",
                lambda req, resp, did=drone_id: self.on_ekf_reset(did, req, resp),
            )

        self.get_logger().info("Fake cf_bridge ready")

    def on_cmd_abs(self, drone_id, msg):
        p = msg.pose.position
        self.last_abs_cmd[drone_id] = (p.x, p.y, p.z)
        self.get_logger().info(
            f"[{drone_id}] cmd_pos x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}"
        )

    def on_cmd_rel(self, drone_id, msg):
        v = msg.linear
        self.last_rel_cmd[drone_id] = (v.x, v.y, v.z)
        self.get_logger().info(
            f"[{drone_id}] cmd_pos_relative dx={v.x:.2f}, dy={v.y:.2f}, dz={v.z:.2f}"
        )

    def on_takeoff(self, drone_id, req, resp):
        self.flying[drone_id] = True
        resp.success = True
        resp.message = f"{drone_id} fake takeoff OK"
        return resp

    def on_land(self, drone_id, req, resp):
        self.flying[drone_id] = False
        resp.success = True
        resp.message = f"{drone_id} fake land OK"
        return resp

    def on_ekf_reset(self, drone_id, req, resp):
        resp.success = True
        resp.message = f"{drone_id} fake ekf reset OK"
        return resp


def main():
    rclpy.init()
    node = FakeCfBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()