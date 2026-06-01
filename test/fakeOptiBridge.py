#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeOptiBridge(Node):
    def __init__(self):
        super().__init__("fake_opti_bridge_node")

        self.ids = ["cf1", "cf2", "cf3"]
        self.pubs = {
            drone_id: self.create_publisher(
                PoseStamped,
                f"/optitrack/{drone_id}/pose",
                10,
            )
            for drone_id in self.ids
        }

        self.t = 0.0
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def tick(self):
        self.t += 0.02

        for i, drone_id in enumerate(self.ids):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.pose.position.x = 0.5 * math.cos(self.t + i)
            msg.pose.position.y = 0.5 * math.sin(self.t + i)
            msg.pose.position.z = 0.4

            msg.pose.orientation.w = 1.0

            self.pubs[drone_id].publish(msg)


def main():
    rclpy.init()
    node = FakeOptiBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()