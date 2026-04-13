from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


@dataclass(frozen=True)
class ContextConfig:
    node_name: str = "cfswarm_sdk"


class RosContext:
    def __init__(self, cfg: ContextConfig | None = None) -> None:
        self._cfg = cfg or ContextConfig()
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._owns_rclpy: bool = False

    @property
    def node(self) -> Node:
        if self._node is None:
            raise RuntimeError("RosContext not started")
        return self._node

    def start(self) -> Node:
        if self._node is not None:
            return self._node

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        self._node = rclpy.create_node(self._cfg.node_name)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        self._thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
        )
        self._thread.start()

        return self._node

    def shutdown(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2)

        if self._node is not None:
            self._node.destroy_node()

        self._executor = None
        self._node = None
        self._thread = None

        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
            self._owns_rclpy = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.shutdown()