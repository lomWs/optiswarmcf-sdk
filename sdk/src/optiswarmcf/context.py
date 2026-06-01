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
        self._spin_error: Optional[BaseException] = None

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
            target=self._spin_loop,
            daemon=True,
        )
        self._thread.start()

        return self._node

    def _spin_loop(self) -> None:
        try:
            if self._executor is not None:
                self._executor.spin()
        except BaseException as e:
            self._spin_error = e

    def check_ok(self) -> None:
        if self._spin_error is not None:
            raise RuntimeError(f"ROS executor stopped with error: {self._spin_error}")

        if self._thread is None or not self._thread.is_alive():
            raise RuntimeError("ROS executor thread is not running")

    def wait_future(self, future, timeout_sec: float) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be > 0")

        self.check_ok()

        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())

        if future.done():
            return

        if not done_event.wait(timeout=timeout_sec):
            self.check_ok()
            raise TimeoutError("Timeout waiting for ROS future")

    def shutdown(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

        if self._node is not None:
            self._node.destroy_node()

        self._executor = None
        self._node = None
        self._thread = None
        self._spin_error = None

        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
            self._owns_rclpy = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.shutdown()