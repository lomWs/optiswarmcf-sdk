from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Iterable

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from .models import Pose3D, Observation


def make_sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


@dataclass(frozen=True)
class OptiTrackConfig:
    """
    drone_id -> topic, usually:
      /mocap/<drone_id>/pose
    """
    pose_topics: Dict[str, str]


class OptiTrack:
    """Reads canonical mocap topics from the backend."""

    def __init__(self, node: Node, cfg: OptiTrackConfig) -> None:
        self._node = node
        self._cfg = cfg

        self._lock = threading.Lock()
        self._poses: Dict[str, Pose3D] = {}
        self._stamp: float = 0.0

        qos = make_sensor_qos()
        self._subs: Dict[str, object] = {}

        for drone_id, topic in cfg.pose_topics.items():
            self._subs[drone_id] = node.create_subscription(
                PoseStamped,
                topic,
                lambda msg, did=drone_id: self._on_pose(did, msg),
                qos,
            )

    def _on_pose(self, drone_id: str, msg: PoseStamped) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        pose = Pose3D(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            qx=float(q.x),
            qy=float(q.y),
            qz=float(q.z),
            qw=float(q.w),
            stamp_sec=float(stamp),
        )
        with self._lock:
            self._poses[drone_id] = pose
            if pose.stamp_sec > self._stamp:
                self._stamp = pose.stamp_sec

    def get_pose(self, drone_id: str) -> Optional[Pose3D]:
        with self._lock:
            return self._poses.get(drone_id)

    def has_pose(self, drone_id: str) -> bool:
        with self._lock:
            return drone_id in self._poses

    def wait_pose(
        self,
        drone_id: str,
        tmax: float = 10.0,
        sleep_dt: float = 0.02,
    ) -> bool:
        """Wait until at least one pose has been received for the given drone."""
        if tmax <= 0.0:
            raise ValueError("tmax must be > 0")
        if sleep_dt <= 0.0:
            raise ValueError("sleep_dt must be > 0")

        deadline = time.monotonic() + tmax
        while time.monotonic() < deadline:
            with self._lock:
                if drone_id in self._poses:
                    return True
            time.sleep(sleep_dt)

        return False

    def wait_all_ready(
        self,
        drone_ids: Optional[Iterable[str]] = None,
        tmax: float = 10.0,
        sleep_dt: float = 0.02,
    ) -> bool:
        """
        Wait until all selected drones have produced at least one pose.
        If drone_ids is None, all drones declared in config are checked.
        """
        if tmax <= 0.0:
            raise ValueError("tmax must be > 0")
        if sleep_dt <= 0.0:
            raise ValueError("sleep_dt must be > 0")

        ids = list(drone_ids) if drone_ids is not None else list(self._cfg.pose_topics.keys())
        missing = set(ids)
        deadline = time.monotonic() + tmax

        while time.monotonic() < deadline:
            with self._lock:
                for drone_id in list(missing):
                    if drone_id in self._poses:
                        missing.remove(drone_id)

            if not missing:
                return True

            time.sleep(sleep_dt)

        return False

    def snapshot(self) -> Observation:
        """Consistency copy for the latest pose for all drones, and the latest timestamp across all poses."""
        with self._lock:
            poses = dict(self._poses)
            stamp = float(self._stamp)
        return Observation(poses=poses, stamp_sec=stamp)