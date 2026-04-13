from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry

from .utils_frames import RigidBodyPose, transform_pose_axis_mode


@dataclass(frozen=True)
class _PoseParts:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_sec: Optional[float]  # if None, caller can use node clock


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _to_poseparts_from_pose(p: Pose) -> _PoseParts:
    return _PoseParts(
        x=float(p.position.x),
        y=float(p.position.y),
        z=float(p.position.z),
        qx=float(p.orientation.x),
        qy=float(p.orientation.y),
        qz=float(p.orientation.z),
        qw=float(p.orientation.w),
        stamp_sec=None,
    )


def _to_poseparts_from_pose_stamped(msg: PoseStamped) -> _PoseParts:
    stamp = _stamp_to_sec(msg.header.stamp)
    pp = _to_poseparts_from_pose(msg.pose)
    return _PoseParts(
        x=pp.x,
        y=pp.y,
        z=pp.z,
        qx=pp.qx,
        qy=pp.qy,
        qz=pp.qz,
        qw=pp.qw,
        stamp_sec=stamp,
    )


def _to_poseparts_from_transform_stamped(msg: TransformStamped) -> _PoseParts:
    stamp = _stamp_to_sec(msg.header.stamp)
    t = msg.transform.translation
    q = msg.transform.rotation
    return _PoseParts(
        x=float(t.x),
        y=float(t.y),
        z=float(t.z),
        qx=float(q.x),
        qy=float(q.y),
        qz=float(q.z),
        qw=float(q.w),
        stamp_sec=stamp,
    )


def _to_poseparts_from_odom(msg: Odometry) -> _PoseParts:
    stamp = _stamp_to_sec(msg.header.stamp)
    pp = _to_poseparts_from_pose(msg.pose.pose)
    return _PoseParts(
        x=pp.x,
        y=pp.y,
        z=pp.z,
        qx=pp.qx,
        qy=pp.qy,
        qz=pp.qz,
        qw=pp.qw,
        stamp_sec=stamp,
    )


def _to_poseparts_from_twist_xyz(msg: Twist) -> _PoseParts:
    # Interpret Twist.linear as position only if upstream uses it that way.
    return _PoseParts(
        x=float(msg.linear.x),
        y=float(msg.linear.y),
        z=float(msg.linear.z),
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        stamp_sec=None,
    )


def poseparts_to_posestamped(
    drone_id: str,
    parts: _PoseParts,
    frame_id: str,
    axis_mode: str,
    now_sec: float,
) -> PoseStamped:
    """
    Build the canonical PoseStamped output for one drone.

    If the input message carried a timestamp, preserve it.
    Otherwise, use the caller-provided node time.
    """
    stamp = parts.stamp_sec if parts.stamp_sec is not None else now_sec

    rb = RigidBodyPose(
        name=drone_id,
        x=parts.x,
        y=parts.y,
        z=parts.z,
        qx=parts.qx,
        qy=parts.qy,
        qz=parts.qz,
        qw=parts.qw,
        stamp_sec=stamp,
    )
    rb = transform_pose_axis_mode(rb, axis_mode)

    msg = PoseStamped()
    msg.header.frame_id = frame_id

    sec = int(stamp)
    nsec = int((stamp - sec) * 1e9)
    msg.header.stamp = Time(sec=sec, nanosec=nsec)

    msg.pose.position.x = rb.x
    msg.pose.position.y = rb.y
    msg.pose.position.z = rb.z
    msg.pose.orientation.x = rb.qx
    msg.pose.orientation.y = rb.qy
    msg.pose.orientation.z = rb.qz
    msg.pose.orientation.w = rb.qw
    return msg


_NORMALIZERS: dict[str, Callable[..., _PoseParts]] = {
    "pose_stamped": _to_poseparts_from_pose_stamped,
    "transform_stamped": _to_poseparts_from_transform_stamped,
    "odom": _to_poseparts_from_odom,
    "pose": _to_poseparts_from_pose,
    "twist_xyz": _to_poseparts_from_twist_xyz,
}


def normalize_any_to_poseparts(msg, input_type: str) -> _PoseParts:
    """
    Convert one supported ROS message into normalized pose parts.
    """
    try:
        fn = _NORMALIZERS[input_type]
    except KeyError as e:
        raise ValueError(f"Unsupported input_type: {input_type}") from e
    return fn(msg)