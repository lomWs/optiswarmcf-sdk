from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class RigidBodyPose:
    """Pose of a rigid body with timestamp expressed in float seconds."""
    name: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_sec: float
    quality: Optional[float] = None


def _quat_normalize(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Tuple[float, float, float, float]:
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if n <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / n, qy / n, qz / n, qw / n


def _with_pose_and_quat(
    p: RigidBodyPose,
    *,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> RigidBodyPose:
    return RigidBodyPose(
        name=p.name,
        x=x,
        y=y,
        z=z,
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        stamp_sec=p.stamp_sec,
        quality=p.quality,
    )


def transform_pose_axis_mode(p: RigidBodyPose, axis_mode: str) -> RigidBodyPose:
    """
    Apply the configured axis transform to position and orientation.

    Supported modes:
    - identity:
        Pass-through pose with quaternion normalization.
    - optitrack_to_enu:
        Mapping validated for this project setup, where OptiTrack/Motive data
        must be converted to the ROS ENU convention by swapping Y and Z.
        Quaternion components are remapped consistently with the same convention.

    Note:
        This mapping is setup-dependent. Keep using 'identity' for synthetic
        tests or whenever upstream data is already expressed in the ROS frame.
    """
    if axis_mode == "identity":
        qx, qy, qz, qw = _quat_normalize(p.qx, p.qy, p.qz, p.qw)
        return _with_pose_and_quat(
            p,
            x=p.x,
            y=p.y,
            z=p.z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

    if axis_mode == "optitrack_to_enu":
        x = p.x
        y = p.z
        z = p.y
        qx, qy, qz, qw = _quat_normalize(p.qx, p.qz, p.qy, p.qw)
        return _with_pose_and_quat(
            p,
            x=x,
            y=y,
            z=z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

    raise ValueError(f"Unknown axis_mode: {axis_mode}")