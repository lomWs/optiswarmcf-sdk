from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class RigidBodyPose:
    """
    Internal pose representation used before frame/axis conversion.
    """
    name: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_sec: Optional[float] = None


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


def transform_pose_axis_mode(
    p: RigidBodyPose,
    axis_mode: str,
) -> RigidBodyPose:
    if axis_mode == "identity":
        qx, qy, qz, qw = _quat_normalize(
            p.qx, p.qy, p.qz, p.qw
        )

        return RigidBodyPose(
            name=p.name,
            x=p.x,
            y=p.y,
            z=p.z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            stamp_sec=p.stamp_sec,
        )

    if axis_mode == "swap_yz":
        qx, qy, qz, qw = _quat_normalize(
            p.qx, p.qz, p.qy, p.qw
        )

        return RigidBodyPose(
            name=p.name,
            x=p.x,
            y=p.z,
            z=p.y,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            stamp_sec=p.stamp_sec,
        )

    raise ValueError(f"Unknown axis_mode: {axis_mode}")


def invert_pose_y(p: RigidBodyPose) -> RigidBodyPose:
    return RigidBodyPose(
        name=p.name,
        x=p.x,
        y=-p.y,
        z=p.z,
        qx=p.qx,
        qy=p.qy,
        qz=p.qz,
        qw=p.qw,
        stamp_sec=p.stamp_sec,
    )
# reminder: for now invert_y is only used if cf_bridge has with_orient = false, which means that the quaternions are not used