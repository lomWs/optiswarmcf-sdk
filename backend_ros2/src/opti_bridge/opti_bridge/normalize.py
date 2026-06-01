from __future__ import annotations

from geometry_msgs.msg import PoseStamped

from .utils_frames import RigidBodyPose, transform_pose_axis_mode, invert_pose_y


def pose_stamped_to_normalized(
    msg: PoseStamped,
    id: str,
    frame_id: str,
    axis_mode: str,
    invert_y: bool,
) -> PoseStamped:
    
    p = msg.pose.position
    q = msg.pose.orientation

    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    rb = RigidBodyPose(
        name=id,
        x=float(p.x),
        y=float(p.y),
        z=float(p.z),
        qx=float(q.x),
        qy=float(q.y),
        qz=float(q.z),
        qw=float(q.w),
        stamp_sec=float(stamp),
    )

    rb = transform_pose_axis_mode(rb, axis_mode)

    if invert_y:
        rb = invert_pose_y(rb)

    out = PoseStamped()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = frame_id

    out.pose.position.x = rb.x
    out.pose.position.y = rb.y
    out.pose.position.z = rb.z

    out.pose.orientation.x = rb.qx
    out.pose.orientation.y = rb.qy
    out.pose.orientation.z = rb.qz
    out.pose.orientation.w = rb.qw

    return out