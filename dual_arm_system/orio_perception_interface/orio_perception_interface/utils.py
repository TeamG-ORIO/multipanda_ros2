"""Utility helpers for the perception interface."""

import math
from geometry_msgs.msg import Quaternion


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert ZYX Euler angles (radians) to a geometry_msgs Quaternion.

    Convention: yaw → pitch → roll (intrinsic ZYX).

    Args:
        roll:  Rotation about X axis (radians).
        pitch: Rotation about Y axis (radians).
        yaw:   Rotation about Z axis (radians).

    Returns:
        geometry_msgs.msg.Quaternion
    """
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def euler_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    """Convenience wrapper that accepts degrees instead of radians."""
    return euler_to_quaternion(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )
