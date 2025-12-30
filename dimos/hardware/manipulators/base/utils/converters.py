# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit conversion utilities for manipulator drivers."""

import math


def degrees_to_radians(degrees: float | list[float]) -> float | list[float]:
    """Convert degrees to radians.

    Args:
        degrees: Angle(s) in degrees

    Returns:
        Angle(s) in radians
    """
    if isinstance(degrees, list):
        return [math.radians(d) for d in degrees]
    return math.radians(degrees)


def radians_to_degrees(radians: float | list[float]) -> float | list[float]:
    """Convert radians to degrees.

    Args:
        radians: Angle(s) in radians

    Returns:
        Angle(s) in degrees
    """
    if isinstance(radians, list):
        return [math.degrees(r) for r in radians]
    return math.degrees(radians)


def mm_to_meters(mm: float | list[float]) -> float | list[float]:
    """Convert millimeters to meters.

    Args:
        mm: Distance(s) in millimeters

    Returns:
        Distance(s) in meters
    """
    if isinstance(mm, list):
        return [m / 1000.0 for m in mm]
    return mm / 1000.0


def meters_to_mm(meters: float | list[float]) -> float | list[float]:
    """Convert meters to millimeters.

    Args:
        meters: Distance(s) in meters

    Returns:
        Distance(s) in millimeters
    """
    if isinstance(meters, list):
        return [m * 1000.0 for m in meters]
    return meters * 1000.0


def rpm_to_rad_per_sec(rpm: float | list[float]) -> float | list[float]:
    """Convert RPM to rad/s.

    Args:
        rpm: Angular velocity in RPM

    Returns:
        Angular velocity in rad/s
    """
    factor = (2 * math.pi) / 60.0
    if isinstance(rpm, list):
        return [r * factor for r in rpm]
    return rpm * factor


def rad_per_sec_to_rpm(rad_per_sec: float | list[float]) -> float | list[float]:
    """Convert rad/s to RPM.

    Args:
        rad_per_sec: Angular velocity in rad/s

    Returns:
        Angular velocity in RPM
    """
    factor = 60.0 / (2 * math.pi)
    if isinstance(rad_per_sec, list):
        return [r * factor for r in rad_per_sec]
    return rad_per_sec * factor


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw).

    Args:
        qx, qy, qz, qw: Quaternion components

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert Euler angles to quaternion.

    Args:
        roll, pitch, yaw: Euler angles in radians

    Returns:
        Tuple of (qx, qy, qz, qw) quaternion components
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def pose_dict_to_list(pose: dict[str, float]) -> list[float]:
    """Convert pose dictionary to list format.

    Args:
        pose: Dict with keys: x, y, z, roll, pitch, yaw

    Returns:
        List [x, y, z, roll, pitch, yaw]
    """
    return [
        pose.get("x", 0.0),
        pose.get("y", 0.0),
        pose.get("z", 0.0),
        pose.get("roll", 0.0),
        pose.get("pitch", 0.0),
        pose.get("yaw", 0.0),
    ]


def pose_list_to_dict(pose: list[float]) -> dict[str, float]:
    """Convert pose list to dictionary format.

    Args:
        pose: List [x, y, z, roll, pitch, yaw]

    Returns:
        Dict with keys: x, y, z, roll, pitch, yaw
    """
    if len(pose) < 6:
        raise ValueError(f"Pose list must have 6 elements, got {len(pose)}")

    return {
        "x": pose[0],
        "y": pose[1],
        "z": pose[2],
        "roll": pose[3],
        "pitch": pose[4],
        "yaw": pose[5],
    }


def twist_dict_to_list(twist: dict[str, float]) -> list[float]:
    """Convert twist dictionary to list format.

    Args:
        twist: Dict with keys: vx, vy, vz, wx, wy, wz

    Returns:
        List [vx, vy, vz, wx, wy, wz]
    """
    return [
        twist.get("vx", 0.0),
        twist.get("vy", 0.0),
        twist.get("vz", 0.0),
        twist.get("wx", 0.0),
        twist.get("wy", 0.0),
        twist.get("wz", 0.0),
    ]


def twist_list_to_dict(twist: list[float]) -> dict[str, float]:
    """Convert twist list to dictionary format.

    Args:
        twist: List [vx, vy, vz, wx, wy, wz]

    Returns:
        Dict with keys: vx, vy, vz, wx, wy, wz
    """
    if len(twist) < 6:
        raise ValueError(f"Twist list must have 6 elements, got {len(twist)}")

    return {
        "vx": twist[0],
        "vy": twist[1],
        "vz": twist[2],
        "wx": twist[3],
        "wy": twist[4],
        "wz": twist[5],
    }


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi].

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def normalize_angles(angles: list[float]) -> list[float]:
    """Normalize angles to [-pi, pi].

    Args:
        angles: Angles in radians

    Returns:
        Normalized angles in [-pi, pi]
    """
    return [normalize_angle(a) for a in angles]
