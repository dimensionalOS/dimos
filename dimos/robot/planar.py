# Copyright 2025-2026 Dimensional Inc.
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

"""Geometry helpers for concrete planar robot implementations, not wire codecs."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math

from dimos.control.contract.keys import Interface
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def twist_to_values(twist: Twist, joints: tuple[str, str, str], *, yaw: float) -> dict[str, float]:
    """Body-frame Twist to declared world-frame planar joint velocities.

    ``yaw`` must be real odometry, never an assumed zero orientation.
    """
    if not math.isfinite(yaw):
        raise ValueError("odometry yaw must be finite")
    c, s = math.cos(yaw), math.sin(yaw)
    vx, vy = twist.linear.x, twist.linear.y
    speeds = (c * vx - s * vy, s * vx + c * vy, twist.angular.z)
    if not all(math.isfinite(value) for value in speeds):
        raise ValueError("Twist conversion produced non-finite velocities")
    return {_key(joint, Interface.VELOCITY): v for joint, v in zip(joints, speeds, strict=True)}


def twist_from_values(
    values: Mapping[str, float], joints: tuple[str, str, str], *, yaw: float
) -> Twist:
    """Declared world-frame planar joint velocities to a body-frame Twist."""
    _require(values, joints, (Interface.VELOCITY,))
    if not math.isfinite(yaw):
        raise ValueError("odometry yaw must be finite")
    vx, vy, wz = [values[_key(joint, Interface.VELOCITY)] for joint in joints]
    c, s = math.cos(yaw), math.sin(yaw)
    body = (c * vx + s * vy, -s * vx + c * vy, wz)
    if not all(math.isfinite(value) for value in body):
        raise ValueError("Twist conversion produced non-finite velocities")
    return Twist(linear=Vector3(body[0], body[1], 0.0), angular=Vector3(0.0, 0.0, body[2]))


def pose_to_values(pose: PoseStamped, joints: tuple[str, str, str]) -> dict[str, float]:
    """World-frame planar coordinates from odometry, bound to model joint names."""
    q = pose.orientation
    if not all(
        math.isfinite(value) for value in (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
    ):
        raise ValueError("odometry pose must be finite")
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isclose(norm, 1.0, abs_tol=1e-3):
        raise ValueError("odometry quaternion must be unit length")
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return {
        _key(joint, Interface.POSITION): value
        for joint, value in zip(joints, (pose.position.x, pose.position.y, yaw), strict=True)
    }


def pose_from_values(
    values: Mapping[str, float],
    joints: tuple[str, str, str],
    ts: float | None = None,
    frame_id: str = "",
) -> PoseStamped:
    """World-frame planar coordinates to an odometry pose."""
    _require(values, joints, (Interface.POSITION,))
    x, y, yaw = [values[_key(joint, Interface.POSITION)] for joint in joints]
    orientation = Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return PoseStamped(
        ts=ts, frame_id=frame_id, position=Vector3(x, y, 0.0), orientation=orientation
    )


def _key(joint: str, interface: str) -> str:
    return f"{joint}/{interface}"


def _require(values: Mapping[str, float], joints: Sequence[str], interfaces: Sequence[str]) -> None:
    missing = [_key(j, i) for j in joints for i in interfaces if _key(j, i) not in values]
    if missing:
        raise ValueError(f"missing values {missing}")
