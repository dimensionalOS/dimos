# Copyright 2026 Dimensional Inc.
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

"""SIYI A8 mini gimbal frame maths: limits, MAVLink flag bits, attitude normalisation, and
the ``gimbal_attitude`` / ``gimbal_target`` JointState format.

The A8 reports yaw relative to the body and pitch and roll stabilised to the earth. A
base-down (inverted) mount is set by config, not read from the reported roll, so a
transient roll reading cannot flip the sign convention. Pure functions, no I/O.
"""

from __future__ import annotations

from collections.abc import Sequence
from enum import IntFlag
import math

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import normalize_angle, quaternion_to_euler

PITCH_MIN_DEG = -90.0
PITCH_MAX_DEG = 25.0
# A8 mechanical yaw limit is +/-135; keep a software margin.
YAW_MIN_DEG = -120.0
YAW_MAX_DEG = 120.0

# gimbal_attitude: radians, MAVLink signs (pitch up, yaw clockwise), GIMBAL_DEVICE_FLAGS and
# failure flags in effort. gimbal_target: the requested pitch and yaw, same units and signs.
GIMBAL_JOINTS = ("gimbal_roll", "gimbal_pitch", "gimbal_yaw")
GIMBAL_TARGET_JOINTS = ("gimbal_pitch", "gimbal_yaw")


class GimbalDeviceFlags(IntFlag):
    """GIMBAL_DEVICE_FLAGS (MAVLink common.xml)."""

    RETRACT = 1
    NEUTRAL = 2
    ROLL_LOCK = 4
    PITCH_LOCK = 8
    YAW_LOCK = 16
    YAW_IN_VEHICLE_FRAME = 32
    YAW_IN_EARTH_FRAME = 64
    ACCEPTS_YAW_IN_EARTH_FRAME = 128


def normalize_attitude(q: Sequence[float], inverted: bool = False) -> tuple[float, float, float]:
    """MAVLink ``[w, x, y, z]`` -> ``(roll, pitch, yaw)`` degrees, pitch and yaw in the
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW convention, roll as reported. ``inverted``: the A8
    sits base down (it then reports roll 180 and yaw +180). Raises ValueError on a zero or
    non-finite quaternion."""
    w, x, y, z = q
    if not all(map(math.isfinite, q)):  # scipy turns an inf component into NaN angles
        raise ValueError("non-finite quaternion")
    euler = quaternion_to_euler(Quaternion(x, y, z, w), degrees=True)  # dimOS order is xyzw
    pitch, yaw = (-euler.y, euler.z + 180.0) if inverted else (euler.y, euler.z)
    return euler.x, pitch, math.degrees(normalize_angle(math.radians(yaw)))


def decode_flags(flags: int) -> str:
    members = GimbalDeviceFlags.__members__.items()
    return "|".join(name for name, bit in members if bit & flags) or "none"


def attitude_msg(
    ts: float,
    roll: float,
    pitch: float,
    yaw: float,
    flags: int = 0,
    failure: int = 0,
    frame_id: str = "gimbal_base",
) -> JointState:
    """A ``gimbal_attitude`` message; angles in radians."""
    return JointState(
        ts=ts,
        frame_id=frame_id,
        name=list(GIMBAL_JOINTS),
        position=[roll, pitch, yaw],
        effort=[float(flags), float(failure), 0.0],
    )


def joints(msg: JointState, names: Sequence[str]) -> list[float] | None:
    """The positions of ``names`` in ``msg``, in that order; None if one is missing."""
    by_name = dict(zip(msg.name, msg.position, strict=False))
    if not all(n in by_name for n in names):
        return None
    return [by_name[n] for n in names]
