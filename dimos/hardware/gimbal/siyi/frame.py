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

"""SIYI A8 mini gimbal frame maths: limits, MAVLink flag bits, attitude normalisation.

The A8 reports yaw relative to the body and pitch and roll stabilised to the earth. The
mount is a ``MountPreset`` chosen by config, not read from the reported roll, so a
transient roll reading cannot flip the sign convention. Pure functions, no I/O.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import IntFlag
import math

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.utils.transform_utils import normalize_angle, quaternion_to_euler

PITCH_MIN_DEG = -90.0
PITCH_MAX_DEG = 25.0
# A8 mechanical yaw limit is +/-135; keep a software margin.
YAW_MIN_DEG = -120.0
YAW_MAX_DEG = 120.0


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


@dataclass(frozen=True)
class MountPreset:
    """How the A8 is mounted.

    ``inverted`` selects the base-down bench orientation, where the A8 reports roll
    180 and yaw +180: pitch is negated and yaw shifted. Signs and offset apply after
    that normalisation.
    """

    name: str
    inverted: bool
    pitch_sign: float = 1.0
    yaw_sign: float = 1.0
    yaw_offset_deg: float = 0.0


# Flight mount: A8 hanging under the frame, roll reads ~0, raw angles used as-is.
FLIGHT_MOUNT = MountPreset(name="flight", inverted=False)
# Bench: base down on the table.
BENCH_MOUNT = MountPreset(name="bench", inverted=True)
MOUNT_PRESETS: dict[str, MountPreset] = {p.name: p for p in (FLIGHT_MOUNT, BENCH_MOUNT)}


def normalize_attitude(q: Sequence[float], mount: MountPreset) -> tuple[float, float, float]:
    """MAVLink ``[w, x, y, z]`` -> ``(roll, pitch, yaw)`` degrees, pitch and yaw in the
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW convention, roll as reported. Raises ValueError
    on a zero or non-finite quaternion."""
    w, x, y, z = q
    if not all(map(math.isfinite, q)):  # scipy turns an inf component into NaN angles
        raise ValueError("non-finite quaternion")
    euler = quaternion_to_euler(Quaternion(x, y, z, w), degrees=True)  # dimOS order is xyzw
    pitch, yaw = (-euler.y, euler.z + 180.0) if mount.inverted else (euler.y, euler.z)
    yaw = mount.yaw_sign * yaw + mount.yaw_offset_deg
    return euler.x, mount.pitch_sign * pitch, math.degrees(normalize_angle(math.radians(yaw)))


def decode_flags(flags: int) -> str:
    members = GimbalDeviceFlags.__members__.items()
    return "|".join(name for name, bit in members if bit & flags) or "none"
