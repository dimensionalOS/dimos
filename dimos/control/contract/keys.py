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

"""Canonical control key names.

A key is ``"<source>/<resource>/<interface>"`` -- exactly three segments of
``[A-Za-z0-9_]+``. The first two segments are the joint name, which is what a
task claims and what ``coordinator_joint_state`` reports; the interface is
appended to address one number on it.

Existing vendor naming already has this shape, so nothing needs renaming::

    arm/joint1          -> arm/joint1/position
    g1/left_hip_pitch   -> g1/left_hip_pitch/kp
    r1pro/torso_joint1  -> r1pro/torso_joint1/velocity
    go2/base            -> go2/base/vx
"""

from __future__ import annotations

from enum import Enum
import re

_SEGMENT = re.compile(r"[A-Za-z0-9_]+")

SEPARATOR = "/"
"""Key segment separator. Never valid inside a segment."""

# Joint interfaces.
POSITION = "position"
VELOCITY = "velocity"
EFFORT = "effort"
KP = "kp"
KD = "kd"

# Base interfaces: body-frame twist, then the integrated pose.
#
# A ground base declares the planar subset (vx, vy, wz over x, y, yaw); a drone
# or a free-flyer declares all six. Both are the same vocabulary, so a task that
# reads vz does not need to know which kind of base it is talking to.
#
# The pose terms (x, y, z, roll, pitch, yaw) are state-only today: a base is
# commanded as a twist, and nothing in the stack commands a pose directly. They
# are named here so odometry has one spelling, not two.
VX = "vx"
VY = "vy"
VZ = "vz"
WX = "wx"
WY = "wy"
WZ = "wz"
X = "x"
Y = "y"
Z = "z"
ROLL = "roll"
PITCH = "pitch"
YAW = "yaw"

# IMU scalars. Orientation quaternion, angular rate, linear acceleration.
QX = "qx"
QY = "qy"
QZ = "qz"
QW = "qw"
GX = "gx"
GY = "gy"
GZ = "gz"
AX = "ax"
AY = "ay"
AZ = "az"


class Unit(Enum):
    """Wire unit of one interface. Declared per interface by the description."""

    RAD = "rad"
    RAD_PER_S = "rad/s"
    NM = "Nm"
    M = "m"
    M_PER_S = "m/s"
    N = "N"
    NORMALIZED = "normalized"
    M_PER_S2 = "m/s^2"
    UNITLESS = "unitless"


def is_valid_segment(segment: str) -> bool:
    """True when ``segment`` is a legal key segment."""
    return _SEGMENT.fullmatch(segment) is not None


def is_valid_key(key: str) -> bool:
    """True when ``key`` is three legal segments joined by ``/``."""
    parts = key.split(SEPARATOR)
    return len(parts) == 3 and all(_SEGMENT.fullmatch(p) for p in parts)


def make_key(source: str, resource: str, interface: str) -> str:
    """Join three segments into a key.

    Raises:
        ValueError: If any segment is not ``[A-Za-z0-9_]+``, naming the offender.
    """
    for label, segment in (("source", source), ("resource", resource), ("interface", interface)):
        if not is_valid_segment(segment):
            raise ValueError(f"invalid {label} segment {segment!r}: expected [A-Za-z0-9_]+")
    return f"{source}{SEPARATOR}{resource}{SEPARATOR}{interface}"


def split_key(key: str) -> tuple[str, str, str]:
    """Split a key into ``(source, resource, interface)``.

    Raises:
        ValueError: If ``key`` is not three legal segments, naming the key.
    """
    parts = key.split(SEPARATOR)
    if len(parts) != 3:
        raise ValueError(f"invalid key {key!r}: expected <source>/<resource>/<interface>")
    for label, segment in zip(("source", "resource", "interface"), parts, strict=True):
        if not is_valid_segment(segment):
            raise ValueError(f"invalid {label} segment {segment!r} in key {key!r}")
    return parts[0], parts[1], parts[2]


def source_of(key: str) -> str:
    """The publisher segment of ``key``."""
    return split_key(key)[0]


def joint_of(key: str) -> str:
    """The ``"<source>/<resource>"`` a key addresses. What tasks claim."""
    source, resource, _ = split_key(key)
    return f"{source}{SEPARATOR}{resource}"


def interface_of(key: str) -> str:
    """The interface segment of ``key``."""
    return split_key(key)[2]
