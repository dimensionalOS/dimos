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

"""How every number a robot sends or receives is named.

A name has three parts: which robot, which part of it, and what about that
part::

    arm/joint1/position         where the arm's first joint is
    g1/left_hip_pitch/kp        how stiffly that hip is held
    go2/base/vx                 how fast the dog is driving forwards

The first two parts together, e.g. ``arm/joint1``, name the part itself. That
is what something asks to take control of. The third part picks one number
about it.

Each part may only contain letters, digits and underscores.
"""

from __future__ import annotations

from enum import Enum
import re

_SEGMENT = re.compile(r"[A-Za-z0-9_]+")

SEPARATOR = "/"
"""What goes between the parts of a name. Never allowed inside one."""

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
    """What a number is measured in. Each robot states its own."""

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
    """Whether one part of a name is spelled legally."""
    return _SEGMENT.fullmatch(segment) is not None


def is_valid_key(key: str) -> bool:
    """Whether a whole name is three legal parts joined by "/"."""
    parts = key.split(SEPARATOR)
    return len(parts) == 3 and all(_SEGMENT.fullmatch(p) for p in parts)


def make_key(source: str, resource: str, interface: str) -> str:
    """Build a name from its three parts.

    Args:
        source: Which robot, e.g. "arm".
        resource: Which part of it, e.g. "joint1".
        interface: What about that part, e.g. "position".

    Returns:
        The three joined by "/", e.g. "arm/joint1/position".

    Raises:
        ValueError: If any part contains anything but letters, digits and
            underscores, naming the one at fault.
    """
    for label, segment in (("source", source), ("resource", resource), ("interface", interface)):
        if not is_valid_segment(segment):
            raise ValueError(f"invalid {label} segment {segment!r}: expected [A-Za-z0-9_]+")
    return f"{source}{SEPARATOR}{resource}{SEPARATOR}{interface}"


def split_key(key: str) -> tuple[str, str, str]:
    """Take a name apart into its three parts.

    Args:
        key: A name such as "arm/joint1/position".

    Returns:
        Which robot, which part, and what about it.

    Raises:
        ValueError: If the name is not three valid parts, naming it.
    """
    parts = key.split(SEPARATOR)
    if len(parts) != 3:
        raise ValueError(f"invalid key {key!r}: expected <source>/<resource>/<interface>")
    for label, segment in zip(("source", "resource", "interface"), parts, strict=True):
        if not is_valid_segment(segment):
            raise ValueError(f"invalid {label} segment {segment!r} in key {key!r}")
    return parts[0], parts[1], parts[2]


def source_of(key: str) -> str:
    """Which robot a name belongs to."""
    return split_key(key)[0]


def joint_of(key: str) -> str:
    """Which part of which robot a name refers to, e.g. "arm/joint1".

    This is what something asks to take control of."""
    source, resource, _ = split_key(key)
    return f"{source}{SEPARATOR}{resource}"


def interface_of(key: str) -> str:
    """What a name says about its part, e.g. "position"."""
    return split_key(key)[2]
