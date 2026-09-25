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


class Key(str):
    """The name of one number on a robot, such as "arm/joint1/position".

    Three parts joined by "/": which robot, which part of it, and what about
    that part. Checked once when built, so reading the parts afterwards costs
    nothing.

    It is a string, so it goes on the wire and works as a dictionary key with
    no conversion.

    Raises:
        ValueError: If the name is not three parts of letters, digits and
            underscores, naming the part at fault.
    """

    __slots__ = ("_interface", "_resource", "_source")

    _source: str
    _resource: str
    _interface: str

    def __new__(cls, value: str) -> Key:
        parts = value.split(SEPARATOR)
        if len(parts) != 3:
            raise ValueError(f"invalid key {value!r}: expected <source>/<resource>/<interface>")
        for label, segment in zip(("source", "resource", "interface"), parts, strict=True):
            if not is_valid_segment(segment):
                raise ValueError(f"invalid {label} segment {segment!r} in key {value!r}")
        key = super().__new__(cls, value)
        key._source, key._resource, key._interface = parts
        return key

    @classmethod
    def of(cls, source: str, resource: str, interface: str) -> Key:
        """Build a name from its three parts.

        Args:
            source: Which robot, e.g. "arm".
            resource: Which part of it, e.g. "joint1".
            interface: What about that part, e.g. "position".
        """
        for label, segment in (
            ("source", source),
            ("resource", resource),
            ("interface", interface),
        ):
            if not is_valid_segment(segment):
                raise ValueError(f"invalid {label} segment {segment!r}: expected [A-Za-z0-9_]+")
        return cls(f"{source}{SEPARATOR}{resource}{SEPARATOR}{interface}")

    @property
    def source(self) -> str:
        """Which robot this belongs to, e.g. "arm"."""
        return self._source

    @property
    def resource(self) -> str:
        """Which part of the robot, e.g. "joint1"."""
        return self._resource

    @property
    def interface(self) -> str:
        """What this says about the part, e.g. "position"."""
        return self._interface

    @property
    def joint(self) -> str:
        """Which part of which robot, e.g. "arm/joint1".

        This is what something asks to take control of.
        """
        return f"{self._source}{SEPARATOR}{self._resource}"
