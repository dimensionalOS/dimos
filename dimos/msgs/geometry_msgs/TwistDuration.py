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

"""TwistDuration: a Twist with a duration field for timed velocity commands.

Used by agent_cmd_vel to specify how long a velocity command should be
executed before automatically stopping.
"""

from __future__ import annotations

from io import BytesIO
import struct
import sys
from typing import Any

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike


class _LCMTwistDuration:
    """LCM-compatible base providing encode/decode for TwistDuration.

    Wire format (big-endian):
        8 bytes  — packed fingerprint
        48 bytes — Twist (linear Vector3 + angular Vector3, 6 doubles)
        8 bytes  — duration (double)
    """

    msg_name = "geometry_msgs.TwistDuration"

    __slots__ = ["angular", "duration", "linear"]

    __typenames__ = ["Vector3", "Vector3", "double"]
    __dimensions__ = [None, None, None]

    linear: Vector3
    angular: Vector3
    duration: float

    def __init__(
        self,
        linear: Any = None,
        angular: Any = None,
        duration: float = 0.0,
    ) -> None:
        from dimos_lcm.geometry_msgs import Vector3 as LCMVector3

        self.linear = linear if linear is not None else LCMVector3()
        self.angular = angular if angular is not None else LCMVector3()
        self.duration = float(duration)

    # -- encode --------------------------------------------------------

    def lcm_encode(self) -> bytes:
        buf = BytesIO()
        buf.write(self._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf: BytesIO) -> None:
        self.linear._encode_one(buf)
        self.angular._encode_one(buf)
        buf.write(struct.pack(">d", self.duration))

    # -- decode --------------------------------------------------------

    @classmethod
    def lcm_decode(cls, data: bytes) -> _LCMTwistDuration:
        buf = BytesIO(data) if not hasattr(data, "read") else data  # type: ignore[arg-type]
        if buf.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error")
        return cls._decode_one(buf)

    @classmethod
    def _decode_one(cls, buf: BytesIO) -> _LCMTwistDuration:
        self = cls.__new__(cls)
        field_type = cls._get_field_type("linear")
        self.linear = field_type._decode_one(buf)
        self.angular = field_type._decode_one(buf)
        (self.duration,) = struct.unpack(">d", buf.read(8))
        return self

    @classmethod
    def _get_field_type(cls, field_name: str) -> Any:
        annotation = cls.__annotations__.get(field_name)
        if annotation is None:
            return None
        if isinstance(annotation, str):
            module = sys.modules[cls.__module__]
            if hasattr(module, annotation):
                return getattr(module, annotation)
            return None
        return annotation

    # -- fingerprint ---------------------------------------------------
    # Hash scheme follows LCM convention. The base hash is an arbitrary
    # 64-bit constant unique to this message type.  Sub-type hashes
    # (Vector3 × 2) are folded in with a rotate-left-1.

    @classmethod
    def _get_hash_recursive(cls, parents: list[type]) -> int:
        if cls in parents:
            return 0
        from dimos_lcm.geometry_msgs import Vector3 as LCMVector3

        newparents = [*parents, cls]
        # Arbitrary base hash for TwistDuration (different from Twist)
        tmphash = (
            0x7B3E9A1F4D2C8B05
            + LCMVector3._get_hash_recursive(newparents)
            + LCMVector3._get_hash_recursive(newparents)
        ) & 0xFFFFFFFFFFFFFFFF
        tmphash = (((tmphash << 1) & 0xFFFFFFFFFFFFFFFF) + (tmphash >> 63)) & 0xFFFFFFFFFFFFFFFF
        return tmphash

    _packed_fingerprint: bytes | None = None

    @classmethod
    def _get_packed_fingerprint(cls) -> bytes:
        if cls._packed_fingerprint is None:
            cls._packed_fingerprint = struct.pack(">Q", cls._get_hash_recursive([]))
        return cls._packed_fingerprint


class TwistDuration(_LCMTwistDuration):
    """A Twist with a duration: execute this velocity for *duration* seconds.

    Attributes:
        linear:   Linear velocity (Vector3).
        angular:  Angular velocity (Vector3).
        duration: How long to apply the velocity (seconds).  0 means
                  "apply once / indefinite" (same as a plain Twist).
    """

    linear: Vector3
    angular: Vector3
    duration: float
    msg_name = "geometry_msgs.TwistDuration"

    def __init__(
        self,
        linear: VectorLike | None = None,
        angular: VectorLike | None = None,
        duration: float = 0.0,
    ) -> None:
        self.linear = Vector3() if linear is None else Vector3(linear)
        self.angular = Vector3() if angular is None else Vector3(angular)
        self.duration = float(duration)

    def to_twist(self) -> Twist:
        """Return just the velocity portion as a plain Twist."""
        return Twist(self.linear, self.angular)

    @classmethod
    def from_twist(cls, twist: Twist, duration: float = 0.0) -> TwistDuration:
        """Create a TwistDuration from an existing Twist."""
        return cls(linear=twist.linear, angular=twist.angular, duration=duration)

    def __repr__(self) -> str:
        return (
            f"TwistDuration(linear={self.linear!r}, "
            f"angular={self.angular!r}, duration={self.duration})"
        )

    def __str__(self) -> str:
        return (
            f"TwistDuration:\n"
            f"  Linear:   {self.linear}\n"
            f"  Angular:  {self.angular}\n"
            f"  Duration: {self.duration}s"
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TwistDuration):
            return False
        return (
            self.linear == other.linear
            and self.angular == other.angular
            and self.duration == other.duration
        )

    def is_zero(self) -> bool:
        """Check if linear and angular velocities are both zero."""
        return self.linear.is_zero() and self.angular.is_zero()


__all__ = ["TwistDuration"]
