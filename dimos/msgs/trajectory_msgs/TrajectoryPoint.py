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

"""
TrajectoryPoint message type.

A single point in a joint trajectory with positions, velocities, and time.
Similar to ROS trajectory_msgs/JointTrajectoryPoint.
"""

from io import BytesIO
import struct


class TrajectoryPoint:
    """
    A single point in a joint trajectory.

    Attributes:
        time_from_start: Time from trajectory start (seconds)
        positions: Joint positions (radians)
        velocities: Joint velocities (rad/s)
    """

    msg_name = "trajectory_msgs.TrajectoryPoint"

    __slots__ = ["num_joints", "positions", "time_from_start", "velocities"]

    def __init__(
        self,
        time_from_start: float = 0.0,
        positions: list[float] | None = None,
        velocities: list[float] | None = None,
    ) -> None:
        """
        Initialize TrajectoryPoint.

        Args:
            time_from_start: Time from trajectory start (seconds)
            positions: Joint positions (radians)
            velocities: Joint velocities (rad/s), defaults to zeros if None
        """
        self.time_from_start = time_from_start
        self.positions = list(positions) if positions else []
        self.num_joints = len(self.positions)

        if velocities is not None:
            self.velocities = list(velocities)
        else:
            self.velocities = [0.0] * self.num_joints

    def lcm_encode(self) -> bytes:
        """Encode for LCM transport."""
        return self.encode()

    def encode(self) -> bytes:
        buf = BytesIO()
        buf.write(TrajectoryPoint._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf: BytesIO) -> None:
        # time_from_start (double)
        buf.write(struct.pack(">d", self.time_from_start))
        # num_joints (int32)
        buf.write(struct.pack(">i", self.num_joints))
        # positions (double[num_joints])
        for p in self.positions:
            buf.write(struct.pack(">d", p))
        # velocities (double[num_joints])
        for v in self.velocities:
            buf.write(struct.pack(">d", v))

    @classmethod
    def lcm_decode(cls, data: bytes) -> "TrajectoryPoint":
        """Decode from LCM transport."""
        return cls.decode(data)

    @classmethod
    def decode(cls, data: bytes) -> "TrajectoryPoint":
        buf = BytesIO(data) if not hasattr(data, "read") else data
        if buf.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error: fingerprint mismatch")
        return cls._decode_one(buf)

    @classmethod
    def _decode_one(cls, buf: BytesIO) -> "TrajectoryPoint":
        self = cls.__new__(cls)
        self.time_from_start = struct.unpack(">d", buf.read(8))[0]
        self.num_joints = struct.unpack(">i", buf.read(4))[0]
        self.positions = [struct.unpack(">d", buf.read(8))[0] for _ in range(self.num_joints)]
        self.velocities = [struct.unpack(">d", buf.read(8))[0] for _ in range(self.num_joints)]
        return self

    _packed_fingerprint = None

    @classmethod
    def _get_hash_recursive(cls, parents):
        if cls in parents:
            return 0
        return 0x1A2B3C4D5E6F7081 & 0xFFFFFFFFFFFFFFFF

    @classmethod
    def _get_packed_fingerprint(cls) -> bytes:
        if cls._packed_fingerprint is None:
            cls._packed_fingerprint = struct.pack(">Q", cls._get_hash_recursive([]))
        return cls._packed_fingerprint

    def __str__(self) -> str:
        return f"TrajectoryPoint(t={self.time_from_start:.3f}s, {self.num_joints} joints)"

    def __repr__(self) -> str:
        return (
            f"TrajectoryPoint(time_from_start={self.time_from_start}, "
            f"positions={self.positions}, velocities={self.velocities})"
        )

    def __eq__(self, other) -> bool:
        if not isinstance(other, TrajectoryPoint):
            return False
        return (
            self.time_from_start == other.time_from_start
            and self.positions == other.positions
            and self.velocities == other.velocities
        )
