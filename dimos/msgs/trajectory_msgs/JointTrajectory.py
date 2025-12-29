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
JointTrajectory message type.

A sequence of joint trajectory points representing a full trajectory.
Similar to ROS trajectory_msgs/JointTrajectory.
"""

from io import BytesIO
import struct
import time

from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


class JointTrajectory:
    """
    A joint-space trajectory consisting of timestamped waypoints.

    Attributes:
        timestamp: When trajectory was created (seconds since epoch)
        joint_names: Names of joints (optional)
        points: Sequence of TrajectoryPoints
        duration: Total trajectory duration (seconds)
    """

    msg_name = "trajectory_msgs.JointTrajectory"

    __slots__ = ["duration", "joint_names", "num_joints", "num_points", "points", "timestamp"]

    def __init__(
        self,
        points: list[TrajectoryPoint] | None = None,
        joint_names: list[str] | None = None,
        timestamp: float | None = None,
    ) -> None:
        """
        Initialize JointTrajectory.

        Args:
            points: List of TrajectoryPoints
            joint_names: Names of joints (optional)
            timestamp: Creation timestamp (defaults to now)
        """
        self.timestamp = timestamp if timestamp is not None else time.time()
        self.points = list(points) if points else []
        self.num_points = len(self.points)
        self.joint_names = list(joint_names) if joint_names else []
        self.num_joints = (
            len(self.joint_names)
            if self.joint_names
            else (self.points[0].num_joints if self.points else 0)
        )

        # Compute duration from last point
        if self.points:
            self.duration = max(p.time_from_start for p in self.points)
        else:
            self.duration = 0.0

    def sample(self, t: float) -> tuple[list[float], list[float]]:
        """
        Sample the trajectory at time t using linear interpolation.

        Args:
            t: Time from trajectory start (seconds)

        Returns:
            Tuple of (positions, velocities) at time t
        """
        if not self.points:
            return [], []

        # Clamp t to valid range
        t = max(0.0, min(t, self.duration))

        # Find bracketing points
        if t <= self.points[0].time_from_start:
            return list(self.points[0].positions), list(self.points[0].velocities)

        if t >= self.points[-1].time_from_start:
            return list(self.points[-1].positions), list(self.points[-1].velocities)

        # Find interval
        for i in range(len(self.points) - 1):
            t0 = self.points[i].time_from_start
            t1 = self.points[i + 1].time_from_start

            if t0 <= t <= t1:
                # Linear interpolation
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                p0 = self.points[i]
                p1 = self.points[i + 1]

                positions = [
                    p0.positions[j] + alpha * (p1.positions[j] - p0.positions[j])
                    for j in range(len(p0.positions))
                ]
                velocities = [
                    p0.velocities[j] + alpha * (p1.velocities[j] - p0.velocities[j])
                    for j in range(len(p0.velocities))
                ]
                return positions, velocities

        # Fallback
        return list(self.points[-1].positions), list(self.points[-1].velocities)

    def lcm_encode(self) -> bytes:
        """Encode for LCM transport."""
        return self.encode()

    def encode(self) -> bytes:
        buf = BytesIO()
        buf.write(JointTrajectory._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf: BytesIO) -> None:
        # timestamp (double)
        buf.write(struct.pack(">d", self.timestamp))
        # duration (double)
        buf.write(struct.pack(">d", self.duration))
        # num_joint_names (int32) - actual count of joint names
        buf.write(struct.pack(">i", len(self.joint_names)))
        # joint_names (string[num_joint_names])
        for name in self.joint_names:
            name_bytes = name.encode("utf-8")
            buf.write(struct.pack(">i", len(name_bytes)))
            buf.write(name_bytes)
        # num_points (int32)
        buf.write(struct.pack(">i", self.num_points))
        # points (TrajectoryPoint[num_points])
        for point in self.points:
            point._encode_one(buf)

    @classmethod
    def lcm_decode(cls, data: bytes) -> "JointTrajectory":
        """Decode from LCM transport."""
        return cls.decode(data)

    @classmethod
    def decode(cls, data: bytes) -> "JointTrajectory":
        buf = BytesIO(data) if not hasattr(data, "read") else data
        if buf.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error: fingerprint mismatch")
        return cls._decode_one(buf)

    @classmethod
    def _decode_one(cls, buf: BytesIO) -> "JointTrajectory":
        self = cls.__new__(cls)
        self.timestamp = struct.unpack(">d", buf.read(8))[0]
        self.duration = struct.unpack(">d", buf.read(8))[0]

        # Read joint names
        num_joint_names = struct.unpack(">i", buf.read(4))[0]
        self.joint_names = []
        for _ in range(num_joint_names):
            name_len = struct.unpack(">i", buf.read(4))[0]
            self.joint_names.append(buf.read(name_len).decode("utf-8"))

        # Read points
        self.num_points = struct.unpack(">i", buf.read(4))[0]
        self.points = [TrajectoryPoint._decode_one(buf) for _ in range(self.num_points)]

        # Set num_joints from joint_names or points
        self.num_joints = (
            len(self.joint_names)
            if self.joint_names
            else (self.points[0].num_joints if self.points else 0)
        )

        return self

    _packed_fingerprint = None

    @classmethod
    def _get_hash_recursive(cls, parents):
        if cls in parents:
            return 0
        return 0x2B3C4D5E6F708192 & 0xFFFFFFFFFFFFFFFF

    @classmethod
    def _get_packed_fingerprint(cls) -> bytes:
        if cls._packed_fingerprint is None:
            cls._packed_fingerprint = struct.pack(">Q", cls._get_hash_recursive([]))
        return cls._packed_fingerprint

    def __str__(self) -> str:
        return f"JointTrajectory({self.num_points} points, duration={self.duration:.3f}s)"

    def __repr__(self) -> str:
        return (
            f"JointTrajectory(points={self.points}, joint_names={self.joint_names}, "
            f"timestamp={self.timestamp})"
        )

    def __len__(self) -> int:
        return self.num_points
