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
TrajectoryStatus message type.

Status feedback for trajectory execution.
"""

from enum import IntEnum
from io import BytesIO
import struct
import time


class TrajectoryState(IntEnum):
    """States for trajectory execution."""

    IDLE = 0  # No trajectory, ready to accept
    EXECUTING = 1  # Currently executing trajectory
    COMPLETED = 2  # Trajectory finished successfully
    ABORTED = 3  # Trajectory was cancelled
    FAULT = 4  # Error occurred, requires reset()


class TrajectoryStatus:
    """
    Status of trajectory execution.

    Attributes:
        timestamp: When status was generated
        state: Current TrajectoryState
        progress: Progress 0.0 to 1.0
        time_elapsed: Seconds since trajectory start
        time_remaining: Estimated seconds remaining
        error: Error message if FAULT state (empty string otherwise)
    """

    msg_name = "trajectory_msgs.TrajectoryStatus"

    __slots__ = ["error", "progress", "state", "time_elapsed", "time_remaining", "timestamp"]

    def __init__(
        self,
        state: TrajectoryState = TrajectoryState.IDLE,
        progress: float = 0.0,
        time_elapsed: float = 0.0,
        time_remaining: float = 0.0,
        error: str = "",
        timestamp: float | None = None,
    ) -> None:
        """
        Initialize TrajectoryStatus.

        Args:
            state: Current execution state
            progress: Progress through trajectory (0.0 to 1.0)
            time_elapsed: Time since trajectory start (seconds)
            time_remaining: Estimated time remaining (seconds)
            error: Error message if in FAULT state
            timestamp: When status was generated (defaults to now)
        """
        self.timestamp = timestamp if timestamp is not None else time.time()
        self.state = state
        self.progress = progress
        self.time_elapsed = time_elapsed
        self.time_remaining = time_remaining
        self.error = error

    @property
    def state_name(self) -> str:
        """Get human-readable state name."""
        return self.state.name

    def is_done(self) -> bool:
        """Check if trajectory execution is finished (completed, aborted, or fault)."""
        return self.state in (
            TrajectoryState.COMPLETED,
            TrajectoryState.ABORTED,
            TrajectoryState.FAULT,
        )

    def is_active(self) -> bool:
        """Check if trajectory is currently executing."""
        return self.state == TrajectoryState.EXECUTING

    def lcm_encode(self) -> bytes:
        """Encode for LCM transport."""
        return self.encode()

    def encode(self) -> bytes:
        buf = BytesIO()
        buf.write(TrajectoryStatus._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf: BytesIO) -> None:
        # timestamp (double)
        buf.write(struct.pack(">d", self.timestamp))
        # state (int32)
        buf.write(struct.pack(">i", int(self.state)))
        # progress (double)
        buf.write(struct.pack(">d", self.progress))
        # time_elapsed (double)
        buf.write(struct.pack(">d", self.time_elapsed))
        # time_remaining (double)
        buf.write(struct.pack(">d", self.time_remaining))
        # error (string)
        error_bytes = self.error.encode("utf-8")
        buf.write(struct.pack(">i", len(error_bytes)))
        buf.write(error_bytes)

    @classmethod
    def lcm_decode(cls, data: bytes) -> "TrajectoryStatus":
        """Decode from LCM transport."""
        return cls.decode(data)

    @classmethod
    def decode(cls, data: bytes) -> "TrajectoryStatus":
        buf = BytesIO(data) if not hasattr(data, "read") else data
        if buf.read(8) != cls._get_packed_fingerprint():
            raise ValueError("Decode error: fingerprint mismatch")
        return cls._decode_one(buf)

    @classmethod
    def _decode_one(cls, buf: BytesIO) -> "TrajectoryStatus":
        self = cls.__new__(cls)
        self.timestamp = struct.unpack(">d", buf.read(8))[0]
        self.state = TrajectoryState(struct.unpack(">i", buf.read(4))[0])
        self.progress = struct.unpack(">d", buf.read(8))[0]
        self.time_elapsed = struct.unpack(">d", buf.read(8))[0]
        self.time_remaining = struct.unpack(">d", buf.read(8))[0]
        error_len = struct.unpack(">i", buf.read(4))[0]
        self.error = buf.read(error_len).decode("utf-8")
        return self

    _packed_fingerprint = None

    @classmethod
    def _get_hash_recursive(cls, parents):
        if cls in parents:
            return 0
        return 0x3C4D5E6F708192A3 & 0xFFFFFFFFFFFFFFFF

    @classmethod
    def _get_packed_fingerprint(cls) -> bytes:
        if cls._packed_fingerprint is None:
            cls._packed_fingerprint = struct.pack(">Q", cls._get_hash_recursive([]))
        return cls._packed_fingerprint

    def __str__(self) -> str:
        return f"TrajectoryStatus({self.state_name}, progress={self.progress:.1%})"

    def __repr__(self) -> str:
        return (
            f"TrajectoryStatus(state={self.state_name}, progress={self.progress}, "
            f"time_elapsed={self.time_elapsed}, time_remaining={self.time_remaining}, "
            f"error='{self.error}')"
        )
