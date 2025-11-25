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

from __future__ import annotations

from datetime import datetime
import time

from dimos_lcm.std_msgs import Header as LCMHeader, Time as LCMTime  # type: ignore[import-untyped]
from plum import dispatch

# Import the actual LCM header type that's returned from decoding
try:
    from lcm_msgs.std_msgs.Header import (  # type: ignore[import-not-found]
        Header as DecodedLCMHeader,
    )
except ImportError:
    DecodedLCMHeader = None


class Header(LCMHeader):  # type: ignore[misc]
    msg_name = "std_msgs.Header"
    ts: float

    @dispatch
    def __init__(self) -> None:
        """Initialize a Header with current time and empty frame_id."""
        self.ts = time.time()
        sec = int(self.ts)
        nsec = int((self.ts - sec) * 1_000_000_000)
        super().__init__(seq=0, stamp=LCMTime(sec=sec, nsec=nsec), frame_id="")

    @dispatch  # type: ignore[no-redef]
    def __init__(self, frame_id: str) -> None:
        """Initialize a Header with current time and specified frame_id."""
        self.ts = time.time()
        sec = int(self.ts)
        nsec = int((self.ts - sec) * 1_000_000_000)
        super().__init__(seq=1, stamp=LCMTime(sec=sec, nsec=nsec), frame_id=frame_id)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, timestamp: float, frame_id: str = "", seq: int = 1) -> None:
        """Initialize a Header with Unix timestamp, frame_id, and optional seq."""
        sec = int(timestamp)
        nsec = int((timestamp - sec) * 1_000_000_000)
        super().__init__(seq=seq, stamp=LCMTime(sec=sec, nsec=nsec), frame_id=frame_id)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, timestamp: datetime, frame_id: str = "") -> None:
        """Initialize a Header with datetime object and frame_id."""
        self.ts = timestamp.timestamp()
        sec = int(self.ts)
        nsec = int((self.ts - sec) * 1_000_000_000)
        super().__init__(seq=1, stamp=LCMTime(sec=sec, nsec=nsec), frame_id=frame_id)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, seq: int, stamp: LCMTime, frame_id: str) -> None:
        """Initialize with explicit seq, stamp, and frame_id (LCM compatibility)."""
        super().__init__(seq=seq, stamp=stamp, frame_id=frame_id)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, header: LCMHeader) -> None:
        """Initialize from another Header (copy constructor)."""
        super().__init__(seq=header.seq, stamp=header.stamp, frame_id=header.frame_id)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, header: object) -> None:
        """Initialize from a decoded LCM header object."""
        # Handle the case where we get an lcm_msgs.std_msgs.Header.Header object
        if hasattr(header, "seq") and hasattr(header, "stamp") and hasattr(header, "frame_id"):
            super().__init__(seq=header.seq, stamp=header.stamp, frame_id=header.frame_id)
        else:
            raise ValueError(f"Cannot create Header from {type(header)}")

    @classmethod
    def now(cls, frame_id: str = "", seq: int = 1) -> Header:
        """Create a Header with current timestamp."""
        ts = time.time()
        return cls(ts, frame_id, seq)

    @property
    def timestamp(self) -> float:
        """Get timestamp as Unix time (float)."""
        return self.stamp.sec + (self.stamp.nsec / 1_000_000_000)  # type: ignore[no-any-return]

    @property
    def datetime(self) -> datetime:
        """Get timestamp as datetime object."""
        return datetime.fromtimestamp(self.timestamp)

    def __str__(self) -> str:
        return f"Header(seq={self.seq}, time={self.timestamp:.6f}, frame_id='{self.frame_id}')"

    def __repr__(self) -> str:
        return f"Header(seq={self.seq}, stamp=Time(sec={self.stamp.sec}, nsec={self.stamp.nsec}), frame_id='{self.frame_id}')"
