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

import struct
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    import os


@dataclass
class Audio(Timestamped):
    """Audio data container."""

    msg_name = "sensor_msgs.Audio"

    data: np.ndarray = field(
        default_factory=lambda: np.zeros((0,), dtype=np.float32)
    )
    sample_rate: int = 16000
    channels: int = 1
    format: str = "F32LE"
    ts: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        if not isinstance(self.data, np.ndarray):
            self.data = np.asarray(self.data, dtype=np.float32)

    def __len__(self) -> int:
        return len(self.data)

    def __str__(self) -> str:
        return (
            f"Audio(len={len(self.data)}, rate={self.sample_rate}, channels={self.channels}, "
            f"ts={self.ts:.3f})"
        )

    def lcm_encode(self) -> bytes:
        """Manual LCM-style encoding since we don't have the generated type."""
        # Simple struct packing: timestamp (double), sample_rate (int), channels (int),
        # data_len (int), data (floats)
        header = struct.pack("<dIII", self.ts, self.sample_rate, self.channels, len(self.data))
        payload = self.data.astype(np.float32).tobytes()
        return header + payload

    @classmethod
    def lcm_decode(cls, data: bytes) -> "Audio":
        """Manual LCM-style decoding."""
        ts, sample_rate, channels, data_len = struct.unpack("<dIII", data[:20])
        payload = data[20:]
        arr = np.frombuffer(payload, dtype=np.float32)
        if len(arr) != data_len:
            raise ValueError(f"Decoded data length {len(arr)} does not match header {data_len}")
        return cls(data=arr, sample_rate=sample_rate, channels=channels, ts=ts)
