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

"""Python overlay for audio messages, mirroring ROS audio_common AudioStamped.

LCM wire type: foxglove_msgs.RawAudio (already in dimos_lcm).

Differences from ROS audio_common:
  - Wire type uses builtin_interfaces.Time (not std_msgs.Header), so frame_id
    and seq are NOT preserved through lcm_encode/lcm_decode.
  - Wire type has a single 'format' string; we encode it as
    "{coding_format}/{sample_format}" (e.g. "pcm/S16LE").
  - AudioInfo and AudioData are not separate LCM sub-types; they are plain
    Python fields on this class.
"""

from __future__ import annotations

import time

from dimos_lcm.builtin_interfaces import Time as LCMTime
from dimos_lcm.foxglove_msgs import RawAudio
import numpy as np

from dimos.msgs.std_msgs.Header import Header
from dimos.types.timestamped import Timestamped


class AudioStamped(Timestamped):
    """Stamped audio chunk with PCM payload.

    Carries one frame of raw PCM audio together with a std_msgs.Header and
    audio metadata.  Serialises to/from foxglove_msgs.RawAudio on the wire.
    """

    msg_name = "foxglove_msgs.RawAudio"  # wire type used for LCM

    def __init__(
        self,
        header: Header,
        sample_rate: int,
        channels: int,
        sample_format: str,
        coding_format: str,
        data: bytes,
    ) -> None:
        super().__init__(ts=header.timestamp)
        self.header = header
        self.sample_rate = sample_rate
        self.channels = channels
        self.sample_format = sample_format
        self.coding_format = coding_format
        self.data = data

    # ------------------------------------------------------------------
    # Factory helpers
    # ------------------------------------------------------------------

    @classmethod
    def from_pcm(
        cls,
        pcm_bytes: bytes,
        sample_rate: int = 16000,
        channels: int = 1,
        sample_format: str = "S16LE",
        coding_format: str = "pcm",
        frame_id: str = "",
        ts: float | None = None,
    ) -> AudioStamped:
        """Construct from raw PCM bytes."""
        t = ts if ts is not None else time.monotonic()
        header = Header(t, frame_id)
        return cls(
            header=header,
            sample_rate=sample_rate,
            channels=channels,
            sample_format=sample_format,
            coding_format=coding_format,
            data=pcm_bytes,
        )

    # ------------------------------------------------------------------
    # Conversion helpers
    # ------------------------------------------------------------------

    def to_numpy(self) -> np.ndarray:
        """Decode PCM bytes to a numpy array.

        Returns shape (n_samples,) for mono or (n_samples, channels) for
        multi-channel.  dtype is int16 for S16LE, float32 for F32LE.
        """
        if self.sample_format == "S16LE":
            dtype = np.dtype("<i2")
        elif self.sample_format in ("F32LE", "F32"):
            dtype = np.dtype("<f4")
        else:
            raise ValueError(f"Unsupported sample_format for to_numpy: {self.sample_format}")

        arr = np.frombuffer(self.data, dtype=dtype)
        if self.channels > 1:
            arr = arr.reshape(-1, self.channels)
        return arr

    # ------------------------------------------------------------------
    # LCM serialisation
    # ------------------------------------------------------------------

    def lcm_encode(self) -> bytes:
        """Encode to foxglove_msgs.RawAudio wire bytes.

        NOTE: frame_id and seq from self.header are NOT preserved (the wire
        type only carries a bare timestamp, not a full std_msgs.Header).
        """
        msg = RawAudio()
        sec = int(self.ts)
        nsec = int((self.ts - sec) * 1_000_000_000)
        msg.timestamp = LCMTime(sec=sec, nanosec=nsec)
        msg.format = f"{self.coding_format}/{self.sample_format}"
        msg.sample_rate = self.sample_rate
        msg.number_of_channels = self.channels
        msg.data_length = len(self.data)
        msg.data = self.data
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, raw: bytes) -> AudioStamped:
        """Decode from foxglove_msgs.RawAudio wire bytes."""
        msg = RawAudio.lcm_decode(raw)

        ts = msg.timestamp.sec + msg.timestamp.nanosec / 1_000_000_000

        fmt = msg.format  # e.g. "pcm/S16LE"
        if "/" in fmt:
            coding_format, sample_format = fmt.split("/", 1)
        else:
            coding_format, sample_format = "pcm", fmt

        return cls.from_pcm(
            pcm_bytes=bytes(msg.data),
            sample_rate=msg.sample_rate,
            channels=msg.number_of_channels,
            sample_format=sample_format,
            coding_format=coding_format,
            frame_id="",  # not stored on wire
            ts=ts,
        )

    # ------------------------------------------------------------------
    # Repr
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        n_samples = len(self.data) // (2 if "16" in self.sample_format else 4)
        return (
            f"AudioStamped(rate={self.sample_rate}, ch={self.channels}, "
            f"fmt={self.sample_format}, samples={n_samples}, ts={self.ts:.6f})"
        )
