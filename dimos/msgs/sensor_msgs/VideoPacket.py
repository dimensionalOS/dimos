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

from __future__ import annotations

from dataclasses import dataclass
import json
import struct
from typing import Any, ClassVar

_MAGIC = b"DVP1"


@dataclass(frozen=True)
class VideoPacket:
    """One encoded video frame/access unit.

    The first supported shape is a complete H.264 Annex B access unit for
    exactly one source image frame. Delta frames are complete encoded-frame
    packets, but they are not necessarily independently decodable without the
    preceding GOP state.
    """

    msg_name: ClassVar[str] = "sensor_msgs.VideoPacket"

    seq: int
    ts: float
    frame_id: str
    width: int
    height: int
    format: str
    codec: str
    bitstream: str
    is_keyframe: bool
    keyframe_seq: int
    pts: int
    data: bytes

    def __post_init__(self) -> None:
        if self.seq < 0:
            raise ValueError("seq must be non-negative")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive")
        if self.codec != "h264":
            raise ValueError(f"Unsupported video codec: {self.codec!r}")
        if self.bitstream != "annex_b":
            raise ValueError(f"Unsupported video bitstream: {self.bitstream!r}")
        if not isinstance(self.data, bytes):
            object.__setattr__(self, "data", bytes(self.data))
        if len(self.data) == 0:
            raise ValueError("VideoPacket data must not be empty")

    def lcm_encode(self) -> bytes:
        """Encode into a compact self-describing binary envelope."""

        header = {
            "seq": self.seq,
            "ts": self.ts,
            "frame_id": self.frame_id,
            "width": self.width,
            "height": self.height,
            "format": self.format,
            "codec": self.codec,
            "bitstream": self.bitstream,
            "is_keyframe": self.is_keyframe,
            "keyframe_seq": self.keyframe_seq,
            "pts": self.pts,
        }
        header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
        return _MAGIC + struct.pack("!I", len(header_bytes)) + header_bytes + self.data

    @classmethod
    def lcm_decode(cls, payload: bytes) -> VideoPacket:
        """Decode a packet produced by :meth:`lcm_encode`."""

        if len(payload) < 8 or payload[:4] != _MAGIC:
            raise ValueError("Invalid VideoPacket payload")
        header_len = struct.unpack("!I", payload[4:8])[0]
        header_start = 8
        header_end = header_start + header_len
        if header_end > len(payload):
            raise ValueError("Truncated VideoPacket header")
        header: dict[str, Any] = json.loads(payload[header_start:header_end].decode("utf-8"))
        return cls(data=payload[header_end:], **header)


__all__ = ["VideoPacket"]
