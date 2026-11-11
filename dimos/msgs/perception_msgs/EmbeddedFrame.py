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

"""One embedded colour frame: its timestamp and camera frame, its sharpness, its embedding.

LCM schema (fingerprint-compatible bindings must be generated from this)::

    package perception_msgs;

    struct EmbeddedFrame {
        int32_t embedding_length;
        std_msgs.Header header;
        double sharpness;
        double embed_hz;
        string embedder;
        float embedding[embedding_length];
    }
"""

from __future__ import annotations

from io import BytesIO
import struct
import time

from dimos_lcm.std_msgs.Header import Header
import numpy as np

from dimos.types.timestamped import Timestamped

# lcm-gen's base hash for the struct above; the encoded fingerprint also includes Header's hash.
_BASE_HASH = 0x11DA65E31E0F46A0


def _packed_fingerprint() -> bytes:
    tmphash = (_BASE_HASH + Header._get_hash_recursive([EmbeddedFrame])) & 0xFFFFFFFFFFFFFFFF
    tmphash = (((tmphash << 1) & 0xFFFFFFFFFFFFFFFF) + (tmphash >> 63)) & 0xFFFFFFFFFFFFFFFF
    return struct.pack(">Q", tmphash)


class EmbeddedFrame(Timestamped):
    msg_name = "perception_msgs.EmbeddedFrame"

    def __init__(
        self,
        embedding: np.ndarray,
        sharpness: float = 0.0,
        embedder: str = "",
        embed_hz: float = 0.0,
        frame_id: str = "",
        ts: float | None = None,
    ) -> None:
        super().__init__(ts if ts is not None else time.time())
        self.frame_id = frame_id
        self.sharpness = float(sharpness)
        self.embed_hz = float(embed_hz)
        self.embedder = embedder
        self.embedding = np.asarray(embedding, dtype=np.float32).reshape(-1)

    def lcm_encode(self) -> bytes:
        header = Header()
        header.seq = 0
        header.frame_id = self.frame_id
        header.stamp.sec = int(self.ts)
        header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)
        embedder = self.embedder.encode()

        buf = BytesIO()
        buf.write(_packed_fingerprint())
        buf.write(struct.pack(">i", len(self.embedding)))
        header._encode_one(buf)
        buf.write(struct.pack(">dd", self.sharpness, self.embed_hz))
        buf.write(struct.pack(">I", len(embedder) + 1) + embedder + b"\0")
        buf.write(self.embedding.astype(">f4").tobytes())
        return buf.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes) -> EmbeddedFrame:
        buf = BytesIO(data)
        if buf.read(8) != _packed_fingerprint():
            raise ValueError("Decode error")
        (length,) = struct.unpack(">i", buf.read(4))
        header = Header._decode_one(buf)
        sharpness, embed_hz = struct.unpack(">dd", buf.read(16))
        (size,) = struct.unpack(">I", buf.read(4))
        embedder = buf.read(size)[:-1].decode()
        embedding = np.frombuffer(buf.read(4 * length), dtype=">f4").astype(np.float32)
        return cls(
            embedding=embedding,
            sharpness=sharpness,
            embedder=embedder,
            embed_hz=embed_hz,
            frame_id=header.frame_id,
            ts=header.stamp.sec + header.stamp.nsec / 1e9,
        )
