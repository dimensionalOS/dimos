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

"""LineSegments3D: 3D line segments with a per-segment weight.

On the wire uses ``nav_msgs/Path``. Consecutive pose pairs form segments and
``orientation.w`` carries the weight.
"""

from __future__ import annotations

import struct
import time
from typing import TYPE_CHECKING, BinaryIO

import numpy as np

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from numpy.typing import ArrayLike, NDArray
    from rerun._baseclasses import Archetype

# Path prefix after the 8 byte fingerprint: poses_length, header seq, stamp sec, stamp nsec, frame_id length.
_PREFIX = struct.Struct(">iiiiI")
# PoseStamped bytes before its frame_id text: header seq, stamp sec, stamp nsec, frame_id length.
_POSE_HEAD = 16
_POSE_DOUBLES = 7
_POSE_TAIL = _POSE_DOUBLES * 8


class LineSegments3D(Timestamped):
    """Line segments as an (N, 2, 3) array plus one weight per segment."""

    msg_name = "nav_msgs.LineSegments3D"
    ts: float
    frame_id: str
    segments: NDArray[np.float64]
    weights: NDArray[np.float64]

    def __init__(
        self,
        ts: float | None = None,
        frame_id: str = "map",
        segments: ArrayLike | None = None,
        weights: ArrayLike | None = None,
    ) -> None:
        self.frame_id = frame_id
        self.ts = time.time() if ts is None else ts
        self.segments = np.asarray(
            segments if segments is not None else np.empty((0, 2, 3)), dtype=np.float64
        ).reshape(-1, 2, 3)
        self.weights = (
            np.ones(len(self.segments))
            if weights is None
            else np.asarray(weights, dtype=np.float64).reshape(-1)
        )

    def lcm_encode(self) -> bytes:
        raise NotImplementedError("Encoded on C++ side")

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> LineSegments3D:
        """Read the Path payload through strided array views.

        Every pose header must carry the same frame_id length, which is what the planner emits.
        """
        raw = data if isinstance(data, bytes) else data.read()
        count, _, sec, nsec, frame_len = _PREFIX.unpack_from(raw, 8)
        offset = 8 + _PREFIX.size
        frame_id = raw[offset : offset + frame_len][:-1].decode("utf-8", "replace")
        offset += frame_len
        ts = sec + nsec / 1e9
        if count == 0:
            return cls(ts=ts, frame_id=frame_id)
        if count % 2:
            raise ValueError(f"LineSegments3D needs pose pairs, got {count} poses")
        (pose_frame_len,) = struct.unpack_from(">I", raw, offset + _POSE_HEAD - 4)
        stride = _POSE_HEAD + pose_frame_len + _POSE_TAIL
        lens = np.ndarray(
            (count,), dtype=">u4", buffer=raw, offset=offset + _POSE_HEAD - 4, strides=(stride,)
        )
        if len(raw) != offset + count * stride or not np.all(lens == pose_frame_len):
            raise ValueError("LineSegments3D poses must share one frame_id length")
        poses = np.ndarray(
            (count, _POSE_DOUBLES),
            dtype=">f8",
            buffer=raw,
            offset=offset + _POSE_HEAD + pose_frame_len,
            strides=(stride, 8),
        )
        return cls(
            ts=ts,
            frame_id=frame_id,
            segments=poses[:, :3].astype(np.float64).reshape(-1, 2, 3),
            weights=poses[0::2, 6].astype(np.float64),
        )

    def to_rerun(self, z_offset: float = 0.0, radii: float = 0.04) -> Archetype:
        """Render as ``rr.LineStrips3D``, green to red by log-scale weight."""
        import rerun as rr

        if len(self.segments) == 0:
            return rr.LineStrips3D([])
        strips = self.segments.astype(np.float32)
        strips[:, :, 2] += z_offset
        log_w = np.log10(np.maximum(self.weights, 1e-6))
        lo, hi = float(log_w.min()), float(log_w.max())
        norm = (log_w - lo) / (hi - lo) if hi > lo else np.zeros_like(log_w)
        r = (255 * norm).astype(np.uint8)
        g = (255 * (1.0 - norm)).astype(np.uint8)
        colors = np.column_stack([r, g, np.full_like(r, 60), np.full_like(r, 220)])
        return rr.LineStrips3D(strips, colors=colors, radii=radii)

    def __len__(self) -> int:
        return len(self.segments)

    def __str__(self) -> str:
        return f"LineSegments3D(frame_id='{self.frame_id}', segments={len(self.segments)})"
