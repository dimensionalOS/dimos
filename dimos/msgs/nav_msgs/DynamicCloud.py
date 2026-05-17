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

"""DynamicCloud: a per-voxel point cloud with a separate sparse event log.

Designed for voxel-grid maps where each "point" is a voxel cell carrying
a quantity (occupancy / health / hit count), and a sparse second array
records timestamped events that reference points by index — useful for
expressing "this voxel was last seen at time T" without paying the
per-point cost when most voxels have no event.

Wire format (little-endian, packed)::

    u64   timestamp_nanos               # overall message timestamp
    f32   voxel_size                    # meters per voxel edge
    u16   frame_id_len
    bytes frame_id                      # utf-8, frame_id_len bytes
    u32   num_points
    i32[N*3]  voxels                    # (x, y, z) interleaved
    u32[N]    quantity                  # per-point quantity
    u32   num_events
    u32[M]    event_indices             # indices into voxels (0 ≤ idx < N)
    u64[M]    event_timestamps          # nanoseconds

`num_events` is independent of `num_points`: events can be empty, can
reference the same point multiple times, and don't need to cover every
point. The Rust mirror lives at
``dimos/mapping/ray_tracing/rust/src/dynamic_cloud.rs`` and must stay
in sync with this format. ``test_dynamic_cloud.py`` pins a known-bytes
fixture that both sides assert against.
"""

from __future__ import annotations

import struct
from typing import TYPE_CHECKING

import numpy as np

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype


_HEADER_FMT = "<QfH"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)
_U32_FMT = "<I"
_U32_SIZE = struct.calcsize(_U32_FMT)


class DynamicCloud(Timestamped):
    """Per-voxel point cloud + sparse timestamped event log."""

    msg_name = "nav_msgs.DynamicCloud"

    def __init__(
        self,
        voxels: np.ndarray | None = None,
        quantity: np.ndarray | None = None,
        event_indices: np.ndarray | None = None,
        event_timestamps: np.ndarray | None = None,
        voxel_size: float = 0.1,
        frame_id: str = "world",
        ts: float | None = None,
    ) -> None:
        self.ts = ts if ts is not None else 0.0  # type: ignore[assignment]
        self.frame_id = frame_id
        self.voxel_size = float(voxel_size)

        if voxels is None:
            voxels = np.zeros((0, 3), dtype=np.int32)
        if quantity is None:
            quantity = np.zeros(0, dtype=np.uint32)
        if event_indices is None:
            event_indices = np.zeros(0, dtype=np.uint32)
        if event_timestamps is None:
            event_timestamps = np.zeros(0, dtype=np.uint64)

        voxels = np.ascontiguousarray(voxels, dtype=np.int32)
        if voxels.ndim != 2 or voxels.shape[1] != 3:
            raise ValueError(f"voxels must have shape (N, 3), got {voxels.shape}")

        quantity = np.ascontiguousarray(quantity, dtype=np.uint32).reshape(-1)
        event_indices = np.ascontiguousarray(event_indices, dtype=np.uint32).reshape(-1)
        event_timestamps = np.ascontiguousarray(event_timestamps, dtype=np.uint64).reshape(-1)

        num_points = voxels.shape[0]
        if quantity.shape[0] != num_points:
            raise ValueError(
                f"voxels/quantity length mismatch: {num_points} vs {quantity.shape[0]}"
            )
        if event_indices.shape[0] != event_timestamps.shape[0]:
            raise ValueError(
                f"event_indices/event_timestamps length mismatch: "
                f"{event_indices.shape[0]} vs {event_timestamps.shape[0]}"
            )
        if num_points == 0 and event_indices.shape[0] > 0:
            raise ValueError("event_indices nonempty but voxels is empty")
        if num_points > 0 and event_indices.shape[0] > 0:
            max_idx = int(event_indices.max())
            if max_idx >= num_points:
                raise ValueError(f"event index {max_idx} out of range for {num_points} points")

        self.voxels = voxels
        self.quantity = quantity
        self.event_indices = event_indices
        self.event_timestamps = event_timestamps

    def __len__(self) -> int:
        return int(self.voxels.shape[0])

    def world_positions(self) -> np.ndarray:
        """Return points reprojected to world space as `(N, 3) float32`."""
        return self.voxels.astype(np.float32) * np.float32(self.voxel_size)

    def per_point_latest_timestamp(self) -> np.ndarray:
        """Return the latest event timestamp per point, 0 if no events touch a point.

        Useful for visualization or "freshness" coloring. Shape ``(N,) uint64``.
        """
        result = np.zeros(len(self), dtype=np.uint64)
        if self.event_indices.size == 0:
            return result
        # For each event, keep the max timestamp per index.
        np.maximum.at(result, self.event_indices, self.event_timestamps)
        return result

    def lcm_encode(self) -> bytes:
        frame_bytes = self.frame_id.encode("utf-8")
        if len(frame_bytes) > 0xFFFF:
            raise ValueError(f"frame_id too long: {len(frame_bytes)} > 65535 bytes")
        timestamp_nanos = int(self.ts * 1_000_000_000) if self.ts else 0
        if timestamp_nanos < 0:
            timestamp_nanos = 0

        header = struct.pack(_HEADER_FMT, timestamp_nanos, self.voxel_size, len(frame_bytes))
        num_points_bytes = struct.pack(_U32_FMT, len(self))
        num_events_bytes = struct.pack(_U32_FMT, int(self.event_indices.shape[0]))
        return b"".join(
            [
                header,
                frame_bytes,
                num_points_bytes,
                self.voxels.tobytes(),
                self.quantity.tobytes(),
                num_events_bytes,
                self.event_indices.tobytes(),
                self.event_timestamps.tobytes(),
            ]
        )

    @classmethod
    def lcm_decode(cls, data: bytes) -> DynamicCloud:
        if len(data) < _HEADER_SIZE:
            raise ValueError(f"DynamicCloud: data too short for header ({len(data)} bytes)")
        timestamp_nanos, voxel_size, frame_id_len = struct.unpack_from(_HEADER_FMT, data, 0)
        offset = _HEADER_SIZE

        if len(data) < offset + frame_id_len + _U32_SIZE:
            raise ValueError("DynamicCloud: data too short for frame_id + num_points")
        frame_id = data[offset : offset + frame_id_len].decode("utf-8")
        offset += frame_id_len

        (num_points,) = struct.unpack_from(_U32_FMT, data, offset)
        offset += _U32_SIZE

        voxels_size = num_points * 3 * 4
        quantity_size = num_points * 4
        if len(data) < offset + voxels_size + quantity_size + _U32_SIZE:
            raise ValueError("DynamicCloud: data too short for voxels + quantity + num_events")

        voxels = np.frombuffer(data, dtype=np.int32, count=num_points * 3, offset=offset).reshape(
            num_points, 3
        )
        offset += voxels_size
        quantity = np.frombuffer(data, dtype=np.uint32, count=num_points, offset=offset)
        offset += quantity_size

        (num_events,) = struct.unpack_from(_U32_FMT, data, offset)
        offset += _U32_SIZE

        events_idx_size = num_events * 4
        events_ts_size = num_events * 8
        expected_tail = events_idx_size + events_ts_size
        if len(data) - offset != expected_tail:
            raise ValueError(
                f"DynamicCloud: payload size mismatch "
                f"(expected {expected_tail} tail bytes, got {len(data) - offset})"
            )

        event_indices = np.frombuffer(data, dtype=np.uint32, count=num_events, offset=offset)
        offset += events_idx_size
        event_timestamps = np.frombuffer(data, dtype=np.uint64, count=num_events, offset=offset)

        return cls(
            voxels=voxels.copy(),
            quantity=quantity.copy(),
            event_indices=event_indices.copy(),
            event_timestamps=event_timestamps.copy(),
            voxel_size=voxel_size,
            frame_id=frame_id,
            ts=timestamp_nanos / 1_000_000_000 if timestamp_nanos > 0 else None,
        )

    def to_rerun(
        self,
        colormap: str = "turbo",
        radii: float | None = None,
        normalize_quantity: bool = True,
    ) -> Archetype:
        """Return an `rr.Points3D` archetype colored by `quantity`.

        Events are not visualized by default (use `per_point_latest_timestamp()`
        if you need to derive a freshness-based visualization).
        """
        import rerun as rr

        positions = self.world_positions()
        if len(positions) == 0:
            return rr.Points3D([])

        colors = self._quantity_colors(colormap, normalize=normalize_quantity)
        radius = self.voxel_size / 2 if radii is None else radii
        return rr.Points3D(positions=positions, colors=colors, radii=radius)

    def _quantity_colors(self, colormap: str, normalize: bool) -> np.ndarray:
        import matplotlib.pyplot as plt

        quantity = self.quantity.astype(np.float32)
        if normalize and quantity.size > 0:
            lo, hi = float(quantity.min()), float(quantity.max())
            spread = hi - lo
            t = (quantity - lo) / spread if spread > 0 else np.zeros_like(quantity)
        else:
            t = np.clip(quantity / 255.0, 0.0, 1.0)
        rgba = plt.get_cmap(colormap)(t)
        return np.asarray(rgba[:, :3] * 255, dtype=np.uint8)
