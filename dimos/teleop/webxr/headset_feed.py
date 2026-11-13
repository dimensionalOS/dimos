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

"""The wrist camera's picture and point cloud, made light enough to stream to a VR headset."""

from __future__ import annotations

import asyncio
import threading
import time

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.voxels.keys import KEY_OFFSET, pack_indices
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A missing transform drops every cloud until it shows up: say so, not at the clouds' rate.
_MISSING_TRANSFORM_WARN_INTERVAL_S = 5.0


class HeadsetFeedConfig(ModuleConfig):
    # The frame the cloud is sent in: the one the arm's links are published in.
    world_frame: str = "world"
    # Edge of the grid each cloud is thinned on, metres: one point is kept per cell.
    cloud_voxel_m: float = Field(default=0.015, gt=0.0)
    # Most clouds taken a second; the ones in between are dropped. A cloud taken is sent,
    # or merged into the map when `accumulate` is on.
    cloud_hz: float = Field(default=4.0, gt=0.0)
    # Send every point the camera has seen, not only its newest cloud: a map kept on a
    # `map_voxel_m` grid, where a newer point replaces the one in its cell, colour included.
    accumulate: bool = True
    # Edge of the map's grid, metres: one point is kept per cell.
    map_voxel_m: float = Field(default=0.02, gt=0.0)
    # Most points the map keeps. Once full it keeps what it has: a point in a cell it
    # holds still replaces the old one, a point in a new cell is dropped until
    # `clear_cloud`. 150,000 is the most the Manipulation app's VR page draws.
    accumulate_max_points: int = Field(default=150_000, gt=0)
    # Most maps sent a second while `accumulate` is on. A point is 16 bytes on the wire
    # (x, y, z and packed rgb, float32 each), so a full map is 150,000 x 16 = 2.4 MB a
    # message. At 0.5 a second that is 1.2 MB/s, plus the pictures (8 a second at roughly
    # 40 KB, 0.32 MB/s) about 1.5 MB/s before the page's zstd, under the ~2 MB/s a Quest
    # on Wi-Fi keeps up with. At 1 a second it would be about 2.7 MB/s.
    accumulated_cloud_hz: float = Field(default=0.5, gt=0.0)
    # How far from a cloud's capture time the camera's transform may be, and how long
    # to wait for one still in flight, seconds. A cloud with no transform is dropped.
    tf_tolerance_s: float = Field(default=0.1, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.2, ge=0.0)
    # Most pictures sent a second, their JPEG quality (1-100) and widest side, pixels.
    camera_hz: float = Field(default=8.0, gt=0.0)
    jpeg_quality: int = Field(default=70, ge=1, le=100)
    camera_max_px: int = Field(default=640, gt=0)


class HeadsetFeedModule(Module):
    """Send the wrist camera to a VR headset over Wi-Fi.

    The picture goes out as JPEG on ``vr_camera``. The point cloud goes out on
    ``vr_cloud`` in the world frame, placed with the camera's pose at the moment
    it was captured and thinned to one point per ``cloud_voxel_m`` cell, colours
    kept. With ``accumulate`` on, ``vr_cloud`` carries every ``map_voxel_m`` cell
    seen so far, up to ``accumulate_max_points``, so the cloud grows as the camera
    looks around; ``clear_cloud`` empties it.
    """

    config: HeadsetFeedConfig

    color_image: In[Image]
    pointcloud: In[PointCloud2]
    tf: In[TFMessage]
    vr_camera: Out[CompressedImage]
    vr_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._next_camera_t = 0.0
        self._next_cloud_t = 0.0
        self._next_map_t = 0.0
        self._missing_warned_at: float | None = None
        self._cloud_map = _CloudMap(self.config.map_voxel_m, self.config.accumulate_max_points)

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()

    @rpc
    def clear_cloud(self) -> None:
        """Forget every point kept so far; the map starts again from the next cloud."""
        self._cloud_map.clear()

    async def handle_color_image(self, image: Image) -> None:
        now = time.monotonic()
        if now < self._next_camera_t:
            return
        self._next_camera_t = _next_due(self._next_camera_t, now, self.config.camera_hz)
        picture = await asyncio.to_thread(self.light_picture, image)
        self.vr_camera.publish(picture)

    async def handle_pointcloud(self, cloud: PointCloud2) -> None:
        now = time.monotonic()
        if now < self._next_cloud_t:
            return
        self._next_cloud_t = _next_due(self._next_cloud_t, now, self.config.cloud_hz)
        light = await asyncio.to_thread(self.light_cloud, cloud)
        if light is None:
            return
        if not self.config.accumulate:
            self.vr_cloud.publish(light)
            return
        cap = self.config.accumulate_max_points
        before = len(self._cloud_map)
        await asyncio.to_thread(self._cloud_map.add, light)
        if before < cap <= len(self._cloud_map):
            logger.info(
                "Headset map is full at %d points: new places are not added until clear_cloud",
                cap,
            )
        if now < self._next_map_t:
            return
        self._next_map_t = _next_due(self._next_map_t, now, self.config.accumulated_cloud_hz)
        self.vr_cloud.publish(self._cloud_map.cloud(self.config.world_frame, light.ts))

    def light_picture(self, image: Image) -> CompressedImage:
        """The picture as a JPEG no wider or taller than ``camera_max_px``."""
        return CompressedImage.from_image(
            image,
            "jpeg",
            quality=self.config.jpeg_quality,
            max_width=self.config.camera_max_px,
        )

    def light_cloud(self, cloud: PointCloud2) -> PointCloud2 | None:
        """The cloud in the world frame, one point per voxel; None while the camera has no pose."""
        world = self.config.world_frame
        if cloud.frame_id == world:
            return cloud.voxel_downsample(self.config.cloud_voxel_m)
        transform = self.tfbuffer.get(
            world,
            cloud.frame_id,
            cloud.ts,
            self.config.tf_tolerance_s,
            forward_tolerance=self.config.tf_forward_tolerance_s,
            warn=False,
        )
        if transform is None:
            now = time.monotonic()
            if (
                self._missing_warned_at is None
                or now - self._missing_warned_at >= _MISSING_TRANSFORM_WARN_INTERVAL_S
            ):
                self._missing_warned_at = now
                logger.warning(
                    "No transform from %s to %s near %.3f; clouds are not sent",
                    world,
                    cloud.frame_id,
                    cloud.ts,
                )
            return None
        # Moved first, so the grid is fixed in the world and still points stay still.
        return cloud.transform(transform).voxel_downsample(self.config.cloud_voxel_m)


class _CloudMap:
    """A cloud that grows: the points of every cloud added, one per cube of a fixed grid, the newest kept.

    Args:
        voxel_m: Edge of a cube, metres.
        max_points: Most points kept. Once full, a point in a cube already held still
            replaces the old one; a point in a new cube is dropped until `clear`.
    """

    def __init__(self, voxel_m: float, max_points: int) -> None:
        self._voxel_m = voxel_m
        self._max_points = max_points
        self._lock = threading.Lock()
        self._keys = np.empty(0, dtype=np.int64)
        self._points = np.empty((0, 3), dtype=np.float32)
        self._colors = np.empty((0, 3), dtype=np.float32)

    def __len__(self) -> int:
        return len(self._keys)

    def add(self, cloud: PointCloud2) -> None:
        """Merge in a cloud already in the map's frame; each of its points replaces the one in its cube.

        A cloud without colours is kept grey.
        """
        points = cloud.points_f32()
        fields = cloud.pointcloud_tensor.point
        colors = (
            fields["colors"].numpy().astype(np.float32)
            if "colors" in fields
            else np.full_like(points, 0.5)
        )
        finite = np.isfinite(points).all(axis=1)
        points, colors = points[finite], colors[finite]
        keys = pack_indices(np.floor(points / self._voxel_m).astype(np.int64) + KEY_OFFSET)
        # One point per cube within the cloud as well: the last one in it.
        keys, last_reversed = np.unique(keys[::-1], return_index=True)
        last = len(points) - 1 - last_reversed
        points, colors = points[last], colors[last]
        with self._lock:
            known = np.isin(keys, self._keys)
            # Every cube already held, and new ones only while there is room.
            take = known.copy()
            take[np.flatnonzero(~known)[: self._max_points - len(self._keys)]] = True
            untouched = ~np.isin(self._keys, keys)
            self._keys = np.concatenate([self._keys[untouched], keys[take]])
            self._points = np.concatenate([self._points[untouched], points[take]])
            self._colors = np.concatenate([self._colors[untouched], colors[take]])

    def cloud(self, frame_id: str, ts: float | None) -> PointCloud2:
        """Every point kept, as one cloud named in `frame_id` and stamped `ts` (seconds)."""
        import open3d.core as o3c  # type: ignore[import-untyped]

        with self._lock:
            points, colors = self._points, self._colors
        cloud = PointCloud2.from_numpy(points, frame_id=frame_id, timestamp=ts)
        cloud.pointcloud_tensor.point["colors"] = o3c.Tensor(colors)
        return cloud

    def clear(self) -> None:
        """Forget every point."""
        with self._lock:
            self._keys = self._keys[:0]
            self._points = self._points[:0]
            self._colors = self._colors[:0]


def _next_due(due: float, now: float, hz: float) -> float:
    """When the next message may go out, after one that was due at `due` went out at `now` (seconds).

    Counted from `due` rather than `now`, so a 5 Hz input thinned to 4 Hz sends 4 a
    second instead of every second one. After a pause it counts from `now`, so no burst.
    """
    period = 1.0 / hz
    return due + period if due + period > now else now + period
