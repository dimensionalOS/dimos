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

"""Unproject a depth image into a PointCloud2 using its CameraInfo intrinsics.

Geometry only — :meth:`PointCloud2.from_rgbd` covers the coloured case but needs a
size-matched colour frame alongside the depth one. Mapping consumers only want the
points, so this skips the colour stream entirely.

Points come out in the optical frame the intrinsics describe (x right, y down,
z forward), tagged with the ``CameraInfo``'s ``frame_id`` so a downstream
consumer resolving it through tf places them correctly. The depth image's own
``frame_id`` is the fallback: a vendor driver often stamps depth with a frame
nobody publishes a transform for, while the intrinsics can be given the link
name the robot actually puts on tf.
"""

from __future__ import annotations

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DepthCloudConfig(ModuleConfig):
    # Keep every Nth pixel on each axis. A 1280x720 depth frame is 920k points;
    # at 4 that is 57k, which is the range the voxel map ray-caster is sized for.
    decimation: int = Field(default=4, ge=1)
    # Depth below this is the sensor's blind zone; above it, stereo range error
    # grows past the voxel size and smears obstacles.
    min_range_m: float = Field(default=0.2, ge=0.0)
    max_range_m: float = Field(default=6.0, gt=0.0)
    # Multiplier onto metres. uint16 depth is millimetres, so 0.001; float32
    # depth is already metres, so the conversion below ignores this.
    depth_scale: float = Field(default=0.001, gt=0.0)


class DepthCloud(Module):
    """Depth image + intrinsics -> PointCloud2 in the camera's optical frame."""

    config: DepthCloudConfig
    depth: In[Image]
    camera_info: In[CameraInfo]
    cloud: Out[PointCloud2]

    _info: CameraInfo | None = None

    @rpc
    def start(self) -> None:
        super().start()
        # Only depth drives a cloud. Intrinsics are effectively static and are
        # republished purely so a late consumer sees them, so pairing the two
        # streams would emit a duplicate cloud on every republish.
        self.register_disposable(self.camera_info.observable().subscribe(self._on_camera_info))
        self.register_disposable(self.depth.observable().subscribe(self._on_depth))

    def _on_camera_info(self, info: CameraInfo) -> None:
        self._info = info

    def _on_depth(self, depth: Image) -> None:
        if self._info is None:
            return
        self.cloud.publish(self._unproject(depth, self._info))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _unproject(self, depth: Image, info: CameraInfo) -> PointCloud2:
        step = self.config.decimation
        depth_data = depth.data
        if depth_data.ndim == 3:
            depth_data = depth_data[:, :, 0]
        depth_data = depth_data[::step, ::step]

        if depth_data.dtype == np.float32 or depth_data.dtype == np.float64:
            metres = depth_data.astype(np.float32)
        else:
            metres = depth_data.astype(np.float32) * self.config.depth_scale

        intrinsics = info.get_K_matrix()
        fx, fy = float(intrinsics[0, 0]), float(intrinsics[1, 1])
        cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
        # Registered depth is often published at a different resolution than the
        # CameraInfo was calibrated at; rescale rather than emit skewed geometry.
        if info.width and info.height:
            fx *= depth.width / info.width
            fy *= depth.height / info.height
            cx *= depth.width / info.width
            cy *= depth.height / info.height

        rows, columns = np.nonzero(
            (metres >= self.config.min_range_m) & (metres <= self.config.max_range_m)
        )
        z = metres[rows, columns]
        # Back to full-resolution pixel coordinates so the intrinsics still apply.
        u = columns.astype(np.float32) * step
        v = rows.astype(np.float32) * step
        points = np.empty((z.size, 3), dtype=np.float32)
        points[:, 0] = (u - cx) * z / fx
        points[:, 1] = (v - cy) * z / fy
        points[:, 2] = z

        return PointCloud2.from_numpy(
            points,
            frame_id=self.config.frame_id or info.frame_id or depth.frame_id,
            timestamp=depth.ts,
        )
