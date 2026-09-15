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

The unprojection itself lives in ``rust/src/unproject.rs``. A 1280x720 frame is
920k pixels to range-gate and divide, per frame, on the critical path of a map
update.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class DepthCloudConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    # The crate is a workspace member, so cargo builds into the repo-root target dir.
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "depth_cloud")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # Keep every Nth pixel on each axis. A 1280x720 depth frame is 920k points;
    # at 4 that is 57k, which is the range the voxel map ray-caster is sized for.
    decimation: int = Field(default=4, ge=1)
    # Depth below this is the sensor's blind zone; above it, stereo range error
    # grows past the voxel size and smears obstacles.
    min_range_m: float = Field(default=0.2, ge=0.0)
    max_range_m: float = Field(default=6.0, gt=0.0)
    # Multiplier onto metres. uint16 depth is millimetres, so 0.001; float32
    # depth is already metres, so the Rust side ignores it for float input.
    depth_scale: float = Field(default=0.001, gt=0.0)
    # Empty rather than None so the native config takes a plain string. Falsy
    # either way, so Module.frame_id still falls back to the class name.
    frame_id: str | None = ""


class DepthCloud(NativeModule):
    """Depth image + intrinsics -> PointCloud2 in the camera's optical frame."""

    config: DepthCloudConfig

    depth: In[Image]
    camera_info: In[CameraInfo]
    cloud: Out[PointCloud2]


if TYPE_CHECKING:
    DepthCloud()
