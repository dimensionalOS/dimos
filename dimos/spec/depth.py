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

"""Depth-mapping pipeline specs: stereo/monocular sensor → backprojection → frame cloud."""

from typing import Protocol

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.mapping import FrameCloud


class StereoDepth(Protocol):
    """Stereo camera with metric depth from triangulation and VIO pose on TF bus."""

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]


class MonocularDepth(Protocol):
    """Monocular neural depth from a single RGB image (scale-relative)."""

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]


class DepthBackprojector(FrameCloud, Protocol):
    """Backprojects depth + RGB into a per-frame world-frame point cloud."""

    frame_cloud: Out[PointCloud2]
