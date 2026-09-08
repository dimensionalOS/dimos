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

"""Records Point-LIO odom+lidar and the RealSense color and depth streams into a memory db."""

from __future__ import annotations

from pydantic import Field

from dimos.core.stream import In
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder, PointlioRecorderConfig
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image


class G1RecorderConfig(PointlioRecorderConfig):
    # don't compress
    stream_codecs: dict[str, str] = Field(
        default_factory=lambda: {"realsense_depth_image": "lz4+lcm"}
    )


class G1Recorder(PointlioRecorder):
    config: G1RecorderConfig

    color_image: In[Image]
    realsense_depth_image: In[Image]
    realsense_camera_info: In[CameraInfo]
    realsense_depth_camera_info: In[CameraInfo]
