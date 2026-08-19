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

"""Record Alfred's D455, Point-LIO and wheel odometry at full rate, uncompressed.

Recorder ports mirror the `RealSenseCamera` and `PointLio` outputs so
`autoconnect` wires them by name.

Every image stream is pinned to the bare ``lcm`` codec. That is the whole point
of this recorder: the default for `Image` is JPEG at quality 50, and the
lossless alternative used elsewhere, ``lz4+lcm``, cannot compress a full-rate
D455 in real time. Bare ``lcm`` writes the pixel bytes as they arrived, so the
only per-frame cost is the LCM envelope and the sqlite write.

Budget that against your disk. At 848x480x30 fps the four image streams come to
roughly 100 MB/s together (colour 8-bit BGR ~35, depth 16-bit ~24, two IR 8-bit
~24), before the lidar.

`wheel_odometry` has no publisher on Alfred yet -- nothing in the robot package
emits `Out[Odometry]` for the base -- so it records an empty stream today. The
port is here so that adding one is a wiring change and not a recorder change.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from pydantic import Field

from dimos.core.stream import In
from dimos.memory.module import OnExisting, Recorder, RecorderConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Uncompressed: the LCM envelope around the original pixel bytes, nothing else.
UNCOMPRESSED_CODEC = "lcm"

IMAGE_STREAMS = ("color_image", "depth_image", "infrared_left", "infrared_right")


class AlfredRecorderConfig(RecorderConfig):
    db_path: str | Path = Field(
        default_factory=lambda: f"alfred_455_{datetime.now():%Y-%m-%d_%H-%M-%S}.db"
    )
    on_existing: OnExisting = OnExisting.APPEND
    # Point-LIO's odometry frame, so recorded frames are anchored in it.
    root_frame: str = "odom"
    stream_codecs: dict[str, str] = Field(
        default_factory=lambda: {stream: UNCOMPRESSED_CODEC for stream in IMAGE_STREAMS}
    )


class AlfredRecorder(Recorder):
    """Records the D455, Point-LIO and wheel odometry to a memory db."""

    config: AlfredRecorderConfig

    color_image: In[Image]
    depth_image: In[Image]
    infrared_left: In[Image]
    infrared_right: In[Image]

    camera_info: In[CameraInfo]
    depth_camera_info: In[CameraInfo]
    infrared_left_camera_info: In[CameraInfo]
    infrared_right_camera_info: In[CameraInfo]

    imu: In[Imu]
    imu_info: In[ImuInfo]

    lidar: In[PointCloud2]
    odometry: In[Odometry]
    wheel_odometry: In[Odometry]
