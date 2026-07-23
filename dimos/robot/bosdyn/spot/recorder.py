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

"""Record every Spot data stream into a memory2 SQLite db.

A ``Recorder`` whose In ports mirror `SpotHighLevel`'s outputs — the five
grayscale cameras, five depth cameras, and body odometry — so `autoconnect`
wires them by name. The base class writes each port (plus the live tf tree) to
``db_path``; poses come from tf, so recorded frames stay spatially anchored.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from pydantic import Field

from dimos.core.stream import In
from dimos.memory2.module import OnExisting, Recorder, RecorderConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.bosdyn.spot.config import CAMERA_STREAM_SUFFIXES

# jpeg codec quantises depth it to ~25cm and adds block artifacts (horrible)
LOSSLESS_CODEC = "lz4+lcm"


class SpotRecorderConfig(RecorderConfig):
    # Timestamped by default so each run lands in its own file instead of appending
    # onto the last one. Override with `-o spotrecorder.db_path=...` to pick a path.
    db_path: str | Path = Field(
        default_factory=lambda: f"spot_recording_{datetime.now():%Y-%m-%d_%H-%M-%S}.db"
    )
    on_existing: OnExisting = OnExisting.APPEND
    root_frame: str = "odom"
    stream_codecs: dict[str, str] = Field(
        default_factory=lambda: {
            f"{kind}_image_{suffix}": LOSSLESS_CODEC
            for kind in ("grayscale", "depth")
            for suffix in CAMERA_STREAM_SUFFIXES
        }
    )


class SpotRecorder(Recorder):
    """Records Spot's fisheye + depth cameras and odometry to a memory2 db."""

    config: SpotRecorderConfig

    grayscale_image_front_left: In[Image]
    grayscale_image_front_right: In[Image]
    grayscale_image_left: In[Image]
    grayscale_image_right: In[Image]
    grayscale_image_back: In[Image]

    depth_image_front_left: In[Image]
    depth_image_front_right: In[Image]
    depth_image_left: In[Image]
    depth_image_right: In[Image]
    depth_image_back: In[Image]

    grayscale_info: In[CameraInfo]
    depth_info: In[CameraInfo]

    odometry: In[Odometry]
