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

from pathlib import Path

from dimos.core.stream import In
from dimos.memory2.module import OnExisting, Recorder, RecorderConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image


class SpotRecorderConfig(RecorderConfig):
    db_path: str | Path = "spot_recording.db"
    # Append into a populated db so re-runs add to the same recording.
    on_existing: OnExisting = OnExisting.APPEND


class SpotRecorder(Recorder):
    """Records Spot's fisheye + depth cameras and odometry to a memory2 db."""

    config: SpotRecorderConfig

    grayscale_image_1: In[Image]
    grayscale_image_2: In[Image]
    grayscale_image_3: In[Image]
    grayscale_image_4: In[Image]
    grayscale_image_5: In[Image]

    depth_image_1: In[Image]
    depth_image_2: In[Image]
    depth_image_3: In[Image]
    depth_image_4: In[Image]
    depth_image_5: In[Image]

    odom: In[Odometry]
