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

"""Replay a G1 recording (``g1_sf_office.db``) onto the G1 sensor streams."""

from __future__ import annotations

from typing import Any

from pydantic import Field

from dimos.core.stream import Out
from dimos.memory.replay_module import ReplayModule, ReplayModuleConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

DEFAULT_G1_DATASET = "g1_sf_office.db"


def _default_g1_dataset(model: dict[str, Any]) -> str:
    replay_db = str(getattr(model.get("g"), "replay_db", "") or "")
    if replay_db in ("", "go2_short"):
        return DEFAULT_G1_DATASET
    return replay_db


class G1ReplayConfig(ReplayModuleConfig):
    dataset: str = Field(default_factory=_default_g1_dataset)


class G1Replay(ReplayModule):
    """Replays a G1 recording (SQLite or MCAP) onto the live G1 stream names."""

    config: G1ReplayConfig
    dedicated_worker = True

    color_image: Out[Image]
    realsense_depth_image: Out[Image]
    realsense_camera_info: Out[CameraInfo]
    realsense_depth_camera_info: Out[CameraInfo]
    pointlio_odometry: Out[Odometry]
    pointlio_lidar: Out[PointCloud2]
    tf: Out[TFMessage]
