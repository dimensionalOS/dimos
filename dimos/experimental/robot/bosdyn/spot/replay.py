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

"""Replay a recorded Spot session onto ports named like `SpotHighLevel`'s outputs.

Plays a recording written by `SpotRecorder`, so the same Rerun visualization wires
up by name with no robot. The recorded ``tf`` tree keeps every frame anchored in 3D.
"""

from __future__ import annotations

from dimos.constants import RECORDINGS_DIR
from dimos.core.stream import Out
from dimos.memory.replay_module import ReplayModule, ReplayModuleConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class SpotReplayConfig(ReplayModuleConfig):
    # Default: the newest recording `SpotRecorder` wrote.
    dataset: str = str(RECORDINGS_DIR / "spot")
    loop: bool = True


class SpotReplay(ReplayModule):
    """Replays Spot's fisheye + depth cameras, odometry, and tf from a recording."""

    config: SpotReplayConfig
    dedicated_worker = True

    grayscale_image_front_left: Out[Image]
    grayscale_image_front_right: Out[Image]
    grayscale_image_left: Out[Image]
    grayscale_image_right: Out[Image]
    grayscale_image_back: Out[Image]

    depth_image_front_left: Out[Image]
    depth_image_front_right: Out[Image]
    depth_image_left: Out[Image]
    depth_image_right: Out[Image]
    depth_image_back: Out[Image]

    grayscale_info: Out[CameraInfo]
    depth_info: Out[CameraInfo]

    odometry: Out[Odometry]
    tf: Out[TFMessage]
