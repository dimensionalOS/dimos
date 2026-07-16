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

"""Shared Booster B1 camera configuration defaults."""

import os

DEFAULT_COLOR_TOPIC = "rt/booster_video_stream"
DEFAULT_DEPTH_TOPIC = "rt/boostercamera/head/depth"
DEFAULT_COLOR_CAMERA_INFO_TOPIC = "rt/boostercamera/head/rgb/camera_info"
DEFAULT_DEPTH_CAMERA_INFO_TOPIC = "rt/boostercamera/head/depth/camera_info"
DEFAULT_DEPTH_SCALE = 0.001


def camera_network_interface_from_env() -> str | None:
    """Return the optional DDS interface selected for the robot connection."""
    return os.environ.get("ROBOT_INTERFACE") or None
