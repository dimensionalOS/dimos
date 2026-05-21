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

"""MuJoCo sim camera intrinsics constant, shared by sim connection modules
(Go2MujocoConnection, G1MujocoConnection) and by external readers that need
the value without pulling in the full mujoco transport class.
"""

from __future__ import annotations

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.simulation.mujoco.constants import (
    VIDEO_CAMERA_FOV,
    VIDEO_HEIGHT,
    VIDEO_WIDTH,
)

# Pinhole model from the MuJoCo camera's vertical FOV:
# f = height / (2 * tan(fovy / 2)).
MUJOCO_CAMERA_INFO_STATIC: CameraInfo = CameraInfo.from_fov(
    fov_deg=VIDEO_CAMERA_FOV,
    width=VIDEO_WIDTH,
    height=VIDEO_HEIGHT,
    axis="vertical",
    frame_id="camera_optical",
)
