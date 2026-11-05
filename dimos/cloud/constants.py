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

"""Console preview sizes (dimos/cloud/preview.py). The server's limits: 4 MB JSON, 32 MB video."""

PREVIEW_FORMAT = "dimos-spatial-preview-v2"
PREVIEW_SCALE = 0.02  # metres per int16 step: +-655 m around the origin
PREVIEW_FRAMES = 24  # timed LiDAR scans and camera thumbnails
PREVIEW_MAP_SCANS = 300  # scans merged into the map; build time stays flat on long recordings
PREVIEW_MAP_VOXEL = 0.1
PREVIEW_MAP_POINTS = 40_000
PREVIEW_SCAN_POINTS = 4_000
PREVIEW_THUMB_PX = 320
PREVIEW_JOY_SAMPLES = 1500  # joystick axes/buttons over time: confirms the controls were recorded
PREVIEW_BAND = (-0.5, 2.0)  # metres around the robot's height; ceilings hide the floor plan
WORLD_FRAMES = ("world", "map", "odom")

TIMELAPSE_MAX_S = 60.0  # longer recordings are sped up to fit
TIMELAPSE_FPS = 24  # smooth playback
TIMELAPSE_HEIGHT = 240
TIMELAPSE_CRF = 30  # H.264 quality: ~1.5 MB per minute at 240p
