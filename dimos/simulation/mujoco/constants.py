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

from pathlib import Path

# Video/Camera constants
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 360
VIDEO_CAMERA_FOV = 45  # MuJoCo default FOV for head_camera (degrees)

# Lidar range/filtering constants
MAX_RANGE = 3
MIN_RANGE = 0.2
MAX_HEIGHT = 1.2

# Lidar constants
LIDAR_RESOLUTION = 0.05
# The recorded Go2 WebRTC voxel maps span 128 x 128 cells at 5 cm.
GO2_LIDAR_HALF_WIDTH = 3.2
# Dense elevation sampling closes the gaps between ground rings near the map's edge.
LIDAR_N_AZIMUTH = 720
LIDAR_N_ELEVATION = 480
LIDAR_ELEVATION_RANGES = {
    # Mid-360: the shallow lower bound leaves the nearby floor out of view.
    "unitree_g1": (-7.0, 52.0),
    # Unitree L1: 90 degrees, tilted down from the head mount.
    "unitree_go1": (-80.0, 10.0),
}

# Simulation timing constants
VIDEO_FPS = 20
LIDAR_FPS = 2

LAUNCHER_PATH = Path(__file__).parent / "mujoco_process.py"
