# Copyright 2025 Dimensional Inc.
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
VIDEO_WIDTH = 320
VIDEO_HEIGHT = 240
DEPTH_CAMERA_FOV = 160

# Depth camera range/filtering constants
MAX_RANGE = 3
MIN_RANGE = 0.2
MAX_HEIGHT = 1.2

# Lidar constants
LIDAR_RESOLUTION = 0.05

# Simulation timing constants
VIDEO_FPS = 20
LIDAR_FPS = 2

LAUNCHER_PATH = Path(__file__).parent / "mujoco_process.py"
