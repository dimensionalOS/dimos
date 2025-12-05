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

DIMOS_PROJECT_ROOT = Path(__file__).parent.parent

DIMOS_LOG_DIR = DIMOS_PROJECT_ROOT / "logs"

"""
Constants for shared memory
Usually, auto-detection for size would be preferred. Sadly, though, channels are made
and frozen *before* the first frame is received.
Therefore, a maximum capacity for color image and depth image transfer should be defined
ahead of time.
"""
# Default color image size: 1920x1080 frame x 3 (RGB) x uint8
DEFAULT_CAPACITY_COLOR_IMAGE = 1920 * 1080 * 3
# Default depth image size: 1280x720 frame * 4 (float32 size)
DEFAULT_CAPACITY_DEPTH_IMAGE = 1280 * 720 * 4

# From https://github.com/lcm-proj/lcm.git
LCM_MAX_CHANNEL_NAME_LENGTH = 63
