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
"""TypeSafe agent constants, by section."""

# --- API -------------------------------------------------------------------
DEFAULT_MODEL = "jev-latest"
DEFAULT_BASE_URL = "https://api.typesafe.ai"
BASE_URL_ENV = "TYPESAFE_BASE_URL"
REQUEST_TIMEOUT_S = 10.0

# --- Perception / world state --------------------------------------------
MAX_OBJECTS = 20  # closest first; the model reads words, not a scene graph
DISTANCE_WORDS = ((0.5, "touching"), (1.5, "near"), (4.0, "mid"), (float("inf"), "far"))
SECTOR_NAMES = (
    "ahead",
    "ahead_left",
    "left",
    "behind_left",
    "behind",
    "behind_right",
    "right",
    "ahead_right",
)
BEARING_WORDS_2D = ("far_left", "left", "center", "right", "far_right")
SIZE_WORDS_2D = ((0.4, "filling_view"), (0.15, "large"), (0.03, "medium"), (-1.0, "small"))
IMAGE_SIZE = (1280, 720)  # for 2D detections
LIDAR_BAND = (-0.2, 0.8, 5.0)  # z_min, z_max, max_range (m) relative to the robot
STALE_S = 2.0  # inputs older than this are ignored

# --- Navigation ------------------------------------------------------------
NAV_MAX_HZ = 2.0  # pose updates arrive faster than the model should be asked
LINEAR_SPEED = 0.5  # m/s
ANGULAR_SPEED = 0.8  # rad/s
LINEAR_ACCEL = 0.8  # m/s^2 ramp
ANGULAR_ACCEL = 1.6  # rad/s^2 ramp
PUBLISH_HZ = 10.0  # cmd_vel rate
MIN_PROBABILITY = 0.5  # a chosen option below this counts as none
STOP_THRESHOLD = 0.7  # noul
REACHED_M = 0.5
GIVE_UP_S = 5.0  # goal clears after this long without motion
SLOW_WITHIN_M = 1.5  # speed scales down inside this distance to the goal
TURN_FULL_AT_DEG = 45.0  # turn rate scales down inside this bearing error
DEADMAN_MIN_S = (
    1.0  # cmd_vel zeroes when no decision arrived for max(this, DEADMAN_PERIODS / max_hz)
)
DEADMAN_PERIODS = 2.5
