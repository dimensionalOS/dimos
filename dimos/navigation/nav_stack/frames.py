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

"""Standard TF frame IDs for the SmartNav navigation stack.

Follows the ROS REP-105 frame convention:

    map → odom → body

- **map**: Global, loop-closure-corrected frame (published by PGO).
- **odom**: Continuous, locally smooth frame with no jumps (published by FastLio2).
- **body**: Robot body / IMU frame.
"""

FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BODY = "body"
FRAME_SENSOR = "sensor"
