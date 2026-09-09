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

"""Values shared by every R1 Pro, independent of the individual unit.

Anything that differs between two R1 Pros — factory camera calibration, host
addresses, dataset paths — is a config field so it can come from the CLI. This
file is only for what the model itself fixes.
"""

# Vendor URDF link names. The head camera joint's rpy is (-1.9199, 0, -1.5708):
# the ROS optical rotation plus a 20 degree down-tilt, so this link already *is*
# the optical frame. Nothing downstream may stack a second optical rotation.
HEAD_CAMERA_LINK = "camera_head_left_link"
LIDAR_LINK = "lidar_chassis_left_link"
BASE_LINK = "base_link"
ODOM_FRAME = "odom"

# First-pass clearances — tune on the robot.
CHASSIS_WIDTH_M = 0.65
ROTATION_DIAMETER_M = 0.9
# Torso-up height plus margin: anything lower than this cannot be driven under.
OVERHEAD_CLEARANCE_M = 1.9
# A wheeled base on flat floors only.
MAX_STEP_HEIGHT_M = 0.03
