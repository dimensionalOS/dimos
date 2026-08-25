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

"""Physical constants for the Unitree Go2."""

from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Robot footprint in meters. Length is forward x, width is left y.
ROBOT_LENGTH = 0.6858
ROBOT_WIDTH = 0.3175
# Ground to the tallest point.
ROBOT_HEIGHT = 0.45

# Mount geometry measured on this rig (metres). Not go2_mid360_static_transforms — that
# is the recording rig: different lidar angle, tree hung off base_link. Both halves of
# the rig read these: GO2Zenoh (zenoh/zenohconnection.py) publishes the tree from the
# laptop, the baked go2_tf (tf/go2_tf.py) publishes it from the robot.
CAMERA_XYZ = Vector3(0.32715, -0.00003, 0.04297)  # base_link -> front_camera
MID360_XYZ = Vector3(-0.032, 0.0, 0.12)  # front_camera -> mid360_link: 3.2cm back, 12cm up

# front_camera -> mid360_link, fixed-axis rpy in degrees, by rig.
MID360_MOUNT_PRESETS: dict[str, tuple[float, float, float]] = {
    # Pointing straight ahead, pitched 60 deg down.
    "SF": (0.0, 60.0, 0.0),
    # The 60 deg tilt lands on roll because this lidar sits yawed 90 deg on its bracket.
    "ATHENS": (-60.0, 0.0, -90.0),
}
