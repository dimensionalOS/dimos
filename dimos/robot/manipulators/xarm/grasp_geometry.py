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

"""The xArm Gripper's shape, as GraspGenX needs it to propose grasps.

Plain values with no imports, so any blueprint can use them without building the
robot. They hold for the real gripper and for the MuJoCo models of it, which share
its geometry and its 0.172 m tool tip.
"""

# The space the open and half-open jaws sweep through, as boxes in metres. The
# frame is xarm_gripper_base_link turned to GraspGenX's convention: the gripper
# approaches along +Z and the jaws close along X. Extents are full box sizes and
# offsets are box centres. fingertip_depth is how far the fingertips reach along Z.
# Measured with mj_forward at the driver joint limits, checked against the URDF.
XARM_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.0889, 0.030, 0.0370),
    "offset_open": (0.0, 0.0, 0.1421),
    "extents_half_open": (0.0479, 0.030, 0.0370),
    "offset_half_open": (0.0, 0.0, 0.1530),
    "fingertip_depth": 0.1606,
}

# The gripper's solid parts with the jaws open, as boxes in the same frame as the
# sweep volume, metres: the palm (as wide as its mounting flange), then the
# knuckles and the finger on each side. GraspGenX checks these against the scene
# around an object. Measured from the xarm_gripper URDF meshes at drive_joint = 0;
# every mesh vertex lies inside a box.
XARM_GRIPPER_BODY = (
    {"extents": (0.1022, 0.075, 0.1121), "offset": (0.0, 0.0, 0.0471)},
    {"extents": (0.0774, 0.0282, 0.0707), "offset": (0.0478, 0.0, 0.0868)},
    {"extents": (0.0774, 0.0282, 0.0707), "offset": (-0.0478, 0.0, 0.0868)},
    {"extents": (0.0319, 0.032, 0.0669), "offset": (0.0605, 0.0, 0.1287)},
    {"extents": (0.0319, 0.032, 0.0669), "offset": (-0.0605, 0.0, 0.1287)},
)

# Where the planner's tool tip (link_tcp) sits in the frame GraspGenX predicts, as
# a 4x4 row-major transform, metres. 0.172 m out along the approach axis
# (xarm_gripper.urdf.xacro joint_tcp), and a quarter turn about it that takes
# GraspGenX's X closing axis onto the xArm gripper's Y.
XARM_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.172),
    (0.0, 0.0, 0.0, 1.0),
)
