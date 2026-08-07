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

"""GraspGenX geometry for the UFACTORY xArm gripper."""

from __future__ import annotations

from dimos.manipulation.grasping.grasp_gen_x import (
    GraspGenXConfig,
    SweepVolumeGripperConfig,
)

# Geometry was derived from UFACTORY's xarm_ros gripper URDF and collision
# meshes at commit 0b5118eb6bf664fc3891c14b203e6ecbd5095dca:
# - link_tcp is 0.172 m along +Z from xarm_gripper_base_link
# - link_tcp's closing axis is 90 degrees counter-clockwise around local +Z
#   from GraspGenX's local +X closing axis
# - the inner finger volume is approximately 0.085 x 0.032 x 0.067 m
# The model's grasp frame is the gripper base; DimOS plans for link_tcp.
XARM_GRASP_FRAME_TO_TCP = (
    (0.0, -1.0, 0.0, 0.0),
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.172),
    (0.0, 0.0, 0.0, 1.0),
)

# Inverse of ``XARM_GRASP_FRAME_TO_TCP``. Rerun receives TCP poses, while the
# sweep geometry below is expressed in the GraspGenX gripper-base frame.
XARM_TCP_TO_GRASP_FRAME = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, -0.172),
    (0.0, 0.0, 0.0, 1.0),
)

XARM_GRIPPER_SWEEP = SweepVolumeGripperConfig(
    extents_open=(0.085, 0.032, 0.067),
    offset_open=(0.0, 0.0, 0.1285),
    extents_half_open=(0.0425, 0.032, 0.067),
    offset_half_open=(0.0, 0.0, 0.1285),
    fingertip_depth=0.162,
    family="revolute_2f",
)


def make_xarm_graspgenx_config() -> GraspGenXConfig:
    """Return the import-safe learned-grasp deployment configuration."""
    return GraspGenXConfig(
        gripper=XARM_GRIPPER_SWEEP,
        grasp_frame_to_tcp=XARM_GRASP_FRAME_TO_TCP,
        max_candidates=100,
    )
