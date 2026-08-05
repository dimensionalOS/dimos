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

# Frame and geometry were measured from data/xarm_grasp_sim/xarm7.xml:
# - link_tcp is 0.172 m along +Z from xarm_gripper_base_link;
# - the finger-pad tips end at approximately +0.162 m, which is the
#   GraspGenX fingertip depth;
# - GraspGenX closes along local X, whereas the xArm closes along local Y.
#
# The transform maps the GraspGenX gripper-base frame to xArm link_tcp. Its
# +90 degree Z rotation maps GraspGenX +X onto xArm +Y while preserving the
# common +Z approach direction.
XARM_GRASP_FRAME_TO_TCP = (
    (0.0, -1.0, 0.0, 0.0),
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.172),
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
