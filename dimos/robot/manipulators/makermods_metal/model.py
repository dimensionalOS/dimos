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

"""Planning model for the six-axis MakerMods Metal arm."""

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_ARM_JOINTS,
    METAL_HARDWARE_ID,
    METAL_MODEL,
)

# dimOS TCP: midpoint of the standard finger pads, rounded to 80 mm along
# gripper_base +Z. In the pinned vendor link7/link8 meshes, their planar contact
# regions span Z=68.15..91.40 and 68.45..91.70 mm. X spans the pad width, Y is
# the jaw opening axis, and +Z points out of the gripper. This is model geometry,
# not a vendor-specified or physically calibrated TCP.
METAL_GRASP_FRAME = "grasp_frame"
METAL_GRASP_XYZ = (0.0, 0.0, 0.08)


def make_metal_model_config() -> RobotModelConfig:
    """Plan to the standard pad center; normalized jaw motion is controlled separately."""
    return RobotModelConfig(
        model=METAL_MODEL.with_subtree_rooted_at("base_link")
        .with_fixed_frame(METAL_GRASP_FRAME, "gripper_base", xyz=METAL_GRASP_XYZ)
        .with_renamed_joints({"gripper_base": "gripper_mount_joint"})
        .with_default_joint_acceleration_limit(1.0),
        joint_names=list(METAL_ARM_JOINTS),
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="arm",
                joint_names=METAL_ARM_JOINTS,
                base_link="base_link",
                tip_link=METAL_GRASP_FRAME,
            )
        ],
        gripper_hardware_id=METAL_HARDWARE_ID,
        auto_convert_meshes=True,
    )
