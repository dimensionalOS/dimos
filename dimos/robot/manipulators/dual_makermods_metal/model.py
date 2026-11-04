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

"""Two official Metal arm trees fixed to the configured mounting base."""

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.dual_makermods_metal.config import ALL_ARM_JOINTS, ARM_JOINTS, SIDES
from dimos.robot.manipulators.makermods_metal.model import make_metal_model_config

DUAL_METAL_BASE = RobotModel.from_root_link("dual_metal_base", name="dual_makermods_metal")

Vector3 = tuple[float, float, float]
# Example bench layout only; no published official bimanual mount transform was found.
DEFAULT_LEFT_XYZ: Vector3 = (0.0, 0.3, 0.0)
DEFAULT_RIGHT_XYZ: Vector3 = (0.0, -0.3, 0.0)
ZERO_RPY: Vector3 = (0.0, 0.0, 0.0)


def dual_metal_model_config(
    *,
    left_xyz: Vector3 = DEFAULT_LEFT_XYZ,
    right_xyz: Vector3 = DEFAULT_RIGHT_XYZ,
    left_rpy: Vector3 = ZERO_RPY,
    right_rpy: Vector3 = ZERO_RPY,
) -> RobotModelConfig:
    """A single collision world and independently selectable pose groups for both arms."""
    if left_xyz == right_xyz:
        raise ValueError("Metal arm bases must have distinct positions")
    single = make_metal_model_config().model
    return RobotModelConfig(
        model=DUAL_METAL_BASE.with_attached_model(
            single,
            parent_link="dual_metal_base",
            joint_name="left_mount",
            prefix="left_",
            xyz=left_xyz,
            rpy=left_rpy,
        ).with_attached_model(
            single,
            parent_link="dual_metal_base",
            joint_name="right_mount",
            prefix="right_",
            xyz=right_xyz,
            rpy=right_rpy,
        ),
        joint_names=list(ALL_ARM_JOINTS),
        base_link="dual_metal_base",
        planning_groups=[
            PlanningGroupDefinition(
                name=f"{side}_arm",
                joint_names=ARM_JOINTS[side],
                base_link="dual_metal_base",
                tip_link=f"{side}_gripper_base",
            )
            for side in SIDES
        ],
        auto_convert_meshes=True,
    )
