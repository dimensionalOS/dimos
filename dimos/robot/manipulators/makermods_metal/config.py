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

"""Pinned MakerMods Metal geometry and arm planning coordinates."""

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

METAL_SOURCE = RobotDescriptionSource(
    url="https://github.com/makermods-robotics/metal-python-ros",
    ref="bccf3c82a21453a7b1784a94966456e16ff0f008",
)
METAL_MODEL = RobotModel.from_file(
    METAL_SOURCE / "metal_ros2/src/metal_description/urdf/metal_with_gripper.urdf",
    package_paths={"metal_description": METAL_SOURCE / "metal_ros2/src/metal_description"},
).with_fixed_joints("joint7", "joint8")
METAL_ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))


def make_metal_model_config() -> RobotModelConfig:
    """Plan arm motion to the gripper mount; opening is a separate control coordinate."""
    return RobotModelConfig(
        model=METAL_MODEL.with_subtree_rooted_at("base_link")
        .with_renamed_joints({"gripper_base": "gripper_mount_joint"})
        .with_default_joint_acceleration_limit(1.0),
        joint_names=list(METAL_ARM_JOINTS),
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="arm",
                joint_names=METAL_ARM_JOINTS,
                base_link="base_link",
                tip_link="gripper_base",
            )
        ],
        auto_convert_meshes=True,
    )
