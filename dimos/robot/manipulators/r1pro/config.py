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

"""Galaxea R1 Pro planning model and fake hardware configuration."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import PlanarBaseConfig, RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

R1PRO_DESCRIPTION_URL = "https://github.com/userguide-galaxea/URDF"
R1PRO_DESCRIPTION_REF = "2e5d31e1784481a34d178006c0d0e18e0a84a82a"
R1PRO_DESCRIPTION_SOURCE = RobotDescriptionSource(
    url=R1PRO_DESCRIPTION_URL,
    ref=R1PRO_DESCRIPTION_REF,
)
R1PRO_PACKAGE = R1PRO_DESCRIPTION_SOURCE / "R1Pro" / "urdf_r1pro_g1z_2026"
R1PRO_MODEL_PATH = R1PRO_PACKAGE / "urdf" / "r1pro_2026.urdf"
R1PRO_PACKAGE_PATHS: dict[str, Path] = {"r1pro_urdf": R1PRO_PACKAGE}

R1PRO_PLANAR_BASE = PlanarBaseConfig(
    position_lower=(-5.0, -5.0, -math.pi),
    position_upper=(5.0, 5.0, math.pi),
    velocity_limits=(1.0, 1.0, 2.0),
    acceleration_limits=(2.0, 2.0, 4.0),
)
R1PRO_TORSO_JOINTS = [f"torso_joint{i}" for i in range(1, 5)]
R1PRO_LEFT_ARM_JOINTS = [f"left_arm_joint{i}" for i in range(1, 8)]
R1PRO_RIGHT_ARM_JOINTS = [f"right_arm_joint{i}" for i in range(1, 8)]
R1PRO_JOINTS = [
    *R1PRO_PLANAR_BASE.joint_names,
    *R1PRO_TORSO_JOINTS,
    *R1PRO_LEFT_ARM_JOINTS,
    *R1PRO_RIGHT_ARM_JOINTS,
]

_R1PRO_FIXED_JOINTS = [
    *(f"steer_motor_joint{i}" for i in range(1, 4)),
    *(f"wheel_motor_joint{i}" for i in range(1, 4)),
    *(f"{side}_gripper_finger_joint{i}" for side in ("left", "right") for i in (1, 2)),
]
R1PRO_MODEL = (
    RobotModel.from_file(
        R1PRO_MODEL_PATH,
        package_paths=R1PRO_PACKAGE_PATHS,
    )
    .with_planar_base(R1PRO_PLANAR_BASE)
    .with_fixed_joints(*_R1PRO_FIXED_JOINTS)
)


def make_r1pro_model_config() -> RobotModelConfig:
    """Build the planar-base, torso, and bimanual R1 Pro planning model."""
    return RobotModelConfig(
        model=R1PRO_MODEL,
        joint_names=list(R1PRO_JOINTS),
        base_link=R1PRO_PLANAR_BASE.root_link,
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=tuple(R1PRO_LEFT_ARM_JOINTS),
                base_link="left_arm_base_link",
                tip_link="left_gripper_link",
            ),
            PlanningGroupDefinition(
                name="right_arm",
                joint_names=tuple(R1PRO_RIGHT_ARM_JOINTS),
                base_link="right_arm_base_link",
                tip_link="right_gripper_link",
            ),
            PlanningGroupDefinition(
                name="torso",
                joint_names=tuple(R1PRO_TORSO_JOINTS),
                base_link="base_link",
            ),
            PlanningGroupDefinition(
                name="moving_base",
                joint_names=R1PRO_PLANAR_BASE.joint_names,
                base_link=R1PRO_PLANAR_BASE.root_link,
            ),
        ],
        auto_convert_meshes=True,
        home_joints=[0.0] * len(R1PRO_JOINTS),
    )


def r1pro_fake_hardware() -> HardwareComponent:
    """Provide zero-state fake hardware for every planned coordinate."""
    return HardwareComponent(
        hardware_id="r1pro",
        hardware_type=HardwareType.MANIPULATOR,
        joints=list(R1PRO_JOINTS),
        adapter_type="mock",
    )
