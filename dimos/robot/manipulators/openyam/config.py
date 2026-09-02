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

"""OpenYAM hardware and planning model configuration."""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.global_config import global_config
from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators._modeling import (
    joint_names,
)
from dimos.utils.data import LfsPath

OPENYAM_DOF = 6
OPENYAM_HARDWARE_ID = "openyam"
OPENYAM_ARM_JOINTS = joint_names(OPENYAM_DOF, prefix="yam_joint")
OPENYAM_GRIPPER_JOINT = "arm/gripper"
OPENYAM_JOINTS = [*OPENYAM_ARM_JOINTS, OPENYAM_GRIPPER_JOINT]
OPENYAM_PACKAGE = LfsPath("yam_description")
OPENYAM_MODEL_PATH = OPENYAM_PACKAGE / "urdf/yam_gripper.urdf.xacro"
OPENYAM_PACKAGE_PATHS: dict[str, Path] = {"yam_description": OPENYAM_PACKAGE}


def openyam_hardware() -> HardwareComponent:
    """Select the physical or in-memory whole-body adapter for OpenYAM."""
    adapter_type = "mock_whole_body" if global_config.simulation else "openyam_damiao"
    adapter_kwargs: dict[str, object] = {}
    limits: JointLimits | None = None
    if global_config.simulation:
        limits = JointLimits(
            position_lower=[*([None] * OPENYAM_DOF), 0.0],
            position_upper=[*([None] * OPENYAM_DOF), 1.0],
            velocity_max=[None] * len(OPENYAM_JOINTS),
        )
    else:
        adapter_kwargs["runtime_config"] = DamiaoRuntimeConfig(
            bus_addresses={"openyam": global_config.can_port or "can0"},
            gravity_comp=True,
        )
    return HardwareComponent(
        hardware_id=OPENYAM_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(OPENYAM_JOINTS),
        adapter_type=adapter_type,
        auto_enable=True,
        limits=limits,
        adapter_kwargs=adapter_kwargs,
        wb_config=WholeBodyConfig(
            kp=(80.0, 80.0, 80.0, 10.0, 10.0, 10.0, 0.0),
            kd=(5.0, 5.0, 5.0, 1.5, 1.5, 1.5, 0.0),
        ),
    )


def make_openyam_model_config(
    *,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    """Build a planning config for the gripper-equipped OpenYAM."""
    model_joint_names = joint_names(OPENYAM_DOF, prefix="yam_joint")
    return RobotModelConfig(
        model=RobotModel.from_file(OPENYAM_MODEL_PATH, package_paths=OPENYAM_PACKAGE_PATHS),
        joint_names=model_joint_names,
        base_link="yam_base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link="yam_base_link",
                tip_link="yam_hand_tcp",
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=[],
        gripper_hardware_id="arm",
        home_joints=home_joints or [0.0] * OPENYAM_DOF,
    )
