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

"""Galaxea A1Z planning model configuration helpers."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.global_config import global_config
from dimos.hardware.manipulators.galaxea_a1z.config import (
    A1ZConfig,
    A1ZGripperConfig,
)
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import (
    joint_names,
    named_for_robot,
)

A1Z_DOF = 6
#: How far the G1Z gripper's jaw opens, in metres.
A1Z_GRIPPER_OPENING_M = A1ZGripperConfig().max_opening_m

A1Z_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("arm_link2", "arm_link5"),
    ("arm_link4", "arm_link6"),
]

A1Z_DESCRIPTION_REPO = "https://github.com/userguide-galaxea/URDF"
A1Z_DESCRIPTION_REF = "2e5d31e1784481a34d178006c0d0e18e0a84a82a"
_A1Z_REPO = RobotDescriptionSource(
    url=A1Z_DESCRIPTION_REPO,
    ref=A1Z_DESCRIPTION_REF,
)
A1Z_G1Z_PACKAGE = _A1Z_REPO / "A1Z" / "A1Z_G1Z"
A1Z_FLANGE_PACKAGE = _A1Z_REPO / "A1Z" / "A1Z_Flange"
A1Z_G1Z_MODEL_PATH = A1Z_G1Z_PACKAGE / "urdf" / "A1Z_G1Z.urdf"
A1Z_FLANGE_MODEL_PATH = A1Z_FLANGE_PACKAGE / "urdf" / "A1Z_Flange.urdf"
A1Z_FK_MODEL = A1Z_FLANGE_MODEL_PATH
A1Z_PACKAGE_PATHS: dict[str, Path] = {
    "A1Z_G1Z": A1Z_G1Z_PACKAGE,
    "A1Z_Flange": A1Z_FLANGE_PACKAGE,
}


def a1z_hardware(
    hw_id: str = "arm",
    *,
    has_gripper: bool = True,
    address: str | None = None,
    dynamics_urdf_path: Path | None = None,
    adapter_config: A1ZConfig | None = None,
) -> HardwareComponent:
    """The A1Z on a CAN interface; a mock stands in until an address is configured, and in simulation."""
    gripper_joints = [f"{hw_id}/gripper"] if has_gripper else []
    joints = [*joint_names(A1Z_DOF, prefix="arm_joint"), *gripper_joints]
    mock_limits = JointLimits(
        position_lower=[*([-math.pi] * A1Z_DOF), *([0.0] * len(gripper_joints))],
        position_upper=[*([math.pi] * A1Z_DOF), *([0.1] * len(gripper_joints))],
        velocity_max=[*([math.pi] * A1Z_DOF), *([0.0] * len(gripper_joints))],
    )
    if global_config.simulation:
        return HardwareComponent(
            hardware_id=hw_id,
            hardware_type=HardwareType.MANIPULATOR,
            joints=joints,
            adapter_type="mock",
            auto_enable=True,
            limits=mock_limits,
        )
    resolved_config = adapter_config or A1ZConfig(
        gripper=A1ZGripperConfig() if has_gripper else None,
    )
    if (resolved_config.gripper is not None) != has_gripper:
        raise ValueError("has_gripper must match adapter_config.gripper")
    if dynamics_urdf_path is not None:
        # Preserve LfsPath laziness: the adapter resolves the model only when
        # connect() constructs the vendor robot.
        resolved_config = replace(resolved_config, urdf_path=dynamics_urdf_path)
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=joints,
        adapter_type="galaxea_a1z",
        address=address,
        auto_enable=True,
        adapter_kwargs={"config": resolved_config},
        mock_limits=mock_limits,
    )


def a1z_model(gripper: bool = True, *, robot: str | None = None) -> RobotModel:
    """The A1Z's URDF, as its connection module and coordinator drive it.

    Args:
        gripper: With the G1Z gripper, driven as a joint ``gripper``: the jaw
            opening, from 0 m (closed) to 0.1 m (open). Its URDF fingers are fixed.
        robot: Name the moving joints under this robot, e.g. "arm" gives
            "arm/arm_joint1" and "arm/gripper". Links keep their names.
    """
    if not gripper:
        model = RobotModel.from_file(A1Z_FLANGE_MODEL_PATH, package_paths=A1Z_PACKAGE_PATHS)
    else:
        model = RobotModel.from_file(
            A1Z_G1Z_MODEL_PATH, package_paths=A1Z_PACKAGE_PATHS
        ).with_added_joints(
            JointDescription(
                name="gripper",
                type="prismatic",
                parent_link="arm_link6",
                child_link="gripper_control_link",
                lower=0.0,
                upper=A1Z_GRIPPER_OPENING_M,
                velocity=0.0,
                effort=0.0,
            )
        )
    if not robot:
        return model
    moving = [*joint_names(A1Z_DOF, prefix="arm_joint"), *(["gripper"] if gripper else [])]
    return model.with_renamed_joints({joint: f"{robot}/{joint}" for joint in moving})


def make_a1z_model_config(
    *,
    has_gripper: bool = True,
    home_joints: list[float] | None = None,
    robot: str | None = None,
) -> RobotModelConfig:
    """The A1Z's planning model.

    Args:
        has_gripper: With the G1Z gripper; the planner aims its jaw centre.
        home_joints: The arm's home, in rad; all zeros if not given.
        robot: Name the arm joints under this robot, e.g. "arm" gives
            "arm/arm_joint1", as a coordinator driving ``A1ZConnection`` names them.
    """
    model_joint_names = joint_names(A1Z_DOF, prefix="arm_joint")
    model = RobotModel.from_file(
        A1Z_G1Z_MODEL_PATH if has_gripper else A1Z_FLANGE_MODEL_PATH,
        package_paths=A1Z_PACKAGE_PATHS,
    ).with_default_joint_acceleration_limit(2.0)
    if has_gripper:
        model = model.with_fixed_frame(
            "gripper_eef_link",
            "arm_link6",
            xyz=(0.0727, 0.0, 0.0),
        )
    config = RobotModelConfig(
        model=model,
        joint_names=model_joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link="base_link",
                tip_link=("gripper_eef_link" if has_gripper else "arm_link6"),
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=A1Z_COLLISION_EXCLUSIONS,
        home_joints=home_joints or [0.0] * A1Z_DOF,
    )
    return named_for_robot(config, robot)
