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

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_joints,
)
from dimos.core.global_config import global_config
from dimos.hardware.manipulators.galaxea_a1z.config import (
    A1ZConfig,
    A1ZGripperConfig,
)
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)

A1Z_DOF = 6

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
    dynamics_urdf_path: Path | None = None,
    adapter_config: A1ZConfig | None = None,
) -> HardwareComponent:
    """Configure mock A1Z hardware unless an explicit CAN port selects the real adapter."""
    resolved_config = adapter_config or A1ZConfig(gripper=A1ZGripperConfig())
    gripper = resolved_config.gripper
    adapter_type = "mock"
    address = None
    adapter_kwargs: dict[str, object] = {}
    if not global_config.simulation and global_config.can_port:
        adapter_type = "galaxea_a1z"
        address = global_config.can_port
        if dynamics_urdf_path is not None:
            # Preserve LfsPath laziness: the adapter resolves the model only when
            # connect() constructs the vendor robot.
            resolved_config = replace(resolved_config, urdf_path=dynamics_urdf_path)
        adapter_kwargs["config"] = resolved_config

    gripper_joints = [f"{hw_id}/gripper"] if gripper is not None else []
    limits: JointLimits | None = None
    if adapter_type == "mock":
        limits = JointLimits(
            position_lower=[*([-math.pi] * A1Z_DOF), *([0.0] * len(gripper_joints))],
            position_upper=[
                *([math.pi] * A1Z_DOF),
                *([gripper.max_opening_m] if gripper is not None else []),
            ],
            velocity_max=[*([math.pi] * A1Z_DOF), *([0.0] * len(gripper_joints))],
        )
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*make_joints(hw_id, A1Z_DOF), *gripper_joints],
        adapter_type=adapter_type,
        address=address,
        auto_enable=True,
        limits=limits,
        adapter_kwargs=adapter_kwargs,
    )


def make_a1z_model_config(
    name: str = "arm",
    *,
    has_gripper: bool = True,
    joint_prefix: str | None = None,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    local_joint_names = joint_names(A1Z_DOF, prefix="arm_joint")
    model = RobotModel.from_file(
        A1Z_G1Z_MODEL_PATH if has_gripper else A1Z_FLANGE_MODEL_PATH,
        package_paths=A1Z_PACKAGE_PATHS,
    )
    if has_gripper:
        model = model.with_fixed_frame(
            "gripper_eef_link",
            "arm_link6",
            xyz=(0.0727, 0.0, 0.0),
        )
    return RobotModelConfig(
        name=name,
        model=model,
        base_pose=base_pose(),
        joint_names=local_joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joint_names),
                base_link="base_link",
                tip_link="gripper_eef_link" if has_gripper else "arm_link6",
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=A1Z_COLLISION_EXCLUSIONS,
        joint_name_mapping=coordinator_joint_mapping(
            name,
            A1Z_DOF,
            joint_prefix=joint_prefix,
            urdf_joint_prefix="arm_",
        ),
        gripper_hardware_id=name if has_gripper else None,
        home_joints=home_joints or [0.0] * A1Z_DOF,
    )
