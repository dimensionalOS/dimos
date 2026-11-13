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
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.global_config import global_config
from dimos.hardware.manipulators.galaxea_a1z.config import (
    A1ZConfig,
    A1ZGripperConfig,
)
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.gripper import GripperConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import (
    joint_names,
)

A1Z_DOF = 6

A1Z_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("arm_link2", "arm_link5"),
    ("arm_link4", "arm_link6"),
]

A1Z_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("arm_link6", "gripper_finger_left_link"),
    ("arm_link6", "gripper_finger_rIght_link"),
    ("gripper_finger_left_link", "gripper_finger_rIght_link"),
]

# Derived from A1Z_G1Z.urdf and its STL meshes (arm_link6.STL,
# gripper_finger_left_link.STL, gripper_finger_rIght_link.STL), expressed in
# GraspGenX's convention (approach along +Z, jaws closing along X) with origin on
# arm_link6: fingertips reach 0.0727 + 0.1105 = 0.1832 m, the distal parallel
# fingertip pad spans 0.0350 m back from the tip (centred at 0.1657 m), and
# stroke is 0.10 m.
A1Z_GRIPPER_SWEEP_VOLUME: dict[str, Any] = {
    "extents_open": (0.100, 0.036, 0.0350),
    "offset_open": (0.0, 0.0, 0.1657),
    "extents_half_open": (0.050, 0.036, 0.0350),
    "offset_half_open": (0.0, 0.0, 0.1657),
    "fingertip_depth": 0.1832,
}

# Maps GraspGenX's grasp frame (origin at arm_link6, +Z approach, X closing) to
# gripper_eef_link (at +0.0727 m along +X of arm_link6, +X approach, Y closing).
A1Z_GRASP_FRAME_TO_TCP: tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
] = (
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0, 0.0727),
    (0.0, 0.0, 0.0, 1.0),
)


def a1z_gripper_config() -> GripperConfig:
    """Which Galaxea G1Z gripper links may touch a grasped object, and how it closes."""
    return GripperConfig(
        pad_links=("gripper_finger_left_link", "gripper_finger_rIght_link"),
        finger_joints={
            "gripper_finger_left_joint": (0.05, 0.0),
            "gripper_finger_rIght_joint": (-0.05, 0.0),
        },
        closing_steps=8,
        approach_axis=(1.0, 0.0, 0.0),
    )


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


def make_a1z_model_config(
    *,
    has_gripper: bool = True,
    home_joints: list[float] | None = None,
    base_pose: PoseStamped | None = None,
    tf_extra_links: list[str] | None = None,
) -> RobotModelConfig:
    model_joint_names = joint_names(A1Z_DOF, prefix="arm_joint")
    model = (
        RobotModel.from_file(
            A1Z_G1Z_MODEL_PATH if has_gripper else A1Z_FLANGE_MODEL_PATH,
            package_paths=A1Z_PACKAGE_PATHS,
        )
        .with_default_joint_acceleration_limit(2.0)
        .with_joint_position_limits("arm_joint4", lower=-1.484, upper=1.484)
    )
    if has_gripper:
        model = (
            model.with_fixed_frame(
                "gripper_eef_link",
                "arm_link6",
                xyz=(0.0727, 0.0, 0.0),
            )
            .with_prismatic_joint(
                "gripper_finger_left_joint",
                axis=(0.0, 1.0, 0.0),
                lower=0.0,
                upper=0.05,
            )
            .with_prismatic_joint(
                "gripper_finger_rIght_joint",
                axis=(0.0, 1.0, 0.0),
                lower=-0.05,
                upper=0.0,
            )
        )
    exclusions = (
        [*A1Z_COLLISION_EXCLUSIONS, *A1Z_GRIPPER_COLLISION_EXCLUSIONS]
        if has_gripper
        else list(A1Z_COLLISION_EXCLUSIONS)
    )
    return RobotModelConfig(
        model=model,
        base_pose=base_pose if base_pose is not None else PoseStamped(),
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
        collision_exclusion_pairs=exclusions,
        gripper=a1z_gripper_config() if has_gripper else None,
        tf_extra_links=list(tf_extra_links or []),
        home_joints=home_joints or [0.0] * A1Z_DOF,
    )
