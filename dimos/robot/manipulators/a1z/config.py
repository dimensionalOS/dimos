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

import math
from pathlib import Path

import attrs

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.core.global_config import global_config
from dimos.hardware.manipulators.galaxea_a1z.config import (
    A1ZConfig,
    A1ZGripperConfig,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)
from dimos.utils.data import LfsPath

A1Z_DOF = 6
A1Z_SIM_GRIPPER_OPEN = 0.052
A1Z_SIM_HOME = [0.0, 0.05, -0.05, 0.0, 0.0, 0.0]
A1Z_GRIPPER_CONTACT_OFFSET = 0.075
_A1Z_GRASP_TO_TCP_PITCH = math.radians(-110.0)
A1Z_GRASP_FRAME_TO_TCP = Pose(
    position=Vector3(
        -A1Z_GRIPPER_CONTACT_OFFSET * math.cos(_A1Z_GRASP_TO_TCP_PITCH),
        0.0,
        A1Z_GRIPPER_CONTACT_OFFSET * math.sin(_A1Z_GRASP_TO_TCP_PITCH),
    ),
    orientation=Quaternion.from_euler(Vector3(0.0, _A1Z_GRASP_TO_TCP_PITCH, 0.0)),
)
A1Z_PRE_GRASP_DIRECTION = Vector3(-1.0, 0.0, 0.0)

A1Z_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("arm_link2", "arm_link5"),
    ("arm_link4", "arm_link6"),
]

A1Z_G1Z_MODEL_PATH = LfsPath("a1z_description") / "A1Z_G1Z/urdf/A1Z_G1Z.urdf"
A1Z_FLANGE_MODEL_PATH = LfsPath("a1z_description") / "A1Z_Flange/urdf/A1Z_Flange.urdf"
A1Z_G1Z_SIM_MODEL_PATH = LfsPath("a1z_description") / "A1Z_G1Z/mujoco/A1Z_G1Z.xml"
A1Z_FK_MODEL = A1Z_FLANGE_MODEL_PATH
A1Z_PACKAGE_PATHS: dict[str, Path] = {
    "A1Z_G1Z": LfsPath("a1z_description") / "A1Z_G1Z",
    "A1Z_Flange": LfsPath("a1z_description") / "A1Z_Flange",
}


def a1z_hardware(
    hw_id: str = "arm",
    *,
    has_gripper: bool = True,
    dynamics_urdf_path: Path | None = None,
    adapter_config: A1ZConfig | None = None,
) -> HardwareComponent:
    """Configure mock A1Z hardware unless an explicit CAN port selects the real adapter."""
    adapter_type = "mock"
    address = None
    adapter_kwargs: dict[str, object] = {}
    if not global_config.simulation and global_config.can_port:
        adapter_type = "galaxea_a1z"
        address = global_config.can_port
        resolved_config = adapter_config or A1ZConfig(
            gripper=A1ZGripperConfig() if has_gripper else None,
        )
        if (resolved_config.gripper is not None) != has_gripper:
            raise ValueError("has_gripper must match adapter_config.gripper")
        if dynamics_urdf_path is not None:
            # Preserve LfsPath laziness: the adapter resolves the model only when
            # connect() constructs the vendor robot.
            resolved_config = attrs.evolve(resolved_config, urdf_path=dynamics_urdf_path)
        adapter_kwargs["config"] = resolved_config

    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, A1Z_DOF),
        adapter_type=adapter_type,
        address=address,
        auto_enable=True,
        gripper_joints=[f"{hw_id}/gripper"] if has_gripper else [],
        gripper_open_position=0.1 if has_gripper else None,
        gripper_closed_position=0.0 if has_gripper else None,
        adapter_kwargs=adapter_kwargs,
    )


def make_a1z_model_config(
    name: str = "arm",
    *,
    has_gripper: bool = True,
    joint_prefix: str | None = None,
    home_joints: list[float] | None = None,
    placement: PoseStamped | None = None,
    pre_grasp_offset: float = 0.10,
    grasp_frame_to_tcp: Pose | None = None,
    pre_grasp_direction: Vector3 | None = None,
) -> RobotModelConfig:
    local_joint_names = joint_names(A1Z_DOF, prefix="arm_joint")
    return RobotModelConfig(
        name=name,
        model_path=A1Z_G1Z_MODEL_PATH if has_gripper else A1Z_FLANGE_MODEL_PATH,
        base_pose=placement if placement is not None else base_pose(),
        joint_names=local_joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joint_names),
                base_link="base_link",
                tip_link=("gripper_eef_link" if has_gripper else "arm_link6"),
            )
        ],
        package_paths=A1Z_PACKAGE_PATHS,
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
        pre_grasp_offset=pre_grasp_offset,
        grasp_frame_to_tcp=grasp_frame_to_tcp or Pose(),
        pre_grasp_direction=pre_grasp_direction or Vector3(0.0, 0.0, -1.0),
    )


def make_a1z_sim_robot_config(robot_base_pose: PoseStamped) -> RobotModelConfig:
    """Configure A1Z planning in the same world frame as its simulator binding."""
    return make_a1z_model_config(
        name="arm",
        has_gripper=True,
        home_joints=A1Z_SIM_HOME,
        placement=robot_base_pose,
        grasp_frame_to_tcp=A1Z_GRASP_FRAME_TO_TCP,
        pre_grasp_direction=A1Z_PRE_GRASP_DIRECTION,
    )
