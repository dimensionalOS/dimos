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

"""OpenYAM hardware and planning model configuration helpers."""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.core.global_config import global_config
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)
from dimos.utils.data import LfsPath

OPENYAM_DOF = 6
OPENYAM_PACKAGE = LfsPath("yam_description")
OPENYAM_MODEL_PATH = OPENYAM_PACKAGE / "urdf/yam_gripper.urdf.xacro"
OPENYAM_GRAVITY_MODEL_PATH = OPENYAM_PACKAGE / "urdf/yam_gripper_gravity.urdf"
OPENYAM_PACKAGE_PATHS: dict[str, Path] = {"yam_description": OPENYAM_PACKAGE}


def make_openyam_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    auto_enable: bool = True,
    home_joints: list[float] | None = None,
    adapter_kwargs: dict[str, object] | None = None,
    include_gripper: bool = True,
) -> HardwareComponent:
    """Create OpenYAM hardware with six arm joints and one gripper channel."""
    kwargs: dict[str, object] = {}
    if adapter_type == "mock" and home_joints is not None:
        kwargs["initial_positions"] = home_joints
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, OPENYAM_DOF),
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        gripper_joints=[f"{hw_id}/gripper"] if include_gripper else [],
        adapter_kwargs=kwargs,
    )


def openyam_hardware(
    hw_id: str = "arm",
    *,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Select mock hardware in simulation and the OpenYAM Damiao adapter on hardware."""
    if global_config.simulation:
        return make_openyam_hardware(hw_id, home_joints=home_joints)
    if not Path(OPENYAM_GRAVITY_MODEL_PATH).is_file():
        raise ValueError(f"OpenYAM gravity model is missing: {OPENYAM_GRAVITY_MODEL_PATH}")
    return make_openyam_hardware(
        hw_id,
        adapter_type="openyam_damiao",
        address=global_config.can_port or "can0",
        # Physical encoder zeros are established by the driver; never pass
        # planning/home positions into a live motor adapter.
        adapter_kwargs={
            "gravity_model_path": OPENYAM_GRAVITY_MODEL_PATH,
        },
        include_gripper=False,
    )


def make_openyam_model_config(
    name: str = "arm",
    *,
    joint_prefix: str | None = None,
    coordinator_task_name: str | None = None,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    """Build a planning config for the gripper-equipped OpenYAM."""
    return RobotModelConfig(
        name=name,
        model_path=OPENYAM_MODEL_PATH,
        base_pose=base_pose(),
        joint_names=joint_names(OPENYAM_DOF, prefix="yam_joint"),
        end_effector_link="yam_hand_tcp",
        base_link="yam_base_link",
        package_paths=OPENYAM_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=[],
        joint_name_mapping=coordinator_joint_mapping(
            name,
            OPENYAM_DOF,
            joint_prefix=joint_prefix,
            urdf_joint_prefix="yam_",
        ),
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        gripper_hardware_id=name,
        home_joints=home_joints or [0.0] * OPENYAM_DOF,
    )
