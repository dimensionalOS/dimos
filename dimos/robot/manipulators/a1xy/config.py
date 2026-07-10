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

"""Galaxea A1X planning model configuration helpers.

The A1X is Galaxea's standalone 6-DOF arm (A1X/A1Y are documented together
as the "A1XY" family; they share the SDK and differ in joint arrangement).
Model source: the official Galaxea URDF repo (github.com/userguide-galaxea/URDF,
``A1X`` ROS package), vendored as the ``a1x_description`` LFS asset.

Notes on the model:

* URDF joint zero is the folded tabletop pose (same convention as the A1X
  arms on the R1 Lite); all six joints at 0 is a valid, collision-free pose.
* The vendor URDF includes the G1 gripper links. ``a1x_no_gripper.urdf``
  (derived, gripper elements stripped) is the FK/IK model so the Pinocchio
  chain is exactly the 6 actuated arm joints — same pattern as the Piper's
  ``piper_no_gripper_description``.
* Hardware wiring is not defined here yet: the transport (Galaxea ROS 2 SDK
  vs ROS 1 ``signal_arm``/``HDAS`` stacks) is pinned during bring-up, and the
  ``ManipulatorAdapter`` lands with it. Until then blueprints run on the
  ``mock`` adapter.
"""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)
from dimos.utils.data import LfsPath

# G1 gripper linkage + wrist pairs that legitimately touch in the full model.
A1X_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("arm_link6", "gripper_link"),
    ("gripper_link", "gripper_finger_link1"),
    ("gripper_link", "gripper_finger_link2"),
    ("gripper_finger_link1", "gripper_finger_link2"),
]

A1X_DOF = 6

A1X_MODEL_PATH = LfsPath("a1x_description") / "urdf/a1x.urdf"
A1X_FK_MODEL = LfsPath("a1x_description") / "urdf/a1x_no_gripper.urdf"
A1X_PACKAGE_PATHS: dict[str, Path] = {"a1x": LfsPath("a1x_description")}


def _adapter_kwargs(home_joints: list[float] | None = None) -> dict[str, object]:
    if home_joints is None:
        return {}
    return {"initial_positions": home_joints}


def make_a1x_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    gripper: bool = False,
    auto_enable: bool = True,
    adapter_kwargs: dict[str, object] | None = None,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    kwargs = _adapter_kwargs(home_joints)
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, A1X_DOF),
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        gripper_joints=[f"{hw_id}/gripper"] if gripper else [],
        adapter_kwargs=kwargs,
    )


def make_a1x_model_config(
    name: str = "arm",
    *,
    add_gripper: bool = False,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    joint_prefix: str | None = None,
    coordinator_task_name: str | None = None,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=A1X_MODEL_PATH,
        base_pose=base_pose(x_offset, y_offset, z_offset),
        joint_names=joint_names(A1X_DOF, prefix="arm_joint"),
        end_effector_link="gripper_link" if add_gripper else "arm_link6",
        base_link="base_link",
        package_paths=A1X_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=A1X_GRIPPER_COLLISION_EXCLUSIONS,
        joint_name_mapping=coordinator_joint_mapping(
            name,
            A1X_DOF,
            joint_prefix=joint_prefix,
            urdf_joint_prefix="arm_",
        ),
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        gripper_hardware_id=name if add_gripper else None,
        home_joints=home_joints or [0.0] * A1X_DOF,
    )
