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

"""Galaxea A1Z planning model + hardware configuration helpers.

The A1Z is Galaxea's standalone 6-DOF arm, driven over classic CAN via the
open-source SDK (github.com/userguide-galaxea/GALAXEA-A1Z) wrapped by the
``a1z`` ManipulatorAdapter. Model = the ``A1Z_Flange`` package (no gripper,
nq=6), vendored as the ``a1z_description`` LFS asset and used directly as the
FK/IK model.
"""

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

A1Z_DOF = 6

# Link pairs Drake flags as overlapping at the home pose (q=0); without these,
# planning reports COLLISION_AT_START. Enumerated via ComputePointPairPenetration.
A1Z_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("base_link", "arm_link1"),
    ("arm_link1", "arm_link2"),
    ("arm_link2", "arm_link3"),
    ("arm_link2", "arm_link5"),
    ("arm_link3", "arm_link4"),
    ("arm_link4", "arm_link5"),
    ("arm_link4", "arm_link6"),
    ("arm_link5", "arm_link6"),
]

# The URDF references meshes as ``package://A1Z_Flange/...``.
A1Z_MODEL_PATH = LfsPath("a1z_description") / "urdf/A1Z_Flange.urdf"
A1Z_FK_MODEL = A1Z_MODEL_PATH  # already gripper-free (nq=6)
A1Z_PACKAGE_PATHS: dict[str, Path] = {"A1Z_Flange": LfsPath("a1z_description")}


def _adapter_kwargs(home_joints: list[float] | None = None) -> dict[str, object]:
    if home_joints is None:
        return {}
    return {"initial_positions": home_joints}


def make_a1z_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
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
        joints=make_joints(hw_id, A1Z_DOF),
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        gripper_joints=[],
        adapter_kwargs=kwargs,
    )


def a1z_hardware(
    hw_id: str = "arm",
    *,
    mock_without_address: bool = True,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Real A1Z hardware when a CAN port is configured, else the mock adapter.

    Uses ``global_config.can_port`` (e.g. ``dimos run ... --can-port can0``).
    There is no MuJoCo sim asset for the A1Z, so the address-less path is the
    kinematic mock (sufficient for the full planning / teleop / viz stack).
    """
    if global_config.can_port:
        return make_a1z_hardware(
            hw_id,
            adapter_type="a1z",
            address=global_config.can_port,
            home_joints=home_joints,
        )
    if mock_without_address:
        return make_a1z_hardware(hw_id, home_joints=home_joints)
    return make_a1z_hardware(hw_id, adapter_type="a1z", address="can0", home_joints=home_joints)


def make_a1z_model_config(
    name: str = "arm",
    *,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    joint_prefix: str | None = None,
    coordinator_task_name: str | None = None,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=A1Z_MODEL_PATH,
        base_pose=base_pose(x_offset, y_offset, z_offset),
        joint_names=joint_names(A1Z_DOF, prefix="arm_joint"),
        end_effector_link="arm_link6",
        base_link="base_link",
        package_paths=A1Z_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=A1Z_COLLISION_EXCLUSIONS,
        joint_name_mapping=coordinator_joint_mapping(
            name,
            A1Z_DOF,
            joint_prefix=joint_prefix,
            urdf_joint_prefix="arm_",
        ),
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        home_joints=home_joints or [0.0] * A1Z_DOF,
    )
