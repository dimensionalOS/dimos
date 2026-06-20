# Copyright 2025-2026 Dimensional Inc.
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

"""OpenArm blueprints. Flip LEFT_CAN / RIGHT_CAN below if arms come up swapped."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.utils.data import LfsPath

OPENARM_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("openarm_left_link5", "openarm_left_link7"),
    ("openarm_right_link5", "openarm_right_link7"),
]

_OPENARM_PKG = LfsPath("openarm_description")
_OPENARM_LEFT_MODEL = _OPENARM_PKG / "urdf/robot/openarm_v10_left.urdf"
_OPENARM_RIGHT_MODEL = _OPENARM_PKG / "urdf/robot/openarm_v10_right.urdf"
OPENARM_V10_FK_MODEL = _OPENARM_PKG / "urdf/robot/openarm_v10_single.urdf"
_OPENARM_PACKAGE_PATHS: dict[str, Path] = {"openarm_description": _OPENARM_PKG}


def _base_pose() -> PoseStamped:
    return PoseStamped(
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def _validate_side(side: str) -> None:
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def _openarm_joints(side: str) -> list[str]:
    _validate_side(side)
    return [f"openarm_{side}_joint{i}" for i in range(1, 8)]


def _openarm_hardware(
    side: str,
    name: str | None = None,
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    adapter_kwargs: dict[str, Any] | None = None,
) -> HardwareComponent:
    _validate_side(side)
    kwargs = {"side": side}
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    return HardwareComponent(
        hardware_id=name or f"{side}_arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=_openarm_joints(side),
        adapter_type=adapter_type,
        address=address,
        adapter_kwargs=kwargs,
    )


def _openarm_model_config(side: str, name: str | None = None) -> RobotModelConfig:
    _validate_side(side)
    resolved_name = name or f"{side}_arm"
    return RobotModelConfig(
        name=resolved_name,
        model_path=_OPENARM_LEFT_MODEL if side == "left" else _OPENARM_RIGHT_MODEL,
        base_pose=_base_pose(),
        joint_names=_openarm_joints(side),
        end_effector_link=f"openarm_{side}_link7",
        base_link="openarm_body_link0",
        package_paths=_OPENARM_PACKAGE_PATHS,
        collision_exclusion_pairs=OPENARM_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        coordinator_task_name=f"traj_{resolved_name}",
        home_joints=[0.0] * 7,
    )


def _openarm_task(hw: HardwareComponent, name: str | None = None) -> TaskConfig:
    return TaskConfig(
        name=name or f"traj_{hw.hardware_id}",
        type="trajectory",
        joint_names=hw.joints,
        priority=10,
    )


def _openarm_single_hardware(
    *,
    adapter_type: str = "mock",
    address: str | None = None,
) -> HardwareComponent:
    return _openarm_hardware(
        "left",
        name="arm",
        adapter_type=adapter_type,
        address=address,
    )


def _openarm_single_model_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=OPENARM_V10_FK_MODEL,
        base_pose=_base_pose(),
        joint_names=_openarm_joints("left"),
        end_effector_link="openarm_left_link7",
        base_link="openarm_body_link0",
        package_paths=_OPENARM_PACKAGE_PATHS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        coordinator_task_name="traj_arm",
        home_joints=[0.0] * 7,
    )


# Mock bimanual: no hardware, great for verifying wiring.
_mock_left = _openarm_hardware(side="left")
_mock_right = _openarm_hardware(side="right")

coordinator_openarm_mock = ControlCoordinator.blueprint(
    hardware=[_mock_left, _mock_right],
    tasks=[
        _openarm_task(_mock_left),
        _openarm_task(_mock_right),
    ],
)

# Single-arm hardware blueprints (first real bring-up targets).
# CAN interface each physical arm is on. Linux assigns can0/can1 in USB
# enumeration order which is not guaranteed stable; if swapped, flip these.
LEFT_CAN = "can1"
RIGHT_CAN = "can0"

# Flip to False to skip the CTRL_MODE=MIT write at connect-time. Leave True for
# normal operation; it is idempotent and ensures motors are in the expected mode.
AUTO_SET_MIT_MODE = True

_ADAPTER_KWARGS = {"auto_set_mit_mode": AUTO_SET_MIT_MODE}
_left_hw = _openarm_hardware(
    side="left",
    address=LEFT_CAN,
    adapter_type="openarm",
    adapter_kwargs=_ADAPTER_KWARGS,
)
_right_hw = _openarm_hardware(
    side="right",
    address=RIGHT_CAN,
    adapter_type="openarm",
    adapter_kwargs=_ADAPTER_KWARGS,
)

coordinator_openarm_left = ControlCoordinator.blueprint(
    hardware=[_left_hw],
    tasks=[_openarm_task(_left_hw)],
)

coordinator_openarm_right = ControlCoordinator.blueprint(
    hardware=[_right_hw],
    tasks=[_openarm_task(_right_hw)],
)

coordinator_openarm_bimanual = ControlCoordinator.blueprint(
    hardware=[_left_hw, _right_hw],
    tasks=[
        _openarm_task(_left_hw),
        _openarm_task(_right_hw),
    ],
)


# Planner + coordinator (mock): Drake plans, mock adapters execute.
openarm_mock_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[
            _openarm_model_config("left"),
            _openarm_model_config("right"),
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
    ),
    ControlCoordinator.blueprint(
        hardware=[_mock_left, _mock_right],
        tasks=[
            _openarm_task(_mock_left),
            _openarm_task(_mock_right),
        ],
    ),
)

# Planner + coordinator (real hw): plan and execute on both arms.
openarm_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[
            _openarm_model_config("left"),
            _openarm_model_config("right"),
        ],
        planning_timeout=10.0,
        visualization={"backend": "meshcat"},
    ),
    ControlCoordinator.blueprint(
        hardware=[_left_hw, _right_hw],
        tasks=[
            _openarm_task(_left_hw),
            _openarm_task(_right_hw),
        ],
    ),
)


# Keyboard teleop (single arm, mock).
_teleop_hw = _openarm_single_hardware()

keyboard_teleop_openarm_mock = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=OPENARM_V10_FK_MODEL,
        ee_joint_id=7,
        joint_names=_teleop_hw.joints,
    ),
    ControlCoordinator.blueprint(
        hardware=[_teleop_hw],
        tasks=[
            TaskConfig(
                name="cartesian_ik_arm",
                type="cartesian_ik",
                joint_names=_teleop_hw.joints,
                priority=10,
                params={"model_path": OPENARM_V10_FK_MODEL, "ee_joint_id": 7},
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_openarm_single_model_config()],
        visualization={"backend": "meshcat"},
    ),
)

# Keyboard teleop (single arm, real hw on can0).
_teleop_real_hw = _openarm_single_hardware(adapter_type="openarm", address=LEFT_CAN)

keyboard_teleop_openarm = autoconnect(
    KeyboardTeleopModule.blueprint(
        model_path=OPENARM_V10_FK_MODEL,
        ee_joint_id=7,
        joint_names=_teleop_real_hw.joints,
    ),
    ControlCoordinator.blueprint(
        hardware=[_teleop_real_hw],
        tasks=[
            TaskConfig(
                name="cartesian_ik_arm",
                type="cartesian_ik",
                joint_names=_teleop_real_hw.joints,
                priority=10,
                params={"model_path": OPENARM_V10_FK_MODEL, "ee_joint_id": 7},
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_openarm_single_model_config()],
        visualization={"backend": "meshcat"},
    ),
)


__all__ = [
    "coordinator_openarm_bimanual",
    "coordinator_openarm_left",
    "coordinator_openarm_mock",
    "coordinator_openarm_right",
    "keyboard_teleop_openarm",
    "keyboard_teleop_openarm_mock",
    "openarm_mock_planner_coordinator",
    "openarm_planner_coordinator",
]
