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

from typing import Any, get_origin, get_type_hints

import pytest

from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.registry import control_task_registry
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.stream import In
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.robot.manipulators.openyam.blueprints.basic import (
    coordinator_openyam,
    openyam_planner_coordinator,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam,
    keyboard_teleop_openyam_planner,
    teleop_webxr_openyam,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_HOME_JOINTS,
    OPENYAM_MODEL_PATH,
    make_openyam_model_config,
)
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.teleop_ik import OpenYamPinkPoseTargetSolver

_MODEL = make_openyam_model_config(robot="arm")
_JOINTS = [
    *_MODEL.joint_names,
    *[f"arm/{name}" for name in OpenYamConnection.robot.gripper_joints.values()],
]
_ARM_JOINTS = list(_MODEL.joint_names)
_GRIPPER_JOINT = "arm/joint7"
from dimos.teleop.webxr.extensions import ArmTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, module_type)
    )


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )


def test_make_openyam_model_config_uses_canonical_arm_joints(mocker) -> None:
    download = mocker.patch(
        "dimos.robot.assets.git_cache.GitAssetCache.resolve",
        side_effect=AssertionError("Config must not download models"),
    )
    config = make_openyam_model_config()

    assert config.model.source_path is OPENYAM_MODEL_PATH
    assert config.joint_names == OPENYAM_ARM_JOINTS
    assert config.base_link == "base"
    assert config.planning_groups[0].tip_link == "gripper_tip"
    assert "gripper_hardware_id" not in config.model_dump()
    assert config.home_joints == OPENYAM_HOME_JOINTS
    download.assert_not_called()


@pytest.mark.self_hosted
def test_openyam_model_contains_canonical_arm_joints() -> None:
    config = make_openyam_model_config()
    model = prepare_robot_model(config).description

    assert OPENYAM_MODEL_PATH.parts[-2:] == ("openyam_single", "openyam_single.urdf")
    assert [joint.name for joint in model.joints if joint.name in config.joint_names] == (
        OPENYAM_ARM_JOINTS
    )
    tip = model.get_joint("gripper_tip_joint")
    assert tip is not None
    assert (tip.parent_link, tip.child_link, tip.origin_xyz) == (
        "arm_tcp",
        "gripper_tip",
        (0.0, 0.0, 0.0),
    )


@pytest.mark.self_hosted
def test_openyam_jaw_is_joint7_and_joint8_copies_it() -> None:
    loaded = make_openyam_model_config(robot="arm").model.load()

    assert [
        joint.name for joint in movable_joints(make_openyam_model_config(robot="arm").model)
    ] == [
        *_ARM_JOINTS,
        _GRIPPER_JOINT,
    ]
    finger = loaded.get_joint("arm/joint7")
    follower = loaded.get_joint("arm/joint8")
    assert finger is not None and follower is not None
    assert (finger.type, finger.lower, finger.upper) == (
        "prismatic",
        0.01081,
        pytest.approx(0.0594641),
    )
    assert follower.mimic == ("arm/joint7", 1.0, 0.0)


def test_make_openyam_model_config_preserves_explicit_home() -> None:
    configured_home = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    config = make_openyam_model_config(home_joints=configured_home)

    assert config.home_joints == configured_home


def test_quest_teleop_matches_dual_openyam_response_tuning() -> None:
    tasks = _coordinator_kwargs(teleop_webxr_openyam)["tasks"]
    teleop = next(task for task in tasks if task.type == "teleop_ik")

    assert teleop.params["pink"].gain == 1.0
    assert teleop.params["solver_type"] is OpenYamPinkPoseTargetSolver
    assert teleop.params["max_joint_velocity_rad_s"] == 2.0
    assert teleop.params["joint_command_filter_cutoff_hz"] == 30.0


def test_openyam_connection_requires_an_explicit_selector() -> None:
    with pytest.raises(ValueError, match="Field required"):
        OpenYamConnection()
    physical = OpenYamConnection(can_port="can1")
    try:
        assert physical.config.source == "yam"
        assert physical.can_ports() == ["can1"]
    finally:
        physical.stop()


@pytest.mark.parametrize("port", ["", " "])
def test_openyam_connection_rejects_empty_selector(port: str) -> None:
    with pytest.raises(ValueError, match="must not be empty"):
        OpenYamConnection(can_port=port)


def test_quest_teleop_module_accepts_blueprint_config() -> None:
    kwargs = _module_kwargs(teleop_webxr_openyam, ArmTeleopModule)

    module = ArmTeleopModule(**kwargs)
    module.stop()


@pytest.mark.parametrize(
    "blueprint",
    [
        coordinator_openyam,
        openyam_planner_coordinator,
    ],
)
def test_openyam_basic_trajectory_accepts_all_deployed_joints(blueprint: Blueprint) -> None:
    kwargs = _coordinator_kwargs(blueprint)

    assert not kwargs.get("hardware")
    assert _module_kwargs(blueprint, OpenYamConnection)["source"] == "arm"
    trajectory = next(task for task in kwargs["tasks"] if task.type == "trajectory")
    gripper = next(task for task in kwargs["tasks"] if task.type == "gripper")

    assert trajectory.joint_names == _JOINTS
    assert (gripper.name, gripper.joint_names) == (
        "arm_gripper",
        [_GRIPPER_JOINT],
    )
    assert gripper.name == "arm_gripper"


@pytest.mark.parametrize(
    "blueprint",
    [
        keyboard_teleop_openyam,
        keyboard_teleop_openyam_planner,
    ],
)
def test_openyam_teleop_blueprints_expose_arm_and_gripper_control(
    blueprint: Blueprint,
) -> None:
    kwargs = _coordinator_kwargs(blueprint)
    claimed_joints = [task.joint_names for task in kwargs["tasks"]]

    assert not kwargs.get("hardware")
    assert _module_kwargs(blueprint, OpenYamConnection)["source"] == "arm"
    assert _ARM_JOINTS in claimed_joints
    assert all(joints in (_ARM_JOINTS, [_GRIPPER_JOINT], _JOINTS) for joints in claimed_joints)


@pytest.mark.parametrize(
    "blueprint",
    [
        keyboard_teleop_openyam,
        keyboard_teleop_openyam_planner,
        teleop_webxr_openyam,
    ],
)
def test_openyam_task_bindings_have_coordinator_inputs(blueprint: Blueprint) -> None:
    atom = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )
    annotations = get_type_hints(atom.module)
    for task in atom.kwargs["tasks"]:
        for binding in control_task_registry.bindings_for(task.type).consumes:
            port = task.stream_bind.get(binding.stream, binding.stream)
            assert get_origin(annotations.get(port)) is In, (task.name, port)


def test_keyboard_teleop_gripper_control_is_independent() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam)["tasks"]
    gripper = next(task for task in tasks if task.name == "arm_gripper")

    assert gripper.joint_names == [_GRIPPER_JOINT]
    assert gripper.type == "gripper"


def test_keyboard_teleop_openyam_overrides_planner_trajectory() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam_planner)["tasks"]
    trajectory = next(task for task in tasks if task.type == "trajectory")
    eef_twist = next(task for task in tasks if task.type == "eef_twist")

    assert trajectory.joint_names == _JOINTS
    assert eef_twist.priority > trajectory.priority


def test_keyboard_teleop_openyam_gripper_opens_toward_joint7_lower_end() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam)["tasks"]
    gripper = next(task for task in tasks if task.name == "arm_gripper")

    assert gripper.joint_names == [_GRIPPER_JOINT]
    assert gripper.params == {"open_at": "lower"}


def test_webxr_teleop_routes_pose_and_gripper_to_separate_tasks() -> None:
    tasks = _coordinator_kwargs(teleop_webxr_openyam)["tasks"]
    teleop = next(task for task in tasks if task.type == "teleop_ik")
    gripper = next(task for task in tasks if task.type == "gripper")

    assert teleop.params["bindings"] == [{"hand": "right", "target_frame": "gripper_tip"}]
    assert gripper.joint_names == [_GRIPPER_JOINT]
    assert gripper.stream_bind == {"gripper_command": "right_gripper_command"}
