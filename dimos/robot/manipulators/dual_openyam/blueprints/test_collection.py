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

from typing import cast

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.collection import (
    DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
    DUAL_OPENYAM_COLLECTION_HOME_JOINTS,
    DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S,
    DUAL_OPENYAM_COLLECTION_MOTION_SCALE,
    DUAL_OPENYAM_COLLECTION_STREAMS,
    DUAL_OPENYAM_PLANNING_GROUPS,
    DUAL_OPENYAM_WRIST_CAMERA_SERIALS,
    DualOpenYamCollectionRecorder,
    DualOpenYamCollectionTeleopModule,
    DualOpenYamHomeButtonConfig,
    DualOpenYamHomeButtonModule,
    dual_openyam_collection_teleop_task,
    dual_openyam_home_targets,
    dual_openyam_wrist_camera,
    scale_controller_pose,
    teleop_webxr_dual_openyam_collection,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.config import dual_openyam_model_config
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS
from dimos.teleop.webxr.extensions import ArmTeleopModule


def _tasks(blueprint: Blueprint) -> list[TaskConfig]:
    coordinator = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, TeleopControlCoordinator)
    )
    return cast("list[TaskConfig]", coordinator.kwargs["tasks"])


def _teleop_task(blueprint: Blueprint) -> TaskConfig:
    return next(task for task in _tasks(blueprint) if task.type == "teleop_ik")


def test_collection_teleop_is_the_stock_task_at_the_collection_speed() -> None:
    ours = dual_openyam_collection_teleop_task()
    stock = _teleop_task(teleop_webxr_dual_openyam)

    speed_keys = {"max_joint_velocity_rad_s", "joint_velocity_limits_rad_s"}
    assert ours.params["max_joint_velocity_rad_s"] == DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S
    assert ours.params["max_joint_velocity_rad_s"] < stock.params["max_joint_velocity_rad_s"]
    assert ours.params["joint_velocity_limits_rad_s"] == {
        "left_joint1": DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
        "right_joint1": DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
    }
    assert (
        DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S < DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S
    )
    assert {k: v for k, v in ours.params.items() if k not in speed_keys} == {
        k: v for k, v in stock.params.items() if k not in speed_keys
    }
    assert (ours.name, ours.priority, ours.joint_names) == (
        stock.name,
        stock.priority,
        stock.joint_names,
    )


def test_controller_pose_scale_keeps_orientation_and_stamp() -> None:
    pose = PoseStamped(
        position=[0.4, -0.2, 0.8], orientation=[0.0, 0.0, 0.7071, 0.7071], ts=12.5, frame_id="right"
    )
    scaled = scale_controller_pose(pose, 0.5)

    assert (scaled.position.x, scaled.position.y, scaled.position.z) == (0.2, -0.1, 0.4)
    assert scaled.orientation == pose.orientation
    assert (scaled.ts, scaled.frame_id) == (12.5, "right")
    assert 0.0 < DUAL_OPENYAM_COLLECTION_MOTION_SCALE <= 1.0
    assert issubclass(DualOpenYamCollectionTeleopModule, ArmTeleopModule)


def test_collection_graph_only_swaps_the_teleop_task_and_module() -> None:
    stock = teleop_webxr_dual_openyam
    ours = teleop_webxr_dual_openyam_collection

    assert sorted(ours.remapping_map.values()) == sorted(stock.remapping_map.values())
    ours_modules = [atom.module for atom in ours.blueprints]
    stock_modules = [atom.module for atom in stock.blueprints]
    assert len(ours_modules) == len(stock_modules)
    for a, b in zip(ours_modules, stock_modules, strict=True):
        assert a is b or (b is ArmTeleopModule and a is DualOpenYamCollectionTeleopModule)
    stock_task, our_task = _teleop_task(stock), _teleop_task(ours)
    assert our_task.name == stock_task.name
    assert our_task.joint_names == stock_task.joint_names
    assert our_task.params["bindings"] == stock_task.params["bindings"]
    assert our_task.params["solver_type"] is stock_task.params["solver_type"]
    assert our_task.params != stock_task.params
    for a, b in zip(ours.blueprints, stock.blueprints, strict=True):
        if a.module is not DualOpenYamCoordinator:
            assert a.kwargs == b.kwargs
            assert a.module is b.module or issubclass(a.module, b.module)
    assert [t for t in _tasks(ours) if t.type != "teleop_ik"] == [
        t for t in _tasks(stock) if t.type != "teleop_ik"
    ]


@pytest.mark.parametrize("side", ["left", "right"])
def test_wrist_camera_streams_carry_the_side_prefix(side: str) -> None:
    camera = dual_openyam_wrist_camera(side)
    atom = camera.blueprints[0]

    assert atom.kwargs["serial_number"] == DUAL_OPENYAM_WRIST_CAMERA_SERIALS[side]
    assert atom.kwargs["enable_depth"] is False
    remapped = {camera.remapping_map[(atom.name, s.name)] for s in atom.streams}
    assert f"{side}_wrist_color_image" in remapped
    assert all("/" not in name for name in remapped)


def test_wrist_camera_rejects_an_unknown_side() -> None:
    with pytest.raises(ValueError):
        dual_openyam_wrist_camera("centre")


def test_recorder_declares_every_collection_stream_as_an_input() -> None:
    atom = DualOpenYamCollectionRecorder.blueprint().blueprints[0]
    inputs = {s.name for s in atom.streams if s.direction == "in"}

    assert set(DUAL_OPENYAM_COLLECTION_STREAMS) <= inputs


def test_home_targets_split_the_pose_per_planning_group() -> None:
    model = dual_openyam_model_config()
    assert tuple(g.name for g in model.planning_groups) == DUAL_OPENYAM_PLANNING_GROUPS

    targets = dual_openyam_home_targets(DUAL_OPENYAM_COLLECTION_HOME_JOINTS)
    assert set(targets) == set(DUAL_OPENYAM_PLANNING_GROUPS)
    names = [n for g in DUAL_OPENYAM_PLANNING_GROUPS for n in targets[g].name]
    positions = [p for g in DUAL_OPENYAM_PLANNING_GROUPS for p in targets[g].position]
    assert names == list(DUAL_OPENYAM_ARM_JOINTS)
    assert positions == DUAL_OPENYAM_COLLECTION_HOME_JOINTS
    with pytest.raises(ValueError):
        dual_openyam_home_targets([0.0] * 6)


def test_home_button_config_rejects_bad_inputs() -> None:
    config = DualOpenYamHomeButtonConfig()
    assert (config.home_button, config.set_home_button) == ("A", "X")
    assert 0.0 < config.speed_scale < 1.0
    assert config.home_joints == DUAL_OPENYAM_COLLECTION_HOME_JOINTS
    with pytest.raises(ValueError):
        DualOpenYamHomeButtonConfig(home_button="Z")
    with pytest.raises(ValueError):
        DualOpenYamHomeButtonConfig(home_joints=[0.0] * 6)
    with pytest.raises(ValueError):
        DualOpenYamHomeButtonConfig(speed_scale=1.5)


def test_home_button_module_binds_buttons_and_the_planner() -> None:
    from dimos.imitation.collection.blueprint import learning_collect_webxr_dual_openyam

    atom = next(
        a
        for a in learning_collect_webxr_dual_openyam.blueprints
        if a.module is DualOpenYamHomeButtonModule
    )
    inputs = {s.name for s in atom.streams if s.direction == "in"}
    assert {"button_pressed", "teleop_buttons", "coordinator_joint_state"} <= inputs
    assert [(r.name, r.spec) for r in atom.module_refs] == [("_manipulation", ManipulationModule)]
    assert {
        DualOpenYamHomeButtonConfig().home_button,
        DualOpenYamHomeButtonConfig().set_home_button,
    } == {"A", "X"}


def test_home_button_skips_the_move_when_already_home() -> None:
    module = DualOpenYamHomeButtonModule.__new__(DualOpenYamHomeButtonModule)
    module.config = DualOpenYamHomeButtonConfig()
    module._lock = __import__("threading").Lock()
    module._grip_held = False
    module._worker = None
    module._home_joints = list(DUAL_OPENYAM_COLLECTION_HOME_JOINTS)
    module._latest_joints = JointState(
        name=[*DUAL_OPENYAM_ARM_JOINTS, "left_arm/gripper", "right_arm/gripper"],
        position=[*(v + 0.01 for v in DUAL_OPENYAM_COLLECTION_HOME_JOINTS), 1.0, 1.0],
    )
    module._manipulation = None  # must not be touched when already home

    module._go_home()
    assert module._worker is None

    with module._lock:
        assert module._distance_to_home_locked() == pytest.approx(0.01)
    module._latest_joints = None
    with module._lock:
        assert module._distance_to_home_locked() is None
