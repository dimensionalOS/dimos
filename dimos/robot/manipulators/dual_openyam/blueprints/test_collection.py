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

from pathlib import Path
from typing import cast

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.collection import (
    DUAL_OPENYAM_CAMERAS,
    DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
    DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S,
    DUAL_OPENYAM_COLLECTION_ZERO_BUTTON,
    dual_openyam_collection_camera,
    dual_openyam_collection_profile,
    dual_openyam_collection_teleop_tasks,
    teleop_webxr_dual_openyam_collection,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.blueprints.workcell_collection import (
    dual_openyam_workcell_collection,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.teleop.webxr.extensions import ArmTeleopModule


def _tasks(blueprint: Blueprint) -> list[TaskConfig]:
    coordinator = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, TeleopControlCoordinator)
    )
    return cast("list[TaskConfig]", coordinator.kwargs["tasks"])


def _teleop_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    return [task for task in _tasks(blueprint) if task.type == "teleop_ik"]


def test_collection_teleop_is_the_stock_task_at_the_collection_speed() -> None:
    ours = dual_openyam_collection_teleop_tasks()
    stock = _teleop_tasks(teleop_webxr_dual_openyam)
    assert len(ours) == len(stock) == 2

    speed_keys = {"max_joint_velocity_rad_s", "joint_velocity_limits_rad_s"}
    for our_task, stock_task in zip(ours, stock, strict=True):
        assert our_task.params["max_joint_velocity_rad_s"] == (
            DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S
        )
        assert (
            our_task.params["max_joint_velocity_rad_s"]
            < (stock_task.params["max_joint_velocity_rad_s"])
        )
        assert our_task.params["joint_velocity_limits_rad_s"] == {
            our_task.joint_names[0]: DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S
        }
        assert {k: v for k, v in our_task.params.items() if k not in speed_keys} == {
            k: v for k, v in stock_task.params.items() if k not in speed_keys
        }
        assert (our_task.name, our_task.priority, our_task.joint_names) == (
            stock_task.name,
            stock_task.priority,
            stock_task.joint_names,
        )


def test_collection_graph_only_swaps_the_teleop_tasks_and_zero_button() -> None:
    stock = teleop_webxr_dual_openyam
    ours = teleop_webxr_dual_openyam_collection

    assert sorted(map(str, ours.remapping_map.values())) == sorted(
        map(str, stock.remapping_map.values())
    )
    assert len(ours.blueprints) == len(stock.blueprints)
    for a, b in zip(ours.blueprints, stock.blueprints, strict=True):
        if b.module is ArmTeleopModule:
            assert a.module is ArmTeleopModule
            assert a.kwargs == {**b.kwargs, "zero_button": DUAL_OPENYAM_COLLECTION_ZERO_BUTTON}
        elif b.module is DualOpenYamCoordinator:
            assert a.module is b.module
        else:
            assert (a.module, a.kwargs) == (b.module, b.kwargs)
    assert [t for t in _tasks(ours) if t.type != "teleop_ik"] == [
        t for t in _tasks(stock) if t.type != "teleop_ik"
    ]


def test_zero_button_does_not_clash_with_the_episode_monitor() -> None:
    monitor = EpisodeMonitorModule.blueprint(task="x").blueprints[0]
    taken = set(monitor.kwargs.get("button_map", {}).values()) or {"B", "Y"}
    assert DUAL_OPENYAM_COLLECTION_ZERO_BUTTON not in taken


@pytest.mark.parametrize("name", DUAL_OPENYAM_CAMERAS)
def test_collection_camera_streams_carry_the_camera_prefix(name: str) -> None:
    camera = dual_openyam_collection_camera(name)
    atom = camera.blueprints[0]

    assert "serial_number" not in atom.kwargs
    assert atom.kwargs["enable_depth"] is False
    remapped = {str(camera.remapping_map[(atom.name, s.name)]) for s in atom.streams}
    assert f"{name}_color_image" in remapped
    assert all("/" not in stream for stream in remapped)


def test_collection_camera_rejects_an_unknown_name() -> None:
    with pytest.raises(ValueError):
        dual_openyam_collection_camera("chest")


def test_profile_records_three_images_the_joints_and_the_commanded_targets() -> None:
    profile = dual_openyam_collection_profile("dual-openyam-quest")

    assert profile.input_types() == {
        **{f"{camera}_color_image": Image for camera in DUAL_OPENYAM_CAMERAS},
        "coordinator_joint_state": JointState,
        "coordinator_joint_command": JointState,
    }
    assert profile.observations["observation.state"].names == list(DUAL_OPENYAM_JOINTS)
    assert profile.actions["action"].source_kind == "joint_position_updates"
    assert profile.metadata == {"name": "dual-openyam-quest", "robot_type": "dual_openyam"}


def test_workcell_collection_records_first_and_takes_normal_configuration(tmp_path: Path) -> None:
    modules = [atom.module for atom in dual_openyam_workcell_collection.active_blueprints]
    assert issubclass(modules[0], CollectionModule)
    assert modules[1] is EpisodeMonitorModule
    outputs = {
        dual_openyam_workcell_collection.remapping_map.get((atom.name, stream.name), stream.name)
        for atom in dual_openyam_workcell_collection.active_blueprints
        for stream in atom.streams
        if stream.direction == "out"
    }
    assert set(dual_openyam_collection_profile("x").input_types()) <= outputs

    parsed = BlueprintConfigParser(dual_openyam_workcell_collection).parse(
        [
            "--recorder.recording",
            str(tmp_path / "session"),
            "--episodes.task",
            "stack the cups",
            "--left-wrist-camera.serial-number",
            "123",
        ],
        environ={},
    )
    recorder = next(
        atom
        for atom in dual_openyam_workcell_collection.active_blueprints
        if atom.name == "recorder"
    )
    assert recorder.kwargs["recording_schema"].robot_type == "dual_openyam"
    assert parsed.module_configs["recorder"]["recording"] == tmp_path / "session"
    assert parsed.module_configs["left_wrist_camera"]["serial_number"] == "123"
