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
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.collection import (
    DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S,
    DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S,
    DUAL_OPENYAM_COLLECTION_STREAMS,
    DUAL_OPENYAM_COLLECTION_ZERO_BUTTON,
    dual_openyam_collection_teleop_tasks,
    dual_openyam_wrist_camera,
    teleop_webxr_dual_openyam_collection,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.collection import (
    DUAL_OPENYAM_COLLECTION_MOTION_SCALE,
    DualOpenYamCollectionRecorder,
    DualOpenYamCollectionTeleopModule,
    scale_controller_pose,
)
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
        base_joint = our_task.joint_names[0]
        assert our_task.params["joint_velocity_limits_rad_s"] == {
            base_joint: DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S
        }
        assert {k: v for k, v in our_task.params.items() if k not in speed_keys} == {
            k: v for k, v in stock_task.params.items() if k not in speed_keys
        }
        assert (our_task.name, our_task.priority, our_task.joint_names) == (
            stock_task.name,
            stock_task.priority,
            stock_task.joint_names,
        )
    assert (
        DUAL_OPENYAM_COLLECTION_BASE_JOINT_SPEED_RAD_S < DUAL_OPENYAM_COLLECTION_JOINT_SPEED_RAD_S
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


def test_collection_graph_only_swaps_the_teleop_tasks_module_and_zero_button() -> None:
    stock = teleop_webxr_dual_openyam
    ours = teleop_webxr_dual_openyam_collection

    assert sorted(ours.remapping_map.values()) == sorted(stock.remapping_map.values())
    assert len(ours.blueprints) == len(stock.blueprints)
    for a, b in zip(ours.blueprints, stock.blueprints, strict=True):
        if b.module is ArmTeleopModule:
            assert a.module is DualOpenYamCollectionTeleopModule
            assert a.kwargs == {**b.kwargs, "zero_button": DUAL_OPENYAM_COLLECTION_ZERO_BUTTON}
        elif b.module is DualOpenYamCoordinator:
            assert a.module is b.module
        else:
            assert (a.module, a.kwargs) == (b.module, b.kwargs)
    for our_task, stock_task in zip(_teleop_tasks(ours), _teleop_tasks(stock), strict=True):
        assert our_task.params["bindings"] == stock_task.params["bindings"]
        assert our_task.params["solver_type"] is stock_task.params["solver_type"]
        assert our_task.params != stock_task.params
    assert [t for t in _tasks(ours) if t.type != "teleop_ik"] == [
        t for t in _tasks(stock) if t.type != "teleop_ik"
    ]


def test_zero_button_does_not_clash_with_the_episode_monitor() -> None:
    monitor = EpisodeMonitorModule.blueprint(task="x").blueprints[0]
    taken = set(monitor.kwargs.get("button_map", {}).values()) or {"B", "Y"}
    assert DUAL_OPENYAM_COLLECTION_ZERO_BUTTON not in taken


@pytest.mark.parametrize("side", ["left", "right"])
def test_wrist_camera_streams_carry_the_side_prefix(side: str) -> None:
    camera = dual_openyam_wrist_camera(side)
    atom = camera.blueprints[0]

    assert "serial_number" not in atom.kwargs
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
