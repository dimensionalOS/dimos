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

"""Construction assertions for WebXR manipulator blueprints."""

from typing import cast

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.manipulators.common.blueprints import TeleopBinding
from dimos.teleop.webxr.blueprints import (
    demo_pico_body_tracking,
    teleop_webxr_dual,
    teleop_webxr_hand_xarm7,
    teleop_webxr_xarm7,
)
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.body_tracking_monitor import BodyTrackingMonitor
from dimos.teleop.webxr.extensions import ArmTeleopModule, HandTeleopModule
from dimos.teleop.webxr.module import WebXRTeleopModule


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    atom = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )
    return cast("list[TaskConfig]", atom.kwargs["tasks"])


def _webxr_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    return [task for task in _coordinator_tasks(blueprint) if task.type == "teleop_ik"]


def _gripper_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    return [task for task in _coordinator_tasks(blueprint) if task.type == "gripper"]


def _binding(task: TaskConfig) -> TeleopBinding:
    bindings = task.params["bindings"]
    assert len(bindings) == 1
    return cast("TeleopBinding", bindings[0])


def test_single_arm_blueprint_uses_one_frame_binding_and_right_stream() -> None:
    tasks = _webxr_tasks(teleop_webxr_xarm7)

    assert len(tasks) == 1
    binding = _binding(tasks[0])
    assert binding["hand"] == "right"
    assert binding["target_frame"] == "link_tcp"
    assert tasks[0].params["robot_model"].name == "arm"
    gripper = _gripper_tasks(teleop_webxr_xarm7)[0]
    assert gripper.stream_bind == {"gripper_command": "right_gripper_command"}
    assert (
        teleop_webxr_xarm7.remapping_map[(ArmTeleopModule.name, "right_controller_output")]
        == "right_cartesian_command"
    )
    assert (
        teleop_webxr_xarm7.remapping_map[(ArmTeleopModule.name, "right_gripper_command")]
        == "right_gripper_command"
    )


def test_single_arm_hand_blueprint_uses_right_card_stream() -> None:
    assert (
        teleop_webxr_hand_xarm7.remapping_map[(HandTeleopModule.name, "right_controller_output")]
        == "right_cartesian_command"
    )


def test_mixed_arm_blueprint_keeps_two_independent_one_binding_tasks() -> None:
    tasks = _webxr_tasks(teleop_webxr_dual)

    assert len(tasks) == 2
    by_name = {task.name: task for task in tasks}
    assert _binding(by_name["teleop_xarm"]) == {
        "hand": "left",
        "target_frame": "link_tcp",
    }
    assert by_name["teleop_xarm"].params["robot_model"].name == "xarm_arm"
    assert _binding(by_name["teleop_piper"]) == {
        "hand": "right",
        "target_frame": "gripper_base",
    }
    assert by_name["teleop_piper"].params["robot_model"].name == "piper_arm"
    grippers = {task.name: task for task in _gripper_tasks(teleop_webxr_dual)}
    assert grippers["xarm_arm_gripper"].stream_bind == {"gripper_command": "left_gripper_command"}
    assert grippers["piper_arm_gripper"].stream_bind == {"gripper_command": "right_gripper_command"}
    assert (
        teleop_webxr_dual.remapping_map[(ArmTeleopModule.name, "left_controller_output")]
        == "left_cartesian_command"
    )
    assert (
        teleop_webxr_dual.remapping_map[(ArmTeleopModule.name, "right_controller_output")]
        == "right_cartesian_command"
    )


def test_pico_body_tracking_demo_connects_required_webxr_to_monitor() -> None:
    modules = {atom.module for atom in demo_pico_body_tracking.blueprints}
    webxr = next(
        atom for atom in demo_pico_body_tracking.blueprints if atom.module is WebXRTeleopModule
    )
    assert modules == {WebXRTeleopModule, BodyTrackingMonitor}
    assert webxr.kwargs["body_tracking_mode"] == "required"
    assert ("body_tracking", BodyTrackingSnapshot) not in demo_pico_body_tracking.transport_map
