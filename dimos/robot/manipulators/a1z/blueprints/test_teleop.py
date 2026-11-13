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

from dimos.control.coordinator import TaskConfig
from dimos.robot.manipulators.a1z.blueprints.teleop import A1ZWebXRCoordinator
from dimos.robot.manipulators.a1z.connection import A1ZConnection
from dimos.teleop.webxr.blueprints import teleop_webxr_a1z


def test_webxr_left_controller_moves_the_gripper_end_through_the_connection() -> None:
    modules = {atom.module: atom.kwargs for atom in teleop_webxr_a1z.blueprints}
    tasks = cast("list[TaskConfig]", modules[A1ZWebXRCoordinator]["tasks"])
    teleop = next(task for task in tasks if task.type == "teleop_ik")
    gripper = next(task for task in tasks if task.type == "gripper")

    assert A1ZConnection in modules
    assert teleop.joint_names == [f"arm/arm_joint{i}" for i in range(1, 7)]
    assert teleop.params["bindings"] == [{"hand": "left", "target_frame": "gripper_eef_link"}]
    assert gripper.stream_bind == {"gripper_command": "left_gripper_command"}
    assert teleop_webxr_a1z.remapping_map == {
        ("armteleopmodule", "left_controller_output"): "left_cartesian_command",
        ("armteleopmodule", "left_gripper_command"): "left_gripper_command",
        ("a1zconnection", "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }
