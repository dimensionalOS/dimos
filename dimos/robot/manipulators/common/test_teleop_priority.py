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

import pytest

from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.openarm.blueprints.teleop import teleop_webxr_openarm
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam_planner,
    teleop_webxr_openyam,
)


@pytest.mark.parametrize(
    "blueprint",
    [
        teleop_webxr_openyam,
        teleop_webxr_dual_openyam,
        teleop_webxr_openarm,
        keyboard_teleop_openyam_planner,
    ],
)
def test_manual_arm_and_gripper_control_outrank_trajectories(blueprint):
    tasks = next(atom.kwargs["tasks"] for atom in blueprint.blueprints if "tasks" in atom.kwargs)
    trajectories = [task for task in tasks if task.type == "trajectory"]
    manual = [task for task in tasks if task.type in {"teleop_ik", "eef_twist", "gripper"}]
    assert len(trajectories) == 1
    assert manual
    assert all(task.priority > trajectories[0].priority for task in manual)
