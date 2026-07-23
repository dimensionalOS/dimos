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

"""Tests for basic xArm blueprints."""

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.xarm.blueprints.basic import dual_xarm6_planner_coordinator


def test_dual_xarm6_uses_roboplan_without_removing_grippers() -> None:
    planner = next(
        atom
        for atom in dual_xarm6_planner_coordinator.blueprints
        if atom.module is ManipulationModule
    )

    assert planner.kwargs["world_backend"] == "roboplan"
    assert all(robot.gripper_hardware_id for robot in planner.kwargs["robots"])
    assert all(robot.xacro_args["add_gripper"] == "true" for robot in planner.kwargs["robots"])
