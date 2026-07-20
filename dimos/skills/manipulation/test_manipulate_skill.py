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

from unittest.mock import MagicMock

from dimos.manipulation.manipulation_interface import ManipulationInterface
from dimos.robot.robot import Robot
from dimos.skills.manipulation.manipulate_skill import Manipulate
from dimos.types.manipulation import TranslationConstraint
from dimos.types.robot_capabilities import RobotCapability


class _RobotWithManipulation(Robot):
    def __init__(self, manipulation_interface: ManipulationInterface) -> None:
        super().__init__()
        self.capabilities = [RobotCapability.MANIPULATION]
        self.manipulation_interface = manipulation_interface

    def cleanup(self) -> None:
        pass


def test_manipulate_submits_task_and_returns_execution_result() -> None:
    """A skill call translates agent input into the task consumed by the interface."""
    # Arrange
    manipulation_interface = MagicMock(spec=ManipulationInterface)
    saved_constraint = TranslationConstraint(
        id="constraint-7", description="keep the handle aligned", translation_axis="x"
    )
    manipulation_interface.get_constraint.return_value = saved_constraint
    source_objects = [
        {"object_id": 7, "label": "cup", "depth": 0.42},
        {"object_id": 8, "label": "table", "depth": 0.8},
    ]
    manipulation_interface.get_latest_objects.return_value = source_objects
    expected_result = {"success": True}
    manipulation_interface.add_manipulation_task.return_value = None
    robot = _RobotWithManipulation(manipulation_interface)
    skill = Manipulate(
        robot=robot,
        description="move the cup to the tray",
        target_object="cup",
        target_point="(120, 240)",
        constraints=["constraint-7"],
        object_tolerances={"7": 0.75},
    )

    # Act
    result = skill()

    # Assert
    assert result == expected_result
    manipulation_interface.get_constraint.assert_called_once_with("constraint-7")
    manipulation_interface.get_latest_objects.assert_called_once_with()
    manipulation_interface.add_manipulation_task.assert_called_once()

    submitted_task = manipulation_interface.add_manipulation_task.call_args.args[0]
    assert submitted_task.description == "move the cup to the tray"
    assert submitted_task.target_object == "cup"
    assert submitted_task.target_point == (120, 240)
    assert submitted_task.get_constraints() == [saved_constraint]
    assert submitted_task.metadata["objects"] == {
        "7": {"object_id": 7, "label": "cup", "depth": 0.42, "movement_tolerance": 0.75},
        "8": {"object_id": 8, "label": "table", "depth": 0.8, "movement_tolerance": 0.0},
    }
    assert submitted_task.metadata["objects"]["7"] is not source_objects[0]
    assert submitted_task.metadata["objects"]["8"] is not source_objects[1]
