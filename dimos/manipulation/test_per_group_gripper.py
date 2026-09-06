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
from unittest.mock import MagicMock

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel


def _two_gripper_config() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/path/to/bimanual.urdf")),
        joint_names=["left/j1", "right/j1", "torso/j1"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                "left_manipulator",
                ("left/j1",),
                "base",
                "left/tool",
                gripper_hardware_id="left_arm",
            ),
            PlanningGroupDefinition(
                "right_manipulator",
                ("right/j1",),
                "base",
                "right/tool",
                gripper_hardware_id="right_arm",
            ),
            # No gripper anywhere: neither on the group nor model-wide.
            PlanningGroupDefinition("torso", ("torso/j1",), "base", "torso/tool"),
        ],
    )


def _module(module_factory, config: RobotModelConfig) -> ManipulationModule:
    module = module_factory()
    module.config.model = config
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry(config.planning_groups)
    module._control_coordinator = MagicMock()
    module._control_coordinator.task_invoke.return_value = True
    return module


def test_each_group_drives_its_own_gripper_task(module_factory) -> None:
    module = _module(module_factory, _two_gripper_config())

    assert {info.id: info.has_gripper for info in module.list_planning_groups()} == {
        "left_manipulator": True,
        "right_manipulator": True,
        "torso": False,
    }

    assert module.set_gripper_position(0.0, "left_manipulator").status is CommandStatus.SUCCEEDED
    assert module.set_gripper_position(1.0, "right_manipulator").status is CommandStatus.SUCCEEDED
    invoked = [call.args[0] for call in module._control_coordinator.task_invoke.call_args_list]
    assert invoked == ["left_arm_gripper", "right_arm_gripper"]

    rejected = module.set_gripper_position(0.0, "torso")
    assert rejected.status is CommandStatus.REJECTED
    assert "gripper-capable" in rejected.message


def test_the_model_wide_gripper_still_covers_groups_that_declare_none(module_factory) -> None:
    config = _two_gripper_config()
    config.planning_groups = [config.planning_groups[2]]
    config.gripper_hardware_id = "arm"
    module = _module(module_factory, config)

    assert [info.has_gripper for info in module.list_planning_groups()] == [True]
    assert module.set_gripper_position(0.5, "torso").status is CommandStatus.SUCCEEDED
    assert module._control_coordinator.task_invoke.call_args.args[0] == "arm_gripper"


def test_pick_and_place_sees_both_arms_as_gripper_capable(module_factory) -> None:
    module = _module(module_factory, _two_gripper_config())
    pick = PickAndPlaceModule()
    pick._manipulation = module

    groups = [info.id for info in module.list_planning_groups() if info.has_gripper]
    assert groups == ["left_manipulator", "right_manipulator"]
    # Two candidates, so an unqualified call stays ambiguous by design.
    assert pick._resolve_group(None) is None
    assert pick._resolve_group("left_manipulator") == "left_manipulator"
    assert pick._resolve_group("right_manipulator") == "right_manipulator"
    assert pick._resolve_group("torso") is None
    pick.stop()
