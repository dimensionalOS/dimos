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

"""Graph-level checks for ``alfred-nav`` and ``alfred-sim``: composition only, no hardware."""

from __future__ import annotations

from typing import Any, cast

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_LIFT_LOWER_M,
    ALFRED_LIFT_UPPER_M,
    alfred_joint_names,
    alfred_model_config,
)
from dimos.robot.diy.alfred.blueprints.alfred_nav import alfred_nav
from dimos.robot.diy.alfred.blueprints.alfred_sim import alfred_sim
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_LIFT_JOINT,
    PillarConnection,
)
from dimos.robot.manipulators.openarm.config import OPENARM_HARDWARE_ID
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop


def _atoms(blueprint: Blueprint, module: type) -> list[Any]:
    return [
        atom
        for atom in blueprint.blueprints
        if isinstance(atom.module, type) and issubclass(atom.module, module)
    ]


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    (atom,) = _atoms(blueprint, ControlCoordinator)
    return cast("dict[str, Any]", atom.kwargs)


def test_alfred_nav_keeps_the_base_out_of_the_coordinator() -> None:
    """AlfredHighLevel owns the FlowBase (Portal + wheel odom); the coordinator must not."""
    assert _atoms(alfred_nav, AlfredHighLevel), "navigation base owner missing"
    hardware_ids = {hw.hardware_id for hw in _coordinator_kwargs(alfred_nav)["hardware"]}
    assert hardware_ids == {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID}


def test_alfred_nav_tasks_cover_lift_and_both_arms() -> None:
    (task,) = cast("list[TaskConfig]", _coordinator_kwargs(alfred_nav)["tasks"])
    assert task.type == "trajectory"
    assert set(task.joint_names) == set(alfred_joint_names())
    limits = task.params["velocity_limits"]
    assert set(limits) == set(task.joint_names)
    assert limits[PILLAR_LIFT_JOINT] == 0.1


def test_alfred_nav_composes_nav_planner_pillar_and_teleop() -> None:
    assert _atoms(alfred_nav, MovementManager)
    assert _atoms(alfred_nav, PillarConnection)
    assert _atoms(alfred_nav, ManipulationModule)
    assert _atoms(alfred_nav, KeyboardTeleop)
    # The operator twist must reach MovementManager's mux, never the base directly.
    (teleop,) = _atoms(alfred_nav, KeyboardTeleop)
    assert alfred_nav.remapping_map[(teleop.name, "cmd_vel")] == "tele_cmd_vel"


def test_alfred_nav_planner_publishes_no_world_rooted_tf() -> None:
    (atom,) = _atoms(alfred_nav, ManipulationModule)
    assert atom.kwargs["model"].tf_extra_links == []


def test_alfred_model_uses_pillar_joint_convention() -> None:
    """Lift is negative below the top switch, exactly as pillar_connection reports it."""
    config = alfred_model_config()
    assert config.joint_names[0] == PILLAR_LIFT_JOINT
    assert ALFRED_LIFT_LOWER_M == -0.5
    assert ALFRED_LIFT_UPPER_M == -0.002
    groups = {group.name: group for group in config.planning_groups}
    assert groups["lift"].joint_names == (PILLAR_LIFT_JOINT,)
    assert set(groups) == {"lift", "left_manipulator", "right_manipulator"}


def test_alfred_sim_still_composes() -> None:
    hardware_ids = {hw.hardware_id for hw in _coordinator_kwargs(alfred_sim)["hardware"]}
    assert {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID, "casters"} <= hardware_ids
