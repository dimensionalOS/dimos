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

"""The simulator runs the navigation the robot runs, with only the step limit lowered to its policy's."""

from dataclasses import replace

import pytest

from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.module import ModuleBase
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.embodiment.m20 import M20
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.deeprobotics.m20.blueprints._nav import m20_nav_stack
from dimos.robot.deeprobotics.m20.blueprints.m20 import m20
from dimos.robot.deeprobotics.m20.rerun import m20_nav_overrides, m20_nav_static
from dimos.simulation.m20_sim.world import SIM_M20, STEP_LIMIT
from dimos.spec.utils import Spec
from dimos.visualization.rerun.bridge import RerunBridgeModule

# The robot's connection gates nav velocity, the sim world takes it directly.
_SIM = m20.solidify(choices={"M20Mode": "sim"})
_SIM_BENCH = m20.solidify(choices={"M20Mode": "sim-bench"})
_REAL = m20.solidify(choices={"M20Mode": "real"})


def _atoms(blueprint: Blueprint) -> dict[str, BlueprintAtom]:
    return {atom.name: atom for atom in blueprint.active_blueprints}


def _remappings(
    blueprint: Blueprint, name: str
) -> dict[tuple[str, str], str | type[ModuleBase] | type[Spec]]:
    return {key: target for key, target in blueprint.remapping_map.items() if key[0] == name}


def test_sim_composes_the_navigation_the_robot_runs() -> None:
    sim, sim_stack, robot_stack = (
        _atoms(_SIM),
        _atoms(m20_nav_stack(SIM_M20).solidify()),
        _atoms(m20_nav_stack().solidify()),
    )
    assert sim_stack.keys() == robot_stack.keys()
    for name in robot_stack:
        assert (sim[name].module, sim[name].kwargs) == (
            sim_stack[name].module,
            sim_stack[name].kwargs,
        )
        assert _remappings(_SIM, name) == _remappings(m20_nav_stack().solidify(), name)


def test_the_sim_plans_only_steps_its_policy_climbs() -> None:
    assert SIM_M20.steppable == STEP_LIMIT
    assert replace(SIM_M20, steppable=M20.steppable) == M20


def test_the_world_feeds_the_streams_the_vendor_lio_and_the_urdf_manager_read() -> None:
    assert _remappings(_SIM, "simm20world") == {
        ("simm20world", "lidar"): "slam_body_points",
        ("simm20world", "odometry"): "slam_odom",
        ("simm20world", "joint_state"): "coordinator_joint_state",
    }


@pytest.mark.parametrize("blueprint", [_REAL, _SIM], ids=["real", "sim"])
def test_teleop_cancel_reaches_the_planner(blueprint: Blueprint) -> None:
    def goal_topic(module: type[ModuleBase]) -> tuple[str | type[ModuleBase] | type[Spec], type]:
        (atom,) = (a for a in blueprint.active_blueprints if a.module is module)
        (stream,) = (s for s in atom.streams if s.name == "goal")
        return blueprint.remapping_map.get((atom.name, "goal"), "goal"), stream.type

    assert goal_topic(MLSPlannerNative) == goal_topic(MovementManager)


def test_nothing_on_the_real_robot_takes_velocity() -> None:
    """Real mode has no motor controller: nothing reads what the navigation sends on cmd_vel."""
    readers = [
        atom.module
        for atom in _REAL.active_blueprints
        for stream in atom.streams
        if stream.type is Twist
        and stream.direction == "in"
        and _REAL.remapping_map.get((atom.name, stream.name), stream.name) == "cmd_vel"
    ]
    assert readers == []


def test_one_movement_manager() -> None:
    assert [a.module for a in _SIM.active_blueprints].count(MovementManager) == 1


def test_viewer_gets_the_navigation_config() -> None:
    (bridge,) = (a for a in _SIM.active_blueprints if a.module is RerunBridgeModule)
    assert m20_nav_overrides().keys() <= bridge.kwargs["visual_override"].keys()
    assert m20_nav_static().keys() <= bridge.kwargs["static"].keys()


@pytest.mark.parametrize("blueprint", [_SIM, _SIM_BENCH], ids=["sim", "sim-bench"])
def test_one_worker_per_module_with_gossip_off(blueprint: Blueprint) -> None:
    overrides = blueprint.global_config_overrides
    assert overrides["n_workers"] == len(blueprint.active_blueprints)
    assert overrides["zenoh_gossip"] is False


def test_real_is_the_default_mode() -> None:
    assert [a.name for a in m20.solidify().active_blueprints] == [
        a.name for a in _REAL.active_blueprints
    ]


def test_sim_bench_is_sim_with_an_episode_driver() -> None:
    assert _atoms(_SIM_BENCH).keys() == _atoms(_SIM).keys() | {"episodedriver"}
