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

"""The simulator runs the navigation the robot runs, unmodified."""

import pytest

from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.module import ModuleBase
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.deeprobotics.m20.blueprints._nav import (
    _m20_nav,
    m20_nav_overrides,
    m20_nav_static,
)
from dimos.robot.deeprobotics.m20.blueprints.m20_kronknav import (
    deeprobotics_m20_kronknav_control,
)
from dimos.robot.deeprobotics.m20.blueprints.m20_sim_nav import m20_sim_nav
from dimos.robot.deeprobotics.m20.blueprints.m20_sim_nav_episode import m20_sim_nav_episode
from dimos.spec.utils import Spec
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _atoms(blueprint: Blueprint) -> dict[str, BlueprintAtom]:
    return {atom.name: atom for atom in blueprint.active_blueprints}


def _remappings(
    blueprint: Blueprint, name: str
) -> dict[tuple[str, str], str | type[ModuleBase] | type[Spec]]:
    return {key: target for key, target in blueprint.remapping_map.items() if key[0] == name}


def test_sim_composes_the_navigation_the_robot_runs() -> None:
    sim, robot = _atoms(m20_sim_nav), _atoms(deeprobotics_m20_kronknav_control)
    for name in _atoms(_m20_nav):
        assert (sim[name].module, sim[name].kwargs) == (robot[name].module, robot[name].kwargs)
        assert _remappings(m20_sim_nav, name) == _remappings(
            deeprobotics_m20_kronknav_control, name
        )


def test_the_world_feeds_the_streams_the_vendor_lio_feeds() -> None:
    assert _remappings(m20_sim_nav, "simm20world") == {
        ("simm20world", "lidar"): "slam_body_points",
        ("simm20world", "odometry"): "slam_odom",
    }


def test_one_movement_manager() -> None:
    assert [a.module for a in m20_sim_nav.active_blueprints].count(MovementManager) == 1


def test_viewer_gets_the_navigation_config() -> None:
    (bridge,) = (a for a in m20_sim_nav.active_blueprints if a.module is RerunBridgeModule)
    assert m20_nav_overrides().keys() <= bridge.kwargs["visual_override"].keys()
    assert m20_nav_static().keys() <= bridge.kwargs["static"].keys()


@pytest.mark.parametrize("blueprint", [m20_sim_nav, m20_sim_nav_episode])
def test_one_worker_per_module_with_gossip_off(blueprint: Blueprint) -> None:
    overrides = blueprint.global_config_overrides
    assert overrides["n_workers"] == len(blueprint.active_blueprints)
    assert overrides["zenoh_gossip"] is False
