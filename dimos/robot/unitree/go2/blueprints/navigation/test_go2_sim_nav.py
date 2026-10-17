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
from dimos.robot.unitree.go2.blueprints.basic.go2_sim import go2_sim
from dimos.robot.unitree.go2.blueprints.navigation.go2_nav import (
    _go2_nav,
    go2_nav_overrides,
    go2_nav_static,
)
from dimos.robot.unitree.go2.blueprints.navigation.go2_sim_nav import go2_sim_nav
from dimos.robot.unitree.go2.dds.blueprints import go2_dds_nav
from dimos.spec.utils import Spec
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _atoms(blueprint: Blueprint) -> dict[str, BlueprintAtom]:
    return {atom.name: atom for atom in blueprint.active_blueprints}


def _remappings(
    blueprint: Blueprint, name: str
) -> dict[tuple[str, str], str | type[ModuleBase] | type[Spec]]:
    return {key: target for key, target in blueprint.remapping_map.items() if key[0] == name}


def test_sim_composes_the_navigation_the_robot_runs() -> None:
    sim, robot = _atoms(go2_sim_nav), _atoms(go2_dds_nav)
    for name in _atoms(_go2_nav):
        assert (sim[name].module, sim[name].kwargs) == (robot[name].module, robot[name].kwargs)
        assert _remappings(go2_sim_nav, name) == _remappings(go2_dds_nav, name)


def test_one_movement_manager() -> None:
    assert [a.module for a in go2_sim_nav.active_blueprints].count(MovementManager) == 1


def test_viewer_gets_the_navigation_config() -> None:
    (bridge,) = (a for a in go2_sim_nav.active_blueprints if a.module is RerunBridgeModule)
    assert go2_nav_overrides().keys() <= bridge.kwargs["visual_override"].keys()
    assert go2_nav_static().keys() <= bridge.kwargs["static"].keys()


@pytest.mark.parametrize("blueprint", [go2_sim, go2_sim_nav])
def test_one_worker_per_module_with_gossip_off(blueprint: Blueprint) -> None:
    overrides = blueprint.global_config_overrides
    assert overrides["n_workers"] == len(blueprint.active_blueprints)
    assert overrides["zenoh_gossip"] is False
