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

from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.navigation.nav_stack.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.nav_stack.modules.path_follower.path_follower import PathFollower
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.navigation.nav_stack.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.nav_stack.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt


def _module_count(blueprint, module_cls: type) -> int:
    return sum(1 for atom in blueprint.blueprints if atom.module is module_cls)


def test_nav_stack_uses_pgo_corrected_odometry_by_default() -> None:
    blueprint = create_nav_stack(planner="far")

    assert _module_count(blueprint, PGO) == 1
    assert blueprint.remapping_map[(PathFollower, "cmd_vel")] == "nav_cmd_vel"
    assert blueprint.remapping_map[(PGO, "global_map")] == "global_map_pgo"
    assert blueprint.remapping_map[(TerrainAnalysis, "odometry")] == "corrected_odometry"
    assert blueprint.remapping_map[(TerrainMapExt, "odometry")] == "corrected_odometry"
    assert blueprint.remapping_map[(FarPlanner, "odometry")] == "corrected_odometry"


def test_nav_stack_can_disable_pgo() -> None:
    blueprint = create_nav_stack(planner="far", use_pgo=False)

    assert _module_count(blueprint, PGO) == 0
    assert (PGO, "global_map") not in blueprint.remapping_map
    assert (TerrainAnalysis, "odometry") not in blueprint.remapping_map
    assert (TerrainMapExt, "odometry") not in blueprint.remapping_map
    assert (FarPlanner, "odometry") not in blueprint.remapping_map


def test_nav_stack_can_keep_pgo_for_viz_without_corrected_nav() -> None:
    blueprint = create_nav_stack(
        planner="far",
        use_pgo=True,
        use_pgo_corrected_odometry=False,
    )

    assert _module_count(blueprint, PGO) == 1
    assert blueprint.remapping_map[(PGO, "global_map")] == "global_map_pgo"
    assert (TerrainAnalysis, "odometry") not in blueprint.remapping_map
    assert (TerrainMapExt, "odometry") not in blueprint.remapping_map
    assert (FarPlanner, "odometry") not in blueprint.remapping_map


def test_nav_stack_corrected_odometry_requires_pgo() -> None:
    with pytest.raises(ValueError, match="use_pgo_corrected_odometry"):
        create_nav_stack(use_pgo=False, use_pgo_corrected_odometry=True)
