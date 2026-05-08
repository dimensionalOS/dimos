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

"""E2E: SimplePlanner (grid A*) cross-wall planning — apples-to-apples mirror of test_cross_wall_planning_far.py."""

from __future__ import annotations

import pytest

pytest.importorskip("gtsam")

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.navigation.nav_stack.tests.conftest import run_cross_wall_test
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_nav_sim import nav_config
from dimos.simulation.unity.module import UnityBridgeModule

pytestmark = [pytest.mark.slow, pytest.mark.skipif_in_ci]


class TestCrossWallPlanningSimple:
    """E2E: cross-wall routing with SimplePlanner (A* on 2D costmap)."""

    def test_cross_wall_sequence_simple(self) -> None:
        blueprint = autoconnect(
            UnityBridgeModule.blueprint(
                unity_scene="home_building_1",
                vehicle_height=nav_config["vehicle_height"],
                lock_z=True,
                publish_images=False,
            ),
            create_nav_stack(**{**nav_config, "planner": "simple"}),
            MovementManager.blueprint(),
        ).global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
        run_cross_wall_test(blueprint, label="simple")
