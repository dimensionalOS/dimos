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

from __future__ import annotations

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.navigation.nav_stack.tests.conftest import run_cross_wall_test
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_nav_sim import (
    nav_config,
    unitree_g1_nav_sim,
)

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_in_ci, pytest.mark.skipif_macos]


class TestCrossWallPlanningRtab:
    """E2E: cross-wall routing with RtabMap as the SLAM provider.

    Mirrors :mod:`test_cross_wall_planning_simple` but swaps PGO for
    :class:`RtabMap` via ``create_nav_stack(slam_choice="rtab")``. Validates
    that RtabMap can stand in as the SLAM provider in the full nav stack —
    publishing ``corrected_odometry`` and the ``map -> odom`` TF correction
    fast enough to keep the planner/follower stack alive.

    ``unitree_g1_nav_sim`` already bakes in a ``create_nav_stack(**nav_config)``
    call that defaults to PGO, so we explicitly disable PGO at the blueprint
    level to avoid two SLAM providers publishing to ``/corrected_odometry``.
    RtabMap is pure Python so this path doesn't need GTSAM either.
    """

    def test_cross_wall_sequence_rtab(self) -> None:
        blueprint = (
            autoconnect(
                unitree_g1_nav_sim,
                create_nav_stack(
                    **{
                        **nav_config,
                        "planner": "simple",
                        "slam_choice": "rtab",
                    }
                ),
            )
            .disabled_modules(PGO)
            .global_config(dtop=True)
        )
        run_cross_wall_test(blueprint, label="rtab")
