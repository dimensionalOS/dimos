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

pytest.importorskip("gtsam")

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.navigation.nav_stack.tests.conftest import run_cross_wall_test
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_nav_sim import (
    nav_config,
    unitree_g1_nav_sim,
)

pytestmark = [pytest.mark.skipif_in_ci, pytest.mark.skipif_macos]


class TestCrossWallPlanning:
    """E2E: cross-wall routing with FAR planner."""

    def test_cross_wall_sequence(self) -> None:
        blueprint = autoconnect(
            unitree_g1_nav_sim,
            create_nav_stack(**{**nav_config, "planner": "far"}),
        ).global_config(dtop=True)
        run_cross_wall_test(blueprint, label="far")
