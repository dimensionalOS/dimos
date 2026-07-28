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

"""Smoke test for the official RoboPlan Cartesian planning binding surface."""

import roboplan.cartesian_planning as roboplan_cartesian
import roboplan.core as roboplan_core


def test_official_cartesian_planning_bindings_are_available() -> None:
    options = roboplan_cartesian.CartesianPlannerOptions()

    assert roboplan_cartesian.CartesianPathPlanner is not None
    assert roboplan_core.CartesianPath is not None
    assert options.speed_mode == roboplan_cartesian.CartesianSpeedMode.Bounded
