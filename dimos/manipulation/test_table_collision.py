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

from unittest.mock import MagicMock

import pytest

from dimos.manipulation.manipulation_module import ManipulationModule


def test_table_collision_is_a_conservative_slab() -> None:
    module = object.__new__(ManipulationModule)
    monitor = MagicMock()
    monitor.update_obstacle.return_value = False
    monitor.add_obstacle.return_value = "calibrated-table"
    module._world_monitor = monitor

    assert module.set_table_collision(0.5, 0.0, 0.35, 0.8, 1.0)

    obstacle = monitor.add_obstacle.call_args.args[0]
    assert obstacle.name == "calibrated-table"
    assert obstacle.dimensions == (0.8, 1.0, 0.2)
    assert obstacle.pose.position.z == pytest.approx(0.25)
