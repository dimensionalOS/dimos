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

import asyncio
from collections.abc import Iterator
from typing import TYPE_CHECKING

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.sim2.world_observations import WorldMirror

if TYPE_CHECKING:
    from dimos.robot.unitree.go2.sim2 import Go2Sim

pytest.importorskip("pxr")


@pytest.fixture
def go2_sim() -> Iterator[Go2Sim]:
    from dimos.robot.unitree.go2.sim2 import Go2Sim

    module = Go2Sim()
    module._world = WorldMirror()
    try:
        yield module
    finally:
        module.stop()


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_a_non_finite_cmd_vel_leaves_the_last_good_command(go2_sim: Go2Sim, bad: float) -> None:
    asyncio.run(go2_sim.handle_cmd_vel(Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))))
    asyncio.run(go2_sim.handle_cmd_vel(Twist(Vector3(bad, 0.0, 0.0), Vector3(0.0, 0.0, bad))))

    assert np.array_equal(go2_sim._desired, [0.5, 0.0, 0.0])
