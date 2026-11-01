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

"""SimFlowBase integrates what it is told through the plant it was given."""

from __future__ import annotations

import math
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.core.module import Module
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevelConfig
from dimos.robot.diy.alfred.sim_flowbase import SimFlowBase, SimFlowBaseConfig

BASE = "flowbase/base"


@pytest.fixture
def base(monkeypatch: pytest.MonkeyPatch) -> Any:
    """SimFlowBase with the framework's Module.__init__ skipped: no RPC session opens."""
    monkeypatch.setattr(Module, "__init__", lambda self, **kwargs: None)
    module = SimFlowBase()
    module.config = SimFlowBaseConfig(plant_gains=(0.5, 1.0, 2.0))
    module.wheel_odometry = MagicMock()
    return module


def _tell(base: Any, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
    base.write({f"{BASE}/vx": vx, f"{BASE}/vy": vy, f"{BASE}/wz": wz})


def _last(base: Any) -> Odometry:
    return base.wheel_odometry.publish.call_args.args[0]


def test_it_moves_through_the_plant(base: Any) -> None:
    _tell(base, vx=0.2)
    for _ in range(100):
        base.step(0.01)
    odometry = _last(base)
    assert odometry.pose.pose.position.x == pytest.approx(0.1), "told 0.2, plant 0.5x, for 1 s"
    assert odometry.twist.twist.linear.x == pytest.approx(0.1)


def test_a_turn_is_scaled_by_its_own_gain(base: Any) -> None:
    _tell(base, wz=0.25)
    for _ in range(100):
        base.step(0.01)
    assert _last(base).pose.pose.orientation.to_euler().z == pytest.approx(0.5)


def test_it_drives_where_it_is_facing(base: Any) -> None:
    base._pose = (0.0, 0.0, math.pi / 2)
    _tell(base, vy=0.1)
    for _ in range(100):
        base.step(0.01)
    position = _last(base).pose.pose.position
    assert (position.x, position.y) == pytest.approx((-0.1, 0.0), abs=1e-6), "left of facing +y"


def test_a_halt_stops_it(base: Any) -> None:
    _tell(base, vx=0.2)
    base.step(0.01)
    base.halt()
    base.step(1.0)
    assert _last(base).twist.twist.linear.x == 0.0


def test_it_publishes_what_alfred_high_level_does() -> None:
    real, sim = AlfredHighLevelConfig(), SimFlowBaseConfig()
    assert sim.source == real.source
    assert sim.base_range == real.base_range
    assert sim.wheel_odometry_hz == real.wheel_odometry_hz
    assert sim.wheel_odom_frame_id == real.wheel_odom_frame_id
    assert sim.base_frame_id == real.base_frame_id
