# Copyright 2025-2026 Dimensional Inc.
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

"""P3-3: ``LocalPlanner`` path controller switch and holonomic integration smoke."""

from __future__ import annotations

import math

import numpy as np
import pytest
from pydantic import ValidationError

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.navigation.replanning_a_star.controllers import (
    HolonomicPathController,
    PController,
    make_local_path_controller,
)
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap


def test_make_local_path_controller_default_is_differential() -> None:
    g = GlobalConfig()
    c = make_local_path_controller(g, 0.5, 10.0)
    assert type(c) is PController


def test_make_local_path_controller_holonomic() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    c = make_local_path_controller(g, 0.5, 10.0)
    assert type(c) is HolonomicPathController


def test_holonomic_path_controller_advance_on_straight_line() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    ctrl = make_local_path_controller(g, 0.5, 10.0)
    assert isinstance(ctrl, HolonomicPathController)
    odom = PoseStamped(
        frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1)
    )
    lookahead = np.array([0.4, 0.0], dtype=np.float64)
    out = ctrl.advance(lookahead, odom)
    # Along +x, expect forward body x and finite yaw rate
    assert math.hypot(float(out.linear.x), float(out.linear.y)) > 0.1
    assert abs(float(out.angular.z)) < 2.0


def test_holonomic_path_controller_rotate_uses_pose_when_passed() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    ctrl = HolonomicPathController(
        g,
        speed=0.55,
        control_frequency=10.0,
        k_position_per_s=2.0,
        k_yaw_per_s=1.5,
    )
    odom = PoseStamped(
        frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1)
    )
    yaw_e = 0.3
    t = ctrl.rotate(yaw_e, odom)
    assert math.isfinite(t.angular.z)
    assert math.hypot(float(t.linear.x), float(t.linear.y)) < 1.0


def test_local_planner_wires_holonomic_when_configured() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.2)
    assert isinstance(lp._controller, HolonomicPathController)  # noqa: SLF001


def test_local_planner_control_rate_from_global_config() -> None:
    """P4-1: loop pacing and ``make_local_path_controller`` share ``GlobalConfig`` Hz."""
    g = GlobalConfig(local_planner_control_rate_hz=25.0, local_planner_path_controller="holonomic")
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.2)
    assert lp._control_frequency == 25.0  # noqa: SLF001
    assert lp._controller._control_frequency == 25.0  # noqa: SLF001


def test_local_planner_control_rate_hz_validation() -> None:
    with pytest.raises(ValidationError):
        GlobalConfig(local_planner_control_rate_hz=0.0)
    with pytest.raises(ValidationError):
        GlobalConfig(local_planner_control_rate_hz=300.0)


def test_replay_flag_does_not_change_holonomic_path_controller() -> None:
    """``replay=True`` selects replay sensor backends; the path follower is independent."""
    g = GlobalConfig(
        replay=True,
        local_planner_path_controller="holonomic",
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.2)
    assert isinstance(lp._controller, HolonomicPathController)  # noqa: SLF001
