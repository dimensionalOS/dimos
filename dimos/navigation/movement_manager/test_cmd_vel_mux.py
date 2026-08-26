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

"""The python mux's decisions, and the parity that keeps it swappable for the rust.

The staleness and preemption predicates are tested by handing them a time
rather than by sleeping: the rust twin injects `now` for the same reason, and a
deadman test that waits out its own timeout is a slow test that still only
proves the clock works.
"""

from __future__ import annotations

from collections.abc import Generator
from dataclasses import dataclass, field

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.movement_manager.cmd_vel_mux import CmdVelMux, CmdVelMuxConfig
from dimos.navigation.movement_manager.cmd_vel_mux_native import (
    CmdVelMuxNative,
    CmdVelMuxNativeConfig,
)

COOLDOWN = 1.0
STALE = 0.5


@dataclass
class Captured:
    cmd_vel: list = field(default_factory=list)
    stop_movement: list = field(default_factory=list)


@pytest.fixture()
def modules() -> Generator[list, None, None]:
    """Modules built by a test, stopped after it."""
    built: list = []
    yield built
    for module in built:
        module.stop()


@pytest.fixture()
def mux_and_captured(modules) -> tuple[CmdVelMux, Captured]:
    module = CmdVelMux(tele_cooldown_sec=COOLDOWN, nav_stale_s=STALE)
    modules.append(module)
    captured = Captured()
    module.cmd_vel.subscribe(captured.cmd_vel.append)
    module.stop_movement.subscribe(captured.stop_movement.append)
    return module, captured


def _twist(lx=0.0, az=0.0):
    return Twist(linear=Vector3(lx, 0, 0), angular=Vector3(0, 0, az))


def test_nav_is_forwarded_raw(mux_and_captured):
    """Nav is the planned command — scaling it would price the plan twice."""
    mux, captured = mux_and_captured
    mux._on_nav(_twist(lx=0.4, az=-0.2))
    assert captured.cmd_vel == [_twist(lx=0.4, az=-0.2)]
    assert captured.stop_movement == []


def test_teleop_scales_and_stops_the_follower(mux_and_captured):
    mux, captured = mux_and_captured
    mux.config.tele_scale_linear = [0.5, 2.0, 0.0]
    mux.config.tele_scale_angular = [1.0, 1.0, 0.25]
    mux._on_teleop(_twist(lx=1.0, az=1.0))

    assert captured.cmd_vel[0].linear.x == 0.5
    assert captured.cmd_vel[0].angular.z == 0.25
    assert [m.data for m in captured.stop_movement] == [True]


def test_teleop_preempts_nav_until_the_cooldown_expires(mux_and_captured):
    mux, captured = mux_and_captured
    mux._on_teleop(_twist(lx=0.1))
    captured.cmd_vel.clear()

    mux._last_teleop = 100.0
    mux._last_nav = None
    assert mux._teleop_holds(100.9)
    assert not mux._teleop_holds(101.1)


def test_a_dropped_nav_still_counts_as_an_arrival(mux_and_captured):
    """The deadman guards the producer's liveness, not what got forwarded.

    Otherwise it trips the instant teleop releases, even though nav was
    streaming the whole time.
    """
    mux, captured = mux_and_captured
    mux._on_teleop(_twist(lx=0.1))
    captured.cmd_vel.clear()
    mux._on_nav(_twist(lx=0.4))

    assert captured.cmd_vel == []  # dropped
    assert mux._last_nav is not None  # but recorded
    assert not mux._nav_is_stale(mux._last_nav + 0.4)


def test_the_watchdog_is_disarmed_until_the_first_nav(mux_and_captured):
    """Nothing has ever driven, so nothing needs stopping."""
    mux, _ = mux_and_captured
    assert not mux._nav_is_stale(1e6)


def test_the_watchdog_fires_once_nav_goes_quiet(mux_and_captured):
    mux, _ = mux_and_captured
    mux._last_nav = 100.0
    assert not mux._nav_is_stale(100.4)
    assert mux._nav_is_stale(100.6)
    # continuous, not edge-triggered: a single zero can be lost
    assert mux._nav_is_stale(200.0)


def test_the_watchdog_is_suppressed_while_teleop_drives(mux_and_captured):
    mux, _ = mux_and_captured
    mux._last_nav = 100.0
    mux._last_teleop = 100.6
    assert not mux._nav_is_stale(101.0)
    # once the teleop cooldown lapses the deadman takes over again
    assert mux._nav_is_stale(101.7)


def test_the_two_muxes_are_swappable(modules):
    """Same ports and same config keys, so a stack swaps one for the other.

    The rust is the one that goes on the robot; this pins the python twin to it
    so a blueprint does not have to know which it got.
    """
    py, native = CmdVelMux(), CmdVelMuxNative()
    modules += [py, native]
    assert set(py.inputs) == set(native.inputs)
    assert set(py.outputs) == set(native.outputs)
    # every key the rust reads is one the python spells the same way
    assert set(CmdVelMuxConfig.model_fields) >= set(CmdVelMuxNativeConfig().to_config_dict())
