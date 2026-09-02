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

"""Microduck policy catalogue, ``PolicyScheduler`` state machine (sim-time
ticks, fake wall clock), ``PolicyBank`` / asset helpers, and an opt-in
headless MuJoCo run of every policy.

The MuJoCo-backed tests carry the ``mujoco`` marker, which the repo's pytest
``addopts`` deselect by default, so they need ``-m mujoco``::

    pytest dimos/robot/microduck/test_policies.py -m mujoco

The per-policy headless matrix (``test_slow_policy_runs_in_headless_sim``,
a few seconds per variant on Apple Silicon) is additionally gated on an env
flag and needs the cached ONNX/MJCF assets under ``~/.cache/dimos/microduck``::

    MICRODUCK_SLOW_TESTS=1 pytest dimos/robot/microduck/test_policies.py \\
        -m mujoco -k headless -s            # default variant, prints outcomes
    MICRODUCK_SLOW_VARIANTS=rollers MICRODUCK_SLOW_TESTS=1 pytest ... -m mujoco -k headless
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
import json
import math
import os
from pathlib import Path
import threading
from typing import Any
import urllib.error

import numpy as np
import pytest

from dimos.robot.microduck import assets_fetch
from dimos.robot.microduck.policies import (
    ACTIVE_BRAKING,
    ACTIVE_STANDING_UP,
    ASSET_MISSING_REASON,
    BASE_POLICIES,
    BRAKE_DURATION_S,
    FALL_GRAVITY_Z,
    GROUND_PICK_END_PHASE,
    GROUND_PICK_PERIOD_S,
    KICK_BALL_OFFSETS,
    KICK_DURATION_S,
    LAST_ERROR_TTL_S,
    ONESHOT_POLICIES,
    POLICY_NAMES,
    POLICY_SPECS,
    ROLLER_BRAKE_THROTTLE,
    ROLLER_THROTTLE_RANGE,
    ROULADE_DURATION_S,
    ROULADE_GRACE_S,
    STAND_UP_DURATION_S,
    PolicyBank,
    PolicyKind,
    PolicyName,
    PolicyScheduler,
    policy_availability,
)

DT = 0.02  # the policies' control period
ROOM_SCENE = Path(__file__).with_name("assets") / "room_scene.xml"
DEFAULT_POLICIES = (
    "walk",
    "stand",
    "sitstand",
    "kick_left",
    "kick_right",
    "roulade",
    "ground_pick",
)
ROLLERS_POLICIES = (
    "walk",
    "stand",
    "roller",
    "roller_crouch",
    "kick_left",
    "kick_right",
    "ground_pick",
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class Harness:
    """A scheduler plus the recorded ball spawns and a helper to run sim ticks."""

    def __init__(self, variant: str = "default", missing: Iterable[str] = ()) -> None:
        self.clock = FakeClock()
        self.spawns: list[tuple[float, float]] = []
        self.sched = PolicyScheduler(
            policy_availability(variant, missing),
            variant,
            spawn_ball=lambda dx, dy: self.spawns.append((dx, dy)),
            clock=self.clock,
        )

    def run(self, seconds: float) -> list[tuple[str, np.ndarray]]:
        n = round(seconds / DT)
        return [self.sched.tick(DT) for _ in range(n)]

    def names(self, seconds: float) -> list[str]:
        return [name for name, _ in self.run(seconds)]


@pytest.fixture
def h() -> Harness:
    return Harness()


@pytest.fixture
def rollers() -> Harness:
    return Harness("rollers")


# --------------------------------------------------------------- catalogue


def test_policy_specs_display_order_kinds_and_files() -> None:
    assert POLICY_NAMES == (
        "walk",
        "stand",
        "roller",
        "roller_crouch",
        "sitstand",
        "kick_left",
        "kick_right",
        "roulade",
        "ground_pick",
    )
    assert [str(n) for n in POLICY_SPECS] == list(POLICY_NAMES)
    kinds = {str(n): s.kind for n, s in POLICY_SPECS.items()}
    assert BASE_POLICIES == {"walk", "stand", "roller", "roller_crouch"}
    assert kinds["sitstand"] is PolicyKind.POSTURE
    assert ONESHOT_POLICIES == {"kick_left", "kick_right", "roulade", "ground_pick"}
    files = {str(n): s.onnx for n, s in POLICY_SPECS.items()}
    assert files == {
        "walk": "alpha_walking.onnx",
        "stand": "alpha_stand.onnx",
        "roller": "roller.onnx",
        "roller_crouch": "roller_crouch.onnx",
        "sitstand": "alpha_sitstand.onnx",
        "kick_left": "ball_kick_left.onnx",
        "kick_right": "ball_kick_right.onnx",
        "roulade": "roulade.onnx",
        "ground_pick": "alpha_ground_pick.onnx",
    }
    assert assets_fetch.POLICY_FILES == {PolicyName(n): f for n, f in files.items()}
    for spec in POLICY_SPECS.values():
        assert (spec.duration is not None) == (spec.kind is PolicyKind.ONESHOT)
    assert str(PolicyName.WALK) == "walk" and PolicyName("kick_left") is PolicyName.KICK_LEFT
    assert json.dumps({"p": PolicyName.ROULADE}) == '{"p": "roulade"}'
    assert FALL_GRAVITY_Z == -0.55  # the sim module's existing threshold


def test_policy_availability_per_variant() -> None:
    default = policy_availability("default")
    assert [n for n, r in default.items() if r is None] == list(DEFAULT_POLICIES)
    assert "rollers" in default["roller"] and "MICRODUCK_VARIANT=rollers" in default["roller"]
    rollers = policy_availability("rollers")
    assert [n for n, r in rollers.items() if r is None] == list(ROLLERS_POLICIES)
    assert rollers["sitstand"] == "not supported on the rollers variant"
    assert rollers["roulade"] == "not supported on the rollers variant"
    with pytest.raises(ValueError):
        policy_availability("wheels")
    missing = policy_availability("default", missing=("roulade", PolicyName.STAND))
    assert missing["roulade"] == ASSET_MISSING_REASON
    assert missing["stand"] == ASSET_MISSING_REASON
    assert missing["walk"] is None


# --------------------------------------------------------------- scheduler


def test_initial_state_and_snapshot_shape(h: Harness) -> None:
    snap = h.sched.snapshot()
    assert list(snap) == [
        "variant",
        "active",
        "base",
        "seated",
        "fallen",
        "locked",
        "oneshot",
        "policies",
        "last_error",
        "t",
    ]
    assert snap["variant"] == "default"
    assert snap["active"] == "walk" and type(snap["active"]) is str
    assert snap["base"] == "walk"
    assert snap["seated"] is False and snap["fallen"] is False and snap["locked"] is False
    assert snap["oneshot"] is None and snap["last_error"] is None
    assert isinstance(snap["t"], float)
    assert [p["name"] for p in snap["policies"]] == list(POLICY_NAMES)
    assert all(set(p) == {"name", "kind", "available", "reason"} for p in snap["policies"])
    by_name = {p["name"]: p for p in snap["policies"]}
    assert by_name["walk"] == {"name": "walk", "kind": "base", "available": True, "reason": None}
    assert by_name["sitstand"]["kind"] == "posture" and by_name["kick_left"]["kind"] == "oneshot"
    assert by_name["roller"]["available"] is False and by_name["roller"]["reason"]
    json.dumps(snap)  # plain JSON, no enum members
    name, cmd = h.sched.tick(DT)
    assert name == "walk" and type(name) is str
    assert cmd.shape == (13,) and cmd.dtype == np.float32 and not cmd.any()


def test_request_is_applied_on_tick_and_latest_pending_wins(h: Harness) -> None:
    assert h.sched.request("stand", "start") == (True, "")
    assert h.sched.active == "walk"  # nothing changes off the sim thread
    assert h.sched.request("kick_left", "start") == (True, "")  # replaces the pending stand
    name, _ = h.sched.tick(DT)
    assert name == "kick_left" and h.sched.base == "walk"


def test_walk_stand_switch_is_immediate(h: Harness) -> None:
    h.sched.request("stand", "start")
    assert h.names(DT) == ["stand"]
    assert h.sched.base == "stand" and not h.sched.locked
    h.sched.request("stand", "stop")  # stop on a base = back to walk
    assert h.names(DT) == ["walk"]
    h.sched.request("stand", "toggle")
    assert h.names(DT) == ["stand"]
    h.sched.request("walk", "toggle")
    assert h.names(DT) == ["walk"]
    assert h.sched.request("walk", "start") == (True, "")  # no-op, still accepted
    assert h.names(DT) == ["walk"]


def test_walk_follows_clipped_twist_and_stand_ignores_it(h: Harness) -> None:
    h.sched.set_twist(1.0, -1.0, 3.0)
    _, cmd = h.sched.tick(DT)
    assert cmd[:3].tolist() == pytest.approx([0.3, -0.2, 1.5])
    assert not cmd[3:].any()
    h.sched.set_twist(-1.0, 0.1, -0.5)
    _, cmd = h.sched.tick(DT)
    assert cmd[:3].tolist() == pytest.approx([-0.25, 0.1, -0.5])
    h.sched.request("stand", "start")
    name, cmd = h.sched.tick(DT)
    assert name == "stand" and not cmd.any()


def test_roller_to_walk_brakes_with_zero_throttle(rollers: Harness) -> None:
    s = rollers.sched
    s.request("roller", "start")
    s.set_twist(0.48, 0.0, 0.3)
    name, cmd = s.tick(DT)
    assert name == "roller" and cmd[0] == pytest.approx(0.48) and cmd[2] == pytest.approx(0.3)
    assert cmd[1] == 0.0
    s.set_twist(2.0, 0.5, -3.0)
    _, cmd = s.tick(DT)
    assert cmd[0] == pytest.approx(ROLLER_THROTTLE_RANGE[1]) and cmd[2] == pytest.approx(-1.0)

    assert s.request("walk", "start") == (True, "")
    name, cmd = s.tick(DT)
    # Braking keeps the roller policy at zero throttle (never reverse thrust,
    # which would drive a resting duck backwards) and ignores the joystick.
    assert name == "roller" and cmd[0] == ROLLER_BRAKE_THROTTLE == 0.0
    assert not cmd.any()
    assert s.active == ACTIVE_BRAKING and s.base == "roller" and s.locked
    assert s.snapshot()["active"] == "braking"
    assert s.request("stand", "start") == (False, "locked: braking")
    names = rollers.names(BRAKE_DURATION_S + 5 * DT)
    braking = 1 + names.count("roller")
    assert braking * DT >= BRAKE_DURATION_S - 1e-9
    assert names[-1] == "walk" and s.base == "walk" and not s.locked


def test_roller_crouch_brakes_on_its_own_loop_when_roller_is_missing() -> None:
    # Regression: braking used to hand the bank ``roller`` unconditionally,
    # which raised KeyError on the sim thread when only roller_crouch loaded.
    crouch_only = Harness("rollers", missing=("roller",))
    s = crouch_only.sched
    assert s.request("roller", "start") == (False, f"roller unavailable: {ASSET_MISSING_REASON}")
    assert s.request("roller_crouch", "start") == (True, "")
    crouch_only.run(0.5)
    assert s.request("walk", "start") == (True, "")
    name, cmd = s.tick(DT)
    assert name == "roller_crouch" and s.active == ACTIVE_BRAKING
    phi = 0.5 / 3.0  # the crouch phase keeps advancing while braking
    assert cmd[0] == pytest.approx(math.cos(2 * math.pi * phi), abs=1e-5)
    assert cmd[1] == pytest.approx(math.sin(2 * math.pi * phi), abs=1e-5)
    _, cmd2 = s.tick(DT)
    assert not np.allclose(cmd2[:2], cmd[:2])
    names = crouch_only.names(BRAKE_DURATION_S)
    assert "roller" not in names and names[-1] == "walk"
    assert s.snapshot()["last_error"] is None


def test_braking_keeps_the_roller_family_base_being_left(rollers: Harness) -> None:
    s = rollers.sched
    s.request("roller_crouch", "start")
    rollers.run(0.3)
    s.request("walk", "start")
    assert rollers.names(BRAKE_DURATION_S) == ["roller_crouch"] * round(BRAKE_DURATION_S / DT)
    assert rollers.names(DT) == ["walk"]


def test_roller_family_switches_do_not_brake_but_leaving_does(rollers: Harness) -> None:
    s = rollers.sched
    s.request("roller", "start")
    s.tick(DT)
    s.request("roller_crouch", "start")
    name, cmd = s.tick(DT)
    assert name == "roller_crouch" and s.base == "roller_crouch"
    assert cmd[0] == pytest.approx(1.0) and cmd[1] == pytest.approx(0.0)  # phase 0
    _, cmd = s.tick(DT)
    phi = DT / 3.0
    assert cmd[0] == pytest.approx(math.cos(2 * math.pi * phi), abs=1e-5)
    assert cmd[1] == pytest.approx(math.sin(2 * math.pi * phi), abs=1e-5)
    s.request("roller", "toggle")
    assert rollers.names(DT) == ["roller"]
    s.request("stand", "start")
    assert rollers.names(DT) == ["roller"] and s.active == ACTIVE_BRAKING
    rollers.run(BRAKE_DURATION_S)
    assert rollers.names(DT) == ["stand"]


def test_roller_stop_means_walk_via_braking(rollers: Harness) -> None:
    s = rollers.sched
    s.request("roller_crouch", "start")
    s.tick(DT)
    s.request("roller_crouch", "stop")
    s.tick(DT)
    assert s.active == ACTIVE_BRAKING
    rollers.run(BRAKE_DURATION_S)
    assert rollers.names(DT) == ["walk"]


def test_sitstand_lifecycle(h: Harness) -> None:
    s = h.sched
    assert s.request("sitstand", "stop") == (False, "not seated")
    assert s.snapshot()["last_error"] == "not seated"
    assert s.request("sitstand", "start") == (True, "")
    assert s.snapshot()["last_error"] is None  # an accepted request clears it
    h.sched.set_twist(0.3, 0.0, 0.0)
    name, cmd = s.tick(DT)
    assert name == "sitstand" and cmd[0] == 1.0 and not cmd[1:].any()
    snap = s.snapshot()
    assert snap["seated"] is True and snap["locked"] is True and snap["active"] == "sitstand"
    assert snap["oneshot"] is None
    assert s.request("stand", "start") == (False, "locked: seated")
    assert s.request("kick_left", "start") == (False, "locked: seated")
    assert s.request("sitstand", "start") == (True, "already seated")
    assert h.names(0.5) == ["sitstand"] * 25

    assert s.request("sitstand", "stop") == (True, "")
    name, cmd = s.tick(DT)
    assert name == "sitstand" and not cmd.any()  # flag 0 = stand up
    snap = s.snapshot()
    assert snap["active"] == ACTIVE_STANDING_UP and snap["seated"] is False
    assert snap["locked"] is True
    assert s.request("walk", "start") == (False, "locked: standing up")
    assert s.request("sitstand", "stop") == (True, "already standing up")
    names = h.names(STAND_UP_DURATION_S)
    assert names[:-1] == ["sitstand"] * (round(STAND_UP_DURATION_S / DT) - 1)
    assert names[-1] == "walk"
    assert not s.locked and s.base == "walk"
    _, cmd = s.tick(DT)
    assert cmd[0] == pytest.approx(0.3)  # twist honoured again


def test_sitstand_toggle_and_policy_less_stop(h: Harness) -> None:
    s = h.sched
    s.request("sitstand", "toggle")
    s.tick(DT)
    assert s.seated
    assert s.request(None, "stop") == (True, "")
    s.tick(DT)
    assert s.active == ACTIVE_STANDING_UP
    h.run(STAND_UP_DURATION_S)
    assert s.active == "walk"
    s.request("sitstand", "toggle")
    s.tick(DT)
    assert s.seated
    s.request("sitstand", "toggle")
    s.tick(DT)
    assert s.active == ACTIVE_STANDING_UP
    assert s.request(None, "start") == (False, "start needs a policy name")
    h.run(STAND_UP_DURATION_S)
    assert s.request(None, "stop") == (True, "")  # idle: harmless
    assert h.names(DT) == ["walk"]


@pytest.mark.parametrize(
    ("name", "expected_ticks"),
    [
        ("kick_left", round(KICK_DURATION_S / DT)),
        ("kick_right", round(KICK_DURATION_S / DT)),
        ("roulade", round(ROULADE_DURATION_S / DT)),
        ("ground_pick", round(GROUND_PICK_END_PHASE * GROUND_PICK_PERIOD_S / DT)),
    ],
)
def test_oneshot_runs_its_window_then_hands_back(
    h: Harness, name: str, expected_ticks: int
) -> None:
    s = h.sched
    s.request("stand", "start")
    s.tick(DT)
    assert s.request(name, "start") == (True, "")
    progress: list[float] = []
    ran = 0
    for _ in range(expected_ticks + 10):
        snap = s.snapshot()
        if snap["oneshot"] is not None:
            assert snap["oneshot"]["name"] == name
            progress.append(snap["oneshot"]["progress"])
        policy, _ = s.tick(DT)
        if policy == name:
            ran += 1
            assert s.locked
    assert ran == expected_ticks
    assert progress[0] == pytest.approx(DT / (expected_ticks * DT), abs=0.01)
    assert progress[-1] == 1.0
    assert progress == sorted(progress)
    snap = s.snapshot()
    assert snap["oneshot"] is None and snap["active"] == "stand" and snap["locked"] is False
    assert h.names(DT) == ["stand"]


def test_ground_pick_command_is_phase_encoded(h: Harness) -> None:
    s = h.sched
    s.request("ground_pick", "start")
    cmds = [cmd for _, cmd in h.run(5 * DT)]
    for k, cmd in enumerate(cmds):
        phi = k * DT / GROUND_PICK_PERIOD_S
        assert cmd[0] == pytest.approx(math.cos(2 * math.pi * phi), abs=1e-5)
        assert cmd[1] == pytest.approx(math.sin(2 * math.pi * phi), abs=1e-5)
        assert not cmd[2:].any()


def test_kick_command_is_zero_and_ignores_twist(h: Harness) -> None:
    h.sched.set_twist(0.3, 0.1, 0.5)
    h.sched.request("kick_right", "start")
    name, cmd = h.sched.tick(DT)
    assert name == "kick_right" and not cmd.any()


@pytest.mark.parametrize("action", ["stop", "toggle"])
def test_oneshot_abort(h: Harness, action: str) -> None:
    s = h.sched
    s.request("roulade", "start")
    h.run(0.5)
    assert s.active == "roulade"
    assert s.request("roulade", action) == (True, "")
    name, _ = s.tick(DT)
    assert name == "walk" and s.active == "walk" and not s.locked
    assert s.snapshot()["oneshot"] is None
    # policy-less stop aborts too
    s.request("kick_left", "start")
    s.tick(DT)
    assert s.request(None, "stop") == (True, "")
    assert h.names(DT) == ["walk"]
    assert s.request("kick_left", "stop") == (False, "kick_left is not running")
    assert s.request("kick_left", "start") == (True, "")
    s.tick(DT)
    assert s.request("kick_left", "start") == (True, "already running")
    assert s.request("roulade", "start") == (False, "locked: kick_left running")


@pytest.mark.parametrize(
    ("name", "action"),
    [("kick_left", "toggle"), ("kick_left", "stop"), ("roulade", "stop"), ("roulade", "toggle")],
)
def test_abort_landing_on_the_final_tick_does_not_restart_or_error(
    h: Harness, name: str, action: str
) -> None:
    """A stop/toggle accepted while the oneshot runs must keep meaning "abort"
    even if the window expires before the sim thread consumes it."""
    s = h.sched
    s.request(name, "start")
    window = POLICY_SPECS[PolicyName(name)].duration or 0.0
    assert set(h.names(window)) == {name}  # the whole window, expiry not yet ticked
    snap = s.snapshot()
    assert snap["locked"] and snap["oneshot"] == {"name": name, "progress": 1.0}
    assert s.request(name, action) == (True, "")
    policy, _ = s.tick(DT)
    assert policy == "walk" and s.active == "walk" and not s.locked
    assert s.snapshot()["last_error"] is None
    assert len(h.spawns) == (1 if name == "kick_left" else 0)
    assert h.names(0.5) == ["walk"] * 25  # nothing restarted later either
    assert len(h.spawns) == (1 if name == "kick_left" else 0)


def test_stop_right_behind_a_pending_start_cancels_it(h: Harness) -> None:
    s = h.sched
    assert s.request("kick_left", "start") == (True, "")
    assert s.request("kick_left", "stop") == (True, "")  # cancels the unconsumed start
    assert h.names(DT) == ["walk"] and h.spawns == [] and s.snapshot()["last_error"] is None
    assert s.request("kick_right", "start") == (True, "")
    assert s.request("kick_right", "toggle") == (True, "")  # toggle behind a start = cancel
    assert h.names(DT) == ["walk"] and h.spawns == []
    assert s.request("sitstand", "start") == (True, "")
    assert s.request("sitstand", "stop") == (True, "")
    assert h.names(DT) == ["walk"] and not s.seated
    assert s.request("sitstand", "toggle") == (True, "")
    assert s.request("sitstand", "toggle") == (True, "")
    assert h.names(DT) == ["walk"] and not s.seated
    # A policy-less stop behind a pending start cancels it too (no "locked:"
    # error later) - and, with nothing to abort, keeps a parked base select.
    assert s.request("ground_pick", "start") == (True, "")
    assert s.request(None, "stop") == (True, "")
    assert h.names(DT) == ["walk"] and s.snapshot()["last_error"] is None
    assert s.request("stand", "start") == (True, "")
    assert s.request(None, "stop") == (True, "")
    assert h.names(DT) == ["stand"]


def test_start_right_behind_a_pending_abort_cancels_it(h: Harness) -> None:
    """The reverse of the test above: the newest intent wins in both
    directions, so an unconsumed abort/stand-up is dropped by a start/sit."""
    s = h.sched
    s.request("kick_left", "start")
    s.tick(DT)
    assert s.request("kick_left", "stop") == (True, "")  # abort pending
    assert s.request("kick_left", "start") == (True, "already running")  # ... cancelled
    assert h.names(DT) == ["kick_left"] and s.snapshot()["last_error"] is None
    assert s.request("kick_left", "toggle") == (True, "")
    assert s.request("kick_left", "toggle") == (True, "")  # toggle behind an abort = cancel
    assert h.names(DT) == ["kick_left"]
    assert s.request(None, "stop") == (True, "")
    assert s.request("kick_left", "start") == (True, "already running")
    assert h.names(DT) == ["kick_left"]
    assert s.request("kick_left", "stop") == (True, "")
    assert s.request("kick_right", "start") == (False, "locked: kick_left running")  # not a cancel
    assert h.names(DT) == ["walk"]
    assert len(h.spawns) == 1  # the kick ran exactly once

    s.request("sitstand", "start")
    s.tick(DT)
    assert s.seated
    assert s.request("sitstand", "stop") == (True, "")  # stand-up pending
    assert s.request("sitstand", "start") == (True, "already seated")  # ... cancelled
    assert h.names(DT) == ["sitstand"] and s.seated
    assert s.request(None, "stop") == (True, "")
    assert s.request("sitstand", "toggle") == (True, "")  # toggle behind a stand-up = cancel
    assert h.names(DT) == ["sitstand"] and s.seated
    assert s.request("sitstand", "toggle") == (True, "")
    assert s.request("sitstand", "toggle") == (True, "")
    assert h.names(DT) == ["sitstand"] and s.seated
    assert s.request("sitstand", "stop") == (True, "")
    assert s.request("sitstand", "stop") == (True, "")  # a repeat is still a stand-up
    s.tick(DT)
    assert s.active == ACTIVE_STANDING_UP


def test_stale_stand_up_after_a_fall_is_dropped_silently(h: Harness) -> None:
    s = h.sched
    s.request("sitstand", "start")
    s.tick(DT)
    assert s.request("sitstand", "stop") == (True, "")
    s.notify_fall(True)  # the sim thread beats the request; the duck is no longer seated
    assert h.names(DT) == ["walk"]
    assert s.active == "walk" and s.snapshot()["last_error"] is None
    # ... whereas a pending *start* that a fall overtook is reported.
    s.notify_fall(False)
    assert s.request("kick_left", "start") == (True, "")
    s.notify_fall(True)
    assert h.names(DT) == ["walk"] and h.spawns == []
    assert s.snapshot()["last_error"] == "locked: fallen"


def test_spawn_ball_failure_does_not_break_the_tick() -> None:
    calls: list[tuple[float, float]] = []

    def broken(dx: float, dy: float) -> None:
        calls.append((dx, dy))
        raise RuntimeError("no ball body in this scene")

    s = PolicyScheduler(policy_availability("default"), "default", spawn_ball=broken)
    s.request("kick_right", "start")
    name, cmd = s.tick(DT)
    assert name == "kick_right" and not cmd.any() and s.locked
    assert calls == [(0.09, -0.042)]
    assert s.snapshot()["oneshot"] == {"name": "kick_right", "progress": pytest.approx(0.04)}


def test_fall_aborts_kick_and_locks_until_recovered(h: Harness) -> None:
    s = h.sched
    s.set_twist(0.3, 0.0, 0.0)
    s.request("kick_left", "start")
    s.tick(DT)
    s.notify_fall(True)
    snap = s.snapshot()
    assert snap["fallen"] is True and snap["locked"] is True
    assert snap["active"] == "walk" and snap["oneshot"] is None
    assert s.request("kick_right", "start") == (False, "locked: fallen")
    assert s.request("stand", "start") == (False, "locked: fallen")
    name, cmd = s.tick(DT)
    assert name == "walk" and not cmd.any()  # twist ignored while fallen
    s.notify_fall(False)
    assert not s.locked
    _, cmd = s.tick(DT)
    assert cmd[0] == pytest.approx(0.3)


def test_fall_does_not_abort_roulade_in_window_and_grace(h: Harness) -> None:
    s = h.sched
    assert not s.suspend_fall_detector
    s.request("roulade", "start")
    s.tick(DT)
    assert s.suspend_fall_detector
    h.run(1.0)
    s.notify_fall(True)  # mid-roll: the duck is upside down by design
    assert s.active == "roulade" and not s.fallen
    remaining = round(ROULADE_DURATION_S / DT) - 1 - round(1.0 / DT)
    names = h.names(remaining * DT)
    assert set(names) == {"roulade"}
    grace_ticks = 0
    while s.suspend_fall_detector:  # window over, then the grace period
        s.notify_fall(True)
        assert not s.fallen
        name, _ = s.tick(DT)
        if name == "walk":
            grace_ticks += 1
        assert grace_ticks < 1000
    assert grace_ticks * DT == pytest.approx(ROULADE_GRACE_S)
    s.notify_fall(True)
    assert s.fallen and s.locked


def test_fall_while_seated_standing_up_or_braking(h: Harness, rollers: Harness) -> None:
    s = h.sched
    s.request("sitstand", "start")
    s.tick(DT)
    s.notify_fall(True)
    assert s.active == "walk" and not s.seated and s.fallen
    s.notify_fall(False)
    s.request("sitstand", "start")
    s.tick(DT)
    s.request("sitstand", "stop")
    s.tick(DT)
    assert s.active == ACTIVE_STANDING_UP
    s.notify_fall(True)
    assert s.active == "walk"

    r = rollers.sched
    r.request("roller", "start")
    r.tick(DT)
    r.request("walk", "start")
    r.tick(DT)
    assert r.active == ACTIVE_BRAKING
    r.notify_fall(True)
    assert r.active == "walk" and r.base == "walk" and r.fallen


def test_rejections_set_last_error_which_expires(h: Harness) -> None:
    s = h.sched
    s.request("kick_left", "start")
    s.tick(DT)
    assert s.request("stand", "start") == (False, "locked: kick_left running")
    assert s.snapshot()["last_error"] == "locked: kick_left running"
    h.clock.advance(LAST_ERROR_TTL_S - 0.5)
    assert s.snapshot()["last_error"] == "locked: kick_left running"
    h.clock.advance(1.0)
    assert s.snapshot()["last_error"] is None
    # A request that was fine when queued but stale by tick time is reported too.
    h.run(1.0)  # kick over
    assert s.request("stand", "start") == (True, "")
    s.notify_fall(True)  # sim thread beats the pending request
    assert h.names(DT) == ["walk"]
    assert s.snapshot()["last_error"] == "locked: fallen"


def test_unavailable_unknown_and_bad_requests(h: Harness, rollers: Harness) -> None:
    ok, reason = h.sched.request("roller", "start")
    assert not ok and reason.startswith("roller unavailable: requires the rollers variant")
    assert h.sched.snapshot()["last_error"] == reason
    ok, reason = rollers.sched.request("sitstand", "toggle")
    assert not ok and reason == "sitstand unavailable: not supported on the rollers variant"
    ok, reason = rollers.sched.request("roulade", "start")
    assert not ok and "not supported on the rollers variant" in reason
    assert h.sched.request("moonwalk", "start") == (False, "unknown policy 'moonwalk'")
    assert h.sched.request("walk", "pause") == (False, "unknown action 'pause'")
    assert h.names(DT) == ["walk"]  # nothing leaked into the pending slot


def test_kick_spawns_ball_with_signed_offsets(h: Harness) -> None:
    s = h.sched
    s.request("kick_left", "start")
    assert h.spawns == []  # spawned on the sim thread, not at request time
    s.tick(DT)
    assert h.spawns == [(0.09, 0.042)]
    s.request("kick_left", "stop")
    s.tick(DT)
    s.request("kick_right", "toggle")
    s.tick(DT)
    assert h.spawns == [(0.09, 0.042), (0.09, -0.042)]
    assert KICK_BALL_OFFSETS == {"kick_left": (0.09, 0.042), "kick_right": (0.09, -0.042)}
    s.request("kick_right", "stop")
    s.tick(DT)
    s.request("roulade", "start")
    s.tick(DT)
    assert len(h.spawns) == 2  # only kicks spawn


def test_missing_assets_and_base_fallback() -> None:
    hh = Harness(missing=("walk", "kick_left"))
    s = hh.sched
    assert s.base == "stand" and hh.names(DT) == ["stand"]
    by_name = {p["name"]: p for p in s.snapshot()["policies"]}
    assert by_name["walk"] == {
        "name": "walk",
        "kind": "base",
        "available": False,
        "reason": ASSET_MISSING_REASON,
    }
    assert s.request("kick_left", "start") == (False, "kick_left unavailable: asset missing")
    assert s.request("stand", "stop") == (False, "walk unavailable: asset missing")
    with pytest.raises(ValueError):
        PolicyScheduler(policy_availability("default", missing=BASE_POLICIES), "default")
    with pytest.raises(ValueError):
        PolicyScheduler(policy_availability("default"), "wheels")
    # Policies the bank never reported are treated as unavailable, not crashed on.
    s2 = PolicyScheduler({"walk": None}, "default")
    assert s2.request("stand", "start") == (False, "stand unavailable: not loaded")


def test_requests_from_other_threads_are_serialised(h: Harness) -> None:
    s = h.sched
    stop = threading.Event()
    errors: list[BaseException] = []

    def hammer(seed: int) -> None:
        rng = np.random.default_rng(seed)
        try:
            while not stop.is_set():
                s.request(
                    str(rng.choice(POLICY_NAMES)), str(rng.choice(["start", "stop", "toggle"]))
                )
                s.set_twist(float(rng.normal()), 0.0, 0.0)
                s.snapshot()
        except BaseException as exc:  # pragma: no cover - failure path
            errors.append(exc)

    threads = [threading.Thread(target=hammer, args=(i,)) for i in range(4)]
    for t in threads:
        t.start()
    for _ in range(2000):
        name, cmd = s.tick(DT)
        assert name in POLICY_NAMES and cmd.shape == (13,)
        if s.fallen:
            s.notify_fall(False)
    stop.set()
    for t in threads:
        t.join(timeout=5)
    assert not errors
    json.dumps(s.snapshot())


# ------------------------------------------------------------------ assets


def test_assets_variant_helpers(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DIMOS_MICRODUCK_ASSETS", str(tmp_path))
    assert assets_fetch.assets_root() == tmp_path
    assert assets_fetch.ROBOT_MJCF_BY_VARIANT == {
        "default": "robot_allcollisions.xml",
        "rollers": "robot_allcollisions_rollers.xml",
    }
    assert (
        assets_fetch.variant_mjcf_path("rollers")
        == tmp_path / "robot" / "robot_allcollisions_rollers.xml"
    )
    assert assets_fetch.robot_mjcf_path() == tmp_path / "robot" / "robot_walk.xml"
    assert assets_fetch.walking_policy_path() == tmp_path / "policies" / "alpha_walking.onnx"
    assert assets_fetch.policy_path("roulade") == tmp_path / "policies" / "roulade.onnx"
    with pytest.raises(ValueError):
        assets_fetch.ensure_assets("wheels")
    assets = assets_fetch.MicroduckAssets(tmp_path / "robot", tmp_path / "policies", ())
    assert assets.robot_mjcf() == tmp_path / "robot" / "robot_allcollisions.xml"
    assert assets.policy_path(PolicyName.KICK_LEFT) == tmp_path / "policies" / "ball_kick_left.onnx"


def _seed_cache(root: Path, policies: Iterable[str]) -> None:
    (root / "robot").mkdir(parents=True)
    for name in ("robot_walk.xml", "robot_allcollisions.xml", "robot_allcollisions_rollers.xml"):
        (root / "robot" / name).write_text("<mujoco/>")
    (root / "policies").mkdir()
    for name in policies:
        (root / "policies" / name).write_bytes(b"x" * 10)


def test_ensure_assets_fetches_only_missing_and_never_deletes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("DIMOS_MICRODUCK_ASSETS", str(tmp_path))
    _seed_cache(tmp_path, ["alpha_walking.onnx", "alpha_stand.onnx"])
    (tmp_path / "policies" / "extra.onnx").write_bytes(b"keep me")
    fetched: list[str] = []

    def fake_fetch(dest: Path) -> None:
        fetched.append(dest.name)
        if dest.name == "roller.onnx":
            raise RuntimeError("pinned source does not serve roller.onnx")
        dest.write_bytes(b"y" * 10)

    monkeypatch.setattr(assets_fetch, "_fetch_policy", fake_fetch)
    monkeypatch.setattr(
        assets_fetch, "_fetch_robot_dir", lambda *a, **k: pytest.fail("robot dir was cached")
    )
    assets = assets_fetch.ensure_assets("default")
    assert assets.robot_dir == tmp_path / "robot" and assets.policy_dir == tmp_path / "policies"
    assert assets.missing == (PolicyName.ROLLER,)
    assert "alpha_walking.onnx" not in fetched and "alpha_stand.onnx" not in fetched
    assert len(fetched) == 7
    assert (tmp_path / "policies" / "extra.onnx").read_bytes() == b"keep me"
    assert (tmp_path / "policies" / "alpha_stand.onnx").read_bytes() == b"x" * 10
    assert (tmp_path / ".complete-1").exists()
    # Second call: everything but roller is cached; only roller is retried.
    fetched.clear()
    assets = assets_fetch.ensure_assets("rollers")
    assert fetched == ["roller.onnx"] and assets.missing == (PolicyName.ROLLER,)


def test_ensure_assets_policy_subset_skips_probes_for_the_rest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("DIMOS_MICRODUCK_ASSETS", str(tmp_path))
    _seed_cache(tmp_path, ["alpha_stand.onnx"])
    fetched: list[str] = []

    def fake_fetch(dest: Path) -> None:
        fetched.append(dest.name)
        dest.write_bytes(b"y" * 10)

    monkeypatch.setattr(assets_fetch, "_fetch_policy", fake_fetch)
    assets = assets_fetch.ensure_assets(policies=("stand",))  # walk is always wanted
    assert fetched == ["alpha_walking.onnx"]
    assert assets.missing == tuple(
        n for n in PolicyName if n not in (PolicyName.WALK, PolicyName.STAND)
    )
    fetched.clear()
    assets = assets_fetch.ensure_assets(policies=[PolicyName.KICK_LEFT, "roulade"])
    assert fetched == ["ball_kick_left.onnx", "roulade.onnx"]
    assert PolicyName.KICK_LEFT not in assets.missing and PolicyName.GROUND_PICK in assets.missing


def test_ensure_assets_offline_marks_rest_missing_but_needs_walk(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("DIMOS_MICRODUCK_ASSETS", str(tmp_path))
    _seed_cache(tmp_path, ["alpha_walking.onnx"])
    attempts: list[str] = []

    def offline(dest: Path) -> None:
        attempts.append(dest.name)
        raise assets_fetch._SourceUnreachableError("no network")

    monkeypatch.setattr(assets_fetch, "_fetch_policy", offline)
    assets = assets_fetch.ensure_assets()
    assert attempts == ["alpha_stand.onnx"]  # one probe, then no more network calls
    assert assets.missing == tuple(n for n in PolicyName if n is not PolicyName.WALK)
    (tmp_path / "policies" / "alpha_walking.onnx").unlink()
    with pytest.raises(RuntimeError, match="walking policy"):
        assets_fetch.ensure_assets()


def test_ensure_assets_fetches_robot_dir_when_variant_mjcf_missing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("DIMOS_MICRODUCK_ASSETS", str(tmp_path))
    _seed_cache(tmp_path, [f for f in assets_fetch.POLICY_FILES.values()])
    (tmp_path / "robot" / "robot_allcollisions_rollers.xml").unlink()
    calls: list[tuple[Path, set[str]]] = []

    def fake_robot(dest: Path, required: Iterable[str]) -> None:
        calls.append((dest, set(required)))
        (dest / "robot_allcollisions_rollers.xml").write_text("<mujoco/>")

    monkeypatch.setattr(assets_fetch, "_fetch_robot_dir", fake_robot)
    monkeypatch.setattr(assets_fetch, "_fetch_policy", lambda d: pytest.fail("policies cached"))
    assets_fetch.ensure_assets("default")
    assert calls == []
    assets_fetch.ensure_assets("rollers")
    assert calls == [(tmp_path / "robot", {"robot_walk.xml", "robot_allcollisions_rollers.xml"})]
    assert (tmp_path / "robot" / "robot_walk.xml").exists()  # nothing removed

    def down(dest: Path, required: Iterable[str]) -> None:
        raise OSError("down")

    monkeypatch.setattr(assets_fetch, "_fetch_robot_dir", down)
    (tmp_path / "robot" / "robot_allcollisions.xml").unlink()
    with pytest.raises(RuntimeError, match="robot models"):
        assets_fetch.ensure_assets("default")


def test_source_serves_classifies_head_results(monkeypatch: pytest.MonkeyPatch) -> None:
    class Resp:
        status = 200

        def __enter__(self) -> Resp:
            return self

        def __exit__(self, *exc: object) -> None:
            return None

    seen: list[str] = []

    def fake_urlopen(request: Any, timeout: float) -> Resp:
        seen.append(request.get_method())
        url = request.full_url
        if url.endswith("missing.onnx"):
            raise urllib.error.HTTPError(url, 404, "nope", {}, None)  # type: ignore[arg-type]
        if url.endswith("offline.onnx"):
            raise urllib.error.URLError("no route")
        return Resp()

    monkeypatch.setattr(assets_fetch.urllib.request, "urlopen", fake_urlopen)
    assert assets_fetch._source_serves("https://x/ok.onnx") is True
    assert assets_fetch._source_serves("https://x/missing.onnx") is False
    with pytest.raises(assets_fetch._SourceUnreachableError):
        assets_fetch._source_serves("https://x/offline.onnx")
    assert seen == ["HEAD"] * 3
    with pytest.raises(RuntimeError, match="does not serve"):
        assets_fetch._fetch_policy(Path("/nonexistent/missing.onnx"))


# --------------------------------------------------------- with the cache


def _cache_or_skip(variant: str = "default") -> None:
    if not assets_fetch.variant_mjcf_path(variant).exists():
        pytest.skip(f"Microduck robot cache not present ({assets_fetch.robot_dir()})")
    if not assets_fetch.walking_policy_path().exists():
        pytest.skip(f"Microduck policy cache not present ({assets_fetch.policy_dir()})")


def test_ensure_assets_is_offline_when_cache_complete(monkeypatch: pytest.MonkeyPatch) -> None:
    _cache_or_skip()
    if not all(p.exists() for p in map(assets_fetch.policy_path, PolicyName)):
        pytest.skip("policy cache incomplete")
    before = sorted(p.name for p in assets_fetch.policy_dir().iterdir())

    def no_network(*a: Any, **k: Any) -> None:
        pytest.fail("ensure_assets touched the network with a complete cache")

    monkeypatch.setattr(assets_fetch.urllib.request, "urlopen", no_network)
    assets = assets_fetch.ensure_assets("default")
    assert assets.missing == ()
    assert assets.robot_mjcf("default").exists() and assets.policy_path("walk").exists()
    assert sorted(p.name for p in assets_fetch.policy_dir().iterdir()) == before


def _compose(variant: str, *, with_ball: bool) -> Any:
    """room_scene + robot MJCF (+ ball) at 200 Hz, like the reference harness."""
    import mujoco

    scene = mujoco.MjSpec.from_file(str(ROOM_SCENE))
    robot = mujoco.MjSpec.from_file(str(assets_fetch.variant_mjcf_path(variant)))
    scene.option.timestep = 0.005
    robot.option.timestep = 0.005
    scene.attach(robot, prefix="", frame=scene.worldbody.add_frame(pos=[0.0, 0.0, 0.0]))
    if with_ball:
        ball = mujoco.MjSpec.from_file(str(assets_fetch.robot_dir() / "ball.xml"))
        scene.attach(ball, prefix="", frame=scene.worldbody.add_frame(pos=[-1.0, 1.0, 0.0]))
    return scene.compile()


def _actuator_perm(model: Any, joint_names: list[str]) -> list[int]:
    import mujoco

    act_joint = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.actuator_trnid[i, 0]))
        for i in range(model.nu)
    ]
    return [act_joint.index(n) for n in joint_names]


@pytest.mark.mujoco
def test_policy_bank_loads_variant_policies_and_steps() -> None:
    pytest.importorskip("onnxruntime")
    mujoco = pytest.importorskip("mujoco")
    _cache_or_skip()
    model = _compose("default", with_ball=False)
    bank = PolicyBank(assets_fetch.policy_dir(), model, variant="default", missing=("roulade",))
    assert bank.names == tuple(n for n in DEFAULT_POLICIES if n != "roulade")
    assert bank.missing == (PolicyName.ROULADE,)
    assert bank.availability["roulade"] == ASSET_MISSING_REASON
    assert bank.availability["roller"] and bank.availability["walk"] is None
    assert bank.num_joints == 14 and len(bank.joint_names) == 14
    assert bank.default_pose.dtype == np.float32 and bank.default_pose.shape == (14,)
    data = mujoco.MjData(model)
    bank.initial_qpos(data)
    mujoco.mj_forward(model, data)
    assert data.qpos[bank.root_qpos_adr + 2] == pytest.approx(0.125)
    assert bank.projected_gravity(data)[2] == pytest.approx(-1.0)
    assert bank.root_yaw(data) == pytest.approx(0.0)
    assert not bank.last_action.any()
    for name in bank.names:
        targets = bank.step(name, np.zeros(13), data)
        assert targets.shape == (14,) and targets.dtype == np.float32
        assert np.isfinite(targets).all()
    assert bank.last_action.any()
    bank.reset()
    assert not bank.last_action.any()
    with pytest.raises(KeyError):
        bank.step("roller", np.zeros(13), data)
    with pytest.raises(ValueError):
        bank.step("walk", np.zeros(3), data)
    sched = PolicyScheduler(bank.availability, bank.variant)
    assert sched.request("roulade", "start") == (False, "roulade unavailable: asset missing")
    with pytest.raises(RuntimeError, match="no Microduck policy"):
        PolicyBank(Path("/nonexistent"), model)


# ------------------------------------------------ opt-in headless sim run


def _slow_variants() -> list[str]:
    raw = os.environ.get("MICRODUCK_SLOW_VARIANTS", "default")
    return [v for v in raw.split(",") if v]


def _slow_cases() -> list[tuple[str, str]]:
    if os.environ.get("MICRODUCK_SLOW_TESTS") != "1":
        return [("default", "walk")]  # placeholder; skipped below
    cases = []
    for variant in _slow_variants():
        for name in DEFAULT_POLICIES if variant == "default" else ROLLERS_POLICIES:
            cases.append((variant, name))
    return cases


class _SimRun:
    """PolicyBank + PolicyScheduler driving a headless MuJoCo model at 50 Hz."""

    FALL_Z = FALL_GRAVITY_Z

    def __init__(self, variant: str, *, spawn_x: float = 0.0) -> None:
        import mujoco

        self.mujoco = mujoco
        self.model = _compose(variant, with_ball=True)
        self.data = mujoco.MjData(self.model)
        self.bank = PolicyBank(assets_fetch.policy_dir(), self.model, variant=variant)
        self.sched = PolicyScheduler(self.bank.availability, variant, spawn_ball=self.spawn_ball)
        self.perm = _actuator_perm(self.model, self.bank.joint_names)
        self.ball_adr = int(
            self.model.jnt_qposadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "ball_free")
            ]
        )
        self.ball_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        self.bank.initial_qpos(self.data)
        self.data.qpos[self.bank.root_qpos_adr] = spawn_x
        mujoco.mj_forward(self.model, self.data)
        self.t = 0.0
        self.fell_unprotected = False
        self.max_grav_z: dict[str, float] = {}  # closest to falling (> -0.55) per policy
        self.log: list[str] = []

    def spawn_ball(self, dx: float, dy: float) -> None:
        yaw = self.bank.root_yaw(self.data)
        px, py = self.data.qpos[self.bank.root_qpos_adr : self.bank.root_qpos_adr + 2]
        x = px + dx * math.cos(yaw) - dy * math.sin(yaw)
        y = py + dx * math.sin(yaw) + dy * math.cos(yaw)
        self.data.qpos[self.ball_adr : self.ball_adr + 7] = [x, y, 0.035, 1.0, 0.0, 0.0, 0.0]
        dof = int(self.model.jnt_dofadr[self.model.body_jntadr[self.ball_body]])
        self.data.qvel[dof : dof + 6] = 0.0

    @property
    def pos(self) -> np.ndarray:
        adr = self.bank.root_qpos_adr
        return np.array(self.data.qpos[adr : adr + 3])

    @property
    def ball_pos(self) -> np.ndarray:
        return np.array(self.data.xpos[self.ball_body])

    def run(self, seconds: float, until: Callable[[], bool] | None = None) -> None:
        steps = round(seconds / 0.005)
        for i in range(steps):
            if i % 4 == 0:
                name, cmd = self.sched.tick(DT)
                targets = self.bank.step(name, cmd, self.data)
                self.data.ctrl[self.perm] = targets
                gz = float(self.bank.projected_gravity(self.data)[2])
                self.max_grav_z[name] = max(self.max_grav_z.get(name, -1.0), gz)
                fallen = gz > self.FALL_Z
                if fallen and not self.sched.suspend_fall_detector:
                    self.fell_unprotected = True
                    self.log.append(
                        f"t={self.t:.2f} FELL during {self.sched.active} grav_z={gz:+.2f}"
                    )
                self.sched.notify_fall(fallen)
                if until is not None and until():
                    return
            self.mujoco.mj_step(self.model, self.data)
            self.t += 0.005

    def upright(self) -> bool:
        return float(self.bank.projected_gravity(self.data)[2]) <= self.FALL_Z


@pytest.mark.mujoco
@pytest.mark.parametrize(("variant", "name"), _slow_cases())
def test_slow_policy_runs_in_headless_sim(variant: str, name: str) -> None:
    if os.environ.get("MICRODUCK_SLOW_TESTS") != "1":
        pytest.skip("set MICRODUCK_SLOW_TESTS=1 to run the headless policy matrix")
    pytest.importorskip("onnxruntime")
    pytest.importorskip("mujoco")
    _cache_or_skip(variant)
    # The wheeled runs cover up to ~2.5 m (push + open-loop glide); the room's
    # east wall is at x=2.05, so start those a metre back.
    sim = _SimRun(variant, spawn_x=-1.0 if name.startswith("roller") else 0.0)
    sched = sim.sched
    sim.run(2.0)  # settle on walk with zero twist
    assert sim.upright(), "did not settle upright"
    start = sim.pos.copy()
    outcome: dict[str, Any] = {"variant": variant, "policy": name}

    if name == "walk":
        sched.set_twist(0.3, 0.0, 0.0)
        sim.run(4.0)
        sched.set_twist(0.0, 0.0, 0.0)
        sim.run(2.0)
        outcome["dx"] = float(sim.pos[0] - start[0])
        assert outcome["dx"] > 0.3, outcome
    elif name == "stand":
        sched.request("stand", "start")
        sim.run(4.0)
        assert sched.active == "stand"
        sched.request("walk", "start")
        sim.run(2.0)
        outcome["dx"] = float(sim.pos[0] - start[0])
    elif name == "sitstand":
        sched.request("sitstand", "start")
        sim.run(4.0)
        assert sched.seated and sched.active == "sitstand"
        outcome["seated_z"] = float(sim.pos[2])
        sched.request("sitstand", "stop")
        sim.run(STAND_UP_DURATION_S + 0.1)
        assert sched.active == "walk" and not sched.locked, sched.snapshot()
        sim.run(3.0)
        outcome["final_z"] = float(sim.pos[2])
        assert outcome["final_z"] > 0.10, outcome
    elif name in ("kick_left", "kick_right"):
        sched.request(name, "start")
        sim.run(DT)
        ball0 = sim.ball_pos.copy()
        outcome["ball_spawn"] = [round(float(v), 3) for v in ball0]
        sim.run(KICK_DURATION_S + 3.0)
        assert sched.active == "walk"
        outcome["ball_dx"] = float(sim.ball_pos[0] - ball0[0])
        assert outcome["ball_dx"] > 0.15, outcome
    elif name == "roulade":
        sched.request("roulade", "start")
        sim.run(ROULADE_DURATION_S + 0.1)
        assert sched.active == "walk" and sched.suspend_fall_detector
        outcome["max_grav_z_roulade"] = sim.max_grav_z.get("roulade")
        sim.run(ROULADE_GRACE_S + 3.0)
        assert not sched.suspend_fall_detector
        outcome["dx"] = float(sim.pos[0] - start[0])
        assert outcome["dx"] > 0.2, outcome
    elif name == "ground_pick":
        sched.request("ground_pick", "start")
        sim.run(GROUND_PICK_END_PHASE * GROUND_PICK_PERIOD_S + 0.1)
        assert sched.active == "walk", sched.snapshot()
        sim.run(3.0)
        outcome["dx"] = float(sim.pos[0] - start[0])
    elif name == "roller":
        # 2 s at throttle 0.48 is ~0.85 m, then zero throttle for the brake
        # window and a little residual glide once walk takes over.
        sched.request("roller", "start")
        sched.set_twist(0.48, 0.0, 0.0)
        sim.run(2.0)
        outcome["rolled_dx"] = float(sim.pos[0] - start[0])
        assert outcome["rolled_dx"] > 0.5, outcome
        sched.request("walk", "start")
        sched.set_twist(0.0, 0.0, 0.0)
        sim.run(DT)
        assert sched.active == ACTIVE_BRAKING
        rolled = sim.pos.copy()
        sim.run(BRAKE_DURATION_S + 0.1)
        assert sched.active == "walk", sched.snapshot()
        braked = sim.pos.copy()
        outcome["brake_dx"] = float(braked[0] - rolled[0])
        sim.run(3.0)
        outcome["walk_glide_dx"] = float(sim.pos[0] - braked[0])
        # Open-loop brake: measured 0.7-1.1 m of forward creep in the window
        # from this entry speed (see BRAKE_DURATION_S), never reverse.
        assert -0.1 < outcome["brake_dx"] < 1.5, outcome
    elif name == "roller_crouch":
        sched.request("roller_crouch", "start")
        sim.run(6.0)
        assert sched.active == "roller_crouch"
        outcome["crouch_dx"] = float(sim.pos[0] - start[0])
        sched.request("walk", "start")
        sim.run(DT)
        assert sched.active == ACTIVE_BRAKING  # brakes on its own loop, no roller needed
        sim.run(BRAKE_DURATION_S + 3.0)
        assert sched.active == "walk"
    else:  # pragma: no cover
        pytest.fail(f"no scenario for {name}")

    outcome["max_grav_z"] = {k: round(v, 2) for k, v in sim.max_grav_z.items()}
    outcome["final_pos"] = [round(float(v), 3) for v in sim.pos]
    outcome["fell_unprotected"] = sim.fell_unprotected
    outcome["upright_at_end"] = sim.upright()
    print(f"\nSLOW_RESULT {json.dumps(outcome)}")
    assert not sim.fell_unprotected, "\n".join(sim.log)
    assert sim.upright(), outcome
    assert not sched.fallen
