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

"""Operator commands flown against a kinematic vehicle: takeoff altitude, go-to, keyboard teleop.

``test_supervisor_core.py`` holds a frozen snapshot in front of the core; here the fake
vehicle flies the setpoints it is sent, so a command is checked by where the aircraft ends
up, how fast it was told to go on the way, and what stops it.
"""

from __future__ import annotations

from dataclasses import replace
import math

import numpy as np
import pytest

from dimos.robot.px4 import mavlink as px
from dimos.robot.px4.config import GuidanceConfig, SupervisorLimits
from dimos.robot.px4.mavlink import (
    ESTIMATOR_POS_HORIZ_ABS,
    ESTIMATOR_POS_VERT_ABS,
    EstimatorStatus,
    GpsFix,
    Heartbeat,
    LocalPosition,
    RcChannels,
    StatusText,
    SysStatus,
    VehicleSnapshot,
)
from dimos.robot.px4.supervisor_core import (
    GOTO_ARRIVED,
    GotoGoal,
    Rejection,
    SupervisorCore,
    TeleopCommand,
    goto_velocity,
)
from dimos.utils.transform_utils import normalize_angle

CFG = SupervisorLimits()
GCFG = GuidanceConfig()
GO = GCFG.goto
DT = 0.05
T0 = 1_000_000.0
# How the fake closes a position setpoint, like PX4's position loop with its speed limit.
_POS_GAIN = 1.0
_POS_SPEED_MPS = 3.0
_YAW_RATE_DPS = 60.0
_LAND_SPEED_MPS = 1.0
# Long enough for the fake's position loop to close a hover tolerance to a few millimetres.
_SETTLE_S = 6.0


class FakeVehicle:
    """A point mass that flies what it is sent. It is the actuator and the snapshot source."""

    def __init__(self, yaw: float = 0.0) -> None:
        self.n = self.e = self.d = 0.0
        self.yaw = yaw
        self.now = T0
        self.armed = False
        self.mode = (px.MAIN_POSCTL, 0)
        self.enable = True
        self.frozen = False  # a vehicle that cannot make way (wind): setpoints move nothing
        self.position_valid = True
        self.local_lost = False  # no LOCAL_POSITION_NED at all
        self.local_age = 0.0  # a frozen estimator: the last sample is this old
        self.arming_denied: str | None = None  # PX4 refuses to arm and says this
        self.said: StatusText | None = None
        self.pos_sp: tuple[float, float, float] | None = None
        self.vel_sp: tuple[float, float, float] | None = None
        self.yaw_sp: float | None = None
        self.yaw_rate_sp = 0.0
        self.vel_sent: list[tuple[float, float, float]] = []

    def set_mode(self, main: int, sub: int = 0) -> None:
        self.mode = (main, sub)

    def arm(self, value: bool) -> None:
        if value and self.arming_denied:
            self.said = StatusText(seq=1, severity=2, text=self.arming_denied, t=self.now)
            return
        self.armed = value

    def send_position_setpoint(self, n: float, e: float, d: float, yaw_rad: float) -> None:
        self.pos_sp, self.vel_sp = (n, e, d), None
        self.yaw_sp, self.yaw_rate_sp = math.degrees(yaw_rad), 0.0

    def send_velocity_setpoint(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float | None = None,
        yaw_rate_rad: float | None = None,
    ) -> None:
        self.pos_sp, self.vel_sp = None, (vn, ve, vd)
        self.vel_sent.append((vn, ve, vd))
        self.yaw_sp = None if yaw_rad is None else math.degrees(yaw_rad)
        self.yaw_rate_sp = 0.0 if yaw_rate_rad is None else math.degrees(yaw_rate_rad)

    def advance(self, dt: float) -> None:
        if self.mode == (px.MAIN_AUTO, px.SUB_AUTO_LAND):
            self.d = min(0.0, self.d + _LAND_SPEED_MPS * dt)
            self.armed = self.d < 0.0
            return
        if self.mode[0] != px.MAIN_OFFBOARD or not self.armed or self.frozen:
            return
        if self.vel_sp is not None:
            vn, ve, vd = self.vel_sp
        elif self.pos_sp is not None:
            err = (self.pos_sp[0] - self.n, self.pos_sp[1] - self.e, self.pos_sp[2] - self.d)
            scale = min(_POS_GAIN, _POS_SPEED_MPS / max(math.hypot(*err), 1e-9))
            vn, ve, vd = (scale * c for c in err)
        else:
            return
        self.n, self.e, self.d = self.n + vn * dt, self.e + ve * dt, self.d + vd * dt
        if self.yaw_sp is not None:
            step = _YAW_RATE_DPS * dt
            err = math.degrees(normalize_angle(math.radians(self.yaw_sp - self.yaw)))
            self.yaw += float(np.clip(err, -step, step))
        self.yaw += self.yaw_rate_sp * dt

    def snapshot(self, now: float) -> VehicleSnapshot:
        self.now = now
        chan = [1500] * 18
        chan[CFG.enable_channel - 1] = 2000 if self.enable else 1000
        landed = px.LANDED_ON_GROUND if self.d > -0.05 else px.LANDED_IN_AIR
        return VehicleSnapshot(
            heartbeat=Heartbeat(self.armed, self.mode[0], self.mode[1], now),
            heartbeat_age=0.0,
            local=None
            if self.local_lost
            else LocalPosition(self.n, self.e, self.d, 0.0, 0.0, 0.0, now),
            local_age=math.inf if self.local_lost else self.local_age,
            gps=GpsFix(fix=3, sats=20, eph=0.8, epv=1.2, t=now),
            sys_status=SysStatus(80, 16.0, 0.0, now),
            rc=RcChannels(tuple(chan), now),
            rc_age=0.0,
            landed_state=landed,
            yaw_deg=self.yaw,
            px4_msg_age=0.0,
            estimator=EstimatorStatus(
                flags=ESTIMATOR_POS_HORIZ_ABS
                | (ESTIMATOR_POS_VERT_ABS if self.position_valid else 0),
                t=now,
            ),
            statustext=self.said,
        )


class Flight:
    """The core and the fake vehicle on one simulated clock."""

    def __init__(self, yaw: float = 0.0) -> None:
        self.core = SupervisorCore(CFG, GCFG)
        self.veh = FakeVehicle(yaw)
        self.now = T0
        self.core.state_since = T0

    def snap(self) -> VehicleSnapshot:
        return self.veh.snapshot(self.now)

    def run(self, seconds: float) -> None:
        for _ in range(round(seconds / DT)):
            self.core.step(self.snap(), self.veh, self.now)
            self.core.stream(self.veh, self.now)
            self.veh.advance(DT)
            self.now += DT

    def run_until(self, state: str, timeout_s: float) -> None:
        deadline = self.now + timeout_s
        while self.core.state != state and self.now < deadline:
            self.run(DT)
        assert self.core.state == state, (self.core.state, self.core.reason)

    def teleop(
        self,
        seconds: float,
        forward: float = 0.0,
        left: float = 0.0,
        up: float = 0.0,
        yaw_rate_ccw: float = 0.0,
    ) -> None:
        """Hold a key: a fresh cmd_vel every tick, the way the viewer sends it."""
        for _ in range(round(seconds / DT)):
            cmd = TeleopCommand(forward, left, up, yaw_rate_ccw, t=self.now)
            assert self.core.on_cmd_vel(cmd, self.now) is None
            self.run(DT)


@pytest.fixture
def flight() -> Flight:
    return Flight()


@pytest.fixture
def hovering() -> Flight:
    """Airborne at 2 m after a real takeoff sequence, facing north."""
    f = Flight()
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("HOVER", 30.0)
    f.run(_SETTLE_S)
    return f


def test_takeoff_climbs_to_the_requested_altitude(hovering: Flight) -> None:
    assert hovering.veh.d == pytest.approx(-2.0, abs=0.05)
    assert hovering.veh.pos_sp == pytest.approx((0.0, 0.0, -2.0))
    assert hovering.core.status(hovering.snap(), hovering.now)["alt_m"] == pytest.approx(
        2.0, abs=0.05
    )


def test_takeoff_without_an_altitude_uses_the_configured_one(flight: Flight) -> None:
    assert flight.core.takeoff_cmd(flight.snap(), flight.now) is None
    flight.run_until("HOVER", 30.0)
    assert flight.veh.pos_sp == pytest.approx((0.0, 0.0, -CFG.takeoff_alt_m))


@pytest.mark.parametrize(
    ("alt_m", "why"),
    [
        (CFG.min_alt_m - 0.1, Rejection.INVALID_ARGUMENT),
        (-3.0, Rejection.INVALID_ARGUMENT),
        (math.nan, Rejection.INVALID_ARGUMENT),
        (math.inf, Rejection.INVALID_ARGUMENT),
        (CFG.max_alt_m - CFG.goal_margin_m + 0.1, Rejection.CEILING),
    ],
)
def test_takeoff_refuses_an_unusable_altitude(flight: Flight, alt_m: float, why: Rejection) -> None:
    assert flight.core.takeoff_cmd(flight.snap(), flight.now, alt_m=alt_m) is why
    assert flight.core.state == "IDLE"
    flight.run(1.0)
    assert not flight.veh.armed and flight.veh.pos_sp is None


def test_a_takeoff_that_never_passes_preflight_says_why(flight: Flight) -> None:
    flight.veh.enable = False
    assert flight.core.takeoff_cmd(flight.snap(), flight.now, alt_m=2.0) is None
    flight.run(5.0)
    assert flight.core.reason == "waiting: enable switch off"
    flight.run_until("IDLE", 10.0)
    flight.run(1.0)
    assert flight.core.reason == "preflight failed: enable switch off"
    assert flight.core.last_rejection is Rejection.ENABLE_SWITCH_OFF
    assert not flight.veh.armed and flight.veh.pos_sp is None


def test_preflight_wants_a_valid_position_estimate(flight: Flight) -> None:
    flight.veh.position_valid = False
    assert flight.core.takeoff_cmd(flight.snap(), flight.now, alt_m=2.0) is None
    flight.run(1.0)
    assert flight.core.state == "PREFLIGHT"
    assert flight.core.reason == "waiting: position estimate not valid"
    assert flight.veh.pos_sp is None
    flight.veh.position_valid = True
    flight.run_until("HOVER", 30.0)


def test_arming_refusal_quotes_px4(flight: Flight) -> None:
    flight.veh.arming_denied = "Arming denied: high accelerometer bias"
    assert flight.core.takeoff_cmd(flight.snap(), flight.now, alt_m=2.0) is None
    flight.run_until("IDLE", 15.0)
    assert flight.core.reason == "arming refused: Arming denied: high accelerometer bias"
    assert flight.core.last_rejection is Rejection.NOT_ARMED
    assert flight.veh.mode == (px.MAIN_AUTO, px.SUB_AUTO_LOITER) and not flight.veh.armed


def test_goto_two_metres_south_at_three_metres(hovering: Flight) -> None:
    f = hovering
    assert f.core.goto_cmd(f.snap(), north_m=-2.0, east_m=0.0, alt_m=3.0, now=f.now) is None
    assert f.core.state == "GOTO"
    f.run_until("HOVER", 30.0)
    assert f.core.reason == GOTO_ARRIVED
    f.run(_SETTLE_S)  # the hover setpoint closes the arrival tolerance
    assert (f.veh.n, f.veh.e, f.veh.d) == pytest.approx((-2.0, 0.0, -3.0), abs=0.05)
    status = f.core.status(f.snap(), f.now)
    assert (status["north_m"], status["east_m"], status["alt_m"]) == pytest.approx(
        (-2.0, 0.0, 3.0), abs=0.05
    )
    assert status["goal"] is None
    # Walking pace all the way, whatever PX4's own limits are.
    assert max(math.hypot(vn, ve) for vn, ve, _ in f.veh.vel_sent) <= GO.v_max_mps + 1e-9
    assert max(abs(vd) for _, _, vd in f.veh.vel_sent) <= GO.vz_max_mps + 1e-9


def test_goto_is_relative_to_the_vehicle_by_default(hovering: Flight) -> None:
    f = hovering
    for _ in range(2):
        assert f.core.goto_cmd(f.snap(), north_m=0.0, east_m=3.0, now=f.now) is None
        f.run_until("HOVER", 30.0)
        f.run(_SETTLE_S)
    assert (f.veh.n, f.veh.e, f.veh.d) == pytest.approx((0.0, 6.0, -2.0), abs=0.05)


def test_goto_from_the_takeoff_point_with_a_heading(hovering: Flight) -> None:
    f = hovering
    assert f.core.goto_cmd(f.snap(), north_m=4.0, east_m=-3.0, now=f.now) is None
    f.run_until("HOVER", 30.0)
    # "Come back and face east": absolute, from the takeoff point.
    assert f.core.goto_cmd(f.snap(), 0.0, 0.0, heading_deg=90.0, relative=False, now=f.now) is None
    f.run_until("HOVER", 30.0)
    f.run(_SETTLE_S)
    assert (f.veh.n, f.veh.e) == pytest.approx((0.0, 0.0), abs=0.05)
    assert f.veh.yaw == pytest.approx(90.0, abs=1.0)


def test_goto_turns_on_the_spot(hovering: Flight) -> None:
    f = hovering
    started = f.now
    assert f.core.goto_cmd(f.snap(), 0.0, 0.0, heading_deg=-90.0, now=f.now) is None
    f.run_until("HOVER", 30.0)
    assert f.veh.yaw == pytest.approx(-90.0, abs=GO.yaw_tolerance_deg)
    assert (f.veh.n, f.veh.e, f.veh.d) == pytest.approx((0.0, 0.0, -2.0), abs=0.05)
    # Rate limited, never a step in the yaw setpoint.
    quarter_turn_s = (90.0 - GO.yaw_tolerance_deg) / GO.max_yaw_rate_dps
    assert f.now - started >= quarter_turn_s


@pytest.mark.parametrize(
    ("kwargs", "why"),
    [
        ({"north_m": CFG.geofence_radius_m - CFG.goal_margin_m + 0.5}, Rejection.FENCE),
        ({"north_m": 20.0, "east_m": 21.0}, Rejection.FENCE),
        ({"alt_m": CFG.max_alt_m}, Rejection.CEILING),
        ({"alt_m": 0.2}, Rejection.INVALID_ARGUMENT),
        ({"north_m": math.nan}, Rejection.INVALID_ARGUMENT),
        ({"heading_deg": math.inf}, Rejection.INVALID_ARGUMENT),
    ],
)
def test_goto_refuses_goals_outside_the_limits(
    hovering: Flight, kwargs: dict[str, float], why: Rejection
) -> None:
    f = hovering
    args = {"north_m": 0.0, "east_m": 0.0, **kwargs}
    assert f.core.goto_cmd(f.snap(), now=f.now, **args) is why
    assert f.core.state == "HOVER" and f.core.goal is None
    assert f.core.last_rejection is why


def test_goto_needs_a_flying_unlatched_supervisor(flight: Flight) -> None:
    f = flight
    assert f.core.goto_cmd(f.snap(), 1.0, 0.0, now=f.now) is Rejection.NOT_ARMED
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("TAKEOFF", 10.0)
    assert f.core.goto_cmd(f.snap(), 1.0, 0.0, now=f.now) is Rejection.WRONG_STATE
    f.run_until("HOVER", 30.0)
    f.core.estop_latched = True
    assert f.core.goto_cmd(f.snap(), 1.0, 0.0, now=f.now) is Rejection.ESTOP_LATCHED
    assert f.core.state == "HOVER"


def test_goto_gives_up_into_a_hover_where_it_is(hovering: Flight) -> None:
    f = hovering
    f.veh.frozen = True
    assert f.core.goto_cmd(f.snap(), north_m=10.0, east_m=0.0, now=f.now) is None
    f.run(GO.timeout_s - 1.0)
    assert f.core.state == "GOTO"
    f.run(2.0)
    assert f.core.state == "HOVER" and "timed out" in f.core.reason
    assert f.core.hover is not None and f.core.hover.n == pytest.approx(0.0)


def test_operator_hover_stops_a_goto(hovering: Flight) -> None:
    f = hovering
    assert f.core.goto_cmd(f.snap(), north_m=10.0, east_m=0.0, now=f.now) is None
    f.run(3.0)
    assert f.core.set_guidance_mode("HOVER", f.now) is None
    stopped_at = f.veh.n
    f.run(5.0)
    assert f.core.state == "HOVER" and f.core.reason != GOTO_ARRIVED
    assert 1.0 < stopped_at < 4.0
    assert f.veh.n == pytest.approx(stopped_at, abs=0.1)


def test_pilot_override_ends_a_goto_without_a_mode_command(hovering: Flight) -> None:
    f = hovering
    assert f.core.goto_cmd(f.snap(), north_m=10.0, east_m=0.0, now=f.now) is None
    f.run(2.0)
    f.veh.mode = (px.MAIN_POSCTL, 0)  # the pilot flicks to Position
    f.run(0.2)
    assert f.core.state == "IDLE" and "PILOT_OVERRIDE" in f.core.reason
    assert f.core.sp is None
    assert f.veh.mode == (px.MAIN_POSCTL, 0)


def test_enable_switch_aborts_a_goto_into_hold(hovering: Flight) -> None:
    f = hovering
    assert f.core.goto_cmd(f.snap(), north_m=10.0, east_m=0.0, now=f.now) is None
    f.run(2.0)
    f.veh.enable = False
    f.run(0.2)
    assert f.core.state == "ABORT" and f.core.sp is None
    assert f.veh.mode == (px.MAIN_AUTO, px.SUB_AUTO_LOITER)


def test_land_after_a_goto(hovering: Flight) -> None:
    f = hovering
    assert f.core.goto_cmd(f.snap(), north_m=-2.0, east_m=0.0, alt_m=3.0, now=f.now) is None
    f.run_until("HOVER", 30.0)
    assert f.core.land_cmd(f.snap(), f.veh, f.now) is None
    f.run_until("IDLE", 30.0)
    assert f.core.reason == "landed and disarmed"
    assert not f.veh.armed and f.veh.d == 0.0


def test_goto_velocity_law() -> None:
    goal = GotoGoal(n=0.0, e=10.0, d=-5.0)
    vn, ve, vd = goto_velocity(0.0, 0.0, -2.0, goal, GO)
    assert (vn, ve) == pytest.approx((0.0, GO.v_max_mps))  # far: capped, along the bearing
    assert vd == pytest.approx(-GO.vz_max_mps)  # 3 m to climb: capped, negative D is up
    vn, ve, vd = goto_velocity(0.0, 9.5, -4.9, goal, GO)
    assert (vn, ve) == pytest.approx((0.0, GO.k_pos * 0.5))  # near: proportional
    assert vd == pytest.approx(GO.k_alt * -0.1)
    assert goto_velocity(0.0, 10.0, -5.0, goal, GO) == (0.0, 0.0, 0.0)


def test_keyboard_flies_the_vehicle_only_in_teleop() -> None:
    f = Flight(yaw=90.0)  # facing east: forward is east, left is north
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("HOVER", 30.0)
    f.run(_SETTLE_S)
    key_w = TeleopCommand(forward=0.5, left=0.0, up=0.0, yaw_rate_ccw=0.0, t=f.now)
    assert f.core.on_cmd_vel(key_w, f.now) is Rejection.NOT_TELEOP
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    assert f.core.state == "TELEOP"

    f.teleop(2.0, forward=0.5)  # W held 2 s
    assert (f.veh.n, f.veh.e) == pytest.approx((0.0, 1.0), abs=0.05)
    f.teleop(2.0, left=0.5)  # Q held 2 s: strafe left of east is north
    assert (f.veh.n, f.veh.e) == pytest.approx((1.0, 1.0), abs=0.05)
    f.teleop(1.0, yaw_rate_ccw=0.8)  # A held 1 s: counter-clockwise lowers the heading
    assert f.veh.yaw == pytest.approx(90.0 - math.degrees(0.8), abs=1.0)

    # Key up: the viewer stops sending, the command goes stale, the vehicle holds.
    f.run(CFG.teleop_stale_s + 0.2)
    held = (f.veh.n, f.veh.e)
    assert f.veh.vel_sp is None and f.veh.pos_sp is not None
    f.run(3.0)
    assert (f.veh.n, f.veh.e) == pytest.approx(held, abs=0.05)
    assert f.veh.d == pytest.approx(-2.0, abs=0.05)


def test_keyboard_is_clamped_and_cannot_change_altitude(hovering: Flight) -> None:
    f = hovering
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    f.teleop(1.0, forward=9.0, up=5.0)  # Shift+W from a viewer with wild speeds
    vn, ve, vd = f.veh.vel_sent[-1]
    assert math.hypot(vn, ve) == pytest.approx(CFG.teleop_v_xy_mps)
    # The up axis is ignored: what is left of vd is the altitude hold's trim.
    assert abs(vd) < 0.01 and f.veh.d == pytest.approx(-2.0, abs=0.05)
    f.teleop(1.0, forward=9.0, left=9.0)  # W+Q: the cap is on the vector, not per axis
    vn, ve, _ = f.veh.vel_sent[-1]
    assert math.hypot(vn, ve) == pytest.approx(CFG.teleop_v_xy_mps)


def test_teleop_holds_the_altitude_it_started_at(hovering: Flight) -> None:
    f = hovering
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    f.veh.d += 0.4  # a sag, as a braking airframe loses height
    f.teleop(4.0, forward=0.5)
    assert f.veh.d == pytest.approx(-2.0, abs=0.1)  # climbing back while it flies
    f.run(_SETTLE_S)  # key up: the hold is at the locked altitude, not where it sagged to
    assert f.core.hover is not None and f.core.hover.d == pytest.approx(-2.0)
    assert f.veh.d == pytest.approx(-2.0, abs=0.05)


def test_goto_takes_over_from_the_keyboard_and_ends_in_hover(hovering: Flight) -> None:
    f = hovering
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    f.teleop(1.0, forward=0.5)
    assert f.core.goto_cmd(f.snap(), 0.0, 0.0, alt_m=3.0, relative=False, now=f.now) is None
    key_w = TeleopCommand(forward=0.5, left=0.0, up=0.0, yaw_rate_ccw=0.0, t=f.now)
    assert f.core.on_cmd_vel(key_w, f.now) is Rejection.NOT_TELEOP
    f.run_until("HOVER", 30.0)
    f.run(CFG.hover_settle_s + 1.0)
    assert f.core.state == "HOVER"  # it does not drift back into TELEOP by itself


def test_a_frozen_local_position_aborts_into_hold(hovering: Flight) -> None:
    f = hovering
    f.veh.local_age = CFG.px4_stale_s + 0.1
    f.run(0.1)
    assert f.core.state == "ABORT" and f.core.reason == "local position lost"
    assert f.core.last_rejection is Rejection.STALE_INPUT and f.core.sp is None
    assert f.veh.mode == (px.MAIN_AUTO, px.SUB_AUTO_LOITER)


def test_a_missing_local_position_aborts_teleop_without_raising(hovering: Flight) -> None:
    f = hovering
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    f.veh.local_lost = True
    f.run(0.1)  # _step_teleop would assert on the missing sample
    assert f.core.state == "ABORT" and f.core.reason == "local position lost"


def test_enable_switch_dropping_while_prestreaming_cancels_the_takeoff(flight: Flight) -> None:
    f = flight
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("STREAMING", 5.0)
    f.veh.enable = False
    f.run(CFG.prestream_s + 1.0)
    assert f.core.state == "IDLE"
    assert f.core.reason == "takeoff cancelled: enable switch off / RC lost"
    assert f.core.last_rejection is Rejection.ENABLE_SWITCH_OFF and f.core.sp is None
    # Never asked for OFFBOARD, never armed, and no mode command: PX4 was not ours yet.
    assert f.veh.mode == (px.MAIN_POSCTL, 0) and not f.veh.armed


def test_enable_switch_dropping_after_the_offboard_request_never_arms(flight: Flight) -> None:
    f = flight
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("OFFBOARD_REQ", 5.0)
    assert f.veh.mode == (px.MAIN_OFFBOARD, 0)
    f.veh.enable = False
    f.run(1.0)
    assert f.core.state == "IDLE" and f.core.reason.startswith("takeoff cancelled")
    assert not f.veh.armed
    assert f.veh.mode == (px.MAIN_AUTO, px.SUB_AUTO_LOITER)  # PX4 handed back in Hold


def test_estop_holds_latches_and_clears_only_in_idle(hovering: Flight) -> None:
    f = hovering
    f.core.estop(f.snap(), f.veh, f.now)
    assert f.core.state == "IDLE" and f.core.estop_latched and f.core.sp is None
    assert f.veh.mode == (px.MAIN_AUTO, px.SUB_AUTO_LOITER)
    assert f.core.takeoff_cmd(f.snap(), f.now) is Rejection.ESTOP_LATCHED
    assert f.core.set_guidance_mode("TELEOP", f.now) is Rejection.ESTOP_LATCHED
    assert f.core.estop_clear() is None and not f.core.estop_latched


def test_estop_never_commands_a_mode_the_pilot_did_not_give_us(hovering: Flight) -> None:
    f = hovering
    f.veh.mode = (px.MAIN_POSCTL, 0)
    f.core.estop(f.snap(), f.veh, f.now)
    assert f.core.estop_latched and f.veh.mode == (px.MAIN_POSCTL, 0)


def test_estop_land_lands_and_stays_latched(hovering: Flight) -> None:
    f = hovering
    f.core.estop_land(f.snap(), f.veh, f.now)
    assert f.core.state == "LANDING" and f.core.estop_clear() is Rejection.WRONG_STATE
    f.run_until("IDLE", 30.0)
    assert f.core.estop_latched and not f.veh.armed


def test_land_and_hold_leave_the_pilot_alone(hovering: Flight) -> None:
    f = hovering
    f.veh.mode = (px.MAIN_POSCTL, 0)
    assert f.core.land_cmd(f.snap(), f.veh, f.now) is Rejection.MODE_NOT_OFFBOARD
    assert f.core.hold_cmd(f.snap(), f.veh, f.now) is None
    assert f.core.state == "IDLE" and f.veh.mode == (px.MAIN_POSCTL, 0)


# The safety rules, one case each: deleting a rule from the core fails its case here.


@pytest.mark.parametrize(
    ("change", "reason", "why"),
    [
        ({"heartbeat_age": 2.0, "px4_msg_age": 2.0}, "PX4 heartbeat lost", Rejection.STALE_INPUT),
        (
            {"rc_age": CFG.rc_stale_s + 0.1},
            "enable switch off / RC lost",
            Rejection.ENABLE_SWITCH_OFF,
        ),
        (
            {"sys_status": SysStatus(CFG.min_batt_pct - 11, 14.0, 0.0, T0)},
            f"battery {CFG.min_batt_pct - 11}%",
            Rejection.BATTERY,
        ),
        (
            {"local": LocalPosition(0.0, 0.0, -(CFG.max_alt_m + 1.0), 0.0, 0.0, 0.0, T0)},
            f"altitude {CFG.max_alt_m + 1.0:.1f} m",
            Rejection.CEILING,
        ),
        (
            {"local": LocalPosition(CFG.geofence_radius_m + 1.0, 0.0, -2.0, 0.0, 0.0, 0.0, T0)},
            f"geofence {CFG.geofence_radius_m + 1.0:.0f} m",
            Rejection.FENCE,
        ),
    ],
)
def test_each_abort_rule_puts_px4_in_hold(
    hovering: Flight, change: dict[str, object], reason: str, why: Rejection
) -> None:
    f = hovering
    f.core.step(replace(f.snap(), **change), f.veh, f.now)  # type: ignore[arg-type]
    assert f.core.state == "ABORT" and f.core.reason == reason
    assert f.core.last_rejection is why and f.core.sp is None
    assert f.veh.mode == (px.MAIN_AUTO, px.SUB_AUTO_LOITER)
    f.run(2.5)  # the dwell, then back to IDLE
    assert f.core.state == "IDLE" and f.core.reason == "after abort"


@pytest.mark.parametrize(
    ("change", "failures"),
    [
        ({"gps": GpsFix(fix=2, sats=5, eph=0.8, epv=1.2, t=T0)}, ["GPS fix"]),
        ({"gps": None}, ["GPS fix"]),
        ({"gps": GpsFix(fix=3, sats=20, eph=3.0, epv=1.2, t=T0)}, ["eph 3.0"]),
        ({"sys_status": SysStatus(30, 15.0, 0.0, T0)}, ["battery 30%"]),
        ({"local_age": 1.0}, ["local position stale"]),
        ({"local": None}, ["local position stale"]),
        ({"heartbeat_age": 2.0, "px4_msg_age": 2.0}, ["PX4 heartbeat stale"]),
        ({"landed_state": px.LANDED_IN_AIR}, ["not on ground"]),
        ({"heartbeat": Heartbeat(True, px.MAIN_POSCTL, 0, T0)}, ["already armed"]),
        ({"rc_age": 5.0}, ["RC stale", "enable switch off"]),
        ({"rc": None}, ["RC stale", "enable switch off"]),
    ],
)
def test_each_preflight_check_refuses(
    flight: Flight, change: dict[str, object], failures: list[str]
) -> None:
    assert flight.core.preflight_failures(flight.snap()) == []
    snap = replace(flight.snap(), **change)  # type: ignore[arg-type]
    assert flight.core.preflight_failures(snap) == failures


def test_an_unknown_gps_accuracy_does_not_block_preflight(flight: Flight) -> None:
    snap = replace(flight.snap(), gps=GpsFix(fix=3, sats=20, eph=math.nan, epv=math.nan, t=T0))
    assert flight.core.preflight_failures(snap) == []


def test_sitl_fakes_the_enable_switch_and_skips_the_rc_checks() -> None:
    core = SupervisorCore(CFG, GCFG, sitl=True)
    snap = replace(FakeVehicle().snapshot(T0), rc=None, rc_age=math.inf)
    assert core.preflight_failures(snap) == ["enable switch off"]
    assert core.sitl_enable(True) is None
    assert core.preflight_failures(snap) == []
    assert SupervisorCore(CFG, GCFG).sitl_enable(True) is Rejection.WRONG_STATE


def test_cmd_vel_is_refused_when_unusable_stale_or_latched(hovering: Flight) -> None:
    f = hovering
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    for bad in (math.nan, math.inf):
        cmd = TeleopCommand(bad, 0.0, 0.0, 0.0, t=f.now)
        assert f.core.on_cmd_vel(cmd, f.now) is Rejection.STALE_INPUT
    old = TeleopCommand(0.5, 0.0, 0.0, 0.0, t=f.now - CFG.teleop_stale_s - 0.5)
    assert f.core.on_cmd_vel(old, f.now) is Rejection.STALE_INPUT
    assert f.core.teleop is None
    f.core.estop_latched = True
    fresh = TeleopCommand(0.5, 0.0, 0.0, 0.0, t=f.now)
    assert f.core.on_cmd_vel(fresh, f.now) is Rejection.ESTOP_LATCHED


def test_the_stream_holds_setpoint_hz_and_survives_a_stepped_clock(hovering: Flight) -> None:
    f = hovering
    before = f.core.setpoint_count
    for i in range(200):  # 1 s of ticks five times faster than the stream
        f.core.stream(f.veh, f.now + i * 0.005)
    assert CFG.setpoint_hz <= f.core.setpoint_count - before <= CFG.setpoint_hz + 1
    # A late tick does not cost the next setpoint.
    sent = [
        f.core.stream(f.veh, f.now + 10.0 + t) for t in (0.0, 0.053, 0.100, 0.150, 0.203, 0.250)
    ]
    assert sent == [True] * 6
    # The wall clock stepped back 5 s (first NTP sync): the stream goes on at once.
    assert f.core.stream(f.veh, f.now + 5.0)
    assert f.core.stream(f.veh, f.now + 5.05)


def test_offboard_that_never_comes_ends_the_takeoff(flight: Flight) -> None:
    f = flight
    f.veh.set_mode = lambda main, sub=0: None  # type: ignore[method-assign]  # PX4 ignores us
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("OFFBOARD_REQ", 5.0)
    f.run_until("IDLE", CFG.ack_timeout_s + 1.0)
    assert f.core.reason.startswith("PX4 did not enter OFFBOARD")
    assert f.core.last_rejection is Rejection.MODE_NOT_OFFBOARD
    assert f.core.sp is None and not f.veh.armed


def test_estop_between_the_offboard_request_and_arming_hands_px4_back(flight: Flight) -> None:
    f = flight
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("OFFBOARD_REQ", 5.0)
    assert f.veh.mode == (px.MAIN_OFFBOARD, 0)
    f.core.estop(f.snap(), f.veh, f.now)
    assert f.core.state == "IDLE" and f.core.estop_latched and not f.veh.armed
    assert f.veh.mode == (px.MAIN_AUTO, px.SUB_AUTO_LOITER)  # not left sitting in OFFBOARD


@pytest.mark.parametrize("ending", ["estop", "estop_land", "land", "hold", "abort"])
def test_a_selected_mode_does_not_outlive_its_flight(hovering: Flight, ending: str) -> None:
    f = hovering
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    assert f.core.state == "TELEOP"
    if ending == "estop":
        f.core.estop(f.snap(), f.veh, f.now)
    elif ending == "estop_land":
        f.core.estop_land(f.snap(), f.veh, f.now)
    elif ending == "land":
        assert f.core.land_cmd(f.snap(), f.veh, f.now) is None
    elif ending == "hold":
        assert f.core.hold_cmd(f.snap(), f.veh, f.now) is None
    else:
        f.veh.enable = False
    f.run_until("IDLE", 40.0)
    assert f.core.guidance_mode == "HOVER"

    # The next flight: a plain takeoff hovers and stays hovering, nobody selected a mode.
    f.veh.enable, f.veh.armed, f.veh.d = True, False, 0.0
    f.veh.mode = (px.MAIN_POSCTL, 0)
    f.core.estop_latched = False
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("HOVER", 30.0)
    f.run(CFG.hover_settle_s + 1.0)
    assert f.core.state == "HOVER"


def test_a_mode_selected_on_the_ground_still_applies_after_the_hover_settles(
    flight: Flight,
) -> None:
    f = flight
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("HOVER", 30.0)
    f.run(CFG.hover_settle_s + 0.5)
    assert f.core.state == "TELEOP"


def test_an_estop_on_the_ground_drops_a_preselected_mode(flight: Flight) -> None:
    f = flight
    assert f.core.set_guidance_mode("TELEOP", f.now) is None
    f.core.estop(f.snap(), f.veh, f.now)  # IDLE to IDLE: no transition, the mode still goes
    assert f.core.estop_clear() is None
    assert f.core.guidance_mode == "HOVER"
