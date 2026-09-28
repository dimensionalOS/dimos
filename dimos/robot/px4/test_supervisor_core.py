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

"""The supervisor state machine against a frozen snapshot and a recording actuator.

``make_state`` is also the one place that drives GPS_RAW_INT, SYS_STATUS, RC_CHANNELS and
EXTENDED_SYS_STATE through ``VehicleState.handle`` into the preflight checks.
"""

from __future__ import annotations

import math
import time
from typing import Any

import pytest

from dimos.robot.px4 import mavlink as px
from dimos.robot.px4.config import GuidanceConfig, SupervisorLimits
from dimos.robot.px4.mavlink import VehicleSnapshot, VehicleState
from dimos.robot.px4.supervisor_core import SupervisorCore, TakeoffPoint, rate_limit_yaw
from dimos.robot.px4.test_mavlink import Msg

CFG = SupervisorLimits()
GCFG = GuidanceConfig()


def test_rate_limit_yaw_wraps() -> None:
    assert rate_limit_yaw(170.0, -170.0, 30.0, 0.5) == pytest.approx(-175.0)
    assert rate_limit_yaw(0.0, 90.0, 30.0, 1.0) == pytest.approx(30.0)


class FakeActuator:
    """Records every vehicle side effect the core requests."""

    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []

    def set_mode(self, main: int, sub: int = 0) -> None:
        self.calls.append(("mode", main, sub))

    def arm(self, value: bool) -> None:
        self.calls.append(("arm", value))

    def send_position_setpoint(self, n: float, e: float, d: float, yaw_rad: float) -> None:
        self.calls.append(("sp", "pos", (n, e, d), (0.0, 0.0, 0.0), yaw_rad))

    def send_velocity_setpoint(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float | None = None,
        yaw_rate_rad: float | None = None,
    ) -> None:
        self.calls.append(("sp", "vel", (0.0, 0.0, 0.0), (vn, ve, vd), yaw_rad))

    def modes(self) -> list[tuple[int, int]]:
        return [(c[1], c[2]) for c in self.calls if c[0] == "mode"]

    def arms(self) -> list[bool]:
        return [c[1] for c in self.calls if c[0] == "arm"]

    def setpoints(self) -> list[tuple[Any, ...]]:
        return [c for c in self.calls if c[0] == "sp"]


def make_state(
    armed: bool = False,
    main: int = px.MAIN_POSCTL,
    landed: int = px.LANDED_ON_GROUND,
    enable: bool = True,
    n: float = 0.0,
    e: float = 0.0,
    d: float = 0.0,
    batt: int = 80,
) -> VehicleSnapshot:
    st = VehicleState()
    cm = main << 16
    st.handle(Msg("HEARTBEAT", base_mode=128 if armed else 0, custom_mode=cm))
    st.handle(Msg("LOCAL_POSITION_NED", x=n, y=e, z=d, vx=0, vy=0, vz=0))
    st.handle(Msg("GPS_RAW_INT", fix_type=3, satellites_visible=20, h_acc=800, v_acc=1200))
    st.handle(Msg("SYS_STATUS", battery_remaining=batt, voltage_battery=16000, current_battery=500))
    st.handle(Msg("EXTENDED_SYS_STATE", landed_state=landed, vtol_state=0))
    st.handle(Msg("ATTITUDE", roll=0.0, pitch=0.0, yaw=math.radians(45.0)))
    chans: dict[str, Any] = {f"chan{i}_raw": 1500 for i in range(1, 19)}
    chans[f"chan{CFG.enable_channel}_raw"] = 2000 if enable else 1000
    st.handle(Msg("RC_CHANNELS", **chans))
    return st.snapshot()


def run(
    sup: SupervisorCore,
    st: VehicleSnapshot,
    m: FakeActuator,
    seconds: float,
    dt: float = 0.05,
    t0: float | None = None,
) -> float:
    """Tick for ``seconds`` from ``t0`` (now when None); returns the time to continue from."""
    t = time.time() if t0 is None else t0
    n = int(seconds / dt)
    for i in range(n):
        sup.step(st, m, t + i * dt)
        sup.stream(m, t + i * dt)
    return t + n * dt


def test_supervisor_nominal_takeoff_and_pilot_override() -> None:
    m = FakeActuator()
    sup = SupervisorCore(CFG, GCFG)
    st = make_state()
    assert sup.takeoff_cmd(st) is None
    assert sup.state == "PREFLIGHT"
    # One clock through the phases: a run() that restarted at time.time() would rewind it.
    t = run(sup, st, m, 0.1)
    assert sup.state == "STREAMING", sup.reason
    t = run(sup, st, m, CFG.prestream_s + 0.2, t0=t)
    assert sup.state == "OFFBOARD_REQ"
    assert (px.MAIN_OFFBOARD, 0) in m.modes()  # OFFBOARD requested
    n_sp = len(m.setpoints())
    assert n_sp >= CFG.setpoint_hz * CFG.prestream_s * 0.8  # streamed before the request
    st = make_state(main=px.MAIN_OFFBOARD)
    t = run(sup, st, m, 0.1, t0=t)
    assert sup.state == "ARMING" and m.arms() == [True]
    assert len(m.setpoints()) > n_sp  # and the stream went on past it
    st = make_state(armed=True, main=px.MAIN_OFFBOARD, landed=px.LANDED_IN_AIR)
    t = run(sup, st, m, 0.1, t0=t)
    assert sup.state == "TAKEOFF"
    st = make_state(
        armed=True, main=px.MAIN_OFFBOARD, landed=px.LANDED_IN_AIR, d=-CFG.takeoff_alt_m
    )
    t = run(sup, st, m, CFG.takeoff_alt_m / CFG.climb_rate_mps + 1.0, t0=t)
    assert sup.state == "HOVER", sup.reason
    sp = m.setpoints()[-1]
    assert sp[2][2] == pytest.approx(-CFG.takeoff_alt_m)  # D setpoint = -3 m
    # Pilot flicks to Position: supervisor stops without commanding any mode.
    before = len(m.modes())
    st = make_state(armed=True, main=px.MAIN_POSCTL, landed=px.LANDED_IN_AIR, d=-3.0)
    run(sup, st, m, 0.1, t0=t)
    assert sup.state == "IDLE" and "PILOT_OVERRIDE" in sup.reason
    assert len(m.modes()) == before and sup.sp is None


def test_supervisor_abort_on_enable_switch_and_geofence() -> None:
    m = FakeActuator()
    sup = SupervisorCore(CFG, GCFG)
    sup.state, sup.entered_offboard = "HOVER", True
    sup.takeoff = TakeoffPoint(n=0.0, e=0.0, d0=0.0, yaw=0.0)
    sup.yaw_cmd = 0.0
    st = make_state(
        armed=True, main=px.MAIN_OFFBOARD, landed=px.LANDED_IN_AIR, d=-3.0, enable=False
    )
    run(sup, st, m, 0.1)
    assert sup.state == "ABORT" and "enable" in sup.reason
    assert (px.MAIN_AUTO, px.SUB_AUTO_LOITER) in m.modes(), m.calls
    sup2 = SupervisorCore(CFG, GCFG)
    sup2.state, sup2.entered_offboard = "HOVER", True
    sup2.takeoff = TakeoffPoint(n=0.0, e=0.0, d0=0.0, yaw=0.0)
    sup2.yaw_cmd = 0.0
    st = make_state(
        armed=True,
        main=px.MAIN_OFFBOARD,
        landed=px.LANDED_IN_AIR,
        n=CFG.geofence_radius_m + 1,
        d=-3.0,
    )
    run(sup2, st, m, 0.1)
    assert sup2.state == "ABORT" and "geofence" in sup2.reason


def test_supervisor_preflight_refuses_without_enable() -> None:
    m = FakeActuator()
    sup = SupervisorCore(CFG, GCFG)
    st = make_state(enable=False)
    assert sup.takeoff_cmd(st) is None
    run(sup, st, m, 0.5)
    assert sup.state == "PREFLIGHT" and "enable switch" in sup.reason
    assert not m.setpoints()  # nothing streamed
