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

"""YAW_TRACK and FOLLOW: the two guidance laws, then the states that fly on them."""

from __future__ import annotations

import math
import time

import pytest

from dimos.robot.px4 import mavlink as px
from dimos.robot.px4.follow import (
    FollowConfig,
    TargetEstimate,
    YawTrackConfig,
    follow_velocity,
    yaw_track_rate,
)
from dimos.robot.px4.supervisor_core import SupervisorCore, TakeoffPoint
from dimos.robot.px4.test_supervisor_core import CFG, GCFG, FakeActuator, make_state, run
from dimos.robot.px4.test_supervisor_operator import Flight

YT = YawTrackConfig()
FO = FollowConfig()


def test_yaw_track_deadband_and_limit() -> None:
    assert yaw_track_rate(10.0, True, YT) == 0.0
    assert yaw_track_rate(40.0, False, YT) == 0.0
    assert yaw_track_rate(None, True, YT) == 0.0
    assert yaw_track_rate(40.0, True, YT) == pytest.approx(YT.k_yaw * 25.0)
    assert yaw_track_rate(-89.0, True, YT) == -YT.max_yaw_rate_dps


def test_follow_geometry() -> None:
    tgt = TargetEstimate(valid=True, n=30.0, e=0.0, vn=0.0, ve=0.0)
    c = follow_velocity(0.0, 0.0, -10.0, 0.0, tgt, FO)
    assert (c.range_m, c.yaw_deg) == pytest.approx((30.0, 0.0))
    assert c.vn > 0 and abs(c.ve) < 1e-9  # closes toward target
    assert math.hypot(c.vn, c.ve) <= FO.v_max_mps + 1e-9
    assert c.vd == pytest.approx(0.0)  # already at 10 m
    tgt = TargetEstimate(valid=True, n=12.5, e=0.0, vn=0.0, ve=0.0)
    c = follow_velocity(0.0, 0.0, -5.0, 0.0, tgt, FO)
    assert c.vn == pytest.approx(0.0)  # inside deadband
    assert c.vd < 0  # climb (negative D velocity) to 10 m
    tgt = TargetEstimate(valid=True, n=5.0, e=0.0, vn=0.0, ve=0.0)
    assert follow_velocity(0, 0, -10, 0, tgt, FO).vn < 0  # too close: back away
    tgt = TargetEstimate(valid=True, n=12.0, e=0.0, vn=0.0, ve=1.0)
    assert follow_velocity(0, 0, -10, 0, tgt, FO).ve == pytest.approx(FO.ff_gain)  # feed-forward


def test_supervisor_yaw_track_and_follow_loss() -> None:
    m = FakeActuator()
    sup = SupervisorCore(CFG, GCFG)
    sup.state, sup.entered_offboard = "HOVER", True
    sup.takeoff = TakeoffPoint(n=0.0, e=0.0, d0=0.0, yaw=0.0)
    sup.yaw_cmd = 0.0
    st = make_state(armed=True, main=px.MAIN_OFFBOARD, landed=px.LANDED_IN_AIR, d=-3.0)
    assert sup.set_guidance_mode("YAW_TRACK") is None
    run(sup, st, m, CFG.hover_settle_s + 0.2)
    assert sup.state == "YAW_TRACK"
    sup.follow.on_target(
        TargetEstimate(valid=False, los_valid=True, gimbal_yaw_body_deg=60.0), time.time()
    )
    y0 = sup.yaw_cmd
    t = time.time()
    for i in range(21):
        sup.step(st, m, t + i * 0.05)  # 1 s at 20 Hz
    expected = min(YT.max_yaw_rate_dps, YT.k_yaw * (60.0 - YT.deadband_deg))
    assert 0.6 * expected <= sup.yaw_cmd - y0 <= 1.2 * expected, (sup.yaw_cmd - y0, expected)
    # FOLLOW with a fresh target, then loss -> hold, then long loss -> HOVER.
    assert sup.set_guidance_mode("FOLLOW") is None
    sup.step(st, m, time.time())
    assert sup.state == "FOLLOW" and sup.sp is not None and sup.sp.kind == "pos"
    sup.follow.on_target(TargetEstimate(valid=True, n=30.0, e=0.0, vn=0.0, ve=0.0), time.time())
    sup.step(st, m, time.time())
    assert sup.state == "FOLLOW"
    sup.step(st, m, time.time())
    assert sup.sp is not None and sup.sp.kind == "vel" and sup.sp.vn > 0
    # Estimator keeps sending packets, but they are invalid: age counts from the last VALID one.
    sup.follow.on_target(TargetEstimate(valid=False), time.time())
    sup.follow.target_valid_t = time.time() - 2.0
    sup.step(st, m, time.time())
    assert sup.sp is not None and sup.sp.kind == "pos" and "holding" in sup.reason
    sup.follow.target_valid_t = time.time() - 6.0
    sup.state_since = time.time() - 10.0  # entered FOLLOW long ago
    sup.step(st, m, time.time())
    assert sup.state == "HOVER" and "lost" in sup.reason, (sup.state, sup.reason)
    assert sup.hover is not None
    assert abs(sup.hover.d - (-3.0)) < 1e-6  # holds current altitude, no descent to takeoff alt


def test_follow_does_not_outlive_an_estop() -> None:
    f = Flight()
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("HOVER", 30.0)
    assert f.core.set_guidance_mode("FOLLOW", f.now) is None
    f.run_until("FOLLOW", 1.0)
    f.core.estop(f.snap(), f.veh, f.now)
    assert f.core.estop_clear() is None
    # The next flight: a plain takeoff hovers and stays hovering, nobody selected FOLLOW again.
    f.veh.armed, f.veh.d, f.veh.mode = False, 0.0, (px.MAIN_POSCTL, 0)
    assert f.core.takeoff_cmd(f.snap(), f.now, alt_m=2.0) is None
    f.run_until("HOVER", 30.0)
    f.run(CFG.hover_settle_s + 1.0)
    assert f.core.state == "HOVER" and f.core.guidance_mode == "HOVER"
