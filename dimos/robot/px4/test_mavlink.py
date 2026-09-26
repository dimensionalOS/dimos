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

"""The MAVLink layer without a socket: frames, vehicle state, the transport hooks."""

from __future__ import annotations

import math
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.robot.px4.config import GuidanceConfig, SupervisorLimits
from dimos.robot.px4.mavlink import (
    ESTIMATOR_POS_HORIZ_ABS,
    ESTIMATOR_POS_VERT_ABS,
    MAIN_OFFBOARD,
    MASK_POS_YAW,
    MASK_VEL_YAW,
    MASK_VEL_YAWRATE,
    MAV_FRAME_LOCAL_NED,
    MavlinkIO,
    VehicleState,
    body_flu_velocity_to_ned,
    frd_to_flu,
    ned_to_flu,
    quaternion_from_ned_euler,
)
from dimos.robot.px4.supervisor_core import SupervisorCore


def test_ned_to_flu_matches_upstream_signs() -> None:
    # Same convention as dimos/robot/drone/test_drone.py::test_ned_to_ros_coordinate_conversion:
    # north -> +x, east -> -y, down -> -z.
    assert ned_to_flu(3.0, 4.0, -1.0) == (3.0, -4.0, 1.0)
    assert frd_to_flu(1.0, 2.0, 9.8) == (1.0, -2.0, -9.8)


def test_a_heading_east_is_a_minus_ninety_degree_flu_yaw() -> None:
    # Same convention as dimos/robot/drone/mavlink_connection.py: roll kept, pitch and yaw negated.
    q = quaternion_from_ned_euler(0.0, 0.0, math.radians(90.0))
    assert (q.x, q.y, q.z, q.w) == pytest.approx((0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)))
    nose_up = quaternion_from_ned_euler(0.0, math.radians(10.0), 0.0)  # FRD pitch up is FLU -y
    assert (nose_up.y, nose_up.w) == pytest.approx(
        (-math.sin(math.radians(5.0)), math.cos(math.radians(5.0)))
    )


def test_body_velocity_rotates_with_heading() -> None:
    # Heading north: forward is north, left is west (negative east).
    assert body_flu_velocity_to_ned(1.0, 0.5, 0.2, 0.0) == pytest.approx((1.0, -0.5, -0.2))
    # Heading east (yaw +90 deg clockwise): forward is east, left is north.
    vn, ve, vd = body_flu_velocity_to_ned(1.0, 0.5, 0.0, math.radians(90))
    assert (vn, ve, vd) == pytest.approx((0.5, 1.0, 0.0), abs=1e-12)


class Msg:
    def __init__(self, typ: str, src: tuple[int, int] = (1, 1), **kw: Any) -> None:
        self._t = typ
        self._src = src
        self.__dict__.update(kw)

    def get_type(self) -> str:
        return self._t

    def get_srcSystem(self) -> int:
        return self._src[0]

    def get_srcComponent(self) -> int:
        return self._src[1]


def test_messages_from_other_components_are_ignored() -> None:
    st = VehicleState()
    st.handle(
        Msg("HEARTBEAT", src=(1, 191), base_mode=128, custom_mode=MAIN_OFFBOARD << 16), now=1.0
    )
    assert st.heartbeat is None
    st.handle(Msg("HEARTBEAT", src=(1, 1), base_mode=128, custom_mode=MAIN_OFFBOARD << 16), now=2.0)
    assert st.heartbeat is not None and st.heartbeat.armed and st.heartbeat.main == MAIN_OFFBOARD


def test_snapshot_ages_use_snapshot_time() -> None:
    st = VehicleState()
    st.handle(Msg("HEARTBEAT", base_mode=0, custom_mode=0), now=100.0)
    st.handle(Msg("LOCAL_POSITION_NED", x=1, y=2, z=-3, vx=0, vy=0, vz=0), now=100.2)
    st.handle(Msg("ATTITUDE", roll=0.0, pitch=0.0, yaw=math.radians(90)), now=100.2)
    snap = st.snapshot(now=101.0)
    assert snap.heartbeat_age == 1.0
    assert math.isclose(snap.local_age, 0.8)
    assert snap.yaw_deg == 90.0
    assert snap.rc is None and snap.rc_age == math.inf


def test_snapshot_carries_estimator_validity_and_the_latest_warning() -> None:
    st = VehicleState()
    st.handle(Msg("ESTIMATOR_STATUS", flags=ESTIMATOR_POS_HORIZ_ABS), now=100.0)
    st.handle(Msg("STATUSTEXT", severity=6, text="Ready"), now=100.1)
    snap = st.snapshot(now=101.0)
    assert snap.estimator is not None and snap.estimator.position_valid is False
    assert snap.statustext is None  # INFO is not worth an operator's attention
    st.handle(
        Msg("ESTIMATOR_STATUS", flags=ESTIMATOR_POS_HORIZ_ABS | ESTIMATOR_POS_VERT_ABS),
        now=102.0,
    )
    st.handle(Msg("STATUSTEXT", severity=2, text=b"Arming denied: not landed\x00"), now=102.5)
    snap = st.snapshot(now=103.0)
    assert snap.estimator is not None and snap.estimator.position_valid
    assert snap.statustext is not None and snap.statustext.text == "Arming denied: not landed"
    assert snap.statustext.t == 102.5


def test_gimbal_attitude_only_from_component_154() -> None:
    st = VehicleState()
    ident = [1.0, 0.0, 0.0, 0.0]
    st.handle(
        Msg("GIMBAL_DEVICE_ATTITUDE_STATUS", src=(1, 1), q=ident, flags=16, failure_flags=0),
        now=1.0,
    )
    assert st.gimbal is None
    st.handle(
        Msg("GIMBAL_DEVICE_ATTITUDE_STATUS", src=(1, 154), q=ident, flags=16, failure_flags=0),
        now=1.0,
    )
    g = st.gimbal
    assert g is not None and g.t == 1.0 and (g.pitch, g.yaw) == pytest.approx((0.0, 0.0))
    assert st.gimbal_flags == 16


@pytest.mark.parametrize("q", ([0.0] * 4, [math.nan, 0.0, 0.0, 0.0]))
def test_gimbal_bad_quaternion_is_dropped_not_raised(q: list[float]) -> None:
    st = VehicleState()
    st.handle(
        Msg("GIMBAL_DEVICE_ATTITUDE_STATUS", src=(1, 154), q=q, flags=16, failure_flags=0),
        now=1.0,
    )
    assert st.gimbal is None


def _rc(chan7: int, **kw: int) -> Msg:
    chans = {f"chan{i}_raw": 1500 for i in range(1, 19)}
    chans["chan7_raw"] = chan7
    return Msg("RC_CHANNELS", **chans, **kw)


@pytest.mark.parametrize(
    ("msg", "enabled"),
    [
        (_rc(1800, chancount=16), True),
        (_rc(1000, chancount=16), False),
        (_rc(65535, chancount=6), False),  # PX4 fills channels past chancount with UINT16_MAX
        (_rc(65535), False),
        (_rc(1800, chancount=6), False),  # a stale high value on a channel the receiver lacks
    ],
)
def test_an_absent_rc_channel_reads_as_enable_switch_off(msg: Msg, enabled: bool) -> None:
    st = VehicleState()
    st.handle(msg, now=1.0)
    core = SupervisorCore(SupervisorLimits(), GuidanceConfig())
    assert core.enable_switch(st.snapshot(now=1.0)) is enabled
    if not enabled:
        assert "enable switch off" in core.preflight_failures(st.snapshot(now=1.0))


def test_gps_accuracy_is_metres_from_h_acc_not_hdop() -> None:
    st = VehicleState()
    # HDOP 0.9 (eph=90) with a 0.45 m horizontal and 0.7 m vertical accuracy.
    st.handle(
        Msg("GPS_RAW_INT", fix_type=3, satellites_visible=14, eph=90, epv=130, h_acc=450, v_acc=700)
    )
    assert st.gps is not None and (st.gps.eph, st.gps.epv) == (0.45, 0.7)
    for unknown in ({"h_acc": 0, "v_acc": 0}, {}):  # 0, or a MAVLink 1 message without the fields
        st.handle(Msg("GPS_RAW_INT", fix_type=3, satellites_visible=14, eph=90, epv=130, **unknown))
        assert st.gps is not None and math.isnan(st.gps.eph) and math.isnan(st.gps.epv)


def test_gps_unknown_satellite_count_is_minus_one() -> None:
    st = VehicleState()
    st.handle(Msg("GPS_RAW_INT", fix_type=0, satellites_visible=255, eph=65535, epv=65535), now=1.0)
    assert st.gps is not None and st.gps.sats == -1


def test_battery_unknowns_are_nan_not_measurements() -> None:
    st = VehicleState()
    st.handle(Msg("SYS_STATUS", battery_remaining=-1, voltage_battery=65535, current_battery=-1))
    ss = st.sys_status
    assert ss is not None and ss.batt_pct == -1 and math.isnan(ss.volt) and math.isnan(ss.current_a)
    st.handle(Msg("SYS_STATUS", battery_remaining=80, voltage_battery=16000, current_battery=520))
    assert st.sys_status is not None
    assert (st.sys_status.volt, st.sys_status.current_a) == (16.0, 5.2)


def _io() -> MavlinkIO:
    return MavlinkIO(VehicleState(), url="", source_system=1, source_component=195)


def test_command_ack_resolves_the_future_through_the_hooks(monkeypatch: pytest.MonkeyPatch) -> None:
    io = _io()
    with pytest.raises(RuntimeError):
        io.send_command(400, 1.0)  # not started: nothing written, so nothing waits for an ack
    assert not io._acks
    written: list[tuple[int, list[float]]] = []
    monkeypatch.setattr(io, "_write_command", lambda command, p: written.append((command, p)))
    fut = io.send_command(400, 1.0)
    assert written == [(400, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
    io._ingest(Msg("COMMAND_ACK", src=(1, 154), command=400, result=0))  # not PX4's ack
    assert not fut.done()
    io._ingest(Msg("COMMAND_ACK", command=400, result=0))
    assert fut.result(timeout=0) == 0


def test_a_setpoint_counts_only_once_it_is_written(monkeypatch: pytest.MonkeyPatch) -> None:
    io = _io()
    monkeypatch.setattr(io, "_write_setpoint", lambda *a: False)
    io.send_position_setpoint(1.0, 2.0, -3.0, 0.5)
    assert io.stats()["setpoints_sent"] == 0


def test_the_bytes_to_the_aircraft() -> None:
    """The real writers against a recorder: CI never runs SITL, a swapped vn/ve would ship green."""
    io = _io()
    conn = io._conn = MagicMock()
    sp = conn.mav.set_position_target_local_ned_send
    io.send_position_setpoint(1.0, 2.0, -3.0, 0.5)
    assert sp.call_args.args[1:] == (
        1, 1, MAV_FRAME_LOCAL_NED, MASK_POS_YAW, 1.0, 2.0, -3.0, 0.0, 0.0, 0.0, 0, 0, 0, 0.5, 0.0
    )  # fmt: skip
    io.send_velocity_setpoint(0.4, -0.2, 0.1, yaw_rad=1.5)
    assert sp.call_args.args[4:] == (MASK_VEL_YAW, 0.0, 0.0, 0.0, 0.4, -0.2, 0.1, 0, 0, 0, 1.5, 0.0)
    io.send_velocity_setpoint(0.4, -0.2, 0.1, yaw_rate_rad=0.2)
    assert sp.call_args.args[4:] == (
        MASK_VEL_YAWRATE, 0.0, 0.0, 0.0, 0.4, -0.2, 0.1, 0, 0, 0, 0.0, 0.2
    )  # fmt: skip
    assert io.stats()["setpoints_sent"] == 3
    io.set_mode(MAIN_OFFBOARD)
    io.arm(True)
    sent = [c.args for c in conn.mav.command_long_send.call_args_list]
    assert sent == [
        (1, 1, 176, 0, 1, MAIN_OFFBOARD, 0, 0.0, 0.0, 0.0, 0.0),
        (1, 1, 400, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ]


def test_the_reader_survives_a_malformed_message() -> None:
    io = _io()
    good = Msg("HEARTBEAT", base_mode=0, custom_mode=MAIN_OFFBOARD << 16)
    feed = iter([Msg("HEARTBEAT", base_mode=0), good])  # the first lacks custom_mode

    def recv_match(**_: Any) -> Msg | None:
        msg = next(feed, None)
        if msg is None:
            io._stop.set()
        return msg

    io._conn = MagicMock(recv_match=recv_match)
    io._reader_loop()
    assert io._state.heartbeat is not None and io._state.heartbeat.main == MAIN_OFFBOARD
    assert io.stats()["bad_data"] == 1
