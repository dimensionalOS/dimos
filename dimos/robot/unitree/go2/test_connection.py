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

"""Tests for go2.connection make_connection routing + Go2WebRTCConnection dispatch.

The generic transport leaf (UnitreeWebRTCConnection) is covered in
dimos/robot/unitree/test_unitree_webrtc.py.
"""

import json
from types import SimpleNamespace
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest
from unitree_webrtc_connect.constants import DATA_CHANNEL_TYPE, RTC_TOPIC, SPORT_CMD

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree import unitree_webrtc as webrtc_mod
from dimos.robot.unitree.go2 import connection as go2_conn, go2_webrtc as go2_webrtc_mod
from dimos.robot.unitree.go2.connection import ConnectionConfig
from dimos.robot.unitree.go2.go2_webrtc import Go2WebRTCConnection, TwistMode


@pytest.fixture
def stub_go2_conn(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace Go2WebRTCConnection so make_connection's webrtc branch doesn't dial out."""
    stub = MagicMock(name="Go2WebRTCConnection")
    monkeypatch.setattr(go2_conn, "Go2WebRTCConnection", stub)
    return stub


def test_make_connection_webrtc_forwards_aes_128_key(stub_go2_conn: MagicMock) -> None:
    cfg = SimpleNamespace(unitree_connection_type="webrtc")
    go2_conn.make_connection("192.168.123.161", cfg, aes_128_key="cafe" * 8)
    stub_go2_conn.assert_called_once_with(
        "192.168.123.161", aes_128_key="cafe" * 8, twist_mode=TwistMode.VELOCITY
    )


def test_make_connection_webrtc_forwards_twist_mode(stub_go2_conn: MagicMock) -> None:
    cfg = SimpleNamespace(unitree_connection_type="webrtc")
    go2_conn.make_connection("192.168.123.161", cfg, twist_mode=TwistMode.JOYSTICK)
    assert stub_go2_conn.call_args.kwargs["twist_mode"] is TwistMode.JOYSTICK


def test_connection_config_twist_mode_defaults_to_velocity() -> None:
    assert ConnectionConfig(g=GlobalConfig(robot_ip="127.0.0.1")).twist_mode is TwistMode.VELOCITY


def test_connection_config_aes_key_defaults_from_global_config() -> None:
    g = GlobalConfig(robot_ip="127.0.0.1", unitree_aes_128_key="dd" * 16)
    assert ConnectionConfig(g=g).aes_128_key == "dd" * 16


def _stub_driver() -> MagicMock:
    driver = MagicMock(name="LegionConnection-instance")
    driver.connect = AsyncMock()
    driver.datachannel.disableTrafficSaving = AsyncMock()
    driver.datachannel.set_decoder = MagicMock()
    driver.datachannel.pub_sub.publish_request_new = AsyncMock()
    return driver


@pytest.fixture
def built_go2(monkeypatch: pytest.MonkeyPatch) -> Any:
    """Build live Go2WebRTCConnections over a stubbed driver (real loop), torn down after."""
    driver = _stub_driver()
    monkeypatch.setattr(webrtc_mod, "LegionConnection", MagicMock(return_value=driver))
    conns = []

    def build(twist_mode: TwistMode = TwistMode.VELOCITY):  # type: ignore[no-untyped-def]
        conn = Go2WebRTCConnection(ip="10.0.0.99", twist_mode=twist_mode)
        conns.append(conn)
        return conn, driver

    try:
        yield build
    finally:
        for conn in conns:
            conn.loop.call_soon_threadsafe(conn.loop.stop)
            conn.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)


def _last_publish(driver: MagicMock) -> Any:
    return driver.datachannel.pub_sub.publish_without_callback.call_args


def test_default_twist_mode_is_velocity(built_go2: Any) -> None:
    conn, _ = built_go2()
    assert conn.twist_mode is TwistMode.VELOCITY


def test_velocity_mode_sends_sport_move(built_go2: Any) -> None:
    conn, driver = built_go2(TwistMode.VELOCITY)
    conn._send_twist(0.5, -0.25, 0.125)

    call = _last_publish(driver)
    assert call.args[0] == RTC_TOPIC["SPORT_MOD"]
    data = call.kwargs["data"]
    assert data["header"]["identity"]["api_id"] == SPORT_CMD["Move"]
    assert json.loads(data["parameter"]) == {"x": 0.5, "y": -0.25, "z": 0.125}
    assert call.kwargs["msg_type"] == DATA_CHANNEL_TYPE["REQUEST"]


def test_joystick_mode_sends_wireless_axes(built_go2: Any) -> None:
    conn, driver = built_go2(TwistMode.JOYSTICK)
    conn._send_twist(0.5, -0.25, 0.125)

    call = _last_publish(driver)
    assert call.args[0] == RTC_TOPIC["WIRELESS_CONTROLLER"]
    assert call.kwargs["data"] == {"lx": 0.25, "ly": 0.5, "rx": -0.125, "ry": 0}


def test_rage_active_forces_joystick_even_in_velocity_mode(built_go2: Any) -> None:
    conn, driver = built_go2(TwistMode.VELOCITY)
    conn._rage_active = True
    conn._send_twist(0.5, -0.25, 0.125)
    assert _last_publish(driver).args[0] == RTC_TOPIC["WIRELESS_CONTROLLER"]


def test_set_rage_mode_tracks_active_state(built_go2: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    conn, _ = built_go2()
    monkeypatch.setattr(conn, "publish_request", lambda *a, **k: True)
    monkeypatch.setattr(go2_webrtc_mod.time, "sleep", lambda *a, **k: None)

    assert conn.set_rage_mode(True) is True
    assert conn._rage_active is True
    assert conn.set_rage_mode(False) is True
    assert conn._rage_active is False


def test_move_uses_loop_timer_not_a_thread(built_go2: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    conn, _ = built_go2()
    # The auto-stop schedules on the event loop; move() must never spawn a per-call thread.
    monkeypatch.setattr(
        go2_webrtc_mod.threading,
        "Timer",
        lambda *a, **k: pytest.fail("move() must not use threading.Timer"),
    )
    assert conn.move(Twist(linear=Vector3(0.1, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))) is True


def test_auto_stop_sends_zero(built_go2: Any) -> None:
    conn, driver = built_go2(TwistMode.JOYSTICK)
    conn._auto_stop()  # simulate the command-timeout firing
    call = _last_publish(driver)
    assert call.args[0] == RTC_TOPIC["WIRELESS_CONTROLLER"]
    assert all(v == 0 for v in call.kwargs["data"].values())


def test_stop_sends_zero_twist_for_safety(built_go2: Any) -> None:
    conn, driver = built_go2(TwistMode.JOYSTICK)
    conn.stop()
    call = _last_publish(driver)  # the only publish_without_callback is the safety stop
    assert call.args[0] == RTC_TOPIC["WIRELESS_CONTROLLER"]
    assert all(v == 0 for v in call.kwargs["data"].values())
