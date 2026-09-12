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

"""Unit tests for UnitreeWebRTCConnection.

Pure-Python test suite with no hardware or network. Covers connect() error propagation,
aes_128_key forwarding, the UNITREE_AES_128_KEY env var via GlobalConfig, and the
movement auto-stop watchdog ordering guarantees.
"""

import asyncio
import json
import threading
import time
from typing import Any
from unittest.mock import ANY, AsyncMock, MagicMock, call

import pytest
from unitree_webrtc_connect.constants import DATA_CHANNEL_TYPE, RTC_TOPIC, SPORT_CMD

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree import connection as conn_mod
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.utils.testing.waiting import wait_until


def _stub_driver(connect_exc: Exception | None = None) -> MagicMock:
    """A LegionConnection instance double covering everything connect() touches."""
    driver = MagicMock(name="LegionConnection-instance")
    driver.connect = AsyncMock(side_effect=connect_exc)
    driver.datachannel.disableTrafficSaving = AsyncMock()
    driver.datachannel.set_decoder = MagicMock()
    driver.datachannel.pub_sub.publish_request_new = AsyncMock()
    return driver


def test_connect_failure_propagates_to_caller(monkeypatch: pytest.MonkeyPatch) -> None:
    """A driver connect failure must raise from the constructor, not hang."""
    driver = _stub_driver(connect_exc=RuntimeError("aes_128_key required (data2=3)"))
    monkeypatch.setattr(conn_mod, "LegionConnection", MagicMock(return_value=driver))

    with pytest.raises(RuntimeError, match="aes_128_key required"):
        UnitreeWebRTCConnection(ip="10.0.0.99")


@pytest.fixture
def built_connection(monkeypatch: pytest.MonkeyPatch) -> Any:
    """A live UnitreeWebRTCConnection over a stubbed driver, torn down (loop
    stopped, thread joined) unconditionally so a failed assert can't leak it."""
    driver = _stub_driver()
    monkeypatch.setattr(conn_mod, "LegionConnection", MagicMock(return_value=driver))

    conn = UnitreeWebRTCConnection(ip="10.0.0.99")
    try:
        yield conn, driver
    finally:
        conn.loop.call_soon_threadsafe(conn.loop.stop)
        conn.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)


def test_connect_success_completes_setup(built_connection: Any) -> None:
    """Happy path: constructor returns after the setup sequence ran."""
    _conn, driver = built_connection

    driver.connect.assert_awaited_once()
    driver.datachannel.pub_sub.publish_request_new.assert_awaited_once()


@pytest.mark.parametrize(
    ("connection_options", "expected_call"),
    [
        pytest.param(
            {},
            call(
                RTC_TOPIC["WIRELESS_CONTROLLER"],
                data={"lx": 0.4, "ly": 1.5, "rx": -0.8, "ry": 0},
            ),
            id="joystick",
        ),
        pytest.param(
            {"velocity_api": True},
            call(
                RTC_TOPIC["SPORT_MOD"],
                data={
                    "header": {
                        "identity": {
                            "id": ANY,
                            "api_id": SPORT_CMD["Move"],
                        }
                    },
                    "parameter": json.dumps({"x": 1.5, "y": -0.4, "z": 0.8}),
                },
                msg_type=DATA_CHANNEL_TYPE["REQUEST"],
            ),
            id="velocity",
        ),
    ],
)
def test_move_api_toggle_sends_selected_wire_command(
    monkeypatch: pytest.MonkeyPatch,
    connection_options: dict[str, bool],
    expected_call: Any,
) -> None:
    driver = _stub_driver()
    monkeypatch.setattr(conn_mod, "LegionConnection", MagicMock(return_value=driver))
    twist = Twist(
        linear=Vector3(1.5, -0.4, 0.0),
        angular=Vector3(0.0, 0.0, 0.8),
    )

    connection = UnitreeWebRTCConnection(ip="10.0.0.99", **connection_options)
    try:
        driver.datachannel.pub_sub.publish_without_callback.reset_mock()
        assert connection.move(twist)
        assert driver.datachannel.pub_sub.publish_without_callback.call_args == expected_call
    finally:
        connection.stop_movement()
        connection.loop.call_soon_threadsafe(connection.loop.stop)
        connection.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)


@pytest.fixture
def stub_legion(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace LegionConnection with a mock and no-op connect() so __init__
    stays inside the aes_128_key resolution without dialing out."""
    monkeypatch.setattr(UnitreeWebRTCConnection, "connect", lambda self: None)
    legion = MagicMock(name="LegionConnection")
    monkeypatch.setattr(conn_mod, "LegionConnection", legion)
    return legion


def _aes_kwarg(legion: MagicMock) -> Any:
    """The aes_128_key passed to LegionConnection, or None if absent."""
    return legion.call_args.kwargs.get("aes_128_key")


def test_no_key_forwards_falsy(stub_legion: MagicMock) -> None:
    """No key → a falsy value reaches the driver, which treats it as no key."""
    UnitreeWebRTCConnection(ip="192.168.123.161")
    assert not _aes_kwarg(stub_legion)


def test_aes_key_forwarded_when_provided(stub_legion: MagicMock) -> None:
    """A provided key is forwarded verbatim to the driver."""
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="aa" * 16)
    assert _aes_kwarg(stub_legion) == "aa" * 16


def test_empty_string_key_forwarded_as_falsy(stub_legion: MagicMock) -> None:
    """Empty-string key stays falsy → the driver treats it as no key."""
    UnitreeWebRTCConnection(ip="192.168.123.161", aes_128_key="")
    assert not _aes_kwarg(stub_legion)


def test_global_config_reads_unitree_aes_128_key_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """The key enters via GlobalConfig, read from the UNITREE_AES_128_KEY env var."""
    monkeypatch.setenv("UNITREE_AES_128_KEY", "ee" * 16)
    assert GlobalConfig().unitree_aes_128_key == "ee" * 16


def _forward_twist(x: float) -> Twist:
    return Twist(linear=Vector3(x, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))


def _joystick_wire(x: float) -> dict[str, float]:
    """WIRELESS_CONTROLLER payload for a pure-forward twist of magnitude x."""
    return {"lx": 0, "ly": x, "rx": 0, "ry": 0}


_ZERO_TWIST_WIRE = _joystick_wire(0)


def _published_data(driver: MagicMock) -> list[dict[str, float]]:
    """Every movement payload sent over the data channel, in publish order."""
    return [
        c.kwargs["data"] for c in driver.datachannel.pub_sub.publish_without_callback.call_args_list
    ]


def _watchdog_armed(conn: UnitreeWebRTCConnection) -> bool:
    """Read the watchdog handle on the connection loop, where it is owned."""

    async def read() -> bool:
        return conn._watchdog_handle is not None

    return asyncio.run_coroutine_threadsafe(read(), conn.loop).result(timeout=2.0)


def test_watchdog_zeroes_movement_after_timeout(built_connection: Any) -> None:
    """With no follow-up command, the watchdog halts the base after cmd_vel_timeout."""
    conn, driver = built_connection
    conn.cmd_vel_timeout = 0.05

    assert conn.move(_forward_twist(1.0))

    wait_until(lambda: len(_published_data(driver)) >= 2, timeout=2.0, interval=0.01)
    assert _published_data(driver) == [_joystick_wire(1.0), _ZERO_TWIST_WIRE]


def test_stale_watchdog_expiry_does_not_stop_newer_command(
    built_connection: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Regression test for #3046: a watchdog expiry that has already started when a
    replacement command arrives must not zero out that command, and the replacement's
    own watchdog must still be armed afterwards."""
    conn, driver = built_connection
    conn.cmd_vel_timeout = 0.01

    expiry_started = threading.Event()
    release_expiry = threading.Event()
    real_expired = conn._watchdog_expired

    def gated_expired() -> None:
        expiry_started.set()
        release_expiry.wait(timeout=2.0)
        real_expired()

    monkeypatch.setattr(conn, "_watchdog_expired", gated_expired)

    assert conn.move(_forward_twist(1.0))
    assert expiry_started.wait(timeout=2.0), "first watchdog never expired"

    # The first expiry is past cancellation and holds the loop. Issue the
    # replacement command while it is in flight; a long timeout keeps the
    # replacement's own watchdog from firing during the assertions below.
    conn.cmd_vel_timeout = 0.5
    mover = threading.Thread(target=lambda: conn.move(_forward_twist(2.0)))
    mover.start()
    time.sleep(0.05)  # let the replacement enqueue behind the gated expiry
    release_expiry.set()
    mover.join(timeout=2.0)
    assert not mover.is_alive()

    # The stale zero landed before the replacement command, never after it.
    data = _published_data(driver)
    assert data[-1] == _joystick_wire(2.0)
    assert data[-2] == _ZERO_TWIST_WIRE

    # The replacement re-armed the watchdog, which then halts the base as usual.
    wait_until(lambda: len(_published_data(driver)) == len(data) + 1, timeout=2.0, interval=0.01)
    assert _published_data(driver)[-1] == _ZERO_TWIST_WIRE


def test_stop_movement_disarms_watchdog(built_connection: Any) -> None:
    """An explicit stop publishes one zero twist and cancels the pending deadline."""
    conn, driver = built_connection
    conn.cmd_vel_timeout = 5.0  # long enough that only stop_movement can publish zero

    assert conn.move(_forward_twist(1.0))
    conn.stop_movement()

    assert _published_data(driver) == [_joystick_wire(1.0), _ZERO_TWIST_WIRE]
    assert not _watchdog_armed(conn)


def test_duration_move_is_not_zeroed_mid_stream(built_connection: Any) -> None:
    """A duration move outliving cmd_vel_timeout keeps re-arming the watchdog:
    the only zero twist is the final stop, never one injected mid-stream."""
    conn, driver = built_connection
    conn.cmd_vel_timeout = 0.1

    assert conn.move(_forward_twist(1.0), duration=0.3)

    data = _published_data(driver)
    assert len(data) >= 3  # repeated move publishes plus the final stop
    assert all(d == _joystick_wire(1.0) for d in data[:-1])
    assert data[-1] == _ZERO_TWIST_WIRE
    assert not _watchdog_armed(conn)
