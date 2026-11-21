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
and aes_128_key forwarding.
"""

import asyncio
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import json
import threading
from typing import Any
from unittest.mock import ANY, AsyncMock, MagicMock, call

import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree import connection as conn_mod
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.webrtc_connect import DATA_CHANNEL_TYPE, RTC_TOPIC, SPORT_CMD

_FORWARD = {"lx": 0.0, "ly": 0.1, "rx": 0.0, "ry": 0}
_STOP = {"lx": 0, "ly": 0, "rx": 0, "ry": 0}


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


@pytest.mark.parametrize("duration", [0.0, 0.15], ids=["watchdog", "timed-loop"])
def test_expired_webrtc_move_cannot_overwrite_replacement(built_connection, monkeypatch, duration):
    conn, driver = built_connection
    deadlines = []
    monkeypatch.setattr(threading.Timer, "start", lambda timer: deadlines.append(timer.function))
    sent = threading.Event()
    publish = driver.datachannel.pub_sub.publish_without_callback
    publish.side_effect = lambda *args, **kwargs: sent.set()
    with ThreadPoolExecutor(max_workers=1) as pool:
        first = pool.submit(conn.move, Twist(linear=[0.1, 0, 0]), duration)
        assert sent.wait(2)
        assert conn.move(Twist(linear=[0, 0.2, 0]))
        assert first.result(timeout=2)

    deadlines[0]()  # The old callback was dispatched before cancel.
    commands = [item.kwargs["data"] for item in publish.call_args_list]
    lateral = {"lx": -0.2, "ly": 0.0, "rx": 0.0, "ry": 0}
    assert commands[commands.index(lateral) :] == [lateral]
    deadlines[1]()
    assert publish.call_args.kwargs["data"] == {"lx": 0, "ly": 0, "rx": 0, "ry": 0}


def _returned(call: Callable[..., Any], *args: Any) -> Any:
    """The call's result; a call that never returns fails the test instead of hanging it."""
    results: list[Any] = []
    thread = threading.Thread(target=lambda: results.append(call(*args)), daemon=True)
    thread.start()
    thread.join(2)
    assert results, f"{call.__name__} did not return"
    return results[0]


@pytest.mark.parametrize(
    ("stalled_in", "delivered", "expected"),
    [
        pytest.param("loop", False, [_STOP], id="loop"),
        pytest.param("driver", True, [_FORWARD, _STOP], id="driver"),
    ],
)
def test_stalled_send_returns_and_the_stop_still_lands(
    built_connection, monkeypatch, stalled_in, delivered, expected
):
    conn, driver = built_connection
    monkeypatch.setattr(conn_mod, "_SEND_TIMEOUT", 0.05)
    deadlines = []
    monkeypatch.setattr(threading.Timer, "start", lambda timer: deadlines.append(timer.function))
    publish = driver.datachannel.pub_sub.publish_without_callback
    stalled, release = threading.Event(), threading.Event()

    def stall(*args, **kwargs):
        stalled.set()
        release.wait(10)

    queue = conn.loop.call_soon_threadsafe
    if stalled_in == "loop":
        queue(stall)
        assert stalled.wait(2)
    else:
        publish.side_effect = stall

        def queue_until_the_driver_holds_it(*args):
            handle = queue(*args)
            stalled.wait(2)
            return handle

        monkeypatch.setattr(conn.loop, "call_soon_threadsafe", queue_until_the_driver_holds_it)

    try:
        # Only a command the driver already holds counts as sent and keeps a watchdog.
        assert _returned(conn.move, Twist(linear=[0.1, 0, 0])) is delivered
        assert bool(deadlines) is delivered
        _returned(conn.stop_movement)
    finally:
        release.set()
    # Let the recovered loop run everything that was queued behind the stall.
    asyncio.run_coroutine_threadsafe(asyncio.sleep(0), conn.loop).result(2)
    assert [item.kwargs["data"] for item in publish.call_args_list] == expected

    assert conn.move(Twist(linear=[0, 0.2, 0]))
    assert publish.call_args.kwargs["data"] == {"lx": -0.2, "ly": 0.0, "rx": 0.0, "ry": 0}


def test_move_fails_at_once_when_the_loop_has_stopped(built_connection, monkeypatch):
    conn, _driver = built_connection
    monkeypatch.setattr(conn_mod, "_SEND_TIMEOUT", 30)
    conn.loop.call_soon_threadsafe(conn.loop.stop)
    conn.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
    assert _returned(conn.move, Twist(linear=[0.1, 0, 0])) is False
