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
aes_128_key forwarding, and the UNITREE_AES_128_KEY env var via GlobalConfig.
"""

import json
import threading
from typing import Any
from unittest.mock import ANY, AsyncMock, MagicMock, call

from dimos_generated.geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3
from dimos_generated.sensor_msgs.msg import Image, PointCloud2
import numpy as np
import pytest
import reactivex as rx
from reactivex.scheduler import ThreadPoolScheduler
from unitree_webrtc_connect.constants import DATA_CHANNEL_TYPE, RTC_TOPIC, SPORT_CMD

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.msgs.image import image_view
from dimos.msgs.pointcloud import pointcloud_xyz
from dimos.robot.unitree import connection as conn_mod
from dimos.robot.unitree.connection import SerializableVideoFrame, UnitreeWebRTCConnection
from dimos.robot.unitree.type.odometry import raw_odometry_msg_sample


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
        linear=Vector3(x=1.5, y=-0.4),
        angular=Vector3(z=0.8),
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


@pytest.fixture
def sensor_scheduler(mocker):
    scheduler = ThreadPoolScheduler(max_workers=2)
    mocker.patch("dimos.utils.reactive.get_scheduler", return_value=scheduler)
    try:
        yield scheduler
    finally:
        scheduler.executor.shutdown(wait=True)


@pytest.mark.parametrize("kind", ["lidar", "video"])
def test_sensor_streams_emit_generated_cdr_with_exact_arrival_time(
    built_connection, mocker, sensor_scheduler, kind
):
    connection, _driver = built_connection
    mocker.patch.object(conn_mod.time, "time_ns", return_value=1700000000123456789)
    values = np.array([[1.25, 2.5, 3.75]], dtype=np.float32)
    pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    if kind == "lidar":
        raw = {"data": {"stamp": 1.0, "data": {"points": values}}}
        mocker.patch.object(connection, "raw_lidar_stream", return_value=rx.just(raw))
        stream = connection.lidar_stream()
    else:
        frame = SerializableVideoFrame(data=pixels)
        mocker.patch.object(connection, "raw_video_stream", return_value=rx.just(frame))
        stream = connection.video_stream()
    received = []
    errors = []
    ready = threading.Event()

    def receive(message):
        received.append(message)
        ready.set()

    def fail(error):
        errors.append(error)
        ready.set()

    subscription = stream.subscribe(receive, fail)
    try:
        assert ready.wait(5), "Sensor conversion did not publish"
        assert not errors
        assert len(received) == 1
        message = received[0]
        assert (message.header.stamp.sec, message.header.stamp.nanosec) == (1700000000, 123456789)
        if kind == "lidar":
            decoded = PointCloud2.decode(message.encode())
            np.testing.assert_array_equal(pointcloud_xyz(decoded), values)
            assert decoded.header.frame_id == "world"
        else:
            decoded = Image.decode(message.encode())
            np.testing.assert_array_equal(image_view(decoded), pixels)
            assert decoded.header.frame_id == "camera_optical"
    finally:
        subscription.dispose()


@pytest.mark.parametrize("as_tf", [False, True])
def test_odometry_stream_preserves_pose_and_uses_exact_arrival_stamp(
    built_connection, mocker, sensor_scheduler, as_tf
):
    connection, _driver = built_connection
    mocker.patch.object(conn_mod.time, "time_ns", return_value=1700000000123456789)
    mocker.patch.object(
        connection, "raw_odom_stream", return_value=rx.just(raw_odometry_msg_sample)
    )
    stream = connection.tf_stream() if as_tf else connection.odom_stream()
    received = []
    errors = []
    ready = threading.Event()

    def receive(message):
        received.append(message)
        ready.set()

    def fail(error):
        errors.append(error)
        ready.set()

    subscription = stream.subscribe(receive, fail)
    try:
        assert ready.wait(5), "Odometry conversion did not publish"
        assert not errors
        assert len(received) == 1
        cls = TransformStamped if as_tf else PoseStamped
        message = cls.decode(received[0].encode())
        assert message.header.frame_id == "world"
        assert (message.header.stamp.sec, message.header.stamp.nanosec) == (1700000000, 123456789)
        if as_tf:
            assert message.child_frame_id == "base_link"
            position, rotation = message.transform.translation, message.transform.rotation
        else:
            position, rotation = message.pose.position, message.pose.orientation
        assert (position.x, position.y, position.z) == (5.961965, -2.916958, 0.319509)
        assert rotation.w == -0.242112
    finally:
        subscription.dispose()
