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

"""Tests for go2.connection: make_connection routing and TF frame naming.

The leaf (UnitreeWebRTCConnection.__init__) is covered in
dimos/robot/unitree/test_connection.py; this pins the go2-local routing.
"""

from collections.abc import Callable, Iterator
from contextlib import ExitStack
from functools import partial
import json
from pathlib import Path
import threading
import time
from unittest.mock import MagicMock

import numpy as np
import pytest
from pytest_mock import MockerFixture
from reactivex import Subject

from dimos.core import transport as transport_module
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import GlobalConfig
from dimos.core.transport_factory import make_transport
from dimos.message_utils import image, pointcloud, time as message_time
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.unitree.go2 import connection as go2_conn
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.simulation.mujoco.config import MujocoConfig


@pytest.fixture
def stub_webrtc(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace UnitreeWebRTCConnection in go2.connection so the webrtc branch
    runs without dialing out."""
    stub = MagicMock(name="UnitreeWebRTCConnection")
    stub.return_value.camera_info_static = go2_conn._camera_info_static()
    monkeypatch.setattr(go2_conn, "UnitreeWebRTCConnection", stub)
    return stub


def test_make_connection_webrtc_forwards_aes_128_key(stub_webrtc: MagicMock) -> None:
    """Webrtc branch forwards aes_128_key as a kwarg to UnitreeWebRTCConnection."""
    cfg = ConnectionConfig(g=GlobalConfig())
    go2_conn.make_connection("192.168.123.161", cfg, aes_128_key="cafe" * 8)
    stub_webrtc.assert_called_once_with(
        "192.168.123.161",
        aes_128_key="cafe" * 8,
        velocity_api=False,
    )


def test_scoped_connection_settings_resolve_sources_and_nested_defaults(tmp_path: Path) -> None:
    path = tmp_path / "robot.json"
    path.write_text(
        json.dumps({"go2connection": {"mujoco": {"room": "custom", "steps_per_frame": 4}}})
    )
    parsed = BlueprintConfigParser(
        GO2Connection.blueprint(obstacle_avoidance=False, mujoco={"shadows": "off"})
    ).parse(
        ["--go2connection.mujoco.steps-per-frame", "9", "--go2connection.dimsim.headless=false"],
        config_path=path,
        environ={
            "GO2CONNECTION__AES_128_KEY": "test-aes-key",
            "GO2CONNECTION__MUJOCO__STEPS_PER_FRAME": "6",
            "GO2CONNECTION__MUJOCO__START_POS": "[2.5, -3]",
        },
    )
    config = ConnectionConfig(**parsed.module_kwargs("go2connection"))

    assert config.mujoco == MujocoConfig(
        room="custom", steps_per_frame=9, start_pos=(2.5, -3), shadows="off"
    )
    assert config.dimsim.headless is False
    assert config.aes_128_key == "test-aes-key"
    assert config.obstacle_avoidance is False
    assert "obstacle_avoidance" not in parsed.global_config_values()


def test_make_connection_replay_forwards_exit_on_complete() -> None:
    """--go2connection.replay-exit reaches ReplayConnection (the lazy `replay` opens no DB)."""
    conn = go2_conn.make_connection(
        None, ConnectionConfig(g=GlobalConfig(replay=True), replay_exit=True)
    )
    assert isinstance(conn, go2_conn.ReplayConnection)
    assert conn._exit_on_complete is True
    conn = go2_conn.make_connection("replay", ConnectionConfig(g=GlobalConfig(replay=True)))
    assert isinstance(conn, go2_conn.ReplayConnection)
    assert conn._exit_on_complete is False


def test_replay_exit_fires_after_the_last_stream_completes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Shutdown is requested exactly once, when every subscribed stream ends."""
    conn = go2_conn.ReplayConnection(exit_on_complete=True)
    fired = threading.Event()
    monkeypatch.setattr(conn, "_request_shutdown", fired.set)

    first: Subject[int] = Subject()
    second: Subject[int] = Subject()
    conn._tracked(first).subscribe()
    conn._tracked(second).subscribe()
    conn.seal_subscriptions()

    first.on_completed()
    assert not fired.wait(0.1)
    second.on_completed()
    assert fired.wait(2)


def test_replay_exit_waits_for_the_seal(monkeypatch: pytest.MonkeyPatch) -> None:
    """A stream completing before the next one subscribes must not fire.

    Subscriptions register one at a time, so a synchronously-completing first
    stream briefly matches the counters (1/1); only the seal says the set is
    complete.
    """
    conn = go2_conn.ReplayConnection(exit_on_complete=True)
    fired = threading.Event()
    monkeypatch.setattr(conn, "_request_shutdown", fired.set)

    first: Subject[int] = Subject()
    conn._tracked(first).subscribe()
    first.on_completed()  # counters now 1/1 with more subscriptions to come
    assert not fired.wait(0.1)

    second: Subject[int] = Subject()
    conn._tracked(second).subscribe()
    conn.seal_subscriptions()
    assert not fired.wait(0.1)

    second.on_completed()
    assert fired.wait(2)


def test_replay_exit_fires_at_the_seal_when_already_complete(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    conn = go2_conn.ReplayConnection(exit_on_complete=True)
    fired = threading.Event()
    monkeypatch.setattr(conn, "_request_shutdown", fired.set)

    only: Subject[int] = Subject()
    conn._tracked(only).subscribe()
    only.on_completed()
    conn.seal_subscriptions()
    assert fired.wait(2)


def test_replay_exit_off_leaves_streams_untouched() -> None:
    source: Subject[int] = Subject()
    conn = go2_conn.ReplayConnection()
    assert conn._tracked(source) is source


def test_odom_to_tf_unprefixed_by_default() -> None:
    odom = PoseStamped(
        Header(message_time.time_from_seconds(1.0), "world"),
        Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
    )
    base, camera_link, camera_optical = GO2Connection._odom_to_tf(odom)
    assert (base.header.frame_id, base.child_frame_id) == ("world", "base_link")
    assert (camera_link.header.frame_id, camera_link.child_frame_id) == ("base_link", "camera_link")
    assert (camera_optical.header.frame_id, camera_optical.child_frame_id) == (
        "camera_link",
        "camera_optical",
    )


@pytest.fixture
def connection(
    stub_webrtc: MagicMock, mocker: MockerFixture
) -> Iterator[Callable[[bool], GO2Connection]]:
    """Build GO2Connections with the tf and odom ports stubbed, and stop them after."""
    built: list[GO2Connection] = []

    def build(publish_tf: bool) -> GO2Connection:
        conn = GO2Connection(
            g=GlobalConfig(robot_ip="127.0.0.1"),
            publish_tf=publish_tf,
            odom_frame_id="go2_odom",
        )
        mocker.patch.object(conn, "tf")
        mocker.patch.object(conn, "odom")
        built.append(conn)
        return conn

    yield build
    for conn in built:
        conn.stop()


def test_publish_tf_off_keeps_odometry_on_its_port(
    connection: Callable[[bool], GO2Connection],
) -> None:
    """Turning tf off hands the base_link edge to another publisher, not the odom port."""
    conn = connection(publish_tf=False)
    conn._publish_tf(
        PoseStamped(
            Header(message_time.time_from_seconds(1.0), "ignored"),
            Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    assert conn.tf.publish.call_count == 0
    assert conn.odom.publish.call_count == 1


def test_publish_tf_on_by_default(connection: Callable[[bool], GO2Connection]) -> None:
    conn = connection(publish_tf=True)
    conn._publish_tf(
        PoseStamped(
            Header(message_time.time_from_seconds(1.0), "ignored"),
            Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    assert conn.tf.publish.call_count == 1
    assert conn.odom.publish.call_count == 1


class _StopLoopError(Exception):
    pass


def test_camera_info_is_restamped_on_each_publish(
    connection: Callable[[bool], GO2Connection], mocker: MockerFixture
) -> None:
    """A frozen stamp collapses a recording's camera_info onto one instant before the run."""
    conn = connection(publish_tf=True)
    mocker.patch.object(conn, "camera_info")
    conn.camera_info_static = go2_conn._camera_info_static()
    conn.camera_info_static.header.stamp = Time(1, 0)

    def sleep(_seconds: float) -> None:
        if conn.camera_info.publish.call_count >= 2:
            raise _StopLoopError

    mocker.patch.object(go2_conn.time, "sleep", side_effect=sleep)
    before = time.time()
    with pytest.raises(_StopLoopError):
        conn.publish_camera_info()

    published = [call.args[0] for call in conn.camera_info.publish.call_args_list]
    assert [message_time.to_seconds(info.header.stamp) >= before for info in published] == [
        True,
        True,
    ]
    assert [info.header.frame_id for info in published] == [
        conn.camera_info_static.header.frame_id
    ] * 2
    assert conn.camera_info_static.header.stamp == Time(1, 0)


def test_odom_to_tf_prefixed() -> None:
    """.namespace() sets frame_id_prefix: robot-local frames get prefixed, the
    odom parent frame stays global so all robots hang off one tree root."""
    odom = PoseStamped(
        Header(message_time.time_from_seconds(1.0), "world"),
        Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
    )
    base, camera_link, camera_optical = GO2Connection._odom_to_tf(odom, prefix="robot0")
    assert (base.header.frame_id, base.child_frame_id) == ("world", "robot0/base_link")
    assert (camera_link.header.frame_id, camera_link.child_frame_id) == (
        "robot0/base_link",
        "robot0/camera_link",
    )
    assert (camera_optical.header.frame_id, camera_optical.child_frame_id) == (
        "robot0/camera_link",
        "robot0/camera_optical",
    )


def test_driver_odometry_boundary_publishes_generated_tf_and_pose(connection):
    conn = connection(publish_tf=True)
    conn._publish_tf(
        PoseStamped(
            Header(message_time.time_from_seconds(0.0), "source"),
            Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    pose = conn.odom.publish.call_args.args[0]
    transforms = conn.tf.publish.call_args.args[0]
    assert type(pose) is PoseStamped
    assert type(transforms) is TFMessage
    assert pose.header == Header(Time(0, 0), "go2_odom")
    assert [(t.header.frame_id, t.child_frame_id) for t in transforms.transforms] == [
        ("go2_odom", "base_link"),
        ("base_link", "camera_link"),
        ("camera_link", "camera_optical"),
    ]
    assert all(t.header.stamp == Time(0, 0) for t in transforms.transforms)


def test_generated_command_is_adapted_only_at_driver_boundary(connection):
    conn = connection(publish_tf=False)
    conn.move(Twist(Vector3(1.0, 2.0, 3.0), Vector3(4.0, 5.0, 6.0)), 0.5)
    command, duration = conn.connection.move.call_args.args
    assert command.linear.x == 1.0
    assert command.linear.y == 2.0
    assert command.linear.z == 3.0
    assert command.angular.x == 4.0
    assert command.angular.y == 5.0
    assert command.angular.z == 6.0
    assert duration == 0.5


@pytest.mark.parametrize(
    "format,encoding,dtype",
    [
        ("rgb8", "rgb8", np.uint8),
        ("16UC1", "16UC1", np.uint16),
    ],
)
def test_driver_image_boundary_copies_pixels_and_prefixes_frame(
    connection, mocker, format, encoding, dtype
):
    conn = connection(publish_tf=False)
    conn.config.frame_id_prefix = "robot0"
    mocker.patch.object(conn, "color_image")
    shape = (2, 3, 3) if format == "rgb8" else (2, 3)
    pixels = np.arange(np.prod(shape), dtype=dtype).reshape(shape)
    source = image.from_array(
        pixels, encoding=encoding, header=Header(Time(0, 0), "camera_optical")
    )
    conn._publish_image(source)
    message = conn.color_image.publish.call_args.args[0]
    assert message.header == Header(Time(0, 0), "robot0/camera_optical")
    assert message.encoding == encoding
    np.testing.assert_array_equal(image.view(message), pixels)
    pixels[:] = 0
    assert image.view(message).max() > 0
    assert source.header.frame_id == "camera_optical"


class MemqRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


def test_go2_default_ports_deliver_cdr_without_robot(stub_webrtc, mocker, monkeypatch):
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    mocker.patch.object(LCMRPC, "start")
    mocker.patch.object(LCMRPC, "serve_module_rpc")
    monkeypatch.setattr(transport_module, "LCM", partial(LCM, url="memq://"))
    config = GlobalConfig(transport="lcm", robot_ip="127.0.0.1")
    received = {}
    condition = threading.Condition()

    def on_message(name, value):
        with condition:
            received[name] = value
            condition.notify_all()

    with ExitStack() as cleanup:
        conn = GO2Connection(g=config, rpc_transport=MemqRPC)
        cleanup.callback(conn.stop)
        for name in ("tf", "odom", "color_image", "lidar"):
            port = conn.outputs[name]
            transport = make_transport(name, port.type, g=config)
            cleanup.callback(transport.stop)
            conn.set_transport(name, transport)
            cleanup.callback(transport.subscribe(partial(on_message, name)))
        source = image.from_array(
            np.arange(18, dtype=np.uint8).reshape(2, 3, 3),
            encoding="rgb8",
            header=Header(Time(0, 0), "camera_optical"),
        )
        conn._publish_image(source)
        conn._publish_tf(
            PoseStamped(
                Header(message_time.time_from_seconds(0.0), "world"),
                Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
            )
        )
        cloud = pointcloud.from_xyz(np.array([[1.0, 2.0, 3.0]]), header=Header(Time(0, 0), ""))
        conn.lidar.publish(cloud)
        with condition:
            assert condition.wait_for(lambda: len(received) == 4, timeout=5)
        assert type(received["odom"]) is PoseStamped
        assert type(received["tf"]) is TFMessage
        assert type(received["lidar"]) is PointCloud2
        assert received["color_image"].encoding == "rgb8"
        assert received["odom"].header.stamp == Time(0, 0)
        assert len(received["tf"].transforms) == 3
        np.testing.assert_array_equal(
            image.view(received["color_image"]), np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        )
        np.testing.assert_array_equal(pointcloud.view(received["lidar"])["z"], [[3.0]])
