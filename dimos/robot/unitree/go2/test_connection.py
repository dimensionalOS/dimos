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
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.sensor_msgs.msg import CameraInfo, RegionOfInterest
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
from pytest_mock import MockerFixture
from reactivex import Subject

from dimos.core.global_config import GlobalConfig
from dimos.msgs.time import to_nanoseconds
from dimos.robot.unitree.go2 import connection as go2_conn
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection


@pytest.fixture
def stub_webrtc(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace UnitreeWebRTCConnection in go2.connection so the webrtc branch
    runs without dialing out."""
    stub = MagicMock(name="UnitreeWebRTCConnection")
    monkeypatch.setattr(go2_conn, "UnitreeWebRTCConnection", stub)
    return stub


def test_make_connection_webrtc_forwards_aes_128_key(stub_webrtc: MagicMock) -> None:
    """Webrtc branch forwards aes_128_key as a kwarg to UnitreeWebRTCConnection."""
    cfg = SimpleNamespace(unitree_connection_type="webrtc")
    go2_conn.make_connection("192.168.123.161", cfg, aes_128_key="cafe" * 8)
    stub_webrtc.assert_called_once_with(
        "192.168.123.161",
        aes_128_key="cafe" * 8,
        velocity_api=False,
    )


def test_make_connection_replay_forwards_exit_on_complete() -> None:
    """--replay-exit reaches ReplayConnection (the lazy `replay` opens no DB)."""
    conn = go2_conn.make_connection(None, GlobalConfig(replay=True, replay_exit=True))
    assert isinstance(conn, go2_conn.ReplayConnection)
    assert conn._exit_on_complete is True
    conn = go2_conn.make_connection("replay", GlobalConfig(replay=True))
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


def test_connection_config_aes_key_defaults_from_global_config() -> None:
    """ConnectionConfig.aes_128_key defaults from GlobalConfig.unitree_aes_128_key."""
    g = GlobalConfig(robot_ip="127.0.0.1", unitree_aes_128_key="dd" * 16)
    assert ConnectionConfig(g=g).aes_128_key == "dd" * 16


def test_odom_to_tf_unprefixed_by_default() -> None:
    odom = PoseStamped(
        header=Header(stamp=Time(sec=1, nanosec=123456789), frame_id="world"),
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
    )
    base, camera_link, camera_optical = GO2Connection._odom_to_tf(odom)
    assert (base.header.frame_id, base.child_frame_id) == ("world", "base_link")
    assert (camera_link.header.frame_id, camera_link.child_frame_id) == ("base_link", "camera_link")
    assert (camera_optical.header.frame_id, camera_optical.child_frame_id) == (
        "camera_link",
        "camera_optical",
    )


@pytest.fixture
def connection(stub_webrtc: MagicMock) -> Iterator[Callable[[bool], GO2Connection]]:
    """Build GO2Connections with the tf and odom ports stubbed, and stop them after."""
    built: list[GO2Connection] = []

    def build(publish_tf: bool) -> GO2Connection:
        conn = GO2Connection(
            g=GlobalConfig(robot_ip="127.0.0.1"),
            publish_tf=publish_tf,
            odom_frame_id="go2_odom",
        )
        conn.tf = MagicMock()
        conn.odom = MagicMock()
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
            header=Header(stamp=Time(sec=1, nanosec=0), frame_id="ignored"),
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
    )
    assert conn.tf.publish.call_count == 0
    assert conn.odom.publish.call_count == 1


def test_publish_tf_on_by_default(connection: Callable[[bool], GO2Connection]) -> None:
    conn = connection(publish_tf=True)
    conn._publish_tf(
        PoseStamped(
            header=Header(stamp=Time(sec=1, nanosec=0), frame_id="ignored"),
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
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
    conn.camera_info = MagicMock()
    conn.camera_info_static = CameraInfo(
        header=Header(frame_id="camera_optical", stamp=Time(sec=1, nanosec=0)),
        height=0,
        width=0,
        distortion_model="",
        d=np.array([], dtype=np.float64),
        k=np.zeros(9, dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )

    def sleep(_seconds: float) -> None:
        if conn.camera_info.publish.call_count >= 2:
            raise _StopLoopError

    mocker.patch.object(go2_conn.time, "sleep", side_effect=sleep)
    mocker.patch.object(
        go2_conn.time, "time_ns", side_effect=[1700000000123456789, 1700000000123456790]
    )
    with pytest.raises(_StopLoopError):
        conn.publish_camera_info()

    published = [call.args[0] for call in conn.camera_info.publish.call_args_list]
    assert [to_nanoseconds(info.header.stamp) for info in published] == [
        1700000000123456789,
        1700000000123456790,
    ]
    assert [info.header.frame_id for info in published] == [
        conn.camera_info_static.header.frame_id
    ] * 2
    assert conn.camera_info_static.header.stamp == Time(sec=1, nanosec=0)


def test_odom_to_tf_prefixed() -> None:
    """.namespace() sets frame_id_prefix: robot-local frames get prefixed, the
    odom parent frame stays global so all robots hang off one tree root."""
    odom = PoseStamped(
        header=Header(stamp=Time(sec=1, nanosec=123456789), frame_id="world"),
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
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


def test_published_tf_copies_source_pose_and_keeps_exact_stamp(connection):
    conn = connection(publish_tf=True)
    source = PoseStamped(
        header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="device_odom"),
        pose=Pose(
            position=Point(x=1.25, y=-2.5, z=0.3),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    conn._publish_tf(source)
    message = conn.odom.publish.call_args.args[0]
    tf = cdr_decode(cdr_encode(conn.tf.publish.call_args.args[0]), TFMessage)
    assert source.header.frame_id == "device_odom"
    assert message.header.frame_id == "go2_odom"
    assert all(edge.header.stamp == source.header.stamp for edge in tf.transforms)
    assert tf.transforms[0].header.frame_id == "go2_odom"
    assert tf.transforms[0].transform.translation.x == 1.25
    message.pose.position.x = 99
    assert source.pose.position.x == 1.25
