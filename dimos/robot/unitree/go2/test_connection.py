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

import asyncio
from collections.abc import Callable, Iterator
from contextlib import ExitStack
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from unittest.mock import MagicMock

import numpy as np
import pytest
from pytest_mock import MockerFixture
from reactivex import Subject

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.unitree.go2 import connection as go2_conn
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection, go2_connection
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.robot.unitree.go2.controllers.locomotion.policy import OnnxGo2Policy
from dimos.robot.unitree.go2.simulation.device import Go2SimDevice
from dimos.robot.unitree.go2.simulation.model import go2_definition
from dimos.sim2.client import SimRobotClient
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.module import SimulationModule
from dimos.sim2.world.scene import scene_path


@pytest.fixture
def stub_webrtc(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace UnitreeWebRTCConnection in go2.connection so the webrtc branch
    runs without dialing out."""
    stub = MagicMock(name="UnitreeWebRTCConnection")
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
        json.dumps({"go2connection": {"policy": "/models/walk.onnx", "spawn": "entry"}})
    )
    parsed = BlueprintConfigParser(GO2Connection.blueprint(obstacle_avoidance=False)).parse(
        ["--go2connection.spawn", "hall", "--go2connection.dimsim.headless=false"],
        config_path=path,
        environ={
            "GO2CONNECTION__AES_128_KEY": "test-aes-key",
            "GO2CONNECTION__SPAWN": "door",
        },
    )
    config = ConnectionConfig(**parsed.module_kwargs("go2connection"))

    assert config.policy == Path("/models/walk.onnx")
    assert config.spawn == "hall"
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
    odom = PoseStamped(ts=1.0, frame_id="world")
    base, camera_link, camera_optical = GO2Connection._odom_to_tf(odom)
    assert (base.frame_id, base.child_frame_id) == ("world", "base_link")
    assert (camera_link.frame_id, camera_link.child_frame_id) == ("base_link", "camera_link")
    assert (camera_optical.frame_id, camera_optical.child_frame_id) == (
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
    conn._publish_tf(PoseStamped(ts=1.0, frame_id="ignored"))
    assert conn.tf.publish.call_count == 0
    assert conn.odom.publish.call_count == 1


def test_publish_tf_on_by_default(connection: Callable[[bool], GO2Connection]) -> None:
    conn = connection(publish_tf=True)
    conn._publish_tf(PoseStamped(ts=1.0, frame_id="ignored"))
    assert conn.tf.publish.call_count == 1
    assert conn.odom.publish.call_count == 1


def test_camera_info_is_restamped_on_each_publish(
    connection: Callable[[bool], GO2Connection], mocker: MockerFixture
) -> None:
    """A frozen stamp collapses a recording's camera_info onto one instant before the run."""
    conn = connection(publish_tf=True)
    conn.camera_info = MagicMock()
    conn.camera_info_static = CameraInfo(frame_id="camera_optical", ts=1.0)

    def wait(_seconds: float) -> None:
        if conn.camera_info.publish.call_count >= 2:
            conn._stop_event.set()

    mocker.patch.object(conn._stop_event, "wait", side_effect=wait)
    before = time.time()
    conn.publish_camera_info()

    published = [call.args[0] for call in conn.camera_info.publish.call_args_list]
    assert [info.ts >= before for info in published] == [True, True]
    assert [info.frame_id for info in published] == [conn.camera_info_static.frame_id] * 2
    assert conn.camera_info_static.ts == 1.0


def test_odom_to_tf_prefixed() -> None:
    """.namespace() sets frame_id_prefix: robot-local frames get prefixed, the
    odom parent frame stays global so all robots hang off one tree root."""
    odom = PoseStamped(ts=1.0, frame_id="world")
    base, camera_link, camera_optical = GO2Connection._odom_to_tf(odom, prefix="robot0")
    assert (base.frame_id, base.child_frame_id) == ("world", "robot0/base_link")
    assert (camera_link.frame_id, camera_link.child_frame_id) == (
        "robot0/base_link",
        "robot0/camera_link",
    )
    assert (camera_optical.frame_id, camera_optical.child_frame_id) == (
        "robot0/camera_link",
        "robot0/camera_optical",
    )


def test_mujoco_selects_internal_device_not_another_world():
    cfg = ConnectionConfig(g=GlobalConfig(simulation="mujoco"))
    device = go2_conn.make_connection(None, cfg)
    assert isinstance(device, Go2SimDevice)
    with pytest.raises(RuntimeError, match="collected sim2 world"):
        device.start()


def test_queued_sim_velocity_does_not_get_a_new_lifetime(mocker):
    with ExitStack() as stack:
        connection = GO2Connection(g=GlobalConfig(simulation="mujoco"))
        stack.callback(connection.stop)
        move = mocker.patch.object(connection.connection, "move")
        mocker.patch("dimos.robot.unitree.go2.connection.time.time", return_value=100.0)
        values = {"go2/base/vx": 0.2, "go2/base/vy": 0.0, "go2/base/wz": 0.0}
        connection.write(values, source_ts=100.0 - CMD_VEL_TIMEOUT / 2)
        assert move.call_args.args[1] == pytest.approx(CMD_VEL_TIMEOUT / 2)
        with pytest.raises(TimeoutError, match="stale"):
            connection.write(values, source_ts=100.0 - CMD_VEL_TIMEOUT * 2)
        assert move.call_count == 1


@pytest.mark.asyncio
async def test_velocity_deadman_preserves_go2_timeout_and_halts(mocker):
    with ExitStack() as stack:
        connection = GO2Connection(g=GlobalConfig(simulation="mujoco"))
        stack.callback(connection.stop)
        mocker.patch.object(connection, "_connected", True)
        mocker.patch.object(connection.connection, "move")
        halt = mocker.patch.object(connection.connection, "stop_movement")
        timer = mocker.patch.object(asyncio.get_running_loop(), "call_later")
        connection._apply({"go2/base/vx": 0.2, "go2/base/vy": 0.0, "go2/base/wz": 0.0})
        assert timer.call_args.args[0] == CMD_VEL_TIMEOUT
        halt.assert_not_called()
        timer.call_args.args[1]()
        halt.assert_called_once()
        assert connection.status().deadman_fired


def test_policy_joint_order_is_bound_to_the_physical_device(mocker):
    names = tuple(j.name for j in go2_definition(camera=False, lidar=False).joints)
    policy = mocker.Mock(spec=OnnxGo2Policy, joint_names=names[::-1])
    mocker.patch("dimos.robot.unitree.go2.simulation.device.load_policy", return_value=policy)
    thread = mocker.patch("dimos.robot.unitree.go2.simulation.device.Thread")
    thread.return_value.is_alive.return_value = False
    device = Go2SimDevice()
    client = mocker.Mock()
    client.descriptor.joint_names = names
    mocker.patch.object(device, "_client", client)
    try:
        device.start()
        state = np.arange(12)
        policy_state = state[device._policy_order]
        assert policy_state.tolist() == list(reversed(range(12)))
        assert policy_state[device._motor_order].tolist() == state.tolist()
    finally:
        device.stop()


def test_reset_during_inference_discards_result_and_restarts_policy(mocker):
    names = tuple(j.name for j in go2_definition(camera=False, lidar=False).joints)
    policy = mocker.Mock(spec=OnnxGo2Policy, joint_names=names, kp=np.ones(12), kd=np.ones(12))
    policy.act.return_value = np.zeros(12)
    mocker.patch("dimos.robot.unitree.go2.simulation.device.load_policy", return_value=policy)
    thread = mocker.patch("dimos.robot.unitree.go2.simulation.device.Thread")
    thread.return_value.is_alive.return_value = False
    frame = StateFrame(
        FrameMetadata(1, 0, 1, 1, 0.005),
        {
            "position": np.zeros(12),
            "velocity": np.zeros(12),
            "imu_gyroscope": np.zeros(3),
            "imu_quaternion": np.array([1, 0, 0, 0]),
            "root_position": np.zeros(3),
            "root_quaternion": np.array([1, 0, 0, 0]),
            "wall_time": np.array([123.0]),
        },
    )
    reset = replace(frame, metadata=replace(frame.metadata, episode_id=1))
    client = mocker.Mock(spec=SimRobotClient, descriptor=mocker.Mock(joint_names=names))
    client.read.side_effect = [frame, reset, reset]
    with ExitStack() as stack:
        device = Go2SimDevice()
        stack.callback(device.stop)
        mocker.patch.object(device, "_client", client)

        def write(*args, **kwargs):
            if client.write_motors.call_count == 1:
                raise RuntimeError("sim2 robot was reset")
            device._stop.set()

        client.write_motors.side_effect = write
        device.start()
        device._run()

        assert device._error is None
        assert policy.reset.call_count == 2
        assert client.write_motors.call_count == 2
        assert client.write_motors.call_args.kwargs["observed"] is reset
        assert client.write_motors.call_args.args[0].timestamp == 123.0


@pytest.mark.parametrize("duration,velocity", [(-1, 0), (1, float("nan")), (float("inf"), 1)])
def test_sim_device_rejects_invalid_movement(duration, velocity):
    with pytest.raises(ValueError, match="finite"):
        Go2SimDevice().move(Twist(Vector3(velocity, 0, 0)), duration)


@pytest.fixture
def walking_stack():
    blueprint = go2_connection()
    parsed = BlueprintConfigParser(blueprint).parse(
        [],
        environ={},
        global_overrides={
            "simulation": "mujoco",
            "transport": "zenoh",
            "viewer": "none",
            "scene_package": str(scene_path(None, "logistics.xml")),
            "n_workers": 2,
        },
    )
    with ExitStack() as stack:
        app = ModuleCoordinator.build(blueprint, parsed)
        stack.callback(app.stop)
        yield app


@pytest.mark.self_hosted
def test_existing_policy_walks_through_public_connection(walking_stack):
    world = walking_stack.get_instance(SimulationModule)
    robot = walking_stack.get_instance(GO2Connection)
    start = world.status()
    positions = []
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        robot.move(Twist(Vector3(0.35, 0, 0)))
        positions.append(world.status()["robots"][GO2Connection.name])
        time.sleep(0.1)  # The commanded input rate.
    robot.stop_movement()
    end = world.status()
    samples = np.asarray(positions)
    assert end["error"] is None
    assert end["steps"] - start["steps"] > 3000
    assert np.isfinite(samples).all()
    assert samples[:, 2].min() > 0.2
    assert samples[-1, 0] - samples[0, 0] > 0.8
    assert robot.status().last_error is None
