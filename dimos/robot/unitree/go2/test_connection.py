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
from contextlib import closing
import json
from pathlib import Path
import sqlite3
import threading
import time
from unittest.mock import MagicMock

from mcap.writer import Writer
import pytest
from pytest_mock import MockerFixture
from reactivex import Subject
from reactivex.disposable import Disposable
from reactivex.testing import TestScheduler

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig, global_config
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.unitree.go2 import connection as go2_conn
from dimos.robot.unitree.go2.connection import ConnectionConfig, GO2Connection
from dimos.simulation.mujoco.config import MujocoConfig


@pytest.fixture
def stub_webrtc(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """Replace UnitreeWebRTCConnection in go2.connection so the webrtc branch
    runs without dialing out."""
    stub = MagicMock(name="UnitreeWebRTCConnection")
    monkeypatch.setattr(go2_conn, "UnitreeWebRTCConnection", stub)
    return stub


def test_make_connection_webrtc_forwards_aes_128_key(stub_webrtc: MagicMock) -> None:
    """Webrtc branch forwards aes_128_key as a kwarg to UnitreeWebRTCConnection."""
    cfg = ConnectionConfig(ip="192.168.123.161", g=GlobalConfig(replay=False, simulation=""))
    go2_conn.make_connection("192.168.123.161", cfg, aes_128_key="cafe" * 8)
    stub_webrtc.assert_called_once_with(
        "192.168.123.161",
        aes_128_key="cafe" * 8,
        velocity_api=False,
    )


@pytest.mark.parametrize("ip", [None, "", "   "])
def test_make_connection_without_an_ip_names_the_ways_out(ip: str | None) -> None:
    cfg = ConnectionConfig(ip="192.0.2.1", g=GlobalConfig(replay=False, simulation=""))
    with pytest.raises(ValueError, match="--robot-ip.*--replay.*--simulation"):
        go2_conn.make_connection(ip, cfg)


@pytest.mark.parametrize("ip", [None, "", "   "])
def test_config_rejects_missing_and_blank_hardware_ips(ip: str | None) -> None:
    with pytest.raises(ValueError, match="--robot-ip.*--replay.*--simulation"):
        ConnectionConfig(g=GlobalConfig(robot_ip=ip, replay=False, simulation=""))


@pytest.mark.parametrize(
    "defaults",
    [
        {"replay": True},
        {"simulation": "mujoco"},
        {"simulation": "dimsim", "transport": "lcm"},
        {"simulation": "true"},
    ],
)
def test_blueprint_offline_defaults_allow_no_ip(defaults, monkeypatch) -> None:
    monkeypatch.setattr(global_config, "robot_ip", None)
    monkeypatch.setattr(global_config, "replay", False)
    monkeypatch.setattr(global_config, "simulation", "")

    parsed = BlueprintConfigParser(GO2Connection.blueprint().global_config(**defaults)).parse(
        environ={}
    )
    cfg = ConnectionConfig(
        g=GlobalConfig(**parsed.global_config_values()), **parsed.module_kwargs("go2connection")
    )

    assert cfg.ip is None
    assert go2_conn.connection_type(replay=cfg.g.replay, simulation=cfg.g.simulation) != "webrtc"


@pytest.mark.parametrize("ip", ["replay", "mock", "fake", "mujoco"])
@pytest.mark.parametrize("simulation", ["", "dimsim"])
def test_config_preserves_offline_ip_aliases(ip: str, simulation: str) -> None:
    cfg = ConnectionConfig(
        ip=ip, g=GlobalConfig(replay=False, simulation=simulation, transport="zenoh")
    )
    assert cfg.ip == ip


@pytest.mark.parametrize(
    ("cli", "environ", "pinned"),
    [
        (["--robot-ip", "192.0.2.1"], {}, {}),
        (["--go2connection.ip", "192.0.2.1"], {}, {}),
        ([], {"ROBOT_IP": "192.0.2.1"}, {}),
        ([], {"GO2CONNECTION__IP": "192.0.2.1"}, {}),
        ([], {}, {"ip": "192.0.2.1"}),
        (["--go2connection.ip", "192.0.2.1"], {"ROBOT_IP": "192.0.2.2"}, {}),
    ],
)
def test_ip_sources_validate_without_process_global_ip(cli, environ, pinned, monkeypatch) -> None:
    monkeypatch.setattr(global_config, "robot_ip", None)
    monkeypatch.setattr(global_config, "replay", False)
    monkeypatch.setattr(global_config, "simulation", "")

    parsed = BlueprintConfigParser(GO2Connection.blueprint(**pinned)).parse(cli, environ=environ)
    cfg = ConnectionConfig(
        g=GlobalConfig(**parsed.global_config_values()), **parsed.module_kwargs("go2connection")
    )

    assert cfg.ip == "192.0.2.1"


def test_missing_ip_cannot_be_hidden_by_stale_process_globals(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.1")
    monkeypatch.setattr(global_config, "replay", True)
    with pytest.raises(BlueprintConfigError, match="go2connection:.*No robot IP"):
        BlueprintConfigParser(GO2Connection.blueprint()).parse(environ={})


def test_namespaced_ip_override_validates() -> None:
    parsed = BlueprintConfigParser(GO2Connection.blueprint().namespace("robot0")).parse(
        ["--robot0/go2connection.ip", "192.0.2.1"], environ={}
    )
    assert parsed.module_kwargs("robot0/go2connection")["ip"] == "192.0.2.1"


def test_hosted_connection_validates_with_the_host_environment() -> None:
    blueprint = GO2Connection.blueprint()
    root = BlueprintConfigParser(blueprint.hosted(host="robot")).parse(
        ["--go2connection.camera", "false"], environ={}
    )

    host = BlueprintConfigParser(blueprint).parse(
        overrides=root.file_and_cli, environ={"ROBOT_IP": "192.0.2.1"}
    )
    assert host.global_config_values()["robot_ip"] == "192.0.2.1"
    assert host.module_kwargs("go2connection")["camera"] is False
    with pytest.raises(BlueprintConfigError, match="No robot IP"):
        BlueprintConfigParser(blueprint).parse(overrides=root.file_and_cli, environ={})


def test_python_build_rejects_missing_ip_before_starting_resources(monkeypatch, mocker) -> None:
    for field, value in {"robot_ip": None, "replay": False, "simulation": ""}.items():
        monkeypatch.setattr(global_config, field, value)
    materialize = mocker.patch("dimos.core.coordination.module_coordinator._materialize_transports")
    start = mocker.patch.object(ModuleCoordinator, "start")

    with pytest.raises(BlueprintConfigError, match="go2connection:.*No robot IP"):
        ModuleCoordinator.build(GO2Connection.blueprint())

    materialize.assert_not_called()
    start.assert_not_called()


@pytest.mark.parametrize("headless", [True, False])
def test_dimsim_rejects_zenoh_before_starting_resources(headless, monkeypatch, mocker) -> None:
    for field, value in {"simulation": "dimsim", "replay": False, "transport": "zenoh"}.items():
        monkeypatch.setattr(global_config, field, value)
    materialize = mocker.patch("dimos.core.coordination.module_coordinator._materialize_transports")
    start = mocker.patch.object(ModuleCoordinator, "start")

    with pytest.raises(
        BlueprintConfigError, match="DimSim publishes over LCM only.*--transport lcm"
    ):
        ModuleCoordinator.build(GO2Connection.blueprint(dimsim={"headless": headless}))

    materialize.assert_not_called()
    start.assert_not_called()


@pytest.mark.parametrize(
    "overrides",
    [{"transport": "lcm"}, {"transport": "zenoh", "replay": True}],
)
def test_dimsim_allows_lcm_or_replay(overrides) -> None:
    parsed = BlueprintConfigParser(GO2Connection.blueprint()).parse(
        environ={}, global_overrides={"simulation": "dimsim", **overrides}
    )

    assert parsed.global_config_values()["transport"] == overrides["transport"]


def test_scoped_connection_settings_resolve_sources_and_nested_defaults(tmp_path: Path) -> None:
    path = tmp_path / "robot.json"
    path.write_text(
        json.dumps(
            {
                "g": {"robot_ip": "192.0.2.1"},
                "go2connection": {"mujoco": {"room": "custom", "steps_per_frame": 4}},
            }
        )
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
    config = ConnectionConfig(
        g=GlobalConfig(**parsed.global_config_values()), **parsed.module_kwargs("go2connection")
    )

    assert config.ip == "192.0.2.1"
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


def test_replay_logs_completion_without_requesting_shutdown(mocker) -> None:
    conn = go2_conn.ReplayConnection(dataset="test-recording.db")
    shutdown = mocker.patch.object(conn, "_request_shutdown")
    log = mocker.patch("dimos.memory.replay_progress.logger")
    with Subject() as first, Subject() as second:
        with conn._tracked(first).subscribe(), conn._tracked(second).subscribe():
            conn.seal_subscriptions()
            first.on_completed()
            log.info.assert_not_called()
            second.on_completed()
            conn.seal_subscriptions()
            log.info.assert_called_once_with("Replay finished", source="test-recording.db")
    shutdown.assert_not_called()


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


def test_stop_disposes_stream_readers_before_closing_connection(connection, mocker) -> None:
    conn = connection(publish_tf=False)
    subscription = conn.register_disposable(Disposable())
    disposed_at_close = []
    mocker.patch.object(
        conn.connection,
        "stop",
        side_effect=lambda: disposed_at_close.append(subscription.is_disposed),
    )

    conn.stop()

    assert disposed_at_close == [True]


class _StopLoopError(Exception):
    pass


def test_camera_info_is_restamped_on_each_publish(
    connection: Callable[[bool], GO2Connection], mocker: MockerFixture
) -> None:
    """A frozen stamp collapses a recording's camera_info onto one instant before the run."""
    conn = connection(publish_tf=True)
    conn.camera_info = MagicMock()
    conn.camera_info_static = CameraInfo(frame_id="camera_optical", ts=1.0)

    def sleep(_seconds: float) -> None:
        if conn.camera_info.publish.call_count >= 2:
            raise _StopLoopError

    mocker.patch.object(conn._camera_info_stop, "wait", side_effect=sleep)
    before = time.time()
    with pytest.raises(_StopLoopError):
        conn.publish_camera_info()

    published = [call.args[0] for call in conn.camera_info.publish.call_args_list]
    assert [info.ts >= before for info in published] == [True, True]
    assert [info.frame_id for info in published] == [conn.camera_info_static.frame_id] * 2
    assert conn.camera_info_static.ts == 1.0


def test_stop_joins_camera_info_publisher(connection, mocker, monkeypatch) -> None:
    conn = connection(publish_tf=False)
    published = threading.Event()
    mocker.patch.object(conn.camera_info, "publish", side_effect=lambda _: published.set())
    thread = threading.Thread(target=conn.publish_camera_info, daemon=True)
    monkeypatch.setattr(conn, "_camera_info_thread", thread)
    thread.start()
    try:
        assert published.wait(timeout=2)

        conn.stop()

        assert not thread.is_alive()
    finally:
        conn.stop()
        thread.join(timeout=2)


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


@pytest.mark.parametrize("dataset_kind", ["sqlite", "named-sqlite", "mcap"])
def test_replay_opens_recording_and_emits_typed_odometry(
    tmp_path, monkeypatch, mocker, dataset_kind
):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    values = [
        PoseStamped(ts=12.5, frame_id="world"),
        PoseStamped(ts=13.5, frame_id="world"),
        PoseStamped(ts=14.0, frame_id="world"),  # beyond seek + duration
    ]
    if dataset_kind == "mcap":
        path = tmp_path / "native.mcap"
        with path.open("wb") as output:
            writer = Writer(output)
            writer.start(profile="dimos")
            channel = writer.register_channel(
                topic="odom",
                message_encoding="lcm",
                schema_id=0,
                metadata={
                    "dimos.payload_type": "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped",
                    "dimos.observation_time": "publish_time",
                },
            )
            for value in values:
                writer.add_message(
                    channel_id=channel,
                    log_time=20_000_000_000,
                    publish_time=int(value.ts * 1e9),
                    data=value.lcm_encode(),
                )
            writer.finish()
    else:
        path = tmp_path / "recording.db"
        with SqliteStore(path=str(path)) as store:
            stream = store.stream("odom", PoseStamped)
            for value in values:
                stream.append(value, ts=value.ts)

    dataset = str(path)
    if dataset_kind == "named-sqlite":
        monkeypatch.chdir(tmp_path)
        dataset = "recording"
    scheduler = TestScheduler()
    received = []
    errors = []
    completed = []
    with go2_conn.ReplayConnection(dataset=dataset, seek=1.0, duration=0.25) as conn:
        with conn.odom_stream().subscribe(
            received.append,
            errors.append,
            lambda: completed.append(True),
            scheduler=scheduler,
        ):
            conn.seal_subscriptions()
            scheduler.advance_by(1.0)
        assert errors == []
        assert completed == [True]
        assert [value.lcm_encode() for value in received] == [values[1].lcm_encode()]
    assert log.info.call_args_list == [
        mocker.call(
            "Replay started", source=dataset, path=str(path), duration_s=0.0, speed=1.0, loop=False
        ),
        mocker.call("Replay finished", source=dataset),
    ]


def test_replay_rejects_legacy_dds_odometry(tmp_path: Path) -> None:
    """A legacy Go2 DDS recording holds nav_msgs.Odometry, which GO2Connection cannot publish."""
    path = tmp_path / "legacy.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start()
        channel = writer.register_channel(
            topic="rt/utlidar/robot_odom", message_encoding="cdr", schema_id=0
        )
        writer.add_message(channel_id=channel, log_time=1, publish_time=1, data=b"")
        writer.finish()

    with go2_conn.ReplayConnection(dataset=str(path)) as conn:
        with pytest.raises(TypeError, match="PoseStamped odometry.*'odom'.*Odometry"):
            conn.odom_stream()


@pytest.mark.parametrize("payload_module", ["builtins.dict", "unavailable_sensor_package.Frame"])
def test_replay_metadata_only_opens_selected_streams(tmp_path, mocker, payload_module):
    path = tmp_path / "poses.db"
    with SqliteStore(path=str(path)) as store:
        for ts in (10.0, 11.0):
            store.stream("odom", PoseStamped).append(PoseStamped(ts=ts), ts=ts)
        for ts in (0.0, 1000.0):
            store.stream("color_image", dict).append({"frame": 1}, ts=ts)
    # An unused stream may require an optional package absent on the replay host.
    with closing(sqlite3.connect(path)) as db, db:
        db.execute(
            "UPDATE _streams SET config = json_set(config, '$.payload_module', ?) "
            "WHERE name = 'color_image'",
            (payload_module,),
        )
    scheduler = TestScheduler()
    received = []
    log = mocker.patch("dimos.memory.replay_progress.logger")
    with go2_conn.ReplayConnection(dataset=str(path)) as conn:
        with conn.odom_stream().subscribe(received.append, scheduler=scheduler):
            conn.seal_subscriptions()
            scheduler.advance_by(5)
            conn.seal_subscriptions()
    assert [message.ts for message in received] == [10.0, 11.0]
    assert log.info.call_args_list == [
        mocker.call(
            "Replay started",
            source=str(path),
            path=str(path),
            duration_s=1.0,
            speed=1.0,
            loop=False,
        ),
        mocker.call("Replay finished", source=str(path)),
    ]


@pytest.mark.parametrize("loop", [False, True])
def test_replay_loop_cli_controls_repeated_playback(tmp_path, mocker, loop):
    path = tmp_path / "poses.db"
    with SqliteStore(path=str(path)) as store:
        odom = store.stream("odom", PoseStamped)
        for ts in (1.0, 2.0):
            odom.append(PoseStamped(ts=ts), ts=ts)
    parsed = BlueprintConfigParser(GO2Connection.blueprint()).parse(
        ["--replay", "true", "--replay-db", str(path), "--go2connection.replay-loop", str(loop)],
        environ={},
    )
    cfg = ConnectionConfig(
        g=GlobalConfig(**parsed.global_config_values()), **parsed.module_kwargs("go2connection")
    )
    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    log = mocker.patch("dimos.memory.replay_progress.logger")
    module = GO2Connection(g=cfg.g, replay_loop=cfg.replay_loop, lidar=False, camera=False)
    mocker.patch.object(module.cmd_vel, "subscribe", return_value=mocker.Mock())
    module.set_transport("odom", mocker.Mock())
    received = []
    try:
        with Disposable(module.odom.subscribe(lambda msg: received.append(msg.ts))):
            module.start()
            scheduler.advance_by(10)
    finally:
        module.stop()
    assert received[:2] == [1.0, 2.0]
    assert (len(received) > 2) is loop
    finished = [call for call in log.info.call_args_list if call.args == ("Replay finished",)]
    assert len(finished) == (0 if loop else 1)
