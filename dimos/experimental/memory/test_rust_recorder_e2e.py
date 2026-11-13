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

from __future__ import annotations

from contextlib import ExitStack
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import threading
import time
from typing import cast
import uuid

from mcap.reader import make_reader
import numpy as np
import pytest
from reactivex.testing import TestScheduler
from typer.testing import CliRunner

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.experimental.memory import rust_cli_recorder
from dimos.experimental.memory.rust_cli_recorder import RustRecordingSession
from dimos.experimental.memory.rust_recorder import (
    RustMcapStoreConfig,
    RustRecorder,
    RustRecordingStoreConfig,
    RustSqliteStoreConfig,
)
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.core import extract_episodes
from dimos.imitation.dataprep.schema import EpisodeExtractor, FeatureSpec, RecordingSchema
from dimos.memory.cli.app import mem_app
from dimos.memory.cli.dataset import open_dataset
from dimos.memory.replay_module import ReplayModule
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool
from dimos.teleop.webxr.module import WebXRTeleopModule
from dimos.utils.testing.waiting import retry_until, wait_until

pytestmark = pytest.mark.self_hosted

_RUST_PACKAGE = DIMOS_PROJECT_ROOT / "dimos" / "experimental" / "memory" / "rust"
_EXECUTABLE = Path(
    os.environ.get(
        "DIMOS_MEMORY_RECORDER_TEST_EXECUTABLE",
        str(_RUST_PACKAGE / "result-registry-v1" / "bin" / "dimos-memory-recorder"),
    )
)
_MCAP_AVAILABLE = importlib.util.find_spec("mcap") is not None


class InteropRustRecorder(RustRecorder):
    status: In[String]
    color_image: In[Image]
    imu: In[Imu]


class FakeTransport:
    def __init__(self, channel: str) -> None:
        self.channel = channel

    def stop(self) -> None:
        pass


@pytest.fixture(scope="module")
def rust_recorder_executable() -> Path:
    # Build locally. Should already be built in CI.
    if not _EXECUTABLE.is_file():
        subprocess.run(
            [
                "nix",
                "--extra-experimental-features",
                "nix-command flakes",
                "build",
                "-L",
                "path:.#dimos-memory-recorder",
                "--out-link",
                "result-registry-v1",
                "--no-write-lock-file",
            ],
            cwd=_RUST_PACKAGE,
            check=True,
        )
    assert _EXECUTABLE.is_file()
    return _EXECUTABLE


def _wait_for_log(process: subprocess.Popen[bytes], message: str) -> None:
    assert process.stderr is not None
    deadline = time.monotonic() + 10.0
    output = bytearray()
    expected = message.encode()
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        readable, _, _ = select.select([process.stderr], [], [], remaining)
        if not readable:
            break
        chunk = os.read(process.stderr.fileno(), 65536)
        if not chunk:
            break
        output.extend(chunk)
        if expected in output:
            return
    pytest.fail(
        f"Rust recorder did not log {message!r}; exit={process.poll()}, "
        f"stderr={output.decode(errors='replace')!r}"
    )


def _free_port() -> int:
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return int(listener.getsockname()[1])


@pytest.mark.parametrize(
    "store_kind",
    [
        "sqlite",
        pytest.param(
            "mcap",
            marks=pytest.mark.skipif(not _MCAP_AVAILABLE, reason="mcap not installed"),
        ),
    ],
)
def test_rust_artifact_is_readable_by_python_memory2(
    tmp_path: Path,
    rust_recorder_executable: Path,
    store_kind: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _capture_native_artifact(tmp_path, rust_recorder_executable, store_kind, monkeypatch)


@pytest.mark.parametrize("store_kind", ["sqlite", "mcap"])
def test_native_collection_directory_preserves_schema_and_messages(
    tmp_path,
    rust_recorder_executable,
    store_kind,
    monkeypatch,
):
    camera = FeatureSpec(
        stream="color_image",
        message_type=Image,
        field="data",
        dtype="video",
        shape=(16, 16, 3),
        names=["height", "width", "channels"],
    )
    state = FeatureSpec(
        stream="imu",
        message_type=Imu,
        field="angular_velocity",
        dtype="float32",
        shape=(3,),
        names=["x", "y", "z"],
    )
    profile = CollectionProfile(
        metadata={"name": "native-test", "robot_type": "test"},
        observations={"image": camera, "state": state},
        actions={"action": state},
    )
    artifact = _capture_native_artifact(
        tmp_path, rust_recorder_executable, store_kind, monkeypatch, profile
    )
    schema = RecordingSchema.read(artifact.parent)
    assert schema.payload == artifact.name
    assert schema.observation["state"].stream == "imu"
    assert schema.name == "native-test"


def _capture_native_artifact(
    tmp_path: Path,
    rust_recorder_executable: Path,
    store_kind: str,
    monkeypatch: pytest.MonkeyPatch,
    profile: CollectionProfile | None = None,
) -> Path:
    suffix = ".db" if store_kind == "sqlite" else ".mcap"
    artifact = tmp_path / f"recording{suffix}"
    store: RustRecordingStoreConfig
    if store_kind == "sqlite":
        store = RustSqliteStoreConfig(path=str(artifact))
    else:
        store = RustMcapStoreConfig(path=str(artifact))
    endpoint = f"tcp/127.0.0.1:{_free_port()}"
    monkeypatch.setattr(global_config, "transport", "zenoh")
    recorder = InteropRustRecorder(
        source_dir=None,
        build_command=None,
        executable=str(rust_recorder_executable),
        store=store,
        record_tf=False,
        encoding_threads=2,
        stream_codecs={"imu": "lz4+lcm", "status": "json"},
        stream_timestamp_fields={"status": "ts"},
        stream_json_schemas={"status": EpisodeStatus.json_schema()},
        session=ZenohConfig(
            mode="peer",
            connect=[],
            listen=[endpoint],
            multicast=False,
            gossip=False,
            connect_timeout=0,
        ),
    )
    if profile is not None:
        kwargs = {
            "source_dir": None,
            "build_command": None,
            "executable": str(rust_recorder_executable),
            "encoding_threads": 2,
            "stream_codecs": {"imu": "lz4+lcm", "status": "json"},
            "stream_timestamp_fields": {"status": "ts"},
            "stream_json_schemas": {"status": EpisodeStatus.json_schema()},
            "session": recorder.config.session,
        }
        recorder.stop()
        atom = collect(
            profile=profile, recording=tmp_path / "session", format=store_kind
        ).active_blueprints[0]
        recorder = atom.module(**atom.kwargs, **kwargs)
        artifact = Path(recorder.config.store.path)
        recorder.status.transport = FakeTransport("dimos/collection_status")
    session_pool = ZenohSessionPool()
    channel_suffix = uuid.uuid4().hex[:8]
    publisher: ZenohTransport[Imu] = ZenohTransport(
        ZenohTopic(f"dimos/rr_imu_{channel_suffix}", Imu),
        session_pool=session_pool,
        mode="client",
        connect=[endpoint],
        multicast=False,
        gossip=False,
        connect_timeout=5,
    )
    image_publisher: ZenohTransport[Image] = ZenohTransport(
        ZenohTopic(f"dimos/rr_image_{channel_suffix}", Image),
        session_pool=session_pool,
        mode="client",
        connect=[endpoint],
        multicast=False,
        gossip=False,
        connect_timeout=5,
    )
    status_publisher: ZenohTransport[String] = ZenohTransport(
        ZenohTopic(f"dimos/rr_status_{channel_suffix}", String),
        session_pool=session_pool,
        mode="client",
        connect=[endpoint],
        multicast=False,
        gossip=False,
        connect_timeout=5,
    )
    recorder.status.transport = FakeTransport(status_publisher.channel)
    imu_topic = publisher.channel
    image_topic = image_publisher.channel
    recorder.imu.transport = FakeTransport(imu_topic)  # type: ignore[assignment]
    recorder.color_image.transport = FakeTransport(image_topic)  # type: ignore[assignment]
    specs = recorder._stream_specs()
    recorder._prepare_store(specs)
    recorder.config.streams = specs
    launch = recorder._stdin_blob(recorder._collect_topics())

    env = {
        **os.environ,
        "DIMOS_TRANSPORT": "zenoh",
        "RUST_LOG": "info,dimos_memory_recorder=debug",
    }
    process = subprocess.Popen(
        [str(rust_recorder_executable)],
        cwd=_RUST_PACKAGE,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    try:
        assert process.stdin is not None
        process.stdin.write(launch)
        process.stdin.close()
        _wait_for_log(process, "memory recorder ready")

        expected = Imu(
            ts=12.5,
            frame_id="imu_link",
            angular_velocity=Vector3(1.0, 2.0, 3.0),
        )
        expected_image = Image(
            data=np.full((16, 16, 3), [20, 80, 140], dtype=np.uint8),
            format=ImageFormat.RGB,
            frame_id="camera",
            ts=12.75,
        )
        for ts, event, state in [(12.0, "start", "recording"), (13.0, "save", "idle")]:
            status_publisher.broadcast(
                None,
                String(
                    EpisodeStatus(
                        ts=ts,
                        state=state,
                        episodes_saved=int(event == "save"),
                        episodes_discarded=0,
                        last_event=event,
                        task_label="拿起积木",
                    ).to_json()
                ),
            )
            _wait_for_log(process, "memory recorder batch written")
        publisher.broadcast(None, expected)
        image_publisher.broadcast(None, expected_image)
        _wait_for_log(process, "memory recorder batch written")

        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10.0) == 0
    finally:
        status_publisher.stop()
        publisher.stop()
        image_publisher.stop()
        session_pool.close_all()
        if process.poll() is None:
            process.kill()
            process.wait(timeout=10.0)
        recorder.stop()

    if store_kind == "mcap":
        with artifact.open("rb") as recording:
            reader = make_reader(recording)
            assert reader.get_header().profile == "dimos"
            summary = reader.get_summary()
            assert summary is not None
            assert summary.statistics is not None
            assert summary.statistics.message_count == 4
            channels = {channel.topic: channel for channel in summary.channels.values()}
            assert set(channels) == {"imu", "color_image", "status"}
            assert channels["imu"].message_encoding == "lz4+lcm"
            assert channels["color_image"].message_encoding == "jpeg"
            assert channels["status"].message_encoding == "json"
            assert summary.schemas[channels["status"].schema_id].name == "status"
            assert channels["imu"].schema_id == channels["color_image"].schema_id == 0
            assert channels["imu"].metadata["dimos.payload_type"] == (
                "dimos.msgs.sensor_msgs.Imu.Imu"
            )
            assert channels["color_image"].metadata["dimos.payload_type"] == (
                "dimos.msgs.sensor_msgs.Image.Image"
            )
            messages = {channel.topic: message for _, channel, message in reader.iter_messages()}
            assert messages["imu"].publish_time == 12_500_000_000
            assert messages["color_image"].publish_time == 12_750_000_000
            assert all(message.log_time > message.publish_time for message in messages.values())

    result = CliRunner().invoke(mem_app, ["summary", str(artifact)])
    assert result.exit_code == 0, result.output
    assert "imu" in result.output
    assert "color_image" in result.output
    assert "raw bytes" not in result.output

    with open_dataset(artifact) as memory:
        assert memory.list_streams() == ["color_image", "imu", "status"]
        assert memory.stream("imu").count() == 1
        assert memory.stream("color_image").count() == 1
        assert memory.stream("status").count() == 2
        episodes = extract_episodes(memory, EpisodeExtractor())
        assert len(episodes) == 1
        assert episodes[0].start_ts == 12.0
        assert episodes[0].end_ts == 13.0
        assert episodes[0].task_label == "拿起积木"

        observation = cast("Observation[Imu]", memory.stream("imu").first())
        assert observation.ts == 12.5
        assert isinstance(observation.data, Imu)
        assert observation.data.lcm_encode() == expected.lcm_encode()
        image_observation = cast("Observation[Image]", memory.stream("color_image").first())
        decoded_image = image_observation.data
        assert image_observation.ts == 12.75
        assert decoded_image.frame_id == "camera"
        assert decoded_image.format is ImageFormat.RGB
        assert decoded_image.data.shape == expected_image.data.shape
        assert (
            np.mean(np.abs(decoded_image.data.astype(float) - expected_image.data.astype(float)))
            < 5
        )

    return artifact


@pytest.mark.parametrize(
    "store_kind",
    ["sqlite", "mcap"],
)
def test_cli_recording_uses_existing_binary_for_both_formats(
    tmp_path: Path,
    rust_recorder_executable: Path,
    store_kind: str,
    monkeypatch: pytest.MonkeyPatch,
    mocker,
) -> None:
    suffix = "db" if store_kind == "sqlite" else "mcap"
    artifact = tmp_path / f"memory.{suffix}"
    lcm_url = f"udpm://239.255.76.67:{_free_port()}?ttl=0"
    monkeypatch.setenv("LCM_DEFAULT_URL", lcm_url)
    monkeypatch.setenv("DIMOS_LOG_LEVEL", "debug")
    batch_written = threading.Event()
    log_debug = rust_cli_recorder.logger.debug

    def observe_batch(message, **fields):
        log_debug(message, **fields)
        if message == "memory recorder batch written":
            batch_written.set()

    mocker.patch.object(rust_cli_recorder.logger, "debug", side_effect=observe_batch)
    monkeypatch.setattr(global_config, "record_engine", "rust")
    monkeypatch.setattr(global_config, "record", store_kind)
    monkeypatch.setattr(global_config, "record_topics", "*")
    monkeypatch.setattr(global_config, "record_encoding_threads", 2)
    monkeypatch.setattr(global_config, "transport", "lcm")
    monkeypatch.setattr(global_config, "build_native", False)
    monkeypatch.setattr(
        rust_cli_recorder, "prepare_rust_recorder", lambda: rust_recorder_executable
    )
    monkeypatch.setattr(rust_cli_recorder, "recording_dir", lambda: tmp_path)
    channel = f"/rust-recorder-{uuid.uuid4().hex[:8]}"
    publisher: LCMTransport[Imu] = LCMTransport(channel, Imu, url=lcm_url)
    idle: LCMTransport[Imu] = LCMTransport(channel + "-idle", Imu, url=lcm_url)
    transports = {("imu", Imu): publisher, ("idle", Imu): idle}
    session = RustRecordingSession(rust_cli_recorder.make_plan(transports))
    expected = Imu(ts=22.5, frame_id="imu_link", angular_velocity=Vector3(1.0, 2.0, 3.0))
    python_path = tmp_path / "python.db"
    with ExitStack() as stack:
        python_store = stack.enter_context(SqliteStore(path=str(python_path)))
        python_recorder = TransportRecorder(python_store)
        stack.callback(python_recorder.close)
        for (name, payload_type), transport in transports.items():
            stack.callback(transport.stop)
            transport.start()
            unsubscribe = python_recorder.tap(name, payload_type, transport)
            assert unsubscribe is not None
            stack.callback(unsubscribe)
        stack.callback(session.stop)
        session.start()
        # The recorder writes after 128 results or 100 ms without one: leave it that gap.
        retry_until(
            batch_written,
            lambda: publisher.broadcast(None, expected),
            timeout=5.0,
            interval=0.2,
        )
        wait_until(
            lambda: python_store.list_streams() == ["imu"]
            and python_store.stream("imu").count() > 0,
            timeout=5.0,
        )

    memory: SqliteStore | McapStore
    if store_kind == "mcap" and not _MCAP_AVAILABLE:
        data = artifact.read_bytes()
        assert data.startswith(b"\x89MCAP0\r\n")
        assert data.endswith(b"\x89MCAP0\r\n")
        return
    if store_kind == "sqlite":
        memory = SqliteStore(path=str(artifact))
    else:
        memory = McapStore(path=str(artifact))
    with memory, SqliteStore(path=str(python_path), must_exist=True) as python_memory:
        assert memory.list_streams() == python_memory.list_streams() == ["imu"]
        observation = cast("Observation[Imu]", memory.stream("imu").first())
        assert observation.ts == 22.5
        assert isinstance(observation.data, Imu)
        assert observation.data.lcm_encode() == expected.lcm_encode()


def test_tf_records_over_zenoh_and_replays_through_python(
    tmp_path: Path,
    rust_recorder_executable: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    artifact = tmp_path / "tf.db"
    endpoint = f"tcp/127.0.0.1:{_free_port()}"
    monkeypatch.setattr(global_config, "transport", "zenoh")
    recorder = RustRecorder(
        source_dir=None,
        build_command=None,
        executable=str(rust_recorder_executable),
        store=RustSqliteStoreConfig(path=str(artifact)),
        record_tf=True,
        encoding_threads=2,
        session=ZenohConfig(
            mode="peer",
            connect=[],
            listen=[endpoint],
            multicast=False,
            gossip=False,
            connect_timeout=0,
        ),
    )
    session_pool = ZenohSessionPool()
    topic = ZenohTopic(f"dimos/rr_tf_{uuid.uuid4().hex[:8]}", TFMessage)
    publisher: ZenohTransport[TFMessage] = ZenohTransport(
        topic,
        session_pool=session_pool,
        mode="client",
        connect=[endpoint],
        multicast=False,
        gossip=False,
        connect_timeout=5,
    )
    recorder.tf.transport = FakeTransport(publisher.channel)  # type: ignore[assignment]
    specs = recorder._stream_specs()
    recorder._prepare_store(specs)
    recorder.config.streams = specs
    launch = recorder._stdin_blob({"tf": publisher.channel})

    env = {
        **os.environ,
        "DIMOS_TRANSPORT": "zenoh",
        "RUST_LOG": "info,dimos_memory_recorder=debug",
    }
    process = subprocess.Popen(
        [str(rust_recorder_executable)],
        cwd=_RUST_PACKAGE,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    expected = TFMessage(
        Transform(
            translation=Vector3(1.0, 2.0, 3.0),
            frame_id="world",
            child_frame_id="base_link",
            ts=10.25,
        ),
        Transform(
            translation=Vector3(4.0, 5.0, 6.0),
            frame_id="base_link",
            child_frame_id="camera",
            ts=11.5,
        ),
    )
    try:
        assert process.stdin is not None
        process.stdin.write(launch)
        process.stdin.close()
        _wait_for_log(process, "memory recorder ready")

        publisher.broadcast(None, expected)
        _wait_for_log(process, "memory recorder batch written")

        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10.0) == 0
    finally:
        publisher.stop()
        session_pool.close_all()
        if process.poll() is None:
            process.kill()
            process.wait(timeout=10.0)
        recorder.stop()

    with SqliteStore(path=str(artifact)) as memory:
        observations = cast(
            "list[Observation[TFMessage]]", memory.stream("tf").order_by("ts").to_list()
        )
        assert [observation.ts for observation in observations] == [10.25, 11.5]
        assert [len(observation.data.transforms) for observation in observations] == [1, 1]
        assert [observation.data.transforms[0].child_frame_id for observation in observations] == [
            "base_link",
            "camera",
        ]


@pytest.mark.parametrize("store_kind", ["sqlite", "mcap"])
def test_native_json_events_replay_to_the_live_quest_hud(
    tmp_path, rust_recorder_executable, store_kind, monkeypatch, mocker
):
    artifact = _capture_native_artifact(tmp_path, rust_recorder_executable, store_kind, monkeypatch)
    received = []
    scheduler = TestScheduler()
    # Replay intentionally skips late subscribers. Keep its clock and scheduler
    # together so host setup latency cannot skip the first status at 100x speed.
    clock = mocker.patch("dimos.memory.replay.time")
    clock.time.side_effect = lambda: scheduler.clock
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    with ExitStack() as cleanup:
        hud = WebXRTeleopModule()
        cleanup.callback(hud.stop)
        replay = ReplayModule(dataset=str(artifact), topics="status", speed=100.0)
        cleanup.callback(replay.stop)

        def broadcast(text):
            received.append(json.loads(text))

        mocker.patch.object(hud, "_broadcast_text", side_effect=broadcast)
        subscription = replay.outputs["status"].subscribe(hud._on_episode_status)
        cleanup.callback(subscription)
        replay.start()
        scheduler.advance_to(1.0)
        assert [(p["ts"], p["last_event"]) for p in received] == [(12.0, "start"), (13.0, "save")]
        assert received[-1]["type"] == "episode_status"
        assert received[-1]["episodes_saved"] == 1
        assert received[-1]["task_label"] == "拿起积木"
        assert received[-1]["elapsed_s"] == 0.0
