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

import importlib.util
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import time
from typing import cast
import uuid

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import Image, Imu
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from mcap.reader import make_reader
import numpy as np
import pytest

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
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.image import image_from_array, image_view
from dimos.msgs.time import time_from_seconds
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool

pytestmark = pytest.mark.self_hosted

_RUST_PACKAGE = DIMOS_PROJECT_ROOT / "dimos" / "experimental" / "memory" / "rust"
_EXECUTABLE = Path(
    os.environ.get(
        "DIMOS_RECORDER_TEST_EXECUTABLE",
        str(_RUST_PACKAGE / "result" / "bin" / "dimos-memory-recorder"),
    )
)
_MCAP_AVAILABLE = importlib.util.find_spec("mcap") is not None


class InteropRustRecorder(RustRecorder):
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
                ".#dimos-memory-recorder",
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
    suffix = ".db" if store_kind == "sqlite" else ".mcap"
    artifact = tmp_path / f"recording{suffix}"
    store: RustRecordingStoreConfig
    memory: SqliteStore | McapStore
    if store_kind == "sqlite":
        store = RustSqliteStoreConfig(path=str(artifact))
    else:
        store = RustMcapStoreConfig(path=str(artifact))
    endpoint = f"tcp/127.0.0.1:{_free_port()}"
    monkeypatch.setattr(global_config, "transport", "zenoh")
    recorder = InteropRustRecorder(
        executable=str(rust_recorder_executable),
        store=store,
        record_tf=False,
        encoding_threads=2,
        stream_codecs={"imu": "lz4+cdr" if store_kind == "sqlite" else "cdr"},
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
    imu_topic = publisher.channel
    image_topic = image_publisher.channel
    recorder.imu.transport = FakeTransport(imu_topic)  # type: ignore[assignment]
    recorder.color_image.transport = FakeTransport(image_topic)  # type: ignore[assignment]
    specs = recorder._stream_specs()
    recorder._prepare_store(specs)
    recorder.config.streams = specs
    launch = recorder._stdin_blob({"imu": imu_topic, "color_image": image_topic})

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
            header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="imu_link"),
            angular_velocity=Vector3(x=1.0, y=2.0, z=3.0),
        )
        expected_image = image_from_array(
            np.full((16, 16, 3), [20, 80, 140], dtype=np.uint8),
            encoding="rgb8",
            header=Header(frame_id="camera", stamp=time_from_seconds(12.75)),
        )
        publisher.broadcast(None, expected)
        image_publisher.broadcast(None, expected_image)
        _wait_for_log(process, "memory recorder batch written")

        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10.0) == 0
    finally:
        publisher.stop()
        image_publisher.stop()
        session_pool.close_all()
        if process.poll() is None:
            process.kill()
            process.wait(timeout=10.0)
        recorder.stop()

    if store_kind == "sqlite":
        memory = SqliteStore(path=str(artifact))
    else:
        with artifact.open("rb") as file:
            recorded_imu = [
                row
                for _schema, channel, row in make_reader(file).iter_messages()
                if channel.topic == "imu"
            ]
        assert len(recorded_imu) == 1
        assert recorded_imu[0].publish_time == 1700000000123456789
        memory = McapStore(path=str(artifact))
    with memory:
        observation = cast("Observation[Imu]", memory.stream("imu").first())
        assert observation.ts == 1700000000.123456789
        assert observation.data.encode() == expected.encode()
        image_observation = cast("Observation[Image]", memory.stream("color_image").first())
        decoded_image = image_observation.data
        assert image_observation.ts == 12.75
        assert decoded_image.header == expected_image.header
        assert decoded_image.encoding == "rgb8"
        np.testing.assert_array_equal(image_view(decoded_image), image_view(expected_image))


@pytest.mark.parametrize(
    "store_kind",
    ["sqlite", "mcap"],
)
def test_cli_recording_uses_existing_binary_for_both_formats(
    tmp_path: Path,
    rust_recorder_executable: Path,
    store_kind: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    suffix = "db" if store_kind == "sqlite" else "mcap"
    artifact = tmp_path / f"memory.{suffix}"
    lcm_url = f"udpm://239.255.76.67:{_free_port()}?ttl=0"
    monkeypatch.setenv("LCM_DEFAULT_URL", lcm_url)
    monkeypatch.setattr(global_config, "record_engine", "rust")
    monkeypatch.setattr(global_config, "record", store_kind)
    monkeypatch.setattr(global_config, "record_topics", "*")
    monkeypatch.setattr(global_config, "record_encoding_threads", 2)
    monkeypatch.setattr(global_config, "transport", "lcm")
    monkeypatch.setattr(global_config, "build_native", False)
    monkeypatch.setattr(rust_cli_recorder, "_EXECUTABLE", rust_recorder_executable)
    monkeypatch.setattr(rust_cli_recorder, "_RUST_DIR", _RUST_PACKAGE)
    monkeypatch.setattr(rust_cli_recorder, "recording_dir", lambda: tmp_path)
    channel = f"/rust-recorder-{uuid.uuid4().hex[:8]}"
    publisher: LCMTransport[Imu] = LCMTransport(channel, Imu, url=lcm_url)
    session = RustRecordingSession(rust_cli_recorder.make_plan({("imu", Imu): publisher}))
    expected = Imu(
        header=Header(stamp=time_from_seconds(22.5), frame_id="imu_link"),
        angular_velocity=Vector3(x=1, y=2, z=3),
    )
    try:
        publisher.start()
        session.start()
        for _ in range(3):
            publisher.broadcast(None, expected)
            time.sleep(0.05)
    finally:
        session.stop()
        publisher.stop()

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
    with memory:
        observation = cast("Observation[Imu]", memory.stream("imu").first())
        assert observation.ts == 22.5
        assert observation.data.encode() == expected.encode()


def test_tf_records_over_zenoh_and_replays_through_python(
    tmp_path: Path,
    rust_recorder_executable: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    artifact = tmp_path / "tf.db"
    endpoint = f"tcp/127.0.0.1:{_free_port()}"
    monkeypatch.setattr(global_config, "transport", "zenoh")
    recorder = RustRecorder(
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
        transforms=[
            TransformStamped(
                header=Header(frame_id="world", stamp=time_from_seconds(10.25)),
                child_frame_id="base_link",
                transform=Transform(translation=Vector3(x=1, y=2, z=3)),
            ),
            TransformStamped(
                header=Header(frame_id="base_link", stamp=time_from_seconds(11.5)),
                child_frame_id="camera",
                transform=Transform(translation=Vector3(x=4, y=5, z=6)),
            ),
        ]
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
