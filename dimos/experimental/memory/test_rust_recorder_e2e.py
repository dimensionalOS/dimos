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

import numpy as np
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.experimental.memory.rust_recorder import (
    RustMcapStoreConfig,
    RustRecorder,
    RustRecordingStoreConfig,
    RustSqliteStoreConfig,
)
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool

pytestmark = pytest.mark.self_hosted_large

_RUST_WORKSPACE = DIMOS_PROJECT_ROOT / "native" / "rust"
_EXECUTABLE = _RUST_WORKSPACE / "result" / "bin" / "dimos-memory-recorder"
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
        cwd=_RUST_WORKSPACE,
        check=True,
    )
    assert _EXECUTABLE.is_file()
    return _EXECUTABLE


def _wait_for_log(process: subprocess.Popen[bytes], message: str) -> None:
    assert process.stderr is not None
    deadline = time.monotonic() + 10.0
    output: list[str] = []
    while process.poll() is None:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        readable, _, _ = select.select([process.stderr], [], [], remaining)
        if not readable:
            break
        line = process.stderr.readline().decode(errors="replace")
        output.append(line)
        if message in line:
            return
    pytest.fail(
        f"Rust recorder did not log {message!r}; exit={process.poll()}, stderr={''.join(output)!r}"
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
) -> None:
    suffix = ".db" if store_kind == "sqlite" else ".mcap"
    artifact = tmp_path / f"recording{suffix}"
    store: RustRecordingStoreConfig
    memory: SqliteStore | McapStore
    if store_kind == "sqlite":
        store = RustSqliteStoreConfig(path=str(artifact))
    else:
        store = RustMcapStoreConfig(path=str(artifact))
    recorder = InteropRustRecorder(
        executable=str(rust_recorder_executable),
        store=store,
        record_tf=False,
        encoding_threads=2,
        stream_codecs={"imu": "lz4+lcm"},
    )
    # LCM appends the payload type to the channel and caps the combined name.
    channel_suffix = uuid.uuid4().hex[:8]
    # The pure-Rust LCM transport currently uses the standard LCM bus directly.
    publisher: LCMTransport[Imu] = LCMTransport(
        f"/rr_imu_{channel_suffix}",
        Imu,
        url="udpm://239.255.76.67:7667?ttl=0",
    )
    image_publisher: LCMTransport[Image] = LCMTransport(
        f"/rr_image_{channel_suffix}",
        Image,
        url="udpm://239.255.76.67:7667?ttl=0",
    )
    imu_topic = str(publisher.topic)
    image_topic = str(image_publisher.topic)
    recorder.imu.transport = FakeTransport(imu_topic)  # type: ignore[assignment]
    recorder.color_image.transport = FakeTransport(image_topic)  # type: ignore[assignment]
    specs = recorder._stream_specs()
    recorder._prepare_store(specs)
    recorder.config.streams = specs
    launch = recorder._stdin_blob({"imu": imu_topic, "color_image": image_topic})

    env = {**os.environ, "DIMOS_TRANSPORT": "lcm", "RUST_LOG": "debug"}
    process = subprocess.Popen(
        [str(rust_recorder_executable)],
        cwd=_RUST_WORKSPACE,
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
        publisher.broadcast(None, expected)
        image_publisher.broadcast(None, expected_image)
        _wait_for_log(process, "memory recorder batch written")

        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10.0) == 0
    finally:
        publisher.stop()
        image_publisher.stop()
        if process.poll() is None:
            process.kill()
            process.wait(timeout=10.0)
        recorder.stop()

    if store_kind == "sqlite":
        memory = SqliteStore(path=str(artifact))
    else:
        memory = McapStore(
            path=str(artifact),
            codecs={"imu": Lz4Codec(LcmCodec(Imu))},
        )
    with memory:
        observation = cast("Observation[Imu]", memory.stream("imu").first())
        assert observation.ts == 12.5
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
    publisher = ZenohTransport(
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

    env = {**os.environ, "DIMOS_TRANSPORT": "zenoh", "RUST_LOG": "debug"}
    process = subprocess.Popen(
        [str(rust_recorder_executable)],
        cwd=_RUST_WORKSPACE,
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
        observations: list[Observation[TFMessage]] = memory.stream("tf").order_by("ts").to_list()
        assert [observation.ts for observation in observations] == [10.25, 11.5]
        assert [len(observation.data.transforms) for observation in observations] == [1, 1]
        assert [observation.data.transforms[0].child_frame_id for observation in observations] == [
            "base_link",
            "camera",
        ]
