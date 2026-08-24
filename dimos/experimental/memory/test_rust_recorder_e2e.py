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
import subprocess
import time
from typing import cast
import uuid

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.stream import In
from dimos.core.transport import LCMTransport
from dimos.experimental.memory.rust_recorder import (
    RustMcapStoreConfig,
    RustRecorder,
    RustRecordingStoreConfig,
    RustSqliteStoreConfig,
)
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu

pytestmark = pytest.mark.self_hosted_large

_RUST_WORKSPACE = DIMOS_PROJECT_ROOT / "native" / "rust"
_EXECUTABLE = _RUST_WORKSPACE / "target" / "debug" / "dimos-memory-recorder"
_MCAP_AVAILABLE = importlib.util.find_spec("mcap") is not None


class InteropRustRecorder(RustRecorder):
    imu: In[Imu]


class FakeTransport:
    def __init__(self, channel: str) -> None:
        self.channel = channel

    def stop(self) -> None:
        pass


@pytest.fixture(scope="module")
def rust_recorder_executable() -> Path:
    subprocess.run(
        ["cargo", "build", "-p", "dimos-memory-recorder"],
        cwd=_RUST_WORKSPACE,
        check=True,
    )
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
    if store_kind == "sqlite":
        store = RustSqliteStoreConfig(path=str(artifact))
    else:
        store = RustMcapStoreConfig(path=str(artifact))
    recorder = InteropRustRecorder(
        executable=str(rust_recorder_executable),
        store=store,
        record_tf=False,
        encoding_threads=2,
    )
    # LCM appends the payload type to the channel and caps the combined name.
    channel = f"/rr_{uuid.uuid4().hex[:8]}"
    # The pure-Rust LCM transport currently uses the standard LCM bus directly.
    publisher: LCMTransport[Imu] = LCMTransport(
        channel,
        Imu,
        url="udpm://239.255.76.67:7667?ttl=0",
    )
    topic = str(publisher.topic)
    recorder.imu.transport = FakeTransport(topic)  # type: ignore[assignment]
    specs = recorder._stream_specs()
    recorder._prepare_store(specs)
    recorder.config.streams = specs
    launch = recorder._stdin_blob({"imu": topic})

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
        publisher.broadcast(None, expected)
        _wait_for_log(process, "memory recorder batch written")

        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10.0) == 0
    finally:
        publisher.stop()
        if process.poll() is None:
            process.kill()
            process.wait(timeout=10.0)
        recorder.stop()

    store_type = SqliteStore if store_kind == "sqlite" else McapStore
    with store_type(path=str(artifact)) as memory:
        observation = cast("Observation[Imu]", memory.stream("imu").first())
        assert observation.ts == 12.5
        assert observation.data.lcm_encode() == expected.lcm_encode()
