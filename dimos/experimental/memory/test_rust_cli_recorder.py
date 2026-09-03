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

from io import BytesIO
import json
from pathlib import Path
import signal
import subprocess
import threading
from types import SimpleNamespace
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.experimental.memory import rust_cli_recorder
from dimos.experimental.memory.rust_cli_recorder import RustRecordingPlan, RustRecordingSession
from dimos.experimental.memory.rust_recorder import RustStreamSpec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic


def _lcm(channel: str, payload_type: type[Any] = PoseStamped) -> LCMTransport[Any]:
    return LCMTransport(channel, payload_type)


def _zenoh(channel: str, payload_type: type[Any] = PoseStamped) -> ZenohTransport[Any]:
    return ZenohTransport(ZenohTopic(channel, payload_type))


@pytest.fixture(autouse=True)
def recording_config(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_engine", "rust")
    monkeypatch.setattr(global_config, "record_topics", "*")
    monkeypatch.setattr(global_config, "record_encoding_threads", None)
    monkeypatch.setattr(global_config, "replay", False)
    monkeypatch.setattr(global_config, "build_native", False)
    monkeypatch.setattr(rust_cli_recorder, "recording_dir", lambda: tmp_path)


def test_plan_uses_actual_lcm_channels_and_memory_codecs(tmp_path: Path) -> None:
    odom = _lcm("/wire/odom")
    camera = _lcm("/wire/camera", Image)
    plan = rust_cli_recorder.make_plan(
        {
            ("odom", PoseStamped): odom,
            ("color_image", Image): camera,
        }
    )

    assert plan.backend == "lcm"
    assert plan.topics == {"stream_0": odom.channel, "stream_1": camera.channel}
    assert [(stream.name, stream.codec) for stream in plan.streams] == [
        ("odom", "lcm"),
        ("color_image", "jpeg"),
    ]
    assert plan.path == tmp_path / "memory.db"


def test_replay_does_not_build_the_native_recorder(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "replay", True)
    monkeypatch.setattr(
        "dimos.experimental.memory.rust_cli_recorder.subprocess.Popen",
        lambda *args, **kwargs: pytest.fail("replay must not build the recorder"),
    )

    rust_cli_recorder.prepare_rust_recorder()


def test_plan_uses_mcap_artifact_for_zenoh(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "record", "mcap")

    plan = rust_cli_recorder.make_plan({("odom", PoseStamped): _zenoh("dimos/odom/PoseStamped")})

    assert plan.backend == "zenoh"
    assert plan.path == tmp_path / "memory.mcap"


def test_plan_rejects_unsupported_transport_before_creating_artifact(
    tmp_path: Path,
) -> None:
    with pytest.raises(ValueError, match=r"unsupported selections: odom \(SimpleNamespace\)"):
        rust_cli_recorder.make_plan({("odom", PoseStamped): SimpleNamespace(channel="/wire/odom")})

    assert list(tmp_path.iterdir()) == []


def test_plan_skips_non_lcm_payload_and_requires_one_recordable_stream() -> None:
    with pytest.raises(ValueError, match="selected no Rust-recordable streams"):
        rust_cli_recorder.make_plan({("values", dict): _lcm("/values")})


def test_plan_rejects_mixed_transport_sessions() -> None:
    with pytest.raises(ValueError, match="mixed LCM and Zenoh"):
        rust_cli_recorder.make_plan(
            {
                ("odom", PoseStamped): _lcm("/odom"),
                ("camera", Image): _zenoh("dimos/camera/Image"),
            }
        )


def test_sqlite_artifact_registration_matches_the_native_plan(tmp_path: Path) -> None:
    plan = rust_cli_recorder.make_plan(
        {("odom", PoseStamped): _lcm("/odom"), ("camera", Image): _lcm("/camera")}
    )

    rust_cli_recorder._prepare_artifact(plan)

    with SqliteStore(path=str(tmp_path / "memory.db"), must_exist=True) as store:
        assert store.list_streams() == ["camera", "odom"]


class _CapturedInput:
    def __init__(self) -> None:
        self.data = bytearray()
        self.closed = False

    def write(self, data: bytes) -> int:
        self.data.extend(data)
        return len(data)

    def close(self) -> None:
        self.closed = True


class _FakeProcess:
    def __init__(
        self,
        stdout: bytes = b'{"fields":{"message":"memory recorder ready"},"level":"INFO"}\n',
        *,
        returncode: int | None = None,
        ignore_term: bool = False,
    ) -> None:
        self.stdin = _CapturedInput()
        self.stdout = BytesIO(stdout)
        self.stderr = BytesIO()
        self.pid = 1234
        self.returncode = returncode
        self.ignore_term = ignore_term
        self.signals: list[int] = []
        self.killed = False
        self._exited = threading.Event()
        if returncode is not None:
            self._exited.set()

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        if timeout is None:
            self._exited.wait()
        elif not self._exited.wait(timeout):
            raise subprocess.TimeoutExpired("rust-recorder", timeout)
        assert self.returncode is not None
        return self.returncode

    def send_signal(self, signum: int) -> None:
        self.signals.append(signum)
        if not self.ignore_term:
            self.exit(0)

    def kill(self) -> None:
        self.killed = True
        self.exit(-signal.SIGKILL)

    def exit(self, returncode: int) -> None:
        self.returncode = returncode
        self._exited.set()


def _plan(tmp_path: Path) -> RustRecordingPlan:
    return RustRecordingPlan(
        backend="lcm",
        topics={"stream_0": "/odom"},
        streams=[
            RustStreamSpec(
                port="stream_0",
                name="odom",
                payload_type="dimos.msgs.geometry_msgs.PoseStamped.PoseStamped",
                codec="lcm",
            )
        ],
        payload_types={"odom": PoseStamped},
        path=tmp_path / "memory.db",
    )


def _start_fake_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    process: _FakeProcess,
) -> RustRecordingSession:
    monkeypatch.setattr(rust_cli_recorder, "prepare_rust_recorder", lambda: None)
    monkeypatch.setattr(
        "dimos.experimental.memory.rust_cli_recorder.subprocess.Popen",
        lambda *args, **kwargs: process,
    )
    session = RustRecordingSession(_plan(tmp_path))
    session.start()
    return session


def test_reuses_existing_memory_recorder_binary() -> None:
    assert rust_cli_recorder._EXECUTABLE.name == "dimos-memory-recorder"


def test_existing_binary_skips_build(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    executable = tmp_path / "dimos-memory-recorder"
    executable.touch()
    monkeypatch.setattr(rust_cli_recorder, "_EXECUTABLE", executable)
    monkeypatch.setattr(
        "dimos.experimental.memory.rust_cli_recorder.subprocess.Popen",
        lambda *args, **kwargs: pytest.fail("existing binary must not be rebuilt"),
    )

    rust_cli_recorder.prepare_rust_recorder()


def test_build_failure_is_reported(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    process = SimpleNamespace(stdout=BytesIO(b"build output\n"), wait=lambda: 7)
    monkeypatch.setattr(rust_cli_recorder, "_EXECUTABLE", tmp_path / "missing")
    monkeypatch.setattr(
        "dimos.experimental.memory.rust_cli_recorder.subprocess.Popen",
        lambda *args, **kwargs: process,
    )

    with pytest.raises(RuntimeError, match="Rust recorder build failed.*exit 7"):
        rust_cli_recorder.prepare_rust_recorder()


@pytest.mark.parametrize("encoding_threads", [None, 8])
def test_launch_config_uses_default_or_configured_threads(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    encoding_threads: int | None,
) -> None:
    monkeypatch.setattr(global_config, "record_encoding_threads", encoding_threads)
    process = _FakeProcess()
    session = _start_fake_session(tmp_path, monkeypatch, process)

    launch = json.loads(process.stdin.data)
    assert launch["config"]["encoding_threads"] == (encoding_threads or 4)
    assert launch["topics"] == {"stream_0": "/odom"}
    assert process.stdin.closed
    session.stop()
    assert process.signals == [signal.SIGTERM]


def test_process_exit_before_ready_fails_startup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    process = _FakeProcess(stdout=b"", returncode=3)
    monkeypatch.setattr(rust_cli_recorder, "prepare_rust_recorder", lambda: None)
    monkeypatch.setattr(
        "dimos.experimental.memory.rust_cli_recorder.subprocess.Popen",
        lambda *args, **kwargs: process,
    )

    with pytest.raises(RuntimeError, match="startup failed: process exited with status 3"):
        RustRecordingSession(_plan(tmp_path)).start()


def test_readiness_timeout_fails_startup(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    process = _FakeProcess(stdout=b"")
    monkeypatch.setattr(rust_cli_recorder, "_READY_TIMEOUT", 0.01)
    monkeypatch.setattr(rust_cli_recorder, "prepare_rust_recorder", lambda: None)
    monkeypatch.setattr(
        "dimos.experimental.memory.rust_cli_recorder.subprocess.Popen",
        lambda *args, **kwargs: process,
    )

    with pytest.raises(RuntimeError, match="did not report ready"):
        RustRecordingSession(_plan(tmp_path)).start()

    assert process.signals == [signal.SIGTERM]


def test_unexpected_runtime_exit_is_logged_without_raising(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    process = _FakeProcess()
    error = mocker.patch.object(rust_cli_recorder.logger, "error")
    session = _start_fake_session(tmp_path, monkeypatch, process)

    process.exit(9)
    session._threads[-1].join(timeout=1)
    session.stop()

    error.assert_called_once_with("Experimental Rust recorder exited", status=9)


def test_stop_kills_process_that_does_not_flush(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    process = _FakeProcess(ignore_term=True)
    monkeypatch.setattr(rust_cli_recorder, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.01)
    session = _start_fake_session(tmp_path, monkeypatch, process)

    session.stop()

    assert process.signals == [signal.SIGTERM]
    assert process.killed
