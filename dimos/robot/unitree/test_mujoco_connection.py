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

from collections.abc import Callable, Iterator
import subprocess
import sys
import threading
from typing import Any

# Imported before subprocess.Popen is patched: mujoco's own import spawns a subprocess.
import mujoco  # noqa: F401
import numpy as np
import pytest
from pytest import MonkeyPatch

from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree import mujoco_connection
from dimos.robot.unitree.mujoco_connection import MujocoConnection


class _FakeShmNames:
    def to_names(self) -> dict[str, str]:
        return {}


class _FakeShmWriter:
    shm = _FakeShmNames()

    def is_ready(self) -> bool:
        return True

    def signal_stop(self) -> None:
        pass

    def cleanup(self) -> None:
        pass


class _FakeLogger:
    def info(self, _message: str) -> None:
        pass

    def warning(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


class _BlockingOutput:
    """Pipe double whose close blocks until the child is terminated."""

    def __init__(self) -> None:
        self.read_started = threading.Event()
        self.child_stopped = threading.Event()

    def __iter__(self) -> Iterator[bytes]:
        self.read_started.set()
        self.child_stopped.wait()
        return iter(())

    def close(self) -> None:
        self.child_stopped.wait()


class _QuietProcess:
    def __init__(self) -> None:
        self.stdout = _BlockingOutput()
        self.stderr = None
        self.returncode: int | None = None
        self.terminations = 0

    def poll(self) -> int | None:
        return self.returncode

    def terminate(self) -> None:
        self.terminations += 1
        self.returncode = 0
        self.stdout.child_stopped.set()

    def wait(self, timeout: float) -> int:
        assert timeout > 0
        assert self.returncode is not None
        return self.returncode

    def kill(self) -> None:
        self.terminate()


def _bare_connection(monkeypatch: MonkeyPatch, popen: Callable[..., Any]) -> MujocoConnection:
    """A MujocoConnection whose subprocess, shared memory and logging are doubles."""
    monkeypatch.setattr(mujoco_connection, "ensure_menagerie", lambda: None)
    monkeypatch.setattr(mujoco_connection, "get_data", lambda _name: None)
    monkeypatch.setattr(mujoco_connection, "ShmWriter", _FakeShmWriter)
    monkeypatch.setattr(mujoco_connection.subprocess, "Popen", popen)
    monkeypatch.setattr(mujoco_connection.atexit, "register", lambda *_args: None)
    monkeypatch.setattr(mujoco_connection, "logger", _FakeLogger())
    return MujocoConnection(GlobalConfig())


def test_start_drains_subprocess_output_larger_than_a_pipe(
    monkeypatch: MonkeyPatch,
) -> None:
    """A noisy simulator must exit instead of blocking on an unread pipe."""
    real_popen = subprocess.Popen
    popen_kwargs: dict[str, Any] = {}

    def noisy_child(_command: list[str], **kwargs: Any) -> subprocess.Popen[bytes]:
        popen_kwargs.update(kwargs)
        return real_popen(
            [
                sys.executable,
                "-c",
                "import sys; "
                "sys.stderr.buffer.write(b'x' * 4_000_000); "
                "sys.stderr.flush(); "
                "sys.stdin.buffer.read(1)",
            ],
            stdin=subprocess.PIPE,
            **kwargs,
        )

    connection = _bare_connection(monkeypatch, noisy_child)

    try:
        connection.start()
        process = connection.process
        assert process is not None
        assert process.stdin is not None
        process.stdin.write(b"x")
        process.stdin.close()
        process.wait(timeout=5)
        output_thread = connection._output_thread
        assert output_thread is not None
        output_thread.join(timeout=5)

        assert not output_thread.is_alive()
        assert popen_kwargs["stdout"] is subprocess.PIPE
        assert popen_kwargs["stderr"] is subprocess.STDOUT
    finally:
        connection.stop()


def test_stop_terminates_child_before_closing_pumped_output(
    monkeypatch: MonkeyPatch,
) -> None:
    """Stopping a quiet child must not deadlock on the pump's read lock."""
    process = _QuietProcess()
    connection = _bare_connection(monkeypatch, lambda *_args, **_kwargs: process)
    assert process.stdout.read_started.wait(timeout=1)

    stopper = threading.Thread(target=connection.stop)
    stopper.start()
    stopper.join(timeout=1)
    if stopper.is_alive():
        # Cleanup for the broken ordering: release close() without pretending
        # terminate() was called, so both the leak and ordering assertions fail.
        process.stdout.child_stopped.set()
        stopper.join(timeout=1)

    assert not stopper.is_alive()
    assert process.terminations == 1


def test_start_waits_on_the_process_launched_at_construction(monkeypatch: MonkeyPatch) -> None:
    launches: list[_QuietProcess] = []

    def popen(*_args: Any, **_kwargs: Any) -> _QuietProcess:
        launches.append(_QuietProcess())
        return launches[-1]

    connection = _bare_connection(monkeypatch, popen)
    assert len(launches) == 1  # launched by __init__

    connection.start()  # the fake shared memory is ready at once: start() only waits
    assert len(launches) == 1

    connection.stop()
    assert launches[0].terminations == 1

    connection.start()  # stop() then start() relaunches
    assert len(launches) == 2
    assert connection.process is launches[1]
    connection.stop()


@pytest.fixture
def quiet_connection(monkeypatch: MonkeyPatch) -> Iterator[MujocoConnection]:
    connection = _bare_connection(monkeypatch, lambda *_args, **_kwargs: _QuietProcess())
    yield connection
    connection.stop()


def test_get_lidar_message_builds_a_point_cloud_once_per_frame(
    quiet_connection: MujocoConnection, monkeypatch: MonkeyPatch
) -> None:
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    assert quiet_connection.shm_data is not None
    monkeypatch.setattr(
        quiet_connection.shm_data, "read_lidar", lambda: ((points, 3.5), 1), raising=False
    )

    message = quiet_connection.get_lidar_message()
    assert message is not None
    assert message.frame_id == "world"
    assert message.ts == 3.5
    np.testing.assert_allclose(message.points_f32(), points)

    assert quiet_connection.get_lidar_message() is None  # same frame again
