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

from collections.abc import Iterator
import subprocess
import sys
import threading
from types import ModuleType
from typing import Any, cast

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


def _bare_connection(monkeypatch: MonkeyPatch) -> MujocoConnection:
    mjx_env = ModuleType("mujoco_playground._src.mjx_env")
    mjx_env.ensure_menagerie_exists = lambda: None
    playground_src = ModuleType("mujoco_playground._src")
    playground_src.mjx_env = mjx_env
    monkeypatch.setitem(sys.modules, "mujoco_playground._src", playground_src)
    monkeypatch.setattr(mujoco_connection, "get_data", lambda _name: None)
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

    connection = _bare_connection(monkeypatch)

    monkeypatch.setattr(mujoco_connection, "ShmWriter", _FakeShmWriter)
    monkeypatch.setattr("dimos.robot.unitree.mujoco_connection.subprocess.Popen", noisy_child)
    monkeypatch.setattr(
        "dimos.robot.unitree.mujoco_connection.atexit.register", lambda *_args: None
    )
    monkeypatch.setattr(mujoco_connection, "logger", _FakeLogger())

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
    monkeypatch.setattr(mujoco_connection, "logger", _FakeLogger())
    connection = _bare_connection(monkeypatch)
    process = _QuietProcess()
    state = cast("Any", connection)
    state.process = process
    state.shm_data = _FakeShmWriter()
    state._output_thread = threading.Thread(
        target=connection._pump_subprocess_output,
        name="mujoco-output-pump",
        daemon=True,
    )
    state._output_thread.start()
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
