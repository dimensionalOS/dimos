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
from io import BytesIO
import os
import socket
import subprocess
import sys
from tempfile import TemporaryDirectory
import threading
import time
from typing import Any

# Imported before subprocess.Popen is patched: mujoco's own import spawns a subprocess.
import mujoco  # noqa: F401
import numpy as np
from numpy.typing import NDArray
import pytest
from pytest import MonkeyPatch
from pytest_mock import MockerFixture
import structlog
from structlog.testing import LogCapture

from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree import mujoco_connection
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.simulation.mujoco.config import MujocoConfig
from dimos.utils import logging_config


class _FakeShmNames:
    def to_names(self) -> dict[str, str]:
        return {}


class _FakeShmWriter:
    shm = _FakeShmNames()

    def __init__(self) -> None:
        self.seq = 0  # the sequence number every sensor read reports
        self.run_signaled = False
        self.command = np.zeros(6, dtype=np.float32)

    def write_command(self, linear: NDArray[Any], angular: NDArray[Any]) -> None:
        self.command = np.concatenate((linear, angular))

    def is_ready(self) -> bool:
        return True

    def signal_run(self) -> None:
        self.run_signaled = True

    def signal_stop(self) -> None:
        pass

    def cleanup(self) -> None:
        pass

    def read_video(self) -> tuple[NDArray[Any], int]:
        return np.zeros((1, 1, 3), dtype=np.uint8), self.seq

    def read_odom(self) -> tuple[tuple[NDArray[Any], NDArray[Any], float], int]:
        return (np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), 0.0), self.seq

    def read_lidar(self) -> tuple[tuple[NDArray[Any], float], int]:
        return (np.zeros((1, 3), dtype=np.float32), 0.0), self.seq


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


def _bare_connection(
    monkeypatch: MonkeyPatch, popen: Callable[..., Any], config: MujocoConfig | None = None
) -> MujocoConnection:
    """A MujocoConnection whose subprocess, shared memory and logging are doubles."""
    monkeypatch.setattr(mujoco_connection, "ensure_menagerie", lambda: None)
    monkeypatch.setattr(mujoco_connection, "get_data", lambda _name: None)
    monkeypatch.setattr(mujoco_connection, "ShmWriter", _FakeShmWriter)
    monkeypatch.setattr(mujoco_connection.subprocess, "Popen", popen)
    monkeypatch.setattr(mujoco_connection.atexit, "register", lambda *_args: None)
    monkeypatch.setattr(mujoco_connection, "logger", _FakeLogger())
    return MujocoConnection(config or MujocoConfig())


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
    assert len(launches) == 1
    first_shm = connection.shm_data
    assert isinstance(first_shm, _FakeShmWriter)
    assert not first_shm.run_signaled  # the world stands still until start()

    connection.start()  # the fake shared memory is ready at once: start() only waits
    assert len(launches) == 1
    assert first_shm.run_signaled
    first_shm.seq = 500
    assert _delivered_sensors(connection) == 3

    connection.stop()
    assert launches[0].terminations == 1

    connection.start()  # stop() then start() relaunches
    assert len(launches) == 2
    assert connection.process is launches[1]
    second_shm = connection.shm_data
    assert isinstance(second_shm, _FakeShmWriter)
    assert second_shm is not first_shm
    assert second_shm.run_signaled
    second_shm.seq = 1  # the new simulator counts frames from zero again
    assert _delivered_sensors(connection) == 3
    connection.stop()


def _delivered_sensors(connection: MujocoConnection) -> int:
    """How many of the three sensors hand out a frame right now."""
    frames = (
        connection.get_video_frame(),
        connection.get_odom_message(),
        connection.get_lidar_message(),
    )
    return sum(frame is not None for frame in frames)


@pytest.fixture
def quiet_connection(monkeypatch: MonkeyPatch) -> Iterator[MujocoConnection]:
    connection = _bare_connection(monkeypatch, lambda *_args, **_kwargs: _QuietProcess())
    yield connection
    connection.stop()


@pytest.fixture
def output_relay(
    monkeypatch: MonkeyPatch, mocker: MockerFixture
) -> Iterator[tuple[MujocoConnection, LogCapture]]:
    # Exercise the real output pump without launching a simulator or a reader thread.
    mocker.patch.object(MujocoConnection, "_launch")
    connection = _bare_connection(monkeypatch, mocker.Mock())
    logs = LogCapture()
    monkeypatch.setattr(
        mujoco_connection,
        "logger",
        structlog.wrap_logger(
            structlog.testing.CapturingLogger(),
            processors=[logs],
            wrapper_class=structlog.stdlib.BoundLogger,
        ),
    )
    yield connection, logs
    connection.stop()


@pytest.mark.parametrize(
    "level,message",
    [
        ("debug", "Checking endpoints"),
        ("info", "No errors or warnings reported"),
        ("warning", "Zenoh endpoints not linked ..."),
        ("error", "Connection lost"),
        ("critical", "Connection lost"),
        ("warning", ""),
    ],
)
def test_output_preserves_formatted_log_level_without_nested_header(
    output_relay: tuple[MujocoConnection, LogCapture],
    monkeypatch: MonkeyPatch,
    mocker: MockerFixture,
    level: str,
    message: str,
) -> None:
    connection, logs = output_relay
    # A child writing to a pipe uses this uncolored renderer. Derive its header
    # here so tests catch mismatches with the actual console format.
    monkeypatch.setattr(logging_config, "_CONSOLE_USE_COLORS", False)
    line = (
        logging_config._compact_console_processor(
            None,
            level,
            {
                "timestamp": "2026-11-08T11:49:45.039Z",
                "level": level,
                "logger": "otocol/service/zenohservice.py",
                "event": message,
            },
        )
        + "\n"
    )
    with BytesIO(line.encode()) as output:
        connection.process = mocker.Mock(stdout=output, stderr=None, poll=lambda: 0)
        connection._pump_subprocess_output()

    assert logs.entries == [
        {
            "event": f"[mujoco] {message}",
            "log_level": level,
            "subprocess_logger": "otocol/service/zenohservice.py",
        }
    ]


@pytest.mark.parametrize(
    "line,level,message",
    [
        (b"Simulator ready\n", "info", "Simulator ready"),
        (b"WARNING: unstable simulation\n", "warning", "WARNING: unstable simulation"),
        (b"ERROR: model unavailable\n", "warning", "ERROR: model unavailable"),
        (b"FATAL: model unavailable\n", "warning", "FATAL: model unavailable"),
        (b"Invalid byte: \xff\n", "info", "Invalid byte: \ufffd"),
    ],
)
def test_output_keeps_unformatted_diagnostics(
    output_relay: tuple[MujocoConnection, LogCapture],
    mocker: MockerFixture,
    line: bytes,
    level: str,
    message: str,
) -> None:
    connection, logs = output_relay
    with BytesIO(b"\n" + line + b"\n") as output:
        connection.process = mocker.Mock(stdout=output, stderr=None, poll=lambda: 0)
        connection._pump_subprocess_output()

    assert logs.entries == [{"event": f"[mujoco] {message}", "log_level": level}]


def test_get_lidar_message_builds_a_point_cloud_once_per_frame(
    quiet_connection: MujocoConnection, monkeypatch: MonkeyPatch
) -> None:
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    assert quiet_connection.shm_data is not None
    monkeypatch.setattr(quiet_connection.shm_data, "read_lidar", lambda: ((points, 3.5), 1))

    message = quiet_connection.get_lidar_message()
    assert message is not None
    assert message.frame_id == "world"
    assert message.ts == 3.5
    np.testing.assert_allclose(message.points_f32(), points)

    assert quiet_connection.get_lidar_message() is None  # same frame again


def test_stop_movement_sends_zero_speed(
    quiet_connection: MujocoConnection, monkeypatch: MonkeyPatch
) -> None:
    sent: list[tuple[list[float], list[float]]] = []
    assert quiet_connection.shm_data is not None
    monkeypatch.setattr(
        quiet_connection.shm_data,
        "write_command",
        lambda linear, angular: sent.append((linear.tolist(), angular.tolist())),
        raising=False,
    )
    quiet_connection.move(Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)))
    quiet_connection.stop_movement()
    assert sent[-1] == ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])


@pytest.mark.parametrize("replacement_duration", [0.0, 2.0], ids=["continuous", "timed"])
def test_expired_move_cannot_stop_a_newer_command(
    quiet_connection: MujocoConnection, mocker: MockerFixture, replacement_duration: float
) -> None:
    writer = quiet_connection.shm_data
    assert isinstance(writer, _FakeShmWriter)
    timers = mocker.patch.object(threading, "Timer", side_effect=[mocker.Mock(), mocker.Mock()])
    quiet_connection.move(Twist(linear=[0.25, 0, 0]), duration=1.0)
    expire_first = timers.call_args.args[1]
    quiet_connection.move(Twist(linear=[0, 0.5, 0]), duration=replacement_duration)

    # A timer can already have entered its callback when cancel() is called.
    expire_first()
    np.testing.assert_array_equal(writer.command, [0, 0.5, 0, 0, 0, 0])

    if replacement_duration:
        timers.call_args.args[1]()
        np.testing.assert_array_equal(writer.command, np.zeros(6))


@pytest.mark.self_hosted
@pytest.mark.skipif(sys.platform != "linux", reason="Linux EGL offscreen rendering")
@pytest.mark.parametrize(
    ("robot", "viewer"),
    [("unitree_go2", "none"), ("unitree_g1", "rerun")],
    ids=["go2-viewer-none", "g1-no-display"],
)
def test_headless_simulation_delivers_frames(
    robot: str, viewer: str, monkeypatch: MonkeyPatch
) -> None:
    for key in ("DISPLAY", "WAYLAND_DISPLAY", "WAYLAND_SOCKET", "MUJOCO_GL"):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setattr(global_config, "viewer", viewer)

    connection = MujocoConnection(MujocoConfig(), robot_model=robot)
    try:
        connection.start()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if (
                connection.get_video_frame() is not None
                and connection.get_odom_message() is not None
            ):
                break
            time.sleep(0.05)
        else:
            pytest.fail("No simulated image and odometry received")
    finally:
        connection.stop()


@pytest.mark.parametrize(
    ("platform", "viewer", "display", "override", "backend", "headless", "child_backend"),
    [
        ("linux", "rerun", None, None, None, True, "egl"),
        ("linux", "rerun", "DISPLAY", None, None, False, None),
        # The name alone is not a display: nothing listens on it here.
        ("linux", "rerun", "WAYLAND_DISPLAY", None, None, True, "egl"),
        ("linux", "none", "DISPLAY", None, None, True, "egl"),
        ("linux", "rerun", "DISPLAY", True, "osmesa", True, "osmesa"),
        ("linux", "none", "DISPLAY", False, None, False, None),
        ("darwin", "rerun", None, True, None, True, None),
        ("darwin", "rerun", None, None, None, False, None),
    ],
)
def test_simulator_window_and_renderer(
    monkeypatch: MonkeyPatch,
    mocker: MockerFixture,
    platform: str,
    viewer: str,
    display: str | None,
    override: bool | None,
    backend: str | None,
    headless: bool,
    child_backend: str | None,
) -> None:
    monkeypatch.setattr(sys, "platform", platform)
    monkeypatch.setattr(global_config, "viewer", viewer)
    monkeypatch.setattr(mujoco_connection, "find_executable", lambda name: name)
    for key in ("DISPLAY", "WAYLAND_DISPLAY", "WAYLAND_SOCKET", "MUJOCO_GL"):
        monkeypatch.delenv(key, raising=False)
    if display:
        monkeypatch.setenv(display, ":1")
    if backend:
        monkeypatch.setenv("MUJOCO_GL", backend)
    config = MujocoConfig() if override is None else MujocoConfig(headless=override)
    popen = mocker.Mock(return_value=_QuietProcess())
    connection = _bare_connection(monkeypatch, popen, config)
    try:
        assert connection.config.headless is headless
        assert popen.call_args.kwargs["env"].get("MUJOCO_GL") == child_backend
        assert os.getenv("MUJOCO_GL") == backend
        executable = "mjpython" if platform == "darwin" and not headless else sys.executable
        assert popen.call_args.args[0][0] == executable
    finally:
        connection.stop()


@pytest.mark.parametrize("named_by", ["name", "path"])
def test_named_wayland_display_needs_a_listener(monkeypatch: MonkeyPatch, named_by: str) -> None:
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(global_config, "viewer", "rerun")
    for key in ("DISPLAY", "WAYLAND_SOCKET", "XDG_RUNTIME_DIR"):
        monkeypatch.delenv(key, raising=False)
    # A pytest tmp_path can exceed the length limit of a Unix socket path.
    with TemporaryDirectory() as runtime:
        path = os.path.join(runtime, "wayland-1")
        if named_by == "name":
            monkeypatch.setenv("XDG_RUNTIME_DIR", runtime)
            monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-1")
        else:
            monkeypatch.setenv("WAYLAND_DISPLAY", path)
        with socket.socket(socket.AF_UNIX) as compositor:
            compositor.bind(path)
            compositor.listen()
            assert not MujocoConfig().headless
        # The socket file outlives the compositor that was listening on it.
        assert MujocoConfig().headless


def test_viewer_child_can_use_inherited_wayland_socket(monkeypatch: MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(global_config, "viewer", "rerun")
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    real_popen = subprocess.Popen

    def viewer(command: list[str], **kwargs: Any) -> subprocess.Popen[bytes]:
        probe = "import os; os.write(int(os.environ['WAYLAND_SOCKET']), b'x')"
        return real_popen([command[0], "-c", probe], **kwargs)

    display, peer = socket.socketpair()
    with display, peer:
        monkeypatch.setenv("WAYLAND_SOCKET", str(display.fileno()))
        connection = _bare_connection(monkeypatch, viewer)
        try:
            assert not connection.config.headless
            assert connection.process is not None
            assert connection.process.wait(timeout=5) == 0
            peer.settimeout(5)
            assert peer.recv(1) == b"x"
        finally:
            connection.stop()
