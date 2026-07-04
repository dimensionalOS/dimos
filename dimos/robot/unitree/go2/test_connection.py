from io import BytesIO
import sys

from dimos.robot.unitree.go2.connection import ReplayConnection
from dimos.robot.unitree.mujoco_connection import (
    _mujoco_subprocess_executable,
    _read_process_stderr,
)


def test_replay_connection_stop_is_noop() -> None:
    connection = ReplayConnection()

    connection.stop()


def test_read_process_stderr_decodes_available_output() -> None:
    class Process:
        stderr = BytesIO(b"first line\nsecond line\n")

    assert _read_process_stderr(Process()) == "first line\nsecond line"


def test_headless_mujoco_uses_current_python_on_macos() -> None:
    assert _mujoco_subprocess_executable(headless=True, platform="darwin") == sys.executable
    assert _mujoco_subprocess_executable(headless=False, platform="darwin") == "mjpython"
