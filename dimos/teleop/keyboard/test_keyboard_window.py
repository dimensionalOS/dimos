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

from collections.abc import Iterator
from contextlib import contextmanager
import multiprocessing
from multiprocessing.connection import Connection
import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest

import dimos.teleop.keyboard.keyboard_teleop_module as keyboard_mod
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.teleop.keyboard.keyboard_window import (
    GripperCommand,
    KeyboardWindow,
    Shutdown,
    TwistCommand,
    WindowError,
    WindowReady,
    pygame,
)


@pytest.fixture(autouse=True)
def headless_sdl(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")


@contextmanager
def window_process() -> Iterator[tuple[Connection, subprocess.Popen[bytes]]]:
    parent, child = multiprocessing.Pipe()
    try:
        with subprocess.Popen(
            [
                sys.executable,
                "-m",
                "dimos.teleop.keyboard.keyboard_window_worker",
                "--connection-fd",
                str(child.fileno()),
                "--linear-speed",
                "0.05",
                "--angular-speed",
                "0.5",
            ],
            pass_fds=(child.fileno(),),
        ) as process:
            child.close()
            try:
                yield parent, process
            finally:
                parent.close()
                if process.poll() is None:
                    process.terminate()
    finally:
        parent.close()
        child.close()


@pytest.fixture
def module(mocker):
    instance = KeyboardTeleopModule()
    mocker.patch.object(instance.ee_twist_command, "publish")
    mocker.patch.object(instance.gripper_command, "publish")
    try:
        yield instance
    finally:
        instance.stop()


def test_worker_ready_shutdown_and_final_zero():
    with window_process() as (connection, process):
        assert connection.poll(10)
        assert connection.recv() == WindowReady()
        connection.send(Shutdown())
        assert connection.poll(10)
        assert connection.recv() == TwistCommand()
        assert process.wait(10) == 0
        with pytest.raises(EOFError):
            connection.recv()


def test_worker_exits_on_parent_eof():
    with window_process() as (connection, process):
        assert connection.poll(10)
        assert connection.recv() == WindowReady()
        connection.close()
        assert process.wait(10) == 0


def test_worker_reports_display_startup_failure(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "not-a-video-driver")
    with window_process() as (connection, process):
        assert connection.poll(10)
        message = connection.recv()
        assert isinstance(message, WindowError)
        assert "not-a-video-driver" in message.detail
        assert process.wait(10) != 0


def test_window_quit_releases_motion_and_pygame(mocker):
    messages = []
    mocker.patch.object(pygame.event, "get", return_value=[pygame.event.Event(pygame.QUIT)])
    quit_pygame = mocker.spy(pygame, "quit")
    window = KeyboardWindow(0.05, 0.5, messages.append, lambda: False)
    window.run()
    assert messages == [WindowReady(), TwistCommand()]
    quit_pygame.assert_called_once_with()


def test_window_error_releases_motion_and_pygame(mocker):
    messages = []
    mocker.patch.object(pygame.event, "get", side_effect=RuntimeError("event failure"))
    quit_pygame = mocker.spy(pygame, "quit")
    window = KeyboardWindow(0.05, 0.5, messages.append, lambda: False)
    with pytest.raises(RuntimeError, match="event failure"):
        window.run()
    assert messages == [WindowReady(), TwistCommand()]
    quit_pygame.assert_called_once_with()


def test_parent_routes_commands_and_stops_on_eof(module):
    parent, child = multiprocessing.Pipe()
    module._connection = parent
    thread = threading.Thread(target=module._publish_window_commands)
    try:
        thread.start()
        child.send(TwistCommand((0.1, 0.2, 0.3), (0.4, 0.5, 0.6)))
        child.send(GripperCommand(1.0))
        child.send(GripperCommand(1.0))
        child.send(GripperCommand(0.0))
        child.close()
        thread.join(5)
        assert not thread.is_alive()
        twists = module.ee_twist_command.publish.call_args_list
        assert len(twists) == 2
        moving, stopped = [call.args[0] for call in twists]
        assert [moving.linear.x, moving.linear.y, moving.linear.z] == [0.1, 0.2, 0.3]
        assert [moving.angular.x, moving.angular.y, moving.angular.z] == [0.4, 0.5, 0.6]
        assert [stopped.linear.x, stopped.linear.y, stopped.linear.z] == [0.0, 0.0, 0.0]
        assert [stopped.angular.x, stopped.angular.y, stopped.angular.z] == [0.0, 0.0, 0.0]
        assert [call.args[0].data for call in module.gripper_command.publish.call_args_list] == [
            1.0,
            0.0,
        ]
    finally:
        child.close()
        thread.join(5)


def test_parent_stops_after_child_crash(module):
    module._start_window_process()
    process = module._window
    thread = threading.Thread(target=module._publish_window_commands)
    module._thread = thread
    thread.start()
    process.kill()
    thread.join(5)
    assert not thread.is_alive()
    assert process.wait(5) != 0
    stopped = module.ee_twist_command.publish.call_args.args[0]
    assert [stopped.linear.x, stopped.linear.y, stopped.linear.z] == [0.0, 0.0, 0.0]
    assert [stopped.angular.x, stopped.angular.y, stopped.angular.z] == [0.0, 0.0, 0.0]


def test_repeated_start_stop_reaps_children_and_closes_connections(module, mocker):
    mocker.patch.object(
        keyboard_mod, "sys", SimpleNamespace(platform="darwin", executable=sys.executable)
    )
    for _ in range(2):
        module.start()
        process, connection, thread = module._window, module._connection, module._thread
        module.start()
        assert module._window is process
        module.stop()
        module.stop()
        assert process.returncode == 0
        assert connection.closed
        assert not thread.is_alive()
        assert module._window is None
        assert module._connection is None


def test_startup_failure_cleans_up_child(module, mocker, monkeypatch):
    mocker.patch.object(
        keyboard_mod, "sys", SimpleNamespace(platform="darwin", executable=sys.executable)
    )
    monkeypatch.setenv("SDL_VIDEODRIVER", "not-a-video-driver")
    with pytest.raises(RuntimeError, match="not-a-video-driver"):
        module.start()
    assert module._window is None
    assert module._connection is None
    assert module._thread is None


def test_startup_timeout_cleans_up_child(module, mocker):
    mocker.patch.object(
        keyboard_mod, "sys", SimpleNamespace(platform="darwin", executable=sys.executable)
    )
    mocker.patch.object(keyboard_mod, "WINDOW_START_TIMEOUT", 0)
    with pytest.raises(TimeoutError, match="did not start"):
        module.start()
    assert module._window is None
    assert module._connection is None
    assert module._thread is None


def test_process_launch_failure_closes_pipe(module, mocker):
    mocker.patch.object(
        keyboard_mod, "sys", SimpleNamespace(platform="darwin", executable=sys.executable)
    )
    mocker.patch.object(keyboard_mod.subprocess, "Popen", side_effect=OSError("launch failed"))
    with pytest.raises(OSError, match="launch failed"):
        module.start()
    assert module._window is None
    assert module._connection is None
    assert module._thread is None


def _launch_from_daemon(connection: Connection) -> None:
    module = KeyboardTeleopModule()
    try:
        module._start_window_process()
        connection.send("ready")
    finally:
        module.stop()
        connection.close()


def test_window_can_start_inside_daemonic_module_worker():
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    process = context.Process(target=_launch_from_daemon, args=(child,), daemon=True)
    try:
        process.start()
        child.close()
        assert parent.poll(15)
        assert parent.recv() == "ready"
        process.join(15)
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(5)
        process.close()
        parent.close()
        child.close()


def test_shutdown_escalates_and_reaps_unresponsive_child(module, mocker):
    process = mocker.Mock(spec=subprocess.Popen)
    process.wait.side_effect = [
        subprocess.TimeoutExpired("keyboard window", 1),
        subprocess.TimeoutExpired("keyboard window", 1),
        0,
    ]
    module._window = process
    module.stop()
    process.terminate.assert_called_once_with()
    process.kill.assert_called_once_with()
    assert process.wait.call_count == 3
    assert module._window is None


def test_parent_does_not_publish_queued_motion_after_stop(module):
    parent, child = multiprocessing.Pipe()
    module._connection = parent
    module._stop_event.set()
    try:
        child.send(TwistCommand((0.1, 0.0, 0.0)))
        child.send(GripperCommand(1.0))
        child.close()
        module._publish_window_commands()
        module.ee_twist_command.publish.assert_called_once()
        stopped = module.ee_twist_command.publish.call_args.args[0]
        assert [stopped.linear.x, stopped.linear.y, stopped.linear.z] == [0.0, 0.0, 0.0]
        module.gripper_command.publish.assert_not_called()
    finally:
        child.close()
