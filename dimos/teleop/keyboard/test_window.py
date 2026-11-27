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

from collections.abc import Iterator
import os
from threading import Event
from typing import Any

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.get_all_blueprints import get_module_by_name
from dimos.teleop.keyboard.window import KeyboardWindow, WindowProcess, check_keyboard_display


@pytest.fixture(params=["keyboard-teleop", "keyboard-teleop-module"])
def module(request) -> Iterator[Any]:
    (atom,) = autoconnect(get_module_by_name(request.param)).solidify().active_blueprints
    instance = atom.module()
    try:
        yield instance
    finally:
        instance.stop()


@pytest.fixture
def velocities(module) -> list[Twist]:
    received: list[Twist] = []
    port = module.outputs.get("cmd_vel") or module.outputs["ee_twist_command"]
    port.subscribe(received.append)
    return received


def test_start_reports_the_display_error(module, velocities, monkeypatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "unavailable-teleop-test")
    with pytest.raises(RuntimeError, match="unavailable-teleop-test"):
        module.start()
    assert velocities == []


def test_start_refuses_automatically_selected_headless_display(
    module, velocities, monkeypatch, tmp_path
) -> None:
    # Give the window child an initialized headless display, independent of the host desktop.
    (tmp_path / "sitecustomize.py").write_text(
        "import os, pygame\n"
        "os.environ['SDL_VIDEODRIVER'] = 'dummy'\n"
        "pygame.display.init()\n"
        "del os.environ['SDL_VIDEODRIVER']\n"
    )
    monkeypatch.setenv("PYTHONPATH", str(tmp_path), prepend=os.pathsep)
    with pytest.raises(RuntimeError, match="no usable display.*dummy"):
        module.start()
    assert velocities == []


def test_offscreen_requires_requesting_that_driver(monkeypatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "x11")
    with pytest.raises(RuntimeError, match="no usable display.*offscreen"):
        check_keyboard_display("offscreen")

    monkeypatch.setenv("SDL_VIDEODRIVER", "x11,offscreen")
    check_keyboard_display("offscreen")


def test_stop_closes_the_open_display(module, monkeypatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    module.start()
    process = module._window._process
    assert process.poll() is None

    module.stop()

    assert process.returncode == 0


def test_idle_window_can_close_and_reopen(module, velocities, monkeypatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    module.start()
    first = module._window
    module.start()
    assert module._window is first

    # Ask the child to exit its event loop without stopping the parent module.
    first._connection.send_bytes(b"")
    first._reader.join(timeout=5)
    assert first._process.returncode == 0
    assert first._connection.closed
    assert velocities == []

    module.start()
    assert module._window is not first
    assert module._window._process.poll() is None


class _MovingWindow(KeyboardWindow):
    def open(self) -> None:
        pass

    def run(self) -> None:
        motion = Twist(linear=Vector3(1, 0, 0))
        self.publish("cmd_vel", motion)
        event = self.connection.recv_bytes()
        self.publish("cmd_vel", Twist() if event == b"release" else motion)


@pytest.mark.parametrize(
    ("event", "expected"),
    [(None, [1.0, 0.0]), (b"quit", [1.0, 1.0, 0.0]), (b"release", [1.0, 0.0])],
    ids=["module-stop", "quit-while-moving", "release-then-quit"],
)
def test_window_stops_outstanding_movement(event: bytes | None, expected: list[float]) -> None:
    output = Out(Twist, "cmd_vel")
    commands: list[float] = []
    moving = Event()

    def record(twist: Twist) -> None:
        commands.append(twist.linear.x)
        if twist.linear.x:
            moving.set()

    output.subscribe(record)
    window = WindowProcess(__name__, {}, {"cmd_vel": output}, lambda: output.publish(Twist()))
    try:
        assert moving.wait(timeout=5)
        if event is not None:
            window._connection.send_bytes(event)
            window._reader.join(timeout=5)
            assert not window._reader.is_alive()
    finally:
        window.stop()

    assert commands == expected


if __name__ == "__main__":
    _MovingWindow.serve()
