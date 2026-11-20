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
from threading import Event
from typing import Any

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.get_all_blueprints import get_module_by_name
from dimos.teleop.keyboard.window import KeyboardWindow, WindowProcess


@pytest.fixture(params=["keyboard-teleop", "keyboard-teleop-module"])
def module(request) -> Iterator[Any]:
    (atom,) = autoconnect(get_module_by_name(request.param)).solidify().active_blueprints
    instance = atom.module()
    try:
        yield instance
    finally:
        instance.stop()


def test_start_reports_the_display_error(module, monkeypatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "unavailable-teleop-test")
    with pytest.raises(RuntimeError, match="unavailable-teleop-test"):
        module.start()


def test_stop_closes_the_open_display(module, monkeypatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    module.start()
    process = module._window._process
    assert process.poll() is None

    module.stop()

    assert process.returncode == 0


class _MovingWindow(KeyboardWindow):
    def open(self) -> None:
        pass

    def run(self) -> None:
        motion = Twist(linear=Vector3(1, 0, 0))
        self.publish("cmd_vel", motion)
        self.connection.recv_bytes()  # Stop requested while a key is held.
        self.publish("cmd_vel", motion)


def test_buffered_movement_cannot_follow_stop() -> None:
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
    finally:
        window.stop()

    assert commands == [1.0, 0.0]


if __name__ == "__main__":
    _MovingWindow.serve()
