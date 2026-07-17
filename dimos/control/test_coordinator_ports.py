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

"""The coordinator's path/speed broadcast: ``_on_path`` -> ``set_path`` and
``_on_speed`` -> ``set_speed`` on every task that exposes them."""

from __future__ import annotations

import threading

from dimos.control.coordinator import ControlCoordinator
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.std_msgs.Float32 import Float32


class _FakeCoord:
    """Binds the real coordinator broadcast methods onto a lightweight object so
    we exercise their logic without constructing a Module (which spawns threads).
    No hardware is wired, so ``_read_base_odom`` returns None."""

    # Bind the real implementations as methods.
    _on_speed = ControlCoordinator._on_speed
    _on_path = ControlCoordinator._on_path
    _read_base_odom = ControlCoordinator._read_base_odom

    def __init__(self, tasks: dict) -> None:
        self._tasks = tasks
        self._task_lock = threading.Lock()
        self._hardware: dict = {}
        self._hardware_lock = threading.Lock()


class _StubTask:
    """A task exposing set_path/set_speed; records what it received."""

    def __init__(self, name: str = "stub") -> None:
        self.name = name
        self.paths: list = []
        self.speeds: list[float] = []

    def set_path(self, path, odom=None) -> None:
        self.paths.append((path, odom))

    def set_speed(self, speed: float) -> None:
        self.speeds.append(speed)


class _NoBroadcastTask:
    """A task with neither set_path nor set_speed — must be skipped, not error."""

    name = "no_broadcast"


def _path() -> Path:
    return Path(
        poses=[
            PoseStamped(position=Vector3(0.0, 0.0, 0.0)),
            PoseStamped(position=Vector3(1.0, 0.0, 0.0)),
        ]
    )


def test_on_speed_broadcasts_to_tasks_with_set_speed():
    stub, other = _StubTask(), _NoBroadcastTask()
    coord = _FakeCoord({"stub": stub, "other": other})
    coord._on_speed(Float32(data=0.6))
    assert stub.speeds == [0.6]


def test_on_path_broadcasts_to_tasks_with_set_path():
    stub = _StubTask()
    coord = _FakeCoord({"stub": stub})
    path = _path()
    coord._on_path(path)
    assert len(stub.paths) == 1
    received_path, received_odom = stub.paths[0]
    assert received_path is path
    # No hardware wired -> no odom snapshot; set_path still called with None.
    assert received_odom is None


def test_on_path_falls_back_to_single_arg_set_path():
    """Tasks with single-arg set_path(path) still work (TypeError fallback)."""
    received: list = []

    class _SingleArg:
        name = "single"

        def set_path(self, path) -> None:  # no odom param
            received.append(path)

    coord = _FakeCoord({"single": _SingleArg()})
    path = _path()
    coord._on_path(path)
    assert received == [path]
