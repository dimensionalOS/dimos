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

from pathlib import Path
import threading

import pytest

from dimos.simulation.manipulators.sim_module import SimulationModule


class _DummyRPC:
    def serve_module_rpc(self, _module) -> None:  # type: ignore[no-untyped-def]
        return None

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None


class _FakeBackend:
    def __init__(self) -> None:
        self._names = ["joint1", "joint2", "joint3"]

    def get_dof(self) -> int:
        return len(self._names)

    def get_joint_names(self) -> list[str]:
        return list(self._names)

    def read_joint_positions(self) -> list[float]:
        return [0.1, 0.2, 0.3]

    def read_joint_velocities(self) -> list[float]:
        return [0.0, 0.0, 0.0]

    def read_joint_efforts(self) -> list[float]:
        return [0.0, 0.0, 0.0]

    def read_state(self) -> dict[str, int]:
        return {"state": 1, "mode": 2}

    def read_error(self) -> tuple[int, str]:
        return 0, ""

    def read_enabled(self) -> bool:
        return True

    def disconnect(self) -> None:
        return None


def _run_single_monitor_iteration(module: SimulationModule, monkeypatch) -> None:  # type: ignore[no-untyped-def]
    def _wait_once(_: float) -> bool:
        module._stop_event.set()
        raise StopIteration

    monkeypatch.setattr(module._stop_event, "wait", _wait_once)
    with pytest.raises(StopIteration):
        module._monitor_loop()


def _run_single_control_iteration(module: SimulationModule, monkeypatch) -> None:  # type: ignore[no-untyped-def]
    def _wait_once(_: float) -> bool:
        module._stop_event.set()
        raise StopIteration

    monkeypatch.setattr(module._stop_event, "wait", _wait_once)
    with pytest.raises(StopIteration):
        module._control_loop()


def test_simulation_module_publishes_joint_state(monkeypatch) -> None:
    module = SimulationModule(
        engine="mujoco",
        config_path=Path("."),
        rpc_transport=_DummyRPC,
    )
    module._backend = _FakeBackend()  # type: ignore[assignment]
    module._stop_event = threading.Event()

    joint_states: list[object] = []
    module.joint_state.subscribe(joint_states.append)
    try:
        _run_single_control_iteration(module, monkeypatch)
    finally:
        module.stop()

    assert len(joint_states) == 1
    assert joint_states[0].name == ["joint1", "joint2", "joint3"]


def test_simulation_module_publishes_robot_state(monkeypatch) -> None:
    module = SimulationModule(
        engine="mujoco",
        config_path=Path("."),
        rpc_transport=_DummyRPC,
    )
    module._backend = _FakeBackend()  # type: ignore[assignment]
    module._stop_event = threading.Event()

    robot_states: list[object] = []
    module.robot_state.subscribe(robot_states.append)
    try:
        _run_single_monitor_iteration(module, monkeypatch)
    finally:
        module.stop()

    assert len(robot_states) == 1
    assert robot_states[0].state == 1
