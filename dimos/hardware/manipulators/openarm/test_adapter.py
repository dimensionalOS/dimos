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

from __future__ import annotations

import time

import pytest

from dimos.hardware.manipulators.openarm.adapter import OpenArmAdapter, register
from dimos.hardware.manipulators.registry import AdapterRegistry
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter


class FakeState:
    def __init__(self, index: int) -> None:
        self.q: float = 0.1 * index
        self.dq: float = 0.2 * index
        self.tau: float = 0.3 * index
        self.t_rotor: float = 30 + index
        self.timestamp: float = time.monotonic()


class FakeOpenArmBus:
    last: FakeOpenArmBus | None = None

    def __init__(self, channel: str, motors: list[object], *, fd: bool, interface: str) -> None:
        FakeOpenArmBus.last = self
        self.channel: str = channel
        self.motors: list[object] = motors
        self.fd: bool = fd
        self.interface: str = interface
        self.opened: bool = False
        self.closed: bool = False
        self.disabled_count: int = 0
        self.ctrl_mode_ids: list[int] = []
        self.mit_commands: list[list[tuple[float, float, float, float, float]]] = []
        self.states: list[FakeState] = [FakeState(index) for index in range(len(motors))]

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.closed = True

    def write_ctrl_mode(self, send_id: int, _mode: int) -> None:
        self.ctrl_mode_ids.append(send_id)

    def get_states(self) -> list[FakeState]:
        return self.states

    def send_mit_many(self, commands: list[tuple[float, float, float, float, float]]) -> None:
        self.mit_commands.append(commands)

    def enable_all(self) -> None:
        return None

    def disable_all(self) -> None:
        self.disabled_count += 1


def test_implements_manipulator_adapter() -> None:
    assert isinstance(OpenArmAdapter(gravity_comp=False), ManipulatorAdapter)


def test_register_preserves_openarm_key() -> None:
    registry = AdapterRegistry()
    register(registry)
    adapter = registry.create("openarm", gravity_comp=False)
    assert isinstance(adapter, OpenArmAdapter)


def test_constructor_validates_dof_side_and_gain_lengths() -> None:
    with pytest.raises(ValueError, match="only supports 7 DOF"):
        _ = OpenArmAdapter(dof=6, gravity_comp=False)
    with pytest.raises(ValueError, match="side must be 'left' or 'right'"):
        _ = OpenArmAdapter(side="middle", gravity_comp=False)
    with pytest.raises(ValueError, match="kp/kd must be length 7"):
        _ = OpenArmAdapter(kp=[1.0], gravity_comp=False)
    with pytest.raises(ValueError, match="kp/kd must be length 7"):
        _ = OpenArmAdapter(kd=[1.0], gravity_comp=False)


def test_side_selects_openarm_identity_limits_and_modes() -> None:
    left = OpenArmAdapter(side="left", gravity_comp=False)
    right = OpenArmAdapter(side="right", gravity_comp=False)

    assert left.get_info().vendor == "Enactic"
    assert left.get_info().model == "OpenArm v10 (left)"
    assert right.get_info().model == "OpenArm v10 (right)"
    assert left.get_limits().position_lower[:2] == [-3.45, -3.30]
    assert right.get_limits().position_lower[:2] == [-1.35, -0.15]
    assert left.set_control_mode(ControlMode.VELOCITY) is True
    assert left.get_control_mode() == ControlMode.VELOCITY
    assert left.set_control_mode(ControlMode.CARTESIAN) is False


def test_disconnected_surface_returns_safe_defaults() -> None:
    adapter = OpenArmAdapter(gravity_comp=False)

    assert adapter.is_connected() is False
    assert adapter.write_joint_positions([0.0] * 7) is False
    assert adapter.write_stop() is False
    assert adapter.write_enable(True) is False


def test_lifecycle_state_commands_and_disconnect(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "dimos.hardware.manipulators.openarm.adapter.OpenArmBus",
        FakeOpenArmBus,
    )
    adapter = OpenArmAdapter(interface="virtual", gravity_comp=False)

    assert adapter.connect() is True
    bus = FakeOpenArmBus.last
    assert bus is not None
    assert bus.opened is True

    assert adapter.write_enable(True) is True
    assert adapter.read_enabled() is True
    assert [round(position, 1) for position in adapter.read_joint_positions()] == [
        0.0,
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6,
    ]

    assert adapter.write_joint_positions([0.1] * 7, velocity=0.5) is True
    assert bus.mit_commands[-1][0] == (0.1, 0.0, 50.0, 1.5, 0.0)
    assert adapter.write_stop() is True
    assert bus.mit_commands[-1][0] == (0.0, 0.0, 100.0, 1.5, 0.0)

    adapter.disconnect()
    assert adapter.read_enabled() is False
    assert bus.closed is True
    assert bus.disabled_count == 1
