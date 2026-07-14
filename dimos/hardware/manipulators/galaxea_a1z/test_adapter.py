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

from collections.abc import Iterator
import importlib
import sys
from types import ModuleType
from typing import Any, ClassVar

import numpy as np
import pytest


class _FakeBus:
    def __init__(self) -> None:
        self.shut_down = False

    def shutdown(self) -> None:
        self.shut_down = True


class _FakeMotor:
    def __init__(self) -> None:
        self.last_feedback = object()  # motor has reported


class _FakeMotorChain:
    def __init__(self) -> None:
        self._motor_a_list = [_FakeMotor() for _ in range(3)]
        self._motor_b_list = [_FakeMotor() for _ in range(3)]


class _FakeArmRobot:
    """Mirrors the a1z ArmRobot surface the adapter relies on.

    Matches the real SDK: is_running/is_estopped are properties, start()
    enables motors and clears the e-stop latch, stop() disables motors.
    """

    instances: ClassVar[list[_FakeArmRobot]] = []

    def __init__(self, **factory_kwargs: Any) -> None:
        self.__class__.instances.append(self)
        self.factory_kwargs = factory_kwargs
        self._running = False
        self._estopped = False
        self._bus = _FakeBus()
        self._default_kp = np.array([30.0, 30.0, 30.0, 20.0, 5.0, 5.0])
        self._default_kd = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])
        self._motor_chain = _FakeMotorChain()
        self.actions: list[Any] = []
        self.state = {
            "pos": np.zeros(6),
            "vel": np.zeros(6),
            "eff": np.zeros(6),
            "error_codes": np.ones(6, dtype=int),  # 0x1 = normal
            "temp_mos": np.full(6, 35.0),
            "temp_rotor": np.full(6, 40.0),
        }

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_estopped(self) -> bool:
        return self._estopped

    def start(self, initial_kp: Any = None, initial_kd: Any = None) -> None:
        self._running = True
        self._estopped = False
        self.actions.append(("start", initial_kp, initial_kd))
        self.actions.append("start")

    def command_joint_state(self, joint_state: dict[str, np.ndarray]) -> None:
        self.actions.append(("command_joint_state", joint_state))

    def stop(self) -> None:
        self._running = False
        self.actions.append("stop")

    def estop(self) -> None:
        self._estopped = True
        self.actions.append("estop")

    def release(self) -> None:
        self._estopped = False
        self.actions.append("release")

    def get_joint_state(self) -> dict[str, np.ndarray]:
        return dict(self.state)

    def command_joint_pos(self, pos: np.ndarray) -> None:
        self.actions.append(("command_joint_pos", pos.tolist()))

    def move_joints(self, target_pos: np.ndarray, speed: float = 0.5) -> None:
        self.actions.append(("move_joints", target_pos.tolist(), speed))

    def command_gripper(self, value: float) -> None:
        if not self.factory_kwargs.get("with_gripper"):
            raise RuntimeError("No gripper attached. Pass gripper= to get_a1z_robot().")
        self.gripper_fraction = value
        self.actions.append(("command_gripper", value))

    def get_gripper_pos(self) -> float | None:
        if not self.factory_kwargs.get("with_gripper"):
            return None
        return getattr(self, "gripper_fraction", 0.0)


@pytest.fixture
def a1z_adapter_module(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    _FakeArmRobot.instances.clear()

    a1z_pkg = ModuleType("a1z")
    a1z_robots = ModuleType("a1z.robots")
    a1z_get_robot = ModuleType("a1z.robots.get_robot")
    a1z_get_robot.get_a1z_robot = _FakeArmRobot  # type: ignore[attr-defined]

    monkeypatch.setitem(sys.modules, "a1z", a1z_pkg)
    monkeypatch.setitem(sys.modules, "a1z.robots", a1z_robots)
    monkeypatch.setitem(sys.modules, "a1z.robots.get_robot", a1z_get_robot)
    sys.modules.pop("dimos.hardware.manipulators.galaxea_a1z.adapter", None)
    yield importlib.import_module("dimos.hardware.manipulators.galaxea_a1z.adapter")
    sys.modules.pop("dimos.hardware.manipulators.galaxea_a1z.adapter", None)


def _connected_adapter(module: ModuleType, **kwargs: Any) -> Any:
    adapter = module.GalaxeaA1ZAdapter(address="can0", **kwargs)
    assert adapter.connect()
    return adapter


def test_connect_constructs_robot_without_powering_motors(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module, gravity_comp_factor=0.7)
    robot = _FakeArmRobot.instances[-1]

    assert adapter.is_connected()
    assert robot.factory_kwargs["can_channel"] == "can0"
    assert robot.factory_kwargs["gravity_comp_factor"] == 0.7
    assert "start" not in robot.actions
    assert not adapter.read_enabled()


def test_activate_starts_control_loop_and_enables(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]

    assert adapter.activate()

    assert "start" in robot.actions
    assert adapter.read_enabled()


def test_safe_start_never_commands_force_toward_unverified_pose(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]
    robot.state["pos"] = np.array([0.1, 0.5, -0.5, 0.2, -0.1, 0.3])

    assert adapter.activate()

    start_call = next(a for a in robot.actions if isinstance(a, tuple) and a[0] == "start")
    assert np.allclose(start_call[1], np.zeros(6))  # kp=0: snap impossible

    engage = next(
        a for a in robot.actions if isinstance(a, tuple) and a[0] == "command_joint_state"
    )
    js = engage[1]
    assert np.allclose(js["pos"], robot.state["pos"])  # hold target = measured pose
    assert np.allclose(js["kp"], robot._default_kp)


def test_safe_start_false_uses_vendor_stock_startup(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module, safe_start=False)
    robot = _FakeArmRobot.instances[-1]

    assert adapter.activate()

    start_call = next(a for a in robot.actions if isinstance(a, tuple) and a[0] == "start")
    assert start_call[1] is None  # vendor defaults, no gain override
    assert not any(isinstance(a, tuple) and a[0] == "command_joint_state" for a in robot.actions)


def test_safe_start_refuses_out_of_limit_pose(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]
    robot.state["pos"] = np.array([0.0, 0.0, 0.0, -1.7, 0.0, 0.0])  # joint4 beyond limit

    assert not adapter.activate()

    assert "stop" in robot.actions  # motors disabled, not yanked into range
    assert not any(isinstance(a, tuple) and a[0] == "command_joint_state" for a in robot.actions)


def test_write_enable_false_stops_control_loop(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]
    assert adapter.activate()

    assert adapter.write_enable(False)

    assert "stop" in robot.actions
    assert not adapter.read_enabled()


def test_disconnect_stops_robot_and_closes_bus(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]
    assert adapter.activate()

    adapter.disconnect()

    assert "stop" in robot.actions
    assert robot._bus.shut_down
    assert not adapter.is_connected()


def test_joint_state_reads_pass_through_si_units(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]

    robot.state["pos"] = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    robot.state["vel"] = np.array([1.0, 1.1, 1.2, 1.3, 1.4, 1.5])
    robot.state["eff"] = np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06])

    assert adapter.read_joint_positions() == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert adapter.read_joint_velocities() == pytest.approx([1.0, 1.1, 1.2, 1.3, 1.4, 1.5])
    assert adapter.read_joint_efforts() == pytest.approx([0.01, 0.02, 0.03, 0.04, 0.05, 0.06])


def test_write_joint_positions_requires_activation(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)

    assert not adapter.write_joint_positions([0.0] * 6)


def test_servo_position_mode_streams_command_joint_pos(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]
    assert adapter.activate()
    assert adapter.set_control_mode(a1z_adapter_module.ControlMode.SERVO_POSITION)

    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert adapter.write_joint_positions(positions)

    assert ("command_joint_pos", pytest.approx(positions)) in [
        (a[0], a[1]) for a in robot.actions if isinstance(a, tuple)
    ]


def test_position_mode_runs_planned_move_in_background(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]
    assert adapter.activate()

    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert adapter.write_joint_positions(positions, velocity=0.5)
    adapter._move_thread.join(timeout=1.0)

    moves = [a for a in robot.actions if isinstance(a, tuple) and a[0] == "move_joints"]
    assert len(moves) == 1
    assert moves[0][1] == pytest.approx(positions)
    assert moves[0][2] == pytest.approx(0.5 * a1z_adapter_module._PLANNED_SPEED_MAX_RAD_S)


def test_estop_latches_and_release_restores_commands(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]
    assert adapter.activate()

    assert adapter.write_stop()
    assert "estop" in robot.actions
    assert not adapter.read_enabled()
    assert not adapter.write_joint_positions([0.0] * 6)
    code, message = adapter.read_error()
    assert code != 0
    assert "e-stop" in message

    assert adapter.write_clear_errors()
    assert "release" in robot.actions
    assert adapter.write_joint_positions([0.0] * 6)


def test_read_state_reports_ints_and_motor_faults(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    robot = _FakeArmRobot.instances[-1]
    assert adapter.activate()

    state = adapter.read_state()
    assert state["state"] == 1
    assert state["error_code"] == 0
    assert isinstance(state["temp_mos_max"], int)

    robot.state["error_codes"] = np.array([1, 1, 8, 1, 1, 1])
    code, message = adapter.read_error()
    assert code == 8
    assert "joint 3" in message
    assert adapter.read_state()["state"] == 2


def test_unsupported_interfaces_signal_cleanly(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)
    assert adapter.activate()

    assert not adapter.write_joint_velocities([0.0] * 6)
    assert not adapter.write_cartesian_position(
        {"x": 0.3, "y": 0.0, "z": 0.4, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    )
    assert adapter.read_gripper_position() is None
    assert not adapter.write_gripper_position(0.05)
    assert adapter.read_force_torque() is None


def test_gripper_round_trips_meters_to_normalized(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module, gripper=True, gripper_max_opening_m=0.1)
    robot = _FakeArmRobot.instances[-1]
    assert robot.factory_kwargs["with_gripper"] is True
    assert adapter.activate()

    assert adapter.write_gripper_position(0.05)  # half open
    assert robot.gripper_fraction == pytest.approx(0.5)
    assert adapter.read_gripper_position() == pytest.approx(0.05)

    # Out-of-range commands clamp to the physical stroke
    assert adapter.write_gripper_position(1.0)
    assert robot.gripper_fraction == pytest.approx(1.0)


def test_gripper_disabled_signals_unsupported(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = _connected_adapter(a1z_adapter_module)  # gripper=False default
    assert adapter.activate()

    assert adapter.read_gripper_position() is None
    assert not adapter.write_gripper_position(0.05)


def test_set_control_mode_rejects_unsupported_modes(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = a1z_adapter_module.GalaxeaA1ZAdapter(address="can0")

    assert not adapter.set_control_mode(a1z_adapter_module.ControlMode.VELOCITY)
    assert not adapter.set_control_mode(a1z_adapter_module.ControlMode.TORQUE)
    assert adapter.set_control_mode(a1z_adapter_module.ControlMode.POSITION)
    assert adapter.set_control_mode(a1z_adapter_module.ControlMode.SERVO_POSITION)


def test_get_limits_match_sdk_joint_limits(
    a1z_adapter_module: ModuleType,
) -> None:
    limits = a1z_adapter_module.GalaxeaA1ZAdapter(address="can0").get_limits()

    assert limits.position_lower == pytest.approx([-2.094, 0.0, -3.142, -1.484, -1.484, -2.007])
    assert limits.position_upper == pytest.approx([2.094, 3.142, 0.0, 1.484, 1.484, 2.007])
    assert len(limits.velocity_max) == 6


def test_connect_fails_gracefully_without_sdk(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    for name in list(sys.modules):
        if name == "a1z" or name.startswith("a1z."):
            monkeypatch.delitem(sys.modules, name)
    monkeypatch.setitem(sys.modules, "a1z", None)  # force ImportError
    sys.modules.pop("dimos.hardware.manipulators.galaxea_a1z.adapter", None)
    module = importlib.import_module("dimos.hardware.manipulators.galaxea_a1z.adapter")

    adapter = module.GalaxeaA1ZAdapter(address="can0")
    assert not adapter.connect()
    assert not adapter.is_connected()


def test_gs_usb_transport_swaps_bus_during_factory_call(
    a1z_adapter_module: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import can

    class _FakeGsBus:
        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs

    import dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus as gs_usb_bus

    monkeypatch.setattr(gs_usb_bus, "GsUsbMacBus", _FakeGsBus)

    seen: dict[str, Any] = {}

    def _factory_calling_can_bus(**kwargs: Any) -> _FakeArmRobot:
        seen["bus"] = can.interface.Bus(channel="can0", bustype="socketcan", bitrate=1_000_000)
        return _FakeArmRobot(**kwargs)

    fake_get_robot = sys.modules["a1z.robots.get_robot"]
    monkeypatch.setattr(fake_get_robot, "get_a1z_robot", _factory_calling_can_bus)

    original_bus = can.interface.Bus
    adapter = a1z_adapter_module.GalaxeaA1ZAdapter(address="can0", transport="gs_usb")
    assert adapter.connect()

    assert isinstance(seen["bus"], _FakeGsBus)
    assert can.interface.Bus is original_bus  # patch is scoped to the call


def test_socketcan_transport_leaves_can_bus_untouched(
    a1z_adapter_module: ModuleType,
) -> None:
    import can

    original_bus = can.interface.Bus
    adapter = a1z_adapter_module.GalaxeaA1ZAdapter(address="can0", transport="socketcan")
    assert adapter.connect()
    assert can.interface.Bus is original_bus


def test_registry_entry_resolves() -> None:
    from dimos.hardware.manipulators.galaxea_a1z._registry import ADAPTER_FACTORIES

    module_path, _, class_name = ADAPTER_FACTORIES["galaxea_a1z"].partition(":")
    module = importlib.import_module(module_path)
    assert hasattr(module, class_name)
