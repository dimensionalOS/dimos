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
from pathlib import Path
import sys
from types import ModuleType
from typing import Any, ClassVar

import can
import numpy as np
import pytest

from dimos.hardware.manipulators.galaxea_a1z import gs_usb_bus
from dimos.hardware.manipulators.galaxea_a1z.config import (
    A1ZConfig,
    A1ZGripperConfig,
    A1ZTeachingConfig,
)


class _FakeBus:
    def __init__(self) -> None:
        self.shut_down = False

    def shutdown(self) -> None:
        self.shut_down = True


class _FakeMotor:
    def __init__(self) -> None:
        self.last_feedback = object()  # motor has reported

    def disable(self) -> None:
        pass


class _FakeGripper:
    def __init__(self) -> None:
        self._motor = _FakeMotor()
        self.feedback_fraction = 0.0

    def get_feedback_norm(self) -> float:
        return self.feedback_fraction


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
        self._default_kp = np.asarray(factory_kwargs["default_kp"], dtype=float)
        self._default_kd = np.asarray(factory_kwargs["default_kd"], dtype=float)
        self.actions: list[Any] = []
        self.gravity_factor_history: list[float] = []
        self.gravity_comp_factor = float(factory_kwargs["gravity_comp_factor"])
        self.full_gravity_velocity_samples: list[np.ndarray] = []
        self._motor_chain = _FakeMotorChain()
        self.move_during_gravity_ramp = False
        self.stop_during_gravity_ramp = False
        self.state = {
            "pos": np.zeros(6),
            "vel": np.zeros(6),
            "eff": np.zeros(6),
            "error_codes": np.ones(6, dtype=int),  # 0x1 = normal
            "temp_mos": np.full(6, 35.0),
            "temp_rotor": np.full(6, 40.0),
        }
        if factory_kwargs["with_gripper"]:
            self.gripper = _FakeGripper()

    @property
    def gravity_comp_factor(self) -> float:
        return self._gravity_comp_factor

    @gravity_comp_factor.setter
    def gravity_comp_factor(self, value: float) -> None:
        self._gravity_comp_factor = value
        self.gravity_factor_history.append(value)
        if value <= 0 or "state" not in self.__dict__:
            return
        if self.move_during_gravity_ramp:
            self.state["vel"][2] = 0.75
        if self.stop_during_gravity_ramp:
            self._running = False

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_estopped(self) -> bool:
        return self._estopped

    def start(self, initial_kp: Any = None, initial_kd: Any = None) -> None:
        self._running = True
        self._estopped = False
        # Model the real SDK boundary that caused the hardware regression:
        # feedforward present when the loop starts can move the arm even at kp=0.
        if self.gravity_comp_factor > 0 and initial_kp is not None and np.allclose(initial_kp, 0):
            self.state["vel"][1] = self.gravity_comp_factor
        self.actions.append(("start", initial_kp, initial_kd, self.gravity_comp_factor))
        self.actions.append("start")

    def command_joint_state(self, joint_state: dict[str, np.ndarray]) -> None:
        self.actions.append(("command_joint_state", joint_state, self.gravity_comp_factor))

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
        state = dict(self.state)
        configured_gravity = float(self.factory_kwargs["gravity_comp_factor"])
        if self.full_gravity_velocity_samples and np.isclose(
            self.gravity_comp_factor, configured_gravity
        ):
            state["vel"] = self.full_gravity_velocity_samples.pop(0).copy()
        return state

    def get_robot_info(self) -> dict[str, Any]:
        return {
            "default_kp": self._default_kp.copy(),
            "default_kd": self._default_kd.copy(),
        }

    def command_joint_pos(self, pos: np.ndarray) -> None:
        self.actions.append(("command_joint_pos", pos.tolist()))

    def move_joints(self, target_pos: np.ndarray, speed: float = 0.5) -> None:
        self.actions.append(("move_joints", target_pos.tolist(), speed))

    def command_gripper(self, value: float) -> None:
        if not self.factory_kwargs.get("with_gripper"):
            raise RuntimeError("No gripper attached. Pass gripper= to get_a1z_robot().")
        self.gripper_fraction = value
        self.gripper.feedback_fraction = value
        self.actions.append(("command_gripper", value))

    def get_gripper_pos(self) -> float | None:
        if not self.factory_kwargs.get("with_gripper"):
            return None
        return self.gripper.feedback_fraction

    def set_gripper_free_drive(self, enabled: bool) -> None:
        if not self.factory_kwargs.get("with_gripper"):
            raise RuntimeError("No gripper attached")
        self.actions.append(("set_gripper_free_drive", enabled))


class _FakeKinematics:
    def __init__(self, urdf_path: str) -> None:
        self.urdf_path = urdf_path

    def fk(self, _positions: np.ndarray) -> np.ndarray:
        return np.eye(4)


@pytest.fixture
def a1z_adapter_module(
    monkeypatch: pytest.MonkeyPatch,
    mocker,
    tmp_path: Path,
) -> Iterator[ModuleType]:
    _FakeArmRobot.instances.clear()

    a1z_pkg = ModuleType("a1z")
    a1z_pkg.__file__ = str(tmp_path / "a1z" / "__init__.py")
    a1z_robots = ModuleType("a1z.robots")
    a1z_arm_robot = ModuleType("a1z.robots.arm_robot")
    a1z_get_robot = ModuleType("a1z.robots.get_robot")
    a1z_kinematics = ModuleType("a1z.robots.kinematics")
    a1z_arm_robot.ArmRobot = _FakeArmRobot
    a1z_get_robot.get_a1z_robot = _FakeArmRobot
    a1z_kinematics.Kinematics = _FakeKinematics

    monkeypatch.setitem(sys.modules, "a1z", a1z_pkg)
    monkeypatch.setitem(sys.modules, "a1z.robots", a1z_robots)
    monkeypatch.setitem(sys.modules, "a1z.robots.arm_robot", a1z_arm_robot)
    monkeypatch.setitem(sys.modules, "a1z.robots.get_robot", a1z_get_robot)
    monkeypatch.setitem(sys.modules, "a1z.robots.kinematics", a1z_kinematics)
    sys.modules.pop("dimos.hardware.manipulators.galaxea_a1z.adapter", None)
    module = importlib.import_module("dimos.hardware.manipulators.galaxea_a1z.adapter")
    mocker.patch.object(module.time, "sleep")
    sys_class_net = tmp_path / "net"
    interface = sys_class_net / "can0"
    driver_path = tmp_path / "drivers" / "gs_usb"
    (interface / "device").mkdir(parents=True)
    driver_path.mkdir(parents=True, exist_ok=True)
    (interface / "flags").write_text("0x1\n")
    (interface / "device" / "driver").symlink_to(driver_path, target_is_directory=True)
    monkeypatch.setattr(module, "_SYS_CLASS_NET", sys_class_net)
    yield module
    sys.modules.pop("dimos.hardware.manipulators.galaxea_a1z.adapter", None)


def _connected_adapter(module: ModuleType, **kwargs: Any) -> tuple[Any, _FakeArmRobot]:
    gripper_enabled = bool(kwargs.pop("gripper", False))
    gripper_free_drive = bool(kwargs.pop("gripper_free_drive", False))
    teaching_enabled = bool(kwargs.pop("zero_gravity", False))
    gripper = (
        A1ZGripperConfig(
            max_torque=kwargs.pop("gripper_max_torque", 0.5),
            max_opening_m=kwargs.pop("gripper_max_opening_m", 0.1),
        )
        if gripper_enabled
        else None
    )
    teaching = (
        A1ZTeachingConfig(gripper_free_drive=gripper_free_drive) if teaching_enabled else None
    )
    config = A1ZConfig(
        gravity_comp_factor=kwargs.pop("gravity_comp_factor", 1.0),
        urdf_path=kwargs.pop("urdf_path", None),
        default_kp=kwargs.pop("default_kp", (80.0, 80.0, 80.0, 50.0, 20.0, 20.0)),
        default_kd=kwargs.pop("default_kd", (3.0, 3.0, 3.0, 0.7, 0.4, 0.4)),
        gripper=gripper,
        teaching=teaching,
    )
    assert not kwargs
    adapter = module.GalaxeaA1ZAdapter(address="can0", config=config, dof=6 + int(gripper_enabled))
    assert adapter.connect()
    return adapter, _FakeArmRobot.instances[-1]


def test_connect_opens_bus_without_powering_motors(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module, gravity_comp_factor=0.7)

    assert adapter.is_connected()
    assert "start" not in robot.actions
    assert not adapter.read_enabled()


def test_connect_forwards_configured_arm_gains_to_sdk(
    a1z_adapter_module: ModuleType,
) -> None:
    kp = (70.0, 70.0, 70.0, 40.0, 15.0, 15.0)
    kd = (2.5, 2.5, 2.5, 0.6, 0.3, 0.3)

    _, robot = _connected_adapter(a1z_adapter_module, default_kp=kp, default_kd=kd)

    assert robot.factory_kwargs["default_kp"] == pytest.approx(kp)
    assert robot.factory_kwargs["default_kd"] == pytest.approx(kd)


def test_safe_start_stages_measured_hold_before_gravity_feedforward(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module, gravity_comp_factor=1.0)
    robot.state["pos"] = np.array([0.1, 0.5, -0.5, 0.2, -0.1, 0.3])

    assert adapter.activate()

    start_call = next(a for a in robot.actions if isinstance(a, tuple) and a[0] == "start")
    assert np.allclose(start_call[1], np.zeros(6))
    assert start_call[3] == 0.0

    hold = [a for a in robot.actions if isinstance(a, tuple) and a[0] == "command_joint_state"]
    assert len(hold) == 1
    assert np.allclose(hold[0][1]["pos"], robot.state["pos"])
    assert hold[0][2] == 0.0
    assert robot.gravity_comp_factor == 1.0


def test_safe_start_requires_feedback_from_all_six_motors(
    a1z_adapter_module: ModuleType,
    mocker,
    capsys: pytest.CaptureFixture[str],
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    robot._motor_chain._motor_b_list[-1].last_feedback = None
    mocker.patch.object(a1z_adapter_module, "_STARTUP_FEEDBACK_TIMEOUT_S", 0.0)

    assert not adapter.activate()

    assert "no feedback from all motors" in capsys.readouterr().out
    assert not adapter.read_enabled()


def test_safe_start_tolerates_one_noisy_settling_velocity_sample(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    stable = np.full(6, 0.05)
    noisy = stable.copy()
    noisy[1] = 0.119
    # The first sample is consumed by the final gravity-ramp validation.
    robot.full_gravity_velocity_samples = [
        np.zeros(6),
        *[stable for _ in range(4)],
        noisy,
        *[stable for _ in range(5)],
    ]

    assert adapter.activate()
    assert adapter.read_enabled()


def test_safe_start_rejects_sustained_motion_during_settling(
    a1z_adapter_module: ModuleType,
    capsys: pytest.CaptureFixture[str],
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    moving = np.zeros(6)
    moving[1] = 0.119
    # Include the final gravity-ramp sample, then exceed the full settling window.
    robot.full_gravity_velocity_samples = [np.zeros(6), *[moving for _ in range(100)]]

    assert not adapter.activate()

    output = capsys.readouterr().out
    assert "arm did not settle within 1.0 s" in output
    assert "velocities=[0.0, 0.119, 0.0, 0.0, 0.0, 0.0]" in output
    assert robot.gravity_comp_factor == 0.0


def test_zero_gravity_uses_vendor_teaching_startup(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(
        a1z_adapter_module,
        gravity_comp_factor=0.7,
        zero_gravity=True,
    )

    assert adapter.activate()

    start_call = next(a for a in robot.actions if isinstance(a, tuple) and a[0] == "start")
    assert start_call[1] is None
    assert start_call[2] is None
    assert start_call[3] == 0.7
    assert not any(
        isinstance(action, tuple) and action[0] == "command_joint_state" for action in robot.actions
    )
    assert 0.0 not in robot.gravity_factor_history
    assert robot.gravity_comp_factor == 0.7


def test_safe_start_aborts_motion_during_gravity_ramp(
    a1z_adapter_module: ModuleType,
    capsys: pytest.CaptureFixture[str],
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    robot.move_during_gravity_ramp = True

    assert not adapter.activate()

    output = capsys.readouterr().out
    assert "arm moving during" in output
    assert "joint3=0.750 rad/s" in output
    assert robot.gravity_comp_factor == 0.0


def test_safe_start_rejects_dead_sdk_control_loop(
    a1z_adapter_module: ModuleType,
    capsys: pytest.CaptureFixture[str],
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    robot.stop_during_gravity_ramp = True

    assert not adapter.activate()

    output = capsys.readouterr().out
    assert "SDK control loop stopped during" in output
    assert robot.gravity_comp_factor == 0.0


def test_safe_start_refuses_out_of_limit_pose(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    robot.state["pos"] = np.array([0.0, 0.0, 0.0, -1.7, 0.0, 0.0])  # joint4 beyond limit

    assert not adapter.activate()

    assert not adapter.read_enabled()


def test_write_enable_false_stops_control_loop(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    assert adapter.activate()

    assert adapter.write_enable(False)

    assert "stop" in robot.actions
    assert not adapter.read_enabled()


def test_disconnect_stops_robot_and_closes_bus(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    assert adapter.activate()

    adapter.disconnect()

    assert "stop" in robot.actions
    assert robot._bus.shut_down
    assert not adapter.is_connected()


def test_servo_position_mode_streams_command_joint_pos(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
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
    adapter, robot = _connected_adapter(a1z_adapter_module)
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
    adapter, robot = _connected_adapter(a1z_adapter_module)
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


def test_motor_fault_is_reported_as_error_state(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module)
    assert adapter.activate()

    robot.state["error_codes"] = np.array([1, 1, 8, 1, 1, 1])

    code, message = adapter.read_error()
    assert code == 8
    assert "joint 3" in message
    assert adapter.read_state()["state"] == 2


def test_gripper_round_trips_meters_to_normalized(
    a1z_adapter_module: ModuleType,
) -> None:
    """The gripper rides the joint array; metres in, SDK fraction out."""
    adapter, robot = _connected_adapter(a1z_adapter_module, gripper=True, gripper_max_opening_m=0.1)
    assert robot.factory_kwargs["with_gripper"] is True
    assert adapter.activate()

    assert adapter.get_dof() == 7
    limits = adapter.get_limits()
    assert len(limits.position_upper) == adapter.get_dof()
    assert limits.position_upper[-1] == pytest.approx(0.1)  # metres, declared

    assert adapter.write_joint_positions([0.0] * 6 + [0.05])  # half open
    assert robot.gripper_fraction == pytest.approx(0.5)
    assert adapter.read_joint_positions()[-1] == pytest.approx(0.05)

    # Out-of-range commands clamp to the physical stroke
    assert adapter.write_joint_positions([0.0] * 6 + [1.0])
    assert robot.gripper_fraction == pytest.approx(1.0)


def test_connect_applies_configured_gripper_force(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(a1z_adapter_module, gripper=True)

    assert adapter.is_connected()
    assert robot.factory_kwargs["gripper_max_torque"] == pytest.approx(0.5)


def test_configured_gripper_free_drive_tracks_adapter_lifecycle(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter, robot = _connected_adapter(
        a1z_adapter_module,
        gripper=True,
        gripper_free_drive=True,
        zero_gravity=True,
    )

    assert adapter.activate()
    assert ("set_gripper_free_drive", True) in robot.actions

    assert adapter.deactivate()
    assert ("set_gripper_free_drive", False) in robot.actions


def test_set_control_mode_rejects_unsupported_modes(
    a1z_adapter_module: ModuleType,
) -> None:
    adapter = a1z_adapter_module.GalaxeaA1ZAdapter(address="can0")

    assert not adapter.set_control_mode(a1z_adapter_module.ControlMode.VELOCITY)
    assert not adapter.set_control_mode(a1z_adapter_module.ControlMode.TORQUE)
    assert adapter.set_control_mode(a1z_adapter_module.ControlMode.POSITION)
    assert adapter.set_control_mode(a1z_adapter_module.ControlMode.SERVO_POSITION)


def test_gs_usb_transport_swaps_bus_during_factory_call(
    a1z_adapter_module: ModuleType,
    mocker,
) -> None:
    mocker.patch.object(a1z_adapter_module.platform, "system", return_value="Darwin")

    class _FakeGsBus:
        def __init__(self, **kwargs: Any) -> None:
            self.kwargs = kwargs

    mocker.patch.object(gs_usb_bus, "GsUsbMacBus", new=_FakeGsBus)

    seen: dict[str, Any] = {}

    def _factory_calling_can_bus(**kwargs: Any) -> _FakeArmRobot:
        seen["bus"] = can.interface.Bus(channel="can0", bustype="socketcan", bitrate=1_000_000)
        return _FakeArmRobot(**kwargs)

    mocker.patch.object(
        a1z_adapter_module,
        "get_a1z_robot",
        side_effect=_factory_calling_can_bus,
    )

    original_bus = can.interface.Bus
    adapter = a1z_adapter_module.GalaxeaA1ZAdapter(address="can0")
    assert adapter.connect()

    assert isinstance(seen["bus"], _FakeGsBus)
    assert can.interface.Bus is original_bus  # patch is scoped to the call


def test_socketcan_connect_fails_closed_before_sdk_construction(
    a1z_adapter_module: ModuleType,
    mocker,
    capsys: pytest.CaptureFixture[str],
) -> None:
    mocker.patch.object(a1z_adapter_module.platform, "system", return_value="Linux")
    mocker.patch.object(
        a1z_adapter_module,
        "_socketcan_channel_error",
        return_value="SocketCAN interface 'can0' belongs to kernel driver 'mttcan'",
    )

    adapter = a1z_adapter_module.GalaxeaA1ZAdapter(address="can0")
    assert not adapter.connect()
    assert not _FakeArmRobot.instances
    assert "belongs to kernel driver 'mttcan'" in capsys.readouterr().out


@pytest.mark.parametrize(
    ("driver", "flags", "expected_error"),
    [
        (
            "mttcan",
            "0x1\n",
            (
                "SocketCAN interface 'can7' belongs to kernel driver 'mttcan', not the HHS "
                "adapter driver 'gs_usb'. Pass the HHS SocketCAN interface with --can-port."
            ),
        ),
        (
            "gs_usb",
            "0x0\n",
            (
                "SocketCAN interface 'can7' is DOWN. Configure it for 1 Mbit/s and bring it UP "
                "before starting DimOS."
            ),
        ),
        ("gs_usb", "0x1\n", None),
    ],
)
def test_socketcan_channel_validation_requires_up_gs_usb_interface(
    a1z_adapter_module: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    driver: str,
    flags: str,
    expected_error: str | None,
) -> None:
    sys_class_net = tmp_path / "net"
    interface = sys_class_net / "can7"
    driver_path = tmp_path / "drivers" / driver
    (interface / "device").mkdir(parents=True)
    driver_path.mkdir(parents=True, exist_ok=True)
    (interface / "flags").write_text(flags)
    (interface / "device" / "driver").symlink_to(driver_path, target_is_directory=True)
    monkeypatch.setattr(a1z_adapter_module, "_SYS_CLASS_NET", sys_class_net)

    error = a1z_adapter_module._socketcan_channel_error("can7")

    assert error == expected_error
