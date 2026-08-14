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

import sys
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest
from typing_extensions import override

piper_sdk_module = ModuleType("piper_sdk")
piper_sdk_module.__dict__["C_PiperInterface_V2"] = lambda **_: None
sys.modules.setdefault("piper_sdk", piper_sdk_module)

from dimos.hardware.manipulators.a750.adapter import A750Adapter
from dimos.hardware.manipulators.openarm.adapter import OpenArmAdapter
from dimos.hardware.manipulators.piper import adapter as piper_adapter
from dimos.hardware.manipulators.piper.adapter import PiperAdapter


class _LifecyclePiperAdapter(PiperAdapter):
    def use_sdk(self, sdk: Any) -> None:
        self._sdk: Any
        self._sdk = sdk


@pytest.fixture
def piper_sdk(mocker: Any) -> Any:
    sdk = mocker.Mock()
    sdk.EnablePiper.return_value = True
    sdk.GetArmStatus.return_value = object()
    sdk.GetArmJointMsgs.return_value = SimpleNamespace(
        joint_state=SimpleNamespace(
            joint_1=0, joint_2=0, joint_3=0, joint_4=0, joint_5=0, joint_6=0
        )
    )
    sdk.gripper_position = 0
    sdk.GripperCtrl.side_effect = lambda position, *_: setattr(sdk, "gripper_position", position)
    sdk.GetArmGripperMsgs.side_effect = lambda: SimpleNamespace(
        gripper_state=SimpleNamespace(grippers_angle=sdk.gripper_position)
    )
    mocker.patch.object(piper_adapter, "C_PiperInterface_V2", lambda **_: sdk)
    mocker.patch.object(piper_adapter.time, "sleep")
    return sdk


def test_piper_lifecycle_enables_then_disables(piper_sdk: Any) -> None:
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(piper_sdk)

    assert adapter.activate()
    assert adapter.deactivate()
    piper_sdk.EnablePiper.assert_called_once_with()


def test_piper_disconnect_gracefully_stops_before_disabling(piper_sdk: Any) -> None:
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(piper_sdk)

    assert adapter.activate()
    adapter.disconnect()

    assert not adapter.is_connected()
    assert not adapter.read_enabled()
    piper_sdk.DisablePiper.assert_called_once_with()
    piper_sdk.DisconnectPort.assert_called_once_with()


def test_piper_explicit_stop_uses_motion_ctrl_1(piper_sdk: Any) -> None:
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(piper_sdk)

    assert adapter.write_stop()
    piper_sdk.MotionCtrl_1.assert_called_once_with(1, 0, 0)


def test_piper_connect_initializes_recovery_enable_zero_pose_and_gripper(
    piper_sdk: Any,
) -> None:
    adapter = PiperAdapter()

    assert adapter.connect()
    assert piper_sdk.ConnectPort.called
    assert piper_sdk.MotionCtrl_1.call_count == 2
    piper_sdk.JointCtrl.assert_called_once_with(0, 0, 0, 0, 0, 0)
    assert piper_sdk.GripperCtrl.called


def test_piper_connect_reset_failure_cleans_up_without_zero(
    mocker: Any,
) -> None:
    sdk = mocker.Mock()
    sdk.GetArmStatus.return_value = object()
    sdk.MotionCtrl_1.side_effect = RuntimeError("reset failed")
    mocker.patch.object(piper_adapter, "C_PiperInterface_V2", lambda **_: sdk)

    adapter = PiperAdapter()

    assert not adapter.connect()
    sdk.DisconnectPort.assert_called_once_with()
    sdk.JointCtrl.assert_not_called()


def test_piper_connect_does_not_enable_during_startup(
    piper_sdk: Any,
) -> None:
    adapter = PiperAdapter()

    assert adapter.connect()
    piper_sdk.EnablePiper.assert_not_called()


def test_piper_connect_joint_failure_cleans_up_without_gripper(
    mocker: Any,
) -> None:
    sdk = mocker.Mock()
    sdk.GetArmStatus.return_value = object()
    sdk.JointCtrl.side_effect = RuntimeError("joint command failed")
    mocker.patch.object(piper_adapter, "C_PiperInterface_V2", lambda **_: sdk)
    mocker.patch.object(piper_adapter.time, "sleep")

    adapter = PiperAdapter()

    assert not adapter.connect()
    sdk.DisconnectPort.assert_called_once_with()
    sdk.GripperCtrl.assert_not_called()


def test_piper_gripper_uses_sdk_units_and_clamps(piper_sdk: Any) -> None:
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(piper_sdk)

    assert adapter.write_gripper_position(0.1)
    assert piper_sdk.gripper_position == 80_000
    assert adapter.read_gripper_position() == 0.08
    assert piper_sdk.GripperCtrl.call_args.args[0] == 80_000


class _OpenArmLifecycle:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def enable_all(self) -> None:
        self.actions.append("enable")

    def disable_all(self) -> None:
        self.actions.append("disable")


class _LifecycleOpenArmAdapter(OpenArmAdapter):
    def __init__(self, lifecycle: _OpenArmLifecycle) -> None:
        super().__init__()
        self._lifecycle: _OpenArmLifecycle
        self._lifecycle = lifecycle

    @override
    def read_joint_positions(self) -> list[float]:
        return [0.0] * 7

    @override
    def _compute_gravity_torques(self, q: list[float]) -> list[float]:
        return [0.0] * len(q)

    @override
    def write_enable(self, enable: bool) -> bool:
        if enable:
            self._lifecycle.enable_all()
        else:
            self._lifecycle.disable_all()
        return True

    @override
    def write_stop(self) -> bool:
        self._lifecycle.actions.append("hold")
        return True


def test_openarm_lifecycle_enables_then_holds_and_disables() -> None:
    lifecycle = _OpenArmLifecycle()
    adapter = _LifecycleOpenArmAdapter(lifecycle)

    assert adapter.activate()
    assert adapter.deactivate()
    assert lifecycle.actions == ["enable", "hold", "disable"]


class _A750Robot:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def start_control_loop(self) -> None:
        self.actions.append("start")

    def stop_control_loop(self) -> None:
        self.actions.append("stop")


class _LifecycleA750Adapter(A750Adapter):
    def use_robot(self, robot: _A750Robot) -> None:
        self._robot: _A750Robot | None
        self._connected: bool
        self._robot = robot
        self._connected = True


def test_a750_lifecycle_starts_then_stops_control_loop() -> None:
    robot = _A750Robot()
    adapter = _LifecycleA750Adapter()
    adapter.use_robot(robot)

    assert adapter.activate()
    assert adapter.deactivate()
    assert robot.actions == ["start", "stop"]
