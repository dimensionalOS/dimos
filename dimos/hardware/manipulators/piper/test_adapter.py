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

from dataclasses import dataclass

import dimos.hardware.manipulators.piper.adapter as adapter_mod
from dimos.hardware.manipulators.piper.adapter import PiperAdapter


@dataclass
class _JointState:
    joint_1: int
    joint_2: int
    joint_3: int
    joint_4: int
    joint_5: int
    joint_6: int


@dataclass
class _JointMsgs:
    joint_state: _JointState


class _FakePiperSdk:
    def __init__(self, joint_positions: list[_JointState]) -> None:
        self.calls: list[tuple[str, tuple[object, ...]]] = []
        self._joint_positions = joint_positions

    def MotionCtrl_2(
        self,
        *,
        ctrl_mode: int,
        move_mode: int,
        move_spd_rate_ctrl: int,
        is_mit_mode: int,
    ) -> None:
        self.calls.append(("MotionCtrl_2", (ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode)))

    def JointCtrl(
        self, joint_1: int, joint_2: int, joint_3: int, joint_4: int, joint_5: int, joint_6: int
    ) -> None:
        self.calls.append(("JointCtrl", (joint_1, joint_2, joint_3, joint_4, joint_5, joint_6)))

    def GetArmJointMsgs(self) -> _JointMsgs:
        joint_state = (
            self._joint_positions.pop(0) if self._joint_positions else _JointState(0, 0, 0, 0, 0, 0)
        )
        return _JointMsgs(joint_state=joint_state)

    def DisablePiper(self) -> None:
        self.calls.append(("DisablePiper", ()))

    def DisconnectPort(self) -> None:
        self.calls.append(("DisconnectPort", ()))

    def EmergencyStop(self) -> None:
        self.calls.append(("EmergencyStop", ()))

    def GripperCtrl(self, angle: int, speed: int, mode: int, effort: int) -> None:
        self.calls.append(("GripperCtrl", (angle, speed, mode, effort)))


def _connected_adapter(fake_sdk: _FakePiperSdk) -> PiperAdapter:
    adapter = PiperAdapter()
    adapter._sdk = fake_sdk
    adapter._connected = True
    adapter._enabled = True
    return adapter


def test_disconnect_moves_to_zero_before_disabling() -> None:
    fake_sdk = _FakePiperSdk([_JointState(0, 0, 0, 0, 0, 0)])
    adapter = _connected_adapter(fake_sdk)

    adapter.disconnect()

    assert fake_sdk.calls == [
        ("MotionCtrl_2", (0x01, 0x01, adapter_mod.PIPER_SHUTDOWN_SPEED_RATE, 0x00)),
        ("JointCtrl", (0, 0, 0, 0, 0, 0)),
        ("GripperCtrl", (0, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x02, 0)),
        ("DisablePiper", ()),
        ("DisconnectPort", ()),
    ]
    assert adapter._sdk is None
    assert adapter._connected is False
    assert adapter._enabled is False


def test_disconnect_disables_and_disconnects_when_zero_move_times_out(monkeypatch) -> None:
    monkeypatch.setattr(adapter_mod, "PIPER_SHUTDOWN_TIMEOUT_S", 0.0)
    fake_sdk = _FakePiperSdk([_JointState(1000, 1000, 1000, 1000, 1000, 1000)])
    adapter = _connected_adapter(fake_sdk)

    adapter.disconnect()

    assert ("JointCtrl", (0, 0, 0, 0, 0, 0)) in fake_sdk.calls
    assert ("GripperCtrl", (0, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x02, 0)) in fake_sdk.calls
    assert ("DisablePiper", ()) in fake_sdk.calls
    assert fake_sdk.calls[-1] == ("DisconnectPort", ())


def test_write_stop_keeps_emergency_stop_behavior() -> None:
    fake_sdk = _FakePiperSdk([_JointState(0, 0, 0, 0, 0, 0)])
    adapter = _connected_adapter(fake_sdk)

    assert adapter.write_stop() is True
    assert fake_sdk.calls == [("EmergencyStop", ())]


def test_write_gripper_position_initializes_gripper_once() -> None:
    fake_sdk = _FakePiperSdk([_JointState(0, 0, 0, 0, 0, 0)])
    adapter = _connected_adapter(fake_sdk)

    assert adapter.write_gripper_position(0.08) is True

    assert fake_sdk.calls == [
        ("GripperCtrl", (0, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x02, 0)),
        ("GripperCtrl", (0, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x01, 0)),
        ("GripperCtrl", (80000, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x01, 0)),
    ]


def test_write_gripper_position_second_call_skips_reinit() -> None:
    fake_sdk = _FakePiperSdk([_JointState(0, 0, 0, 0, 0, 0)])
    adapter = _connected_adapter(fake_sdk)

    assert adapter.write_gripper_position(0.08) is True
    assert adapter.write_gripper_position(0.0) is True

    assert fake_sdk.calls == [
        ("GripperCtrl", (0, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x02, 0)),
        ("GripperCtrl", (0, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x01, 0)),
        ("GripperCtrl", (80000, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x01, 0)),
        ("GripperCtrl", (0, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x01, 0)),
    ]


def test_write_gripper_position_clamps_to_max_stroke() -> None:
    fake_sdk = _FakePiperSdk([_JointState(0, 0, 0, 0, 0, 0)])
    adapter = _connected_adapter(fake_sdk)

    assert adapter.write_gripper_position(200.0) is True

    assert fake_sdk.calls[-1] == (
        "GripperCtrl",
        (80000, adapter_mod.DEFAULT_GRIPPER_SPEED, 0x01, 0),
    )
