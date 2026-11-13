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

import struct
from typing import Any

import pytest

from dimos.core.coordination.module_coordinator import (
    resolve_module_refs,
    verify_no_name_conflicts,
)
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.perception.localize.demo_blueprints.galaxea_a1z_localize import (
    check_usb_hardware,
    galaxea_a1z_localize,
)
from dimos.teleop.spacemouse.spacemouse_teleop_module import (
    LINUX_KEYCODES,
    SPNAV_EVENT_BUTTON,
    SPNAV_EVENT_BUTTON_RELEASE,
    SPNAV_EVENT_MOTION,
    SPNAV_PACKET_FORMAT,
    SpaceMouseTeleopModule,
    _test_bit,
    parse_spnav_button_packet,
    parse_spnav_packet,
)


class _DummyOut:
    def __init__(self) -> None:
        self.published: list[TwistStamped] = []
        self.transport: Any = object()

    def publish(self, msg: TwistStamped) -> None:
        self.published.append(msg)


def test_parse_spnav_motion_packet() -> None:
    pkt = struct.pack(SPNAV_PACKET_FORMAT, SPNAV_EVENT_MOTION, 100, -50, -200, 30, -40, 60, 16)
    assert parse_spnav_packet(pkt) == (100, -50, -200, 30, -40, 60)


def test_parse_spnav_non_motion_packet_returns_none() -> None:
    pkt = struct.pack(SPNAV_PACKET_FORMAT, 1, 0, 1, 0, 0, 0, 0, 0)
    assert parse_spnav_packet(pkt) is None


def test_spacemouse_moves_only_while_x_deadman_pressed_and_locks_on_release() -> None:
    x_pressed = False
    module = SpaceMouseTeleopModule(deadman_fn=lambda: x_pressed)
    try:
        dummy_out = _DummyOut()
        module.ee_twist_command = dummy_out  # type: ignore[assignment]

        # 1. Deflect SpaceMouse without pressing X: arm must NOT receive any twist.
        module.ingest_motion((175, 0, -350, 0, 0, 175), now=10.0)
        assert module.step_once(now=10.02) is None
        assert len(dummy_out.published) == 0

        # 2. Hold X deadman key: 6-DoF twist is published.
        x_pressed = True
        twist = module.step_once(now=10.04)
        assert twist is not None
        assert len(dummy_out.published) == 1
        assert twist.linear.x == pytest.approx(0.15)
        assert twist.linear.y == pytest.approx(-0.075)
        assert twist.angular.x == pytest.approx(-0.30)

        # 3. Release (depress) X deadman key while SpaceMouse is still deflected:
        # must immediately emit a zero twist so the arm locks in place in the air.
        x_pressed = False
        lock_twist = module.step_once(now=10.06)
        assert lock_twist is not None
        assert len(dummy_out.published) == 2
        assert (lock_twist.linear.x, lock_twist.linear.y, lock_twist.linear.z) == (0.0, 0.0, 0.0)
        assert (lock_twist.angular.x, lock_twist.angular.y, lock_twist.angular.z) == (
            0.0,
            0.0,
            0.0,
        )

        # 4. Subsequent ticks with X still unpressed emit nothing further.
        assert module.step_once(now=10.08) is None
        assert len(dummy_out.published) == 2
    finally:
        module._close_module()


def test_check_usb_hardware_prints_both_devices_and_exits_when_missing(
    capsys: pytest.CaptureFixture[str],
) -> None:
    valid_lsusb = (
        "Bus 007 Device 006: ID a8fa:8598 Com Equipment CANFD Analyser\n"
        "Bus 004 Device 011: ID 8086:0b5b Intel Corp. Intel(R) RealSense(TM) Depth Camera 405\n"
    )
    a1z_line, d405_line = check_usb_hardware(lsusb_output=valid_lsusb)
    out = capsys.readouterr().out
    assert "a8fa:8598" in a1z_line
    assert "8086:0b5b" in d405_line
    assert "• Galaxea A1Z USB-CANFD Adapter:" in out
    assert "• Intel RealSense D405 Depth Camera:" in out

    missing_lsusb = "Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub\n"
    with pytest.raises(SystemExit) as exc_info:
        check_usb_hardware(lsusb_output=missing_lsusb)
    assert exc_info.value.code == 1


def test_galaxea_a1z_localize_blueprint_wires_cleanly() -> None:
    verify_no_name_conflicts(galaxea_a1z_localize)
    refs = resolve_module_refs(galaxea_a1z_localize)
    assert refs["pickandplacemodule", "_scene"] == "manipulationlocalize"
    assert refs["pickandplacemodule", "_grasp_generator"] == "graspgenxmodule"
    assert refs["pickandplacemodule", "_manipulation"] == "manipulationmodule"
    assert refs["manipulationskills", "manipulation"] == "manipulationmodule"
    assert refs["manipulationmodule", "_control_coordinator"] == "ControlCoordinator"


def test_evdev_key_bitmask() -> None:
    key_x = LINUX_KEYCODES["x"]
    buf = bytearray(96)
    assert not _test_bit(buf, key_x)
    buf[key_x // 8] |= 1 << (key_x % 8)
    assert _test_bit(buf, key_x)


def test_spacemouse_buttons_toggle_translation_only_and_gripper() -> None:
    # spacenavd packet layout: (ev_type, bnum, press, ...)
    pkt_left_press = struct.pack(SPNAV_PACKET_FORMAT, SPNAV_EVENT_BUTTON, 0, 1, 0, 0, 0, 0, 0)
    pkt_right_press = struct.pack(SPNAV_PACKET_FORMAT, SPNAV_EVENT_BUTTON, 1, 1, 0, 0, 0, 0, 0)
    pkt_left_release = struct.pack(
        SPNAV_PACKET_FORMAT, SPNAV_EVENT_BUTTON_RELEASE, 0, 0, 0, 0, 0, 0, 0
    )
    assert parse_spnav_button_packet(pkt_left_press) == (0, True)
    assert parse_spnav_button_packet(pkt_right_press) == (1, True)
    assert parse_spnav_button_packet(pkt_left_release) == (0, False)

    module = SpaceMouseTeleopModule(
        deadman_fn=lambda: True,
        translation_only=True,
        decouple_arm_and_gripper=True,
    )
    try:
        dummy_out = _DummyOut()
        dummy_grip = _DummyOut()
        module.ee_twist_command = dummy_out  # type: ignore[assignment]
        module.gripper_command = dummy_grip  # type: ignore[assignment]

        # 1. Default ARM Mode: moves only arm X/Y/Z, gripper orientation locked (angular == 0)
        module.ingest_motion((175, 0, -350, 175, 175, 175), now=20.0)
        twist_arm = module.step_once(now=20.02)
        assert twist_arm is not None
        assert twist_arm.linear.x == pytest.approx(0.15)
        assert (twist_arm.angular.x, twist_arm.angular.y, twist_arm.angular.z) == (0.0, 0.0, 0.0)

        # 2. Left button (0) switches to GRIPPER Mode: arm stays still (linear == 0),
        # only gripper tilts/rotates (angular != 0, with +angular.y tilting down).
        module.on_button_event(0, True)
        twist_grip = module.step_once(now=20.04)
        assert twist_grip is not None
        assert (twist_grip.linear.x, twist_grip.linear.y, twist_grip.linear.z) == (0.0, 0.0, 0.0)
        assert twist_grip.angular.x == pytest.approx(-0.30)
        assert twist_grip.angular.y == pytest.approx(0.60)

        # 3. Left button (0) again switches back to ARM Mode (linear != 0, angular == 0)
        module.on_button_event(0, True)
        twist_arm_again = module.step_once(now=20.06)
        assert twist_arm_again is not None
        assert twist_arm_again.linear.x == pytest.approx(0.15)
        assert (
            twist_arm_again.angular.x,
            twist_arm_again.angular.y,
            twist_arm_again.angular.z,
        ) == (0.0, 0.0, 0.0)

        # 4. Right button (1) toggles gripper close (0.0) then open (1.0)
        module.on_button_event(1, True)
        module.on_button_event(1, True)
        assert len(dummy_grip.published) == 2
        assert dummy_grip.published[0].data == pytest.approx(0.0)  # type: ignore[attr-defined]
        assert dummy_grip.published[1].data == pytest.approx(1.0)  # type: ignore[attr-defined]
    finally:
        module._close_module()
