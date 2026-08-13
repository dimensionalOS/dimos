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

"""OpenYAM adapter tests — no hardware, ``can.Bus(interface="virtual")``."""

from __future__ import annotations

import time

import pytest

can = pytest.importorskip("can")

from dimos.hardware.manipulators.openarm.driver import float_to_uint
from dimos.hardware.manipulators.openyam.adapter import OpenYamAdapter
from dimos.hardware.manipulators.openyam.driver import (
    DEFAULT_GRIPPER_SEND_ID,
    MotorType,
    make_yam_motors,
    resolve_transport,
)
from dimos.hardware.manipulators.spec import ControlMode

# ------------------------------------------------------------ resolve_transport


def test_resolve_transport_socketcan_on_linux(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("dimos.hardware.manipulators.openyam.driver.sys.platform", "linux")
    interface, channel, kwargs = resolve_transport("can0")
    assert (interface, channel, kwargs) == ("socketcan", "can0", {})


def test_resolve_transport_can_name_falls_back_to_gs_usb_on_macos(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("dimos.hardware.manipulators.openyam.driver.sys.platform", "darwin")
    interface, _, kwargs = resolve_transport("can0")
    assert interface == "gs_usb"
    assert kwargs == {"index": 0, "bitrate": 1_000_000}


def test_resolve_transport_gs_usb_index() -> None:
    interface, _, kwargs = resolve_transport("gs_usb:1", bitrate=500_000)
    assert interface == "gs_usb"
    assert kwargs == {"index": 1, "bitrate": 500_000}


def test_resolve_transport_serial_is_slcan() -> None:
    interface, channel, kwargs = resolve_transport("/dev/tty.usbmodem1101")
    assert interface == "slcan"
    assert channel == "/dev/tty.usbmodem1101"
    assert kwargs == {"bitrate": 1_000_000}


def test_resolve_transport_explicit_interface_wins() -> None:
    interface, _, kwargs = resolve_transport("yam-test", interface="virtual")
    assert interface == "virtual"
    assert kwargs == {}


# ---------------------------------------------------------------- motor layout


def test_make_yam_motors_default_layout() -> None:
    motors = make_yam_motors(6)
    assert [m.send_id for m in motors] == [1, 2, 3, 4, 5, 6]
    assert [m.effective_recv_id for m in motors] == [0x11, 0x12, 0x13, 0x14, 0x15, 0x16]
    assert [m.motor_type for m in motors] == [MotorType.DM4340] * 3 + [MotorType.DM4310] * 3


def test_make_yam_motors_validates_lengths() -> None:
    with pytest.raises(ValueError, match="arm_motor_types"):
        make_yam_motors(6, arm_motor_types=[MotorType.DM4310])
    with pytest.raises(ValueError, match="motor_ids"):
        make_yam_motors(6, motor_ids=[1, 2, 3])


# -------------------------------------------------------------------- adapter


def test_adapter_rejects_wrong_dof() -> None:
    with pytest.raises(ValueError, match="6 DOF"):
        OpenYamAdapter(dof=7)


def _forge_state(sender: can.BusABC, motor_id: int, q: float, motor_type: MotorType) -> None:
    from dimos.hardware.manipulators.openarm.driver import _MOTOR_LIMITS

    p_max, v_max, t_max = _MOTOR_LIMITS[motor_type]
    q_u = float_to_uint(q, -p_max, p_max, 16)
    dq_u = float_to_uint(0.0, -v_max, v_max, 12)
    tau_u = float_to_uint(0.0, -t_max, t_max, 12)
    payload = bytes(
        [
            motor_id,
            (q_u >> 8) & 0xFF,
            q_u & 0xFF,
            (dq_u >> 4) & 0xFF,
            ((dq_u & 0xF) << 4) | ((tau_u >> 8) & 0xF),
            tau_u & 0xFF,
            30,
            28,
        ]
    )
    sender.send(can.Message(arbitration_id=motor_id | 0x10, data=payload, is_extended_id=False))


def _forge_all_states(sender: can.BusABC, q: float = 0.0) -> None:
    types = [MotorType.DM4340] * 3 + [MotorType.DM4310] * 3
    for motor_id, mt in zip(range(1, 7), types, strict=True):
        _forge_state(sender, motor_id, q, mt)
    _forge_state(sender, DEFAULT_GRIPPER_SEND_ID, q, MotorType.DM4310)


def _wait_for_positions(adapter: OpenYamAdapter, timeout: float = 0.5) -> list[float]:
    deadline = time.monotonic() + timeout
    last: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return adapter.read_joint_positions()
        except RuntimeError as e:
            last = e
            time.sleep(0.01)
    raise AssertionError(f"no joint state within {timeout}s: {last}")


@pytest.fixture
def virtual_pair():
    channel = "openyam-test"
    adapter = OpenYamAdapter(address=channel, interface="virtual", auto_set_mit_mode=False)
    peer = can.Bus(interface="virtual", channel=channel)
    assert adapter.connect()
    try:
        yield adapter, peer
    finally:
        adapter.disconnect()
        peer.shutdown()


def test_connect_read_write_roundtrip(virtual_pair) -> None:
    adapter, peer = virtual_pair
    assert adapter.is_connected()

    _forge_all_states(peer, q=0.25)
    positions = _wait_for_positions(adapter)
    assert all(abs(p - 0.25) < 0.01 for p in positions)

    # Enable then command; peer should observe one frame per joint.
    # Drain the enable frames (6 joints + gripper) first.
    assert adapter.write_enable(True)
    drained = 0
    deadline = time.monotonic() + 0.5
    while drained < 7 and time.monotonic() < deadline:
        if peer.recv(timeout=0.1) is not None:
            drained += 1
    assert drained == 7

    _forge_all_states(peer, q=0.25)
    assert adapter.write_joint_positions([0.1, 0.2, 0.3, -0.1, -0.2, -0.3])
    seen: set[int] = set()
    deadline = time.monotonic() + 0.5
    while len(seen) < 6 and time.monotonic() < deadline:
        msg = peer.recv(timeout=0.1)
        if msg is not None and 1 <= msg.arbitration_id <= 6:
            seen.add(int(msg.arbitration_id))
    assert seen == {1, 2, 3, 4, 5, 6}


def test_gripper_read_write(virtual_pair) -> None:
    adapter, peer = virtual_pair
    _forge_state(peer, DEFAULT_GRIPPER_SEND_ID, 0.8, MotorType.DM4310)
    deadline = time.monotonic() + 0.5
    pos = None
    while pos is None and time.monotonic() < deadline:
        pos = adapter.read_gripper_position()
        time.sleep(0.01)
    assert pos is not None
    assert abs(pos - 0.8) < 0.01

    assert adapter.write_enable(True)
    assert adapter.write_gripper_position(0.5)
    deadline = time.monotonic() + 0.5
    found = False
    while not found and time.monotonic() < deadline:
        msg = peer.recv(timeout=0.1)
        if msg is not None and msg.arbitration_id == DEFAULT_GRIPPER_SEND_ID and len(msg.data) == 8:
            found = True
    assert found


def test_stale_state_raises(virtual_pair) -> None:
    adapter, peer = virtual_pair
    _forge_all_states(peer)
    _wait_for_positions(adapter)
    time.sleep(0.15)  # > _STATE_MAX_AGE_S
    with pytest.raises(RuntimeError, match="stale"):
        adapter.read_joint_positions()


def test_write_requires_enable(virtual_pair) -> None:
    adapter, peer = virtual_pair
    _forge_all_states(peer)
    _wait_for_positions(adapter)
    assert not adapter.write_joint_positions([0.0] * 6)
    assert not adapter.write_gripper_position(0.0)


def test_control_modes() -> None:
    adapter = OpenYamAdapter(address="unused", interface="virtual")
    assert adapter.set_control_mode(ControlMode.VELOCITY)
    assert adapter.get_control_mode() == ControlMode.VELOCITY
    assert not adapter.set_control_mode(ControlMode.CARTESIAN)


def test_get_limits_shapes() -> None:
    adapter = OpenYamAdapter(address="unused", interface="virtual")
    limits = adapter.get_limits()
    assert len(limits.position_lower) == 6
    assert len(limits.position_upper) == 6
    assert len(limits.velocity_max) == 6
