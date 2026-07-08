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

import pytest

import dimos.teleop.openarm_mini.tools.setup_motor_id as setup_motor_id_module
from dimos.teleop.openarm_mini.tools.setup_motor_id import (
    FEETECH_ID_ADDRESS,
    FEETECH_TORQUE_ENABLE,
    FEETECH_TORQUE_ENABLE_ADDRESS,
    find_single_motor_id,
    setup_motor_id,
    write_motor_id,
)


class _FakePacketHandler:
    def __init__(self, responding_ids: set[int]) -> None:
        self.responding_ids = responding_ids
        self.calls: list[tuple[str, int, int | None, int | None]] = []
        self.fail_id_write = False

    def ping(self, scs_id: int) -> tuple[int, int, int]:
        self.calls.append(("ping", scs_id, None, None))
        return (1234, 0, 0) if scs_id in self.responding_ids else (0, -1, 0)

    def write1ByteTxRx(self, scs_id: int, address: int, value: int) -> tuple[int, int]:
        self.calls.append(("write1", scs_id, address, value))
        if scs_id not in self.responding_ids or (
            address == FEETECH_ID_ADDRESS and self.fail_id_write
        ):
            return (-1, 0)
        if address == FEETECH_ID_ADDRESS:
            self.responding_ids = (self.responding_ids - {scs_id}) | {value}
        return (0, 0)

    def unLockEprom(self, scs_id: int) -> tuple[int, int]:
        self.calls.append(("unlock", scs_id, None, None))
        return (0, 0)

    def LockEprom(self, scs_id: int) -> tuple[int, int]:
        self.calls.append(("lock", scs_id, None, None))
        return (0, 0) if scs_id in self.responding_ids else (-1, 0)


class _FakePortHandler:
    def __init__(self) -> None:
        self.opened = False
        self.closed = False
        self.baudrate: int | None = None

    def openPort(self) -> bool:
        self.opened = True
        return True

    def setBaudRate(self, baudrate: int) -> bool:
        self.baudrate = baudrate
        return True

    def closePort(self) -> None:
        self.closed = True


def test_write_motor_id_writes_sequence_and_verifies_new_id() -> None:
    packet_handler = _FakePacketHandler({3})

    write_motor_id(packet_handler, old_id=3, new_id=7)

    assert packet_handler.calls == [
        ("ping", 3, None, None),
        ("write1", 3, FEETECH_TORQUE_ENABLE_ADDRESS, 0),
        ("unlock", 3, None, None),
        ("write1", 3, FEETECH_ID_ADDRESS, 7),
        ("lock", 7, None, None),
        ("ping", 7, None, None),
    ]
    assert packet_handler.responding_ids == {7}


def test_write_motor_id_locks_and_restores_torque_after_failure() -> None:
    packet_handler = _FakePacketHandler({3})
    packet_handler.fail_id_write = True

    with pytest.raises(RuntimeError, match="write motor ID"):
        write_motor_id(packet_handler, old_id=3, new_id=7)

    assert ("lock", 3, None, None) in packet_handler.calls
    assert (
        "write1",
        3,
        FEETECH_TORQUE_ENABLE_ADDRESS,
        FEETECH_TORQUE_ENABLE,
    ) in packet_handler.calls
    assert packet_handler.responding_ids == {3}


def test_find_single_motor_id_rejects_multiple_connected_motors() -> None:
    with pytest.raises(RuntimeError, match="multiple Feetech motors"):
        find_single_motor_id(_FakePacketHandler({2, 4}))


def test_setup_motor_id_scans_and_closes_port(monkeypatch: pytest.MonkeyPatch) -> None:
    packet_handler = _FakePacketHandler({5})
    port_handler = _FakePortHandler()

    def create_handlers(_port: str) -> tuple[_FakePortHandler, _FakePacketHandler]:
        return port_handler, packet_handler

    monkeypatch.setattr(setup_motor_id_module, "_create_sdk_handlers", create_handlers)

    previous_id = setup_motor_id("/dev/test-feetech", baudrate=123456, new_id=9)

    assert previous_id == 5
    assert port_handler.opened and port_handler.closed
    assert port_handler.baudrate == 123456
    assert packet_handler.responding_ids == {9}


def test_setup_motor_id_rejects_invalid_ids() -> None:
    with pytest.raises(ValueError, match="new-id"):
        setup_motor_id("/dev/test-feetech", baudrate=1_000_000, new_id=254, old_id=1)
