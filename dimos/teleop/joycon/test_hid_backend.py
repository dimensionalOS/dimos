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

import pytest

from dimos.teleop.joycon import hid_backend
from dimos.teleop.joycon.hid_backend import (
    HidJoyConBackend,
    StickCalibration,
    _Device,
    decode_report,
)


@pytest.fixture
def calibration():
    return StickCalibration(center=(2000, 2100), below=(1000, 1000), above=(1000, 1000))


def test_factory_calibration_and_normalization_use_upright_axes():
    # Factory bytes read from the attached original left Joy-Con.
    cal = StickCalibration.decode(bytes.fromhex("e78443058889c57449"), "left")
    assert cal.center == (2053, 2200)
    assert cal.normalize(cal.center) == (0, 0)
    assert cal.normalize((3308, 1025)) == (1, 1)
    assert cal.normalize((832, 3280)) == (-1, -1)


def test_right_calibration_has_a_different_field_order():
    cal = StickCalibration.decode(bytes.fromhex("058889c57449e78443"), "right")
    assert cal.center == (2053, 2200)
    assert cal.normalize((3308, 1025)) == (1, 1)
    with pytest.raises(ValueError, match="invalid"):
        StickCalibration.decode(b"\xff" * 9, "left")


def test_full_report_preserves_left_labels_and_does_not_rotate_the_stick(calibration):
    packet = bytes([0x30, 1, 0x40, 0, 0x29, 0xE2, 0xD0, 0x47, 0x83, 0, 0, 0, 0])
    reading = decode_report(packet, "left", calibration)
    assert reading.pressed == frozenset({"minus", "capture", "stick", "SL", "L", "ZL", "up"})
    assert reading.stick == (0, 0)


def test_full_report_preserves_right_labels(calibration):
    packet = bytes([0x30, 1, 0x40, 0xDA, 0x16, 0, 0, 0, 0, 0xD0, 0x47, 0x83, 0])
    reading = decode_report(packet, "right", calibration)
    assert reading.pressed == frozenset({"X", "A", "R", "ZR", "SR", "plus", "stick", "home"})
    assert reading.stick == (0, 0)
    assert decode_report(packet[:8], "right", calibration) is None
    assert decode_report(b"\x21" + packet[1:], "right", calibration) is None


def test_disconnected_hid_device_is_absent_and_closed(mocker, calibration):
    clock = mocker.patch.object(hid_backend.time, "monotonic", return_value=0)
    handle = mocker.Mock()
    handle.read.return_value = []
    device = _Device(handle, "left")
    device.calibration = calibration
    backend = HidJoyConBackend()
    backend.devices["left"] = device
    mocker.patch.object(backend, "_scan")
    try:
        clock.return_value = 0.21
        assert backend.read() == {"left": None, "right": None}
        handle.close.assert_called_once_with()
        assert not backend.devices
    finally:
        backend.close()


def test_queued_reports_are_drained_and_old_input_is_not_refreshed(mocker, calibration):
    mocker.patch.object(hid_backend.time, "monotonic", return_value=0)
    handle = mocker.Mock()
    pressed = bytes([0x30, 1, 0x40, 0, 1, 0, 0xD0, 0x47, 0x83, 0, 0, 0, 0])
    released = bytes([0x30, 2, 0x40, 0, 0, 0, 0xD0, 0x47, 0x83, 0, 0, 0, 0])
    handle.read.side_effect = [pressed, released, [], []]
    device = _Device(handle, "left")
    device.calibration = calibration
    assert device.read(0, 0.2).pressed == frozenset()
    with pytest.raises(OSError, match="stopped sending"):
        device.read(0.21, 0.2)


def test_excessive_queue_cannot_grant_fresh_authority(mocker, calibration):
    handle = mocker.Mock()
    handle.read.return_value = [0x30] + [0] * 48
    device = _Device(handle, "left")
    device.calibration = calibration
    with pytest.raises(OSError, match="freshness budget"):
        device.read(1, 0.2)
    assert device.reading is None
