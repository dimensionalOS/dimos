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

"""Picking a camera out of the USB bus, on either kind of bus listing."""

from pathlib import Path

import pytest

from dimos.hardware.sensors.camera.realsense import discovery


def _usb_device(root: Path, name: str, vendor: str, product: str, serial: str | None) -> None:
    device = root / name
    device.mkdir()
    (device / "idVendor").write_text(f"{vendor}\n")
    (device / "idProduct").write_text(f"{product}\n")
    if serial is not None:
        (device / "serial").write_text(f"{serial}\n")


def test_sysfs_picks_the_d455_out_of_a_mixed_bus(tmp_path: Path) -> None:
    _usb_device(tmp_path, "2-1", "8086", "0b3a", "142322070123")  # rear D435i
    _usb_device(tmp_path, "2-2", "8086", "0b5c", "260922302422")  # mast D455
    _usb_device(tmp_path, "2-3", "046d", "0b5c", "keyboard")  # same product id, not Intel
    _usb_device(tmp_path, "usb2", "1d6b", "0003", None)  # root hub, no serial

    found = discovery._scan_sysfs(tmp_path)
    assert found[discovery.PRODUCT_IDS["D455"]] == ["260922302422"]
    assert found[discovery.PRODUCT_IDS["D435i"]] == ["142322070123"]


def test_ioreg_walks_the_hub_tree() -> None:
    root = {
        "IORegistryEntryChildren": [
            {
                "IORegistryEntryName": "hub",
                "IORegistryEntryChildren": [
                    {"idVendor": 0x8086, "idProduct": 0x0B5C, "USB Serial Number": "260922302422"},
                    {"idVendor": 0x8086, "idProduct": 0x0B3A, "USB Serial Number": "142322070123"},
                    {"idVendor": 0x05AC, "idProduct": 0x0B5C, "USB Serial Number": "not-a-camera"},
                ],
            }
        ]
    }
    found = discovery._parse_ioreg(root)
    assert found == {0x0B5C: ["260922302422"], 0x0B3A: ["142322070123"]}


def test_two_of_a_model_refuse_to_pick(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        discovery, "_serials_by_product_id", lambda: {0x0B5C: ["one", "two"], 0x0B3A: ["only"]}
    )
    assert discovery.find_serials("D455") == ["one", "two"]
    assert discovery.find_serial("D455") is None
    assert discovery.find_serial("D435i") == "only"
    assert discovery.find_serial("D435") is None


def test_unknown_model_is_a_typo_not_an_absent_camera() -> None:
    with pytest.raises(ValueError, match="D999"):
        discovery.find_serials("D999")
