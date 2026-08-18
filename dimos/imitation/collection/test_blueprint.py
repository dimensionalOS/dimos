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

"""Tests for the rig camera USB-port resolver used by the multicam blueprint."""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.imitation.collection.blueprint import (
    _MULTICAM_USB_PORTS,
    _video_index_for_usb_port,
)


def _add_camera(sysfs_root: Path, video_index: int, port: str) -> None:
    """Create a fake video4linux node whose device symlink resolves to `port`."""
    device_dir = sysfs_root / "devices" / "usb3" / port.split(".")[0] / f"{port}:1.0"
    device_dir.mkdir(parents=True, exist_ok=True)
    node = sysfs_root / "class" / "video4linux" / f"video{video_index}"
    node.mkdir(parents=True, exist_ok=True)
    (node / "device").symlink_to(device_dir)


def test_resolves_the_lower_numbered_node_of_a_pair(tmp_path: Path) -> None:
    root = tmp_path / "class" / "video4linux"
    _add_camera(tmp_path, 4, "3-5.1")
    _add_camera(tmp_path, 5, "3-5.1")

    assert _video_index_for_usb_port("3-5.1", root) == 4


def test_distinguishes_two_cameras_on_different_ports(tmp_path: Path) -> None:
    root = tmp_path / "class" / "video4linux"
    _add_camera(tmp_path, 4, "3-5.1")
    _add_camera(tmp_path, 6, "3-5.2")

    assert _video_index_for_usb_port("3-5.1", root) == 4
    assert _video_index_for_usb_port("3-5.2", root) == 6


def test_missing_port_raises_instead_of_silently_picking_another_camera(
    tmp_path: Path,
) -> None:
    root = tmp_path / "class" / "video4linux"
    _add_camera(tmp_path, 4, "3-5.1")

    with pytest.raises(RuntimeError, match="No camera found"):
        _video_index_for_usb_port("3-5.2", root)


def test_rig_ports_are_four_distinct_labeled_ports() -> None:
    assert set(_MULTICAM_USB_PORTS) == {"chest", "left_hand", "right_hand", "waist"}
    assert len(set(_MULTICAM_USB_PORTS.values())) == 4
