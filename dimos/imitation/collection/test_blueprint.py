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

"""Tests for the collection blueprints and the rig camera USB-port resolver."""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.imitation.collection.blueprint import (
    _MULTICAM_USB_PORTS,
    _video_index_for_usb_port,
    learning_collect_quest_openarm,
    learning_collect_quest_openarm_multicam,
    learning_collect_quest_openarm_webcam,
    learning_collect_quest_piper,
    learning_collect_quest_xarm7,
)
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.msgs.sensor_msgs.JointState import JointState

AGGREGATE = "coordinator_joint_state"

ALL_COLLECTION_BLUEPRINTS = [
    learning_collect_quest_xarm7,
    learning_collect_quest_piper,
    learning_collect_quest_openarm,
    learning_collect_quest_openarm_webcam,
    learning_collect_quest_openarm_multicam,
]


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_quest_xarm7, learning_collect_quest_piper],
)
def test_collection_streams_are_poseless(blueprint: Blueprint) -> None:
    recorder = next(atom for atom in blueprint.blueprints if atom.module is CollectionRecorder)

    assert recorder.kwargs["poseless_streams"] == [
        "color_image",
        "coordinator_joint_state",
        "status",
    ]
    assert recorder.kwargs["record_tf"] is False


@pytest.mark.parametrize("blueprint", ALL_COLLECTION_BLUEPRINTS)
def test_collection_recorder_stops_after_producers(blueprint: Blueprint) -> None:
    assert blueprint.active_blueprints[0].module is CollectionRecorder


@pytest.mark.parametrize("blueprint", ALL_COLLECTION_BLUEPRINTS)
def test_episode_monitor_stops_after_input_producers(blueprint: Blueprint) -> None:
    assert blueprint.active_blueprints[1].module is EpisodeMonitorModule


def _joint_streams(blueprint: Blueprint) -> dict[tuple[str, str], str]:
    """(instance, port) -> effective stream name, as ModuleCoordinator pairs streams."""
    return {
        (atom.name, stream.name): blueprint.remapping_map.get((atom.name, stream.name), stream.name)
        for atom in blueprint.active_blueprints
        for stream in atom.streams
        if stream.type is JointState
    }


@pytest.mark.parametrize("blueprint", [learning_collect_quest_xarm7, learning_collect_quest_piper])
def test_recorder_reads_aggregate_joint_state(blueprint: Blueprint) -> None:
    streams = _joint_streams(blueprint)

    # Plain name pairing on both ends, no remap in between. The coordinator
    # atom carries its explicit instance_name (the RPC lookup contract).
    assert streams[("collectionrecorder", AGGREGATE)] == AGGREGATE
    assert streams[("ControlCoordinator", AGGREGATE)] == AGGREGATE
    # This base still exposes per-robot *_joints coordinator ports; the
    # recorder must not read them.
    assert not [
        port
        for instance, port in streams
        if instance == "collectionrecorder" and port.endswith("_joints")
    ]


# ── rig camera USB-port resolver ─────────────────────────────────────────────


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
