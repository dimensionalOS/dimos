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

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.transport import pSHMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.imitation.collection.blueprint import (
    learning_collect_quest_openyam,
    learning_collect_quest_piper,
    learning_collect_quest_xarm7,
)
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS
from dimos.teleop.quest.quest_extensions import ArmTeleopModule

AGGREGATE = "coordinator_joint_state"


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_quest_xarm7, learning_collect_quest_piper],
)
def test_collection_streams_are_poseless(blueprint: Blueprint) -> None:
    recorder = next(atom for atom in blueprint.blueprints if atom.module is CollectionRecorder)

    assert recorder.kwargs["record_tf"] is False


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_quest_xarm7, learning_collect_quest_piper],
)
def test_collection_recorder_stops_after_producers(blueprint: Blueprint) -> None:
    assert blueprint.active_blueprints[0].module is CollectionRecorder


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_quest_xarm7, learning_collect_quest_piper],
)
def test_episode_monitor_stops_after_input_producers(blueprint: Blueprint) -> None:
    assert blueprint.active_blueprints[1].module is EpisodeMonitorModule


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_quest_xarm7, learning_collect_quest_piper],
)
def test_collection_status_is_wired_to_quest_hud(blueprint: Blueprint) -> None:
    hud = next(atom for atom in blueprint.blueprints if atom.module is ArmTeleopModule)
    status = next(stream for stream in hud.streams if stream.name == "status")

    assert status.direction == "in"
    assert status.type.__name__ == "EpisodeStatus"


def _joint_streams(blueprint: Blueprint) -> dict[tuple[str, str], str]:
    """(instance, port) -> effective stream name, as ModuleCoordinator pairs streams."""
    return {
        (atom.name, stream.name): blueprint.remapping_map.get((atom.name, stream.name), stream.name)
        for atom in blueprint.active_blueprints
        for stream in atom.streams
        if stream.type is JointState
    }


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_quest_xarm7, learning_collect_quest_piper, learning_collect_quest_openyam],
)
def test_recorder_reads_aggregate_joint_state(blueprint: Blueprint) -> None:
    streams = _joint_streams(blueprint)

    # Plain name pairing on both ends, no remap in between. The coordinator
    # atom carries its explicit instance_name (the RPC lookup contract).
    assert streams[("collectionrecorder", AGGREGATE)] == AGGREGATE
    assert streams[("ControlCoordinator", AGGREGATE)] == AGGREGATE


def test_openyam_collection_has_one_wrist_webcam_and_all_joints() -> None:
    camera_atoms = [
        atom
        for atom in learning_collect_quest_openyam.active_blueprints
        if atom.module is CameraModule
    ]
    coordinator = next(
        atom
        for atom in learning_collect_quest_openyam.active_blueprints
        if atom.instance_name == "ControlCoordinator"
    )

    assert len(camera_atoms) == 1
    assert camera_atoms[0].instance_name == "WristCamera"
    webcam = camera_atoms[0].kwargs["webcam"]
    assert webcam.width == 640
    assert webcam.height == 480
    assert webcam.fps == 30.0
    hardware = coordinator.kwargs["hardware"]
    assert len(hardware) == 1
    assert hardware[0].joints == OPENYAM_JOINTS


def test_openyam_collection_records_wrist_camera_over_shared_memory() -> None:
    transport = learning_collect_quest_openyam.transport_map[("color_image", Image)]

    assert isinstance(transport, pSHMTransport)
    assert transport.shm.config.default_capacity == DEFAULT_CAPACITY_COLOR_IMAGE


@pytest.mark.parametrize(
    ("argument", "expected"),
    [("2", 2), ("/dev/v4l/by-id/usb-wrist-camera", "/dev/v4l/by-id/usb-wrist-camera")],
)
def test_openyam_wrist_camera_device_is_configurable_from_cli(
    argument: str, expected: int | str
) -> None:
    parsed = BlueprintConfigParser(learning_collect_quest_openyam).parse(
        ["--WristCamera.webcam.camera-index", argument, "--task", "pick up the block"],
        environ={},
    )

    assert parsed.module_kwargs("WristCamera")["webcam"]["camera_index"] == expected
