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

from dimos.core.coordination.blueprints import Blueprint
from dimos.imitation.collection.blueprint import (
    learning_collect_webxr_piper,
    learning_collect_webxr_xarm7,
)
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.webxr.extensions import ArmTeleopModule

AGGREGATE = "coordinator_joint_state"


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7, learning_collect_webxr_piper],
)
def test_collection_streams_are_poseless(blueprint: Blueprint) -> None:
    recorder = next(atom for atom in blueprint.blueprints if atom.module is CollectionRecorder)

    assert recorder.kwargs["poseless_streams"] == [
        "color_image",
        "coordinator_joint_state",
        "status",
    ]
    assert recorder.kwargs["record_tf"] is False


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7, learning_collect_webxr_piper],
)
def test_collection_recorder_stops_after_producers(blueprint: Blueprint) -> None:
    assert blueprint.active_blueprints[0].module is CollectionRecorder


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7, learning_collect_webxr_piper],
)
def test_episode_monitor_stops_after_input_producers(blueprint: Blueprint) -> None:
    assert blueprint.active_blueprints[1].module is EpisodeMonitorModule


@pytest.mark.parametrize(
    "blueprint",
    [learning_collect_webxr_xarm7, learning_collect_webxr_piper],
)
def test_collection_status_is_wired_to_webxr_hud(blueprint: Blueprint) -> None:
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


@pytest.mark.parametrize("blueprint", [learning_collect_webxr_xarm7, learning_collect_webxr_piper])
def test_recorder_reads_aggregate_joint_state(blueprint: Blueprint) -> None:
    streams = _joint_streams(blueprint)

    # Plain name pairing on both ends, no remap in between. The coordinator
    # atom carries its explicit instance_name (the RPC lookup contract).
    assert streams[("collectionrecorder", AGGREGATE)] == AGGREGATE
    assert streams[("ControlCoordinator", AGGREGATE)] == AGGREGATE
    assert not [port for _instance, port in streams if port.endswith("_joints")]
