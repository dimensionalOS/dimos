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
    learning_collect_quest_piper,
    learning_collect_quest_xarm7,
)
from dimos.msgs.sensor_msgs.JointState import JointState

AGGREGATE = "coordinator_joint_state"


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
    assert not [port for _instance, port in streams if port.endswith("_joints")]
