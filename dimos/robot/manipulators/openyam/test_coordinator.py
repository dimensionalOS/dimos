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

"""Which connection each OpenYAM collection and rollout stack deploys."""

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.manipulators.openyam.blueprints.learning_collection import (
    OPENYAM_TEACH_JOINTS,
    openyam_teach_collection,
)
from dimos.robot.manipulators.openyam.blueprints.learning_quest_collection import (
    openyam_quest_collection,
)
from dimos.robot.manipulators.openyam.blueprints.learning_rollout import build_openyam_rollout
from dimos.robot.manipulators.openyam.connection import (
    OpenYamConnection,
    OpenYamTeachConnection,
)

ROLLOUT = build_openyam_rollout()
QUEST_ROLLOUT = build_openyam_rollout(quest_control=True)


@pytest.mark.parametrize(
    ("blueprint", "module_name", "connection"),
    [
        (openyam_teach_collection, "openyamteachconnection", OpenYamTeachConnection),
        (openyam_quest_collection, "openyamconnection", OpenYamConnection),
        (ROLLOUT, "openyamconnection", OpenYamConnection),
        (QUEST_ROLLOUT, "openyamconnection", OpenYamConnection),
    ],
    ids=["teach", "quest", "rollout", "quest-rollout"],
)
def test_can_selector_belongs_to_the_connection(blueprint, module_name, connection, tmp_path):
    flags = [f"--{module_name}.can-port", "/dev/ttyACM0"]
    if blueprint in (openyam_teach_collection, openyam_quest_collection):
        flags.extend(["--recorder.recording", str(tmp_path / "session"), "--episodes.task", "pick"])
    else:
        flags.extend(["--policy.policy-path", "checkpoint", "--policy.task", "pick"])
    parsed = BlueprintConfigParser(blueprint).parse(flags, environ={})

    assert parsed.module_kwargs(module_name)["can_port"] == "/dev/ttyACM0"
    atom = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, OpenYamConnection)
    )
    assert atom.module is connection
    assert atom.kwargs["source"] == "arm"
    coordinator = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, ControlCoordinator)
    )
    assert not coordinator.kwargs.get("hardware")
    if connection is OpenYamTeachConnection:
        assert coordinator.kwargs["zero_stiffness_joints"] == OPENYAM_TEACH_JOINTS
    else:
        assert coordinator.kwargs["zero_stiffness_joints"] == []
