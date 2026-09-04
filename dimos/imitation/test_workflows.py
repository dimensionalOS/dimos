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

from dimos.imitation.workflows import WORKFLOWS, get_workflow
from dimos.robot.manipulators.openyam.learning import (
    OPENYAM_LEARNING_PROFILE,
    OPENYAM_TEACH_LEARNING_PROFILE,
)


def test_registry_has_two_explicit_openyam_workflows() -> None:
    assert list(WORKFLOWS) == ["openyam-teach", "openyam-quest"]
    assert WORKFLOWS["openyam-teach"].collection_method == ("gravity-compensated hand guidance")
    assert "Quest headset" not in WORKFLOWS["openyam-teach"].required_hardware
    assert "Quest headset" in WORKFLOWS["openyam-quest"].required_hardware


def test_workflows_select_the_correct_action_contract() -> None:
    teach = get_workflow("openyam-teach").load_dataprep_profile()
    quest = get_workflow("openyam-quest").load_dataprep_profile()

    assert teach is OPENYAM_TEACH_LEARNING_PROFILE
    assert quest is OPENYAM_LEARNING_PROFILE
    assert teach.dataprep_config().action["action"].stream == "coordinator_joint_state"
    assert quest.dataprep_config().action["action"].stream == ("applied_joint_position_command")
    assert teach.dataprep_config().quality.mode == "strict"


def test_unknown_workflow_lists_valid_choices() -> None:
    with pytest.raises(ValueError, match="openyam-quest, openyam-teach"):
        get_workflow("missing")
