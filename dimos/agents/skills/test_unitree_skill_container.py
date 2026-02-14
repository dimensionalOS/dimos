# Copyright 2025-2026 Dimensional Inc.
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

import difflib

from langchain_core.messages import HumanMessage
import pytest

from dimos.robot.unitree.unitree_skill_container import _UNITREE_COMMANDS, UnitreeSkillContainer


class MockedUnitreeSkill(UnitreeSkillContainer):
    rpc_calls: list[str] = []

    def __init__(self):
        super().__init__()
        # Provide a fake RPC so the real execute_sport_command runs end-to-end.
        self._bound_rpc_calls["GO2Connection.publish_request"] = lambda *args, **kwargs: None


@pytest.mark.integration
def test_pounce(agent_setup) -> None:
    history = agent_setup(
        blueprints=[MockedUnitreeSkill.blueprint()],
        messages=[HumanMessage("Pounce! Use the execute_sport_command tool.")],
    )

    response = history[-1].content.lower()
    assert "pounce" in response


def test_did_you_mean() -> None:
    suggestions = difflib.get_close_matches("Pounce", _UNITREE_COMMANDS.keys(), n=3, cutoff=0.6)
    assert "FrontPounce" in suggestions
    assert "Pose" in suggestions
