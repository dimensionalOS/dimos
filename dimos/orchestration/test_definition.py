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

from pydantic import BaseModel, JsonValue, ValidationError
import pytest

from dimos.orchestration.definition import Node, OrchestratorDefinition, Transition

REJECTED: list[tuple[type[BaseModel], dict[str, JsonValue], str]] = [
    (Transition, {"goto": "A", "trigger": "estop", "after": 5}, "at most one of `trigger`"),
    (Node, {"name": "A", "children": ["B"], "parallel": ["C"]}, "at most one of `children`"),
]


def test_patrol_definition(patrol: OrchestratorDefinition) -> None:
    nodes = {node.name: node for node in patrol.nodes}
    assert nodes["Navigating"].transitions[0].mode == "graceful"
    assert nodes["NextWaypoint"].transitions[1].set == {"patrol.index": "0"}
    from_file = Transition.model_validate({"goto": "A", "if": "x < 1", "after": 60})
    assert from_file == Transition(goto="A", if_="x < 1", after="60")


@pytest.mark.parametrize(("struct", "fields", "problem"), REJECTED)
def test_field_rules(struct: type[BaseModel], fields: dict[str, JsonValue], problem: str) -> None:
    with pytest.raises(ValidationError, match=problem):
        struct.model_validate(fields)
