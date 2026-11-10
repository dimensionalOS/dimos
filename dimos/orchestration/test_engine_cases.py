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

from pathlib import Path
import sys

import pytest

pytest.importorskip("cel_expr_python")

from pydantic import BaseModel, ConfigDict, Field, JsonValue

from dimos.orchestration.definition import load_definition
from dimos.orchestration.engine import (
    Effect,
    Engine,
    Event,
    ExternalEvent,
    InputMessage,
    Publish,
    StateMachineError,
)

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

CASES = Path(__file__).parent / "cases"


class CaseStep(BaseModel):
    model_config = ConfigDict(extra="forbid")

    now: float = 0.0
    start: bool = False
    input: str | None = None
    msg: dict[str, JsonValue] = Field(default_factory=dict)
    external: str | None = None
    data: dict[str, JsonValue] = Field(default_factory=dict)
    effects: list[dict[str, JsonValue]] = Field(default_factory=list)
    active: list[str] = Field(default_factory=list)
    vars: dict[str, JsonValue] = Field(default_factory=dict)
    error: str | None = None


class Case(BaseModel):
    model_config = ConfigDict(extra="forbid")

    state_machine: str
    step: list[CaseStep]


def _event(step: CaseStep) -> Event:
    if step.input is not None:
        return InputMessage(step.input, step.msg)
    assert step.external is not None, "a step needs start, input or external"
    return ExternalEvent(step.external, step.data)


def _as_table(effect: Effect) -> dict[str, JsonValue]:
    if isinstance(effect, Publish):
        return {"publish": effect.output, "value": effect.value}
    return {
        "call": effect.target,
        "goal_id": effect.goal_id,
        "args": effect.args,
        "nowait": effect.nowait,
    }


@pytest.mark.parametrize("path", sorted(CASES.glob("*.toml")), ids=lambda path: path.stem)
def test_case(path: Path) -> None:
    case = Case.model_validate(tomllib.loads(path.read_text()))
    engine = Engine(load_definition(CASES / "state_machines" / case.state_machine), id_prefix="g")
    for index, step in enumerate(case.step):
        where = f"step {index}"
        if step.error is not None:
            with pytest.raises(StateMachineError, match=step.error):
                engine.handle(_event(step), step.now)
            continue
        effects = engine.start(step.now) if step.start else engine.handle(_event(step), step.now)
        assert [_as_table(effect) for effect in effects] == step.effects, where
        assert list(engine.active) == step.active, where
        assert step.vars.items() <= engine.vars.items(), where
