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

"""A dimos module that runs a state machine file."""

from __future__ import annotations

from collections.abc import Iterable
import importlib
from pathlib import Path
import re

from dimos_lcm.actionlib_msgs import GoalStatus
from pydantic import Field, JsonValue, model_validator
from typing_extensions import Self

from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.dynamic_module import (
    MethodSpec,
    PortSpec,
    RefSpec,
    make_module_class,
    make_spec_class,
)
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.String import String
from dimos.orchestration.definition import (
    ActionActivity,
    CallActivity,
    CallStep,
    DefinitionError,
    StateMachineDefinition,
    load_definition,
)
from dimos.orchestration.expressions import kind_of

MACHINES_DIR = STATE_DIR / "machines"

_MESSAGE_TYPE = re.compile(r"([A-Za-z_]\w*)\.([A-Za-z_]\w*)")


class StateMachineConfig(ModuleConfig):
    definition: StateMachineDefinition
    state_file: Path | None = None
    params: dict[str, JsonValue] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _check_params(self) -> Self:
        defaults = self.definition.params
        for name, value in self.params.items():
            if name not in defaults:
                raise ValueError(f"params: unknown param {name!r}")
            if kind_of(value) != kind_of(defaults[name]):
                raise ValueError(
                    f"params: {name} is {kind_of(value)}, expected {kind_of(defaults[name])}"
                )
        return self


class StateMachineModule(Module):
    """Runs one state machine definition, with ports and role refs generated from its file."""

    config: StateMachineConfig

    action_result: In[GoalStatus]
    machine_state: Out[String]


def message_type(name: str) -> type:
    """Resolve a message type name like std_msgs.String, from dimos.msgs first, then dimos_lcm."""
    match = _MESSAGE_TYPE.fullmatch(name)
    if match is None:
        raise DefinitionError(f"expected a message type like std_msgs.String, got {name!r}")
    package, type_name = match.groups()
    for module_name in (f"dimos.msgs.{package}.{type_name}", f"dimos_lcm.{package}"):
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        found = getattr(module, type_name, None)
        if isinstance(found, type):
            return found
    raise DefinitionError(f"unknown message type {name!r}")


def _role_methods(definition: StateMachineDefinition) -> dict[str, dict[str, set[str]]]:
    roles: dict[str, dict[str, set[str]]] = {}

    def add(target: str, params: Iterable[str]) -> None:
        role, method = target.split(".")
        roles.setdefault(role, {}).setdefault(method, set()).update(params)

    for state in definition.states:
        steps = [*state.entry, *state.exit, *(step for t in state.transitions for step in t.run)]
        for step in steps:
            if isinstance(step, CallStep):
                add(step.call, step.args)
        if isinstance(state.do, CallActivity):
            add(state.do.call, state.do.args)
        elif isinstance(state.do, ActionActivity):
            add(state.do.action, [*state.do.args, "goal_id"])
            add(f"{state.do.action.split('.')[0]}.cancel_action", ["goal_id"])
    return roles


def _refs(definition: StateMachineDefinition) -> list[RefSpec]:
    refs: list[RefSpec] = []
    for role, methods in _role_methods(definition).items():
        spec_name = "".join(part.title() for part in role.split("_")) + "Role"
        method_specs = [MethodSpec(name, tuple(sorted(params))) for name, params in methods.items()]
        refs.append(RefSpec(role, make_spec_class(spec_name, method_specs)))
    return refs


def _ports(definition: StateMachineDefinition) -> list[PortSpec]:
    return [
        PortSpec(name, message_type(type_name), "in")
        for name, type_name in definition.inputs.items()
    ] + [
        PortSpec(name, message_type(type_name), "out")
        for name, type_name in definition.outputs.items()
    ]


def state_machine(path: Path | str) -> Blueprint:
    """Load a state machine file and return a blueprint that runs it."""
    definition = load_definition(path)
    try:
        cls = make_module_class(StateMachineModule, _ports(definition), refs=_refs(definition))
    except (DefinitionError, TypeError, ValueError) as error:
        raise DefinitionError(f"{path}: {error}") from error
    return cls.blueprint(definition=definition, instance_name=definition.state_machine.name)
