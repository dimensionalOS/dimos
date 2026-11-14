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

"""The checks of a definition against the running blueprint."""

from collections.abc import Iterator, Mapping
from dataclasses import dataclass, replace
from inspect import Parameter
from typing import Literal

from dimos.core.introspection.module.info import RpcInfo
from dimos.orchestration.checks import CheckedDefinition, check
from dimos.orchestration.definition import OrchestratorDefinition
from dimos.orchestration.expressions import UNKNOWN, Shape
from dimos.orchestration.plain import shape_of

_VAR_KEYWORD = Parameter.VAR_KEYWORD.name
_VARIADIC = (Parameter.VAR_POSITIONAL.name, _VAR_KEYWORD)
_BY_NAME = (Parameter.POSITIONAL_OR_KEYWORD.name, Parameter.KEYWORD_ONLY.name)


@dataclass(frozen=True)
class Deployed:
    """What the running blueprint offers the modules a definition names."""

    modules: frozenset[str]
    rpcs: Mapping[str, RpcInfo]
    """By `Module.rpc`."""
    streams: Mapping[str, type]
    """Message types by `Module.stream`."""


def bind(definition: OrchestratorDefinition, deployed: Deployed) -> CheckedDefinition:
    """Check a definition against the running blueprint. Its problems hold every one found."""
    shapes, shape_problems = _shapes(definition, deployed)
    parameters = {
        activity.name: {
            param.name: shape_of(param.annotation) for param in deployed.rpcs[activity.start].params
        }
        for activity in definition.activities
        if activity.start in deployed.rpcs
    }
    checked = check(definition, shapes, parameters)
    problems = [
        *_activity_problems(definition, deployed),
        *_args_problems(definition, deployed),
        *shape_problems,
        *checked.problems,
    ]
    return replace(checked, problems=tuple(dict.fromkeys(problems)))


def _missing(
    deployed: Deployed, target: str, offered: Mapping[str, object], kind: Literal["RPC", "stream"]
) -> str:
    """Why the blueprint does not offer a `Module.name` target. Empty when it does."""
    module, _, name = target.partition(".")
    if module not in deployed.modules:
        return f"names `{target}`, and `{module}` is not a deployed module"
    if target not in offered:
        return f"names `{target}`, and `{module}` has no {kind} `{name}`"
    return ""


def _required(info: RpcInfo) -> list[str]:
    return [p.name for p in info.params if p.default is None and p.kind not in _VARIADIC]


def _activity_problems(definition: OrchestratorDefinition, deployed: Deployed) -> Iterator[str]:
    for activity in definition.activities:
        for field, target in (("start", activity.start), ("cancel", activity.cancel)):
            missing = _missing(deployed, target, deployed.rpcs, "RPC") if target else ""
            if missing:
                yield f"activity `{activity.name}`: `{field}` {missing}"
        cancel = deployed.rpcs.get(activity.cancel or "")
        for name in _required(cancel) if cancel is not None else ():
            yield (
                f"activity `{activity.name}`: `cancel` RPC `{activity.cancel}` needs `{name}`, "
                "and a cancel is called with no arguments"
            )


def _args_problems(definition: OrchestratorDefinition, deployed: Deployed) -> Iterator[str]:
    starts = {activity.name: activity.start for activity in definition.activities}
    for node in definition.nodes:
        target = starts.get(node.activity or "", "")
        info = deployed.rpcs.get(target)
        if info is None:
            continue
        named = {p.name for p in info.params if p.kind in _BY_NAME}
        takes_any = any(p.kind == _VAR_KEYWORD for p in info.params)
        for name in node.args:
            if name not in named and not takes_any:
                yield (
                    f"node `{node.name}`: `args` names `{name}`, "
                    f"which is not a parameter of `{target}`"
                )
        for name in _required(info):
            if name not in node.args:
                yield f"node `{node.name}`: `args` has no entry for `{name}`, which `{target}` needs"


def _shapes(
    definition: OrchestratorDefinition, deployed: Deployed
) -> tuple[dict[str, Shape], list[str]]:
    """The shape of every signal and result key, and what is wrong with any of them."""
    shapes: dict[str, Shape] = {}
    problems: list[str] = []
    for signal in definition.signals:
        missing = _missing(deployed, signal.stream, deployed.streams, "stream")
        if missing:
            problems.append(f"signal `{signal.name}`: `stream` {missing}")
            continue
        message = deployed.streams[signal.stream]
        shape = shape_of(message)
        if signal.field is not None and isinstance(shape, dict) and signal.field not in shape:
            problems.append(
                f"signal `{signal.name}`: `field` names `{signal.field}`, "
                f"which is not a field of `{message.__name__}`"
            )
        elif signal.field is not None:
            shape = shape[signal.field] if isinstance(shape, dict) else UNKNOWN
        shapes[signal.name] = shape

    starts = {activity.name: activity.start for activity in definition.activities}
    writers: dict[str, tuple[str, Shape]] = {}
    for node in definition.nodes:
        info = deployed.rpcs.get(starts.get(node.activity or "", ""))
        if node.result is None or info is None:
            continue
        shape = shape_of(info.return_annotation)
        first, earlier = writers.setdefault(node.result, (node.name, shape))
        if earlier is not UNKNOWN and shape is not UNKNOWN and repr(earlier) != repr(shape):
            problems.append(
                f"key `{node.result}` is written by `{first}` and `{node.name}`, "
                "and their `start` RPCs return different shapes"
            )
    shapes.update({key: shape for key, (_, shape) in writers.items()})
    return shapes, problems
