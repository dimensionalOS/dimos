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

"""Reads an orchestrator file into its definition."""

from collections.abc import Iterable, Iterator, Mapping
from pathlib import Path
import re
import sys

from pydantic import JsonValue, ValidationError
from pydantic_core import ErrorDetails

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from dimos.orchestration.checks import check
from dimos.orchestration.definition import OrchestratorDefinition

# The file's tables are singular, and the fields they fill are plural.
_FILE_TABLES = {"signal": "signals", "event": "events", "activity": "activities", "node": "nodes"}
_NODE_TABLES = {"transition": "transitions"}
_ITEM_NAMES = {
    **{field: table for table, field in (_FILE_TABLES | _NODE_TABLES).items()},
    "requires": "`requires` check",
    "ensures": "`ensures` check",
}


class LoadError(Exception):
    """Everything wrong with one orchestrator file."""

    def __init__(self, path: Path, problems: Iterable[str]) -> None:
        self.problems = tuple(problems)
        super().__init__(f"{path}:\n" + "\n".join(f"  {problem}" for problem in self.problems))


def load(path: str | Path) -> OrchestratorDefinition:
    """Read an orchestrator file and run the static checks. Raises `LoadError` with every problem."""
    path = Path(path)
    try:
        raw: dict[str, JsonValue] = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as error:
        raise LoadError(path, [f"not valid TOML: {error}"]) from None
    data = _renamed(raw, _FILE_TABLES)
    nodes = data.get("nodes")
    if isinstance(nodes, list):
        data["nodes"] = [
            _renamed(node, _NODE_TABLES) if isinstance(node, dict) else node for node in nodes
        ]
    try:
        definition = OrchestratorDefinition.model_validate(data)
    except ValidationError as error:
        raise LoadError(path, [_problem(found, data) for found in error.errors()]) from None
    problems = [*check(definition).problems, *_quoting_problems(definition)]
    if problems:
        raise LoadError(path, problems)
    return definition


def _renamed(table: dict[str, JsonValue], names: Mapping[str, str]) -> dict[str, JsonValue]:
    return {names.get(key, key): value for key, value in table.items()}


def _inside(data: JsonValue, key: int | str) -> JsonValue:
    if isinstance(data, dict) and isinstance(key, str):
        return data.get(key)
    if isinstance(data, list) and isinstance(key, int) and key < len(data):
        return data[key]
    return None


def _problem(error: ErrorDetails, data: JsonValue) -> str:
    """One validation error, with where it is in the words the file uses."""
    where: list[str] = []
    loc = list(error["loc"])
    while loc:
        key = loc.pop(0)
        data = _inside(data, key)
        index = loc[0] if loc else None
        if isinstance(key, str) and key in _ITEM_NAMES and isinstance(index, int):
            loc.pop(0)
            data = _inside(data, index)
            name = data.get("name") if isinstance(data, dict) else None
            label = f"`{name}`" if isinstance(name, str) else str(index + 1)
            where.append(f"{_ITEM_NAMES[key]} {label}")
        else:
            where.append(f"`{key}`" if isinstance(key, str) else f"item {key + 1}")
    if error["type"] == "value_error":
        return ": ".join([*where, str(error["ctx"]["error"])])
    if error["type"] == "missing":
        said = "is required"
    elif error["type"] == "extra_forbidden":
        said = "is not a field"
    else:
        said = re.sub("^[Ii]nput ", "", error["msg"]).replace("tuple", "list")
    return ": ".join([*where[:-1], f"{where[-1]} {said}"])


def _quoting_problems(definition: OrchestratorDefinition) -> Iterator[str]:
    """Each key a `set` names that TOML read under `[vars]` as a map and a field of it."""
    keys = {key for node in definition.nodes for t in node.transitions for key in t.set}
    for key in sorted(keys):
        head, *fields = key.split(".")
        value = definition.vars.get(head)
        for field in fields:
            value = value.get(field) if isinstance(value, dict) else None
        if fields and value is not None:
            yield (
                f"`set` names `{key}`, and `[vars]` holds it as a field of the map `{head}`. "
                f'Write the key in quotes there: `"{key}"`'
            )
