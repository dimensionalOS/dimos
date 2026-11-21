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

"""Every BaseAgent in the repo names its input and output types."""

import ast
import importlib
from pathlib import Path
from typing import TypeVar

import pytest

from dimos.agents.base_agent import BaseAgent
from dimos.constants import DIMOS_PROJECT_ROOT

_PACKAGE = DIMOS_PROJECT_ROOT / "dimos"


def _base_names(node: ast.ClassDef) -> list[tuple[str, bool]]:
    """Each base's class name, and whether it is subscripted."""
    names = []
    for base in node.bases:
        subscripted = isinstance(base, ast.Subscript)
        target = base.value if isinstance(base, ast.Subscript) else base
        if isinstance(target, ast.Name):
            names.append((target.id, subscripted))
        elif isinstance(target, ast.Attribute):
            names.append((target.attr, subscripted))
    return names


def _importable(path: Path) -> bool:
    return not path.name.startswith(("test_", "demo_", "conftest"))


def _agent_classes() -> tuple[list[tuple[Path, str]], list[str]]:
    """(file, class) of every BaseAgent subclass, and the ones that subclass it bare."""
    sources = {
        path: path.read_text()
        for path in _PACKAGE.rglob("*.py")
        if ".venv" not in path.parts and "node_modules" not in path.parts
    }
    known = {"BaseAgent"}
    found: dict[tuple[Path, str], None] = {}
    bare: list[str] = []
    grew = True
    while grew:
        grew = False
        for path, text in sources.items():
            if not any(name in text for name in known):
                continue
            for node in ast.walk(ast.parse(text)):
                if not isinstance(node, ast.ClassDef) or (path, node.name) in found:
                    continue
                bases = _base_names(node)
                if not any(name in known for name, _ in bases):
                    continue
                found[(path, node.name)] = None
                if ("BaseAgent", False) in bases:
                    bare.append(f"{path.relative_to(DIMOS_PROJECT_ROOT)}: {node.name}")
                # Classes in tests are matched by name only, so they are not followed further.
                if _importable(path) and node.name not in known:
                    known.add(node.name)
                    grew = True
    return list(found), bare


_AGENTS, _BARE = _agent_classes()


def test_no_agent_subclasses_base_agent_without_types() -> None:
    assert not _BARE, (
        "these subclass BaseAgent without naming types; write BaseAgent[Input, Output]: "
        + ", ".join(_BARE)
    )


@pytest.mark.parametrize(
    "path, name",
    [agent for agent in _AGENTS if _importable(agent[0])],
    ids=lambda value: value if isinstance(value, str) else "",
)
def test_agent_has_concrete_input_and_output_types(path: Path, name: str) -> None:
    module = ".".join(path.relative_to(DIMOS_PROJECT_ROOT).with_suffix("").parts)
    agent = getattr(importlib.import_module(module), name)

    if not issubclass(agent, BaseAgent):
        return  # another class's base shares a name with an agent
    if getattr(agent, "__parameters__", ()):
        return  # generic itself; its subclasses bind the types
    assert not isinstance(agent.input_type, TypeVar), f"{name} has no input type"
    assert not isinstance(agent.output_type, TypeVar), f"{name} has no output type"
