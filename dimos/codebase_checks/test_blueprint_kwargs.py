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

import inspect
from typing import Any, get_type_hints

from pydantic import BaseModel
import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import ModuleBase
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.get_all_blueprints import get_blueprint_by_name
from dimos.robot.test_all_blueprints import (
    OPTIONAL_DEPENDENCIES,
    OPTIONAL_ERROR_SUBSTRINGS,
    SELF_HOSTED_BLUEPRINTS,
)


def _get_blueprint_or_skip(blueprint_name: str) -> Blueprint:
    try:
        return get_blueprint_by_name(blueprint_name)
    except ModuleNotFoundError as e:
        if e.name in OPTIONAL_DEPENDENCIES:
            pytest.skip(f"Skipping due to missing optional dependency: {e.name}")
        raise
    except Exception as e:
        message = str(e)
        if any(substring in message for substring in OPTIONAL_ERROR_SUBSTRINGS):
            pytest.skip(f"Skipping due to missing optional dependency: {message}")
        raise


def _config_kwarg_names(module: type[ModuleBase]) -> set[str]:
    config_type = get_type_hints(module).get("config")
    if isinstance(config_type, type) and issubclass(config_type, BaseModel):
        return set(config_type.model_fields)
    return set()


def _declares_config(module: type[ModuleBase]) -> bool:
    annotations: dict[str, Any] = module.__dict__.get("__annotations__", {})
    return "config" in annotations


def _init_kwarg_names(module: type[ModuleBase]) -> set[str]:
    signature = inspect.signature(module.__init__)
    names = {
        name
        for name, parameter in signature.parameters.items()
        if name != "self"
        and parameter.kind
        in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
    }
    if any(p.kind is inspect.Parameter.VAR_KEYWORD for p in signature.parameters.values()):
        names.update(_config_kwarg_names(module))
    return names


def _allowed_kwarg_names(module: type[ModuleBase]) -> set[str]:
    if _declares_config(module):
        return _config_kwarg_names(module)
    return _init_kwarg_names(module)


def _blueprint_params() -> list[str | pytest.ParameterSet]:
    self_hosted = set(SELF_HOSTED_BLUEPRINTS)
    return [
        pytest.param(name, marks=pytest.mark.self_hosted) if name in self_hosted else name
        for name in sorted(all_blueprints)
    ]


@pytest.mark.parametrize("blueprint_name", _blueprint_params())
def test_blueprint_atom_kwargs_match_module_config(blueprint_name: str) -> None:
    """Fail when blueprint kwargs cannot be consumed by their target module."""
    blueprint = _get_blueprint_or_skip(blueprint_name)

    violations: list[str] = []
    for atom in blueprint.blueprints:
        unknown_kwargs = sorted(set(atom.kwargs) - _allowed_kwarg_names(atom.module))
        if unknown_kwargs:
            violations.append(
                f"{atom.module.__module__}.{atom.module.__name__}: unknown kwargs {unknown_kwargs}"
            )

    if violations:
        listing = "\n".join(f"  - {violation}" for violation in violations)
        raise AssertionError(
            f"Blueprint {blueprint_name!r} passes unknown module kwargs:\n{listing}\n\n"
            "Blueprint kwargs are forwarded into the module constructor. For modules "
            "with an explicit `config` annotation, use fields from that config model; "
            "for legacy modules with direct constructor parameters, use the declared "
            "`__init__` keyword names."
        )
