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

import difflib
import re
import sys
from typing import NoReturn

import typer

from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.all_blueprints import all_blueprints, all_modules

all_names = sorted(set(all_blueprints.keys()) | set(all_modules.keys()))


def class_name_to_registry_key(class_name: str) -> str:
    """Convert a CamelCase class name to its kebab-case registry key."""
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", class_name)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    return s.lower().replace("_", "-")


def _raise_unknown(name: str, candidates: list[str]) -> NoReturn:
    msg = f"Unknown blueprint or module: {name!r}"
    suggestions = difflib.get_close_matches(name, candidates, n=5, cutoff=0.4)
    if suggestions:
        msg += f". Did you mean: {', '.join(suggestions)}?"
    raise ValueError(msg)


def get_blueprint_by_name(name: str) -> Blueprint:
    if name not in all_blueprints:
        _raise_unknown(name, list(all_blueprints.keys()))
    module_path, attr = all_blueprints[name].split(":")
    module = __import__(module_path, fromlist=[attr])
    return getattr(module, attr)  # type: ignore[no-any-return]


def get_module_by_name(name: str) -> Blueprint:
    if name not in all_modules:
        _raise_unknown(name, list(all_modules.keys()))
    module_path, class_name = all_modules[name].rsplit(".", 1)
    python_module = __import__(module_path, fromlist=[class_name])
    return getattr(python_module, class_name).blueprint()  # type: ignore[no-any-return]


def get_by_name(name: str) -> Blueprint:
    if name in all_blueprints:
        return get_blueprint_by_name(name)
    elif name in all_modules:
        return get_module_by_name(name)
    else:
        _raise_unknown(name, all_names)


def _fail_or_exit(name: str, candidates: list[str]) -> NoReturn:
    typer.echo(typer.style(f"Unknown blueprint or module: {name}", fg=typer.colors.RED), err=True)
    suggestions = difflib.get_close_matches(name, candidates, n=5, cutoff=0.4)
    if suggestions:
        typer.echo("Did you mean one of these?", err=True)
        for s in suggestions:
            typer.echo(f"  {s}", err=True)
    sys.exit(1)


def get_by_name_or_exit(name: str) -> Blueprint:
    if name in all_blueprints:
        return get_blueprint_by_name(name)
    elif name in all_modules:
        return get_module_by_name(name)
    else:
        _fail_or_exit(name, all_names)


def get_module_by_name_or_exit(name: str) -> Blueprint:
    if name not in all_modules:
        _fail_or_exit(name, list(all_modules.keys()))
    return get_module_by_name(name)
