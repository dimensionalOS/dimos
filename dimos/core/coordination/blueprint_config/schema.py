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

"""Option schema: the configuration targets a blueprint exposes."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import json
from typing import Any, Literal

from pydantic import BaseModel

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.fields import (
    all_annotation_types,
    scalar_annotation_types,
)
from dimos.core.coordination.blueprints import BlueprintAtom, TransportSpec


@dataclass(frozen=True, slots=True)
class ModuleSchema:
    atom: BlueprintAtom
    config_cls: type[BaseModel]


@dataclass(frozen=True, slots=True)
class TransportSchema:
    name: str
    config_cls: type[BaseModel]
    specs: tuple[TransportSpec, ...]


@dataclass(frozen=True, slots=True)
class OptionTarget:
    section: Literal["module", "global", "transport"]
    root: str
    path: tuple[str, ...]
    annotations: tuple[Any, ...]

    @property
    def identity(self) -> tuple[str, str, tuple[str, ...]]:
        return (self.section, self.root, self.path)

    @property
    def qualified_name(self) -> str:
        if self.section == "global":
            parts = ("g", *self.path)
        elif self.section == "transport":
            parts = ("transports", self.root, *self.path)
        else:
            parts = (self.root, *self.path)
        return cli_path(parts)

    @property
    def relative_name(self) -> str:
        if self.section == "transport":
            return cli_path((self.root, *self.path))
        return cli_path(self.path)

    @property
    def scalar_types(self) -> set[Any]:
        scalar_types: set[Any] = set()
        for annotation in self.annotations:
            scalar_types.update(scalar_annotation_types(annotation))
        return scalar_types

    @property
    def is_bool(self) -> bool:
        return self.scalar_types == {bool}

    @property
    def allows_none(self) -> bool:
        return any(type(None) in all_annotation_types(a) for a in self.annotations)


@dataclass(frozen=True, slots=True)
class ParserSchema:
    modules: tuple[ModuleSchema, ...]
    transports: tuple[TransportSchema, ...]
    targets: tuple[OptionTarget, ...]
    aliases: Mapping[str, tuple[OptionTarget, ...]]


def cli_path(parts: tuple[str, ...]) -> str:
    return ".".join(part.replace("_", "-") for part in parts)


def normalize_option_name(name: str) -> str:
    return ".".join(part.replace("-", "_").lower() for part in name.split("."))


def display_normalized_option(name: str) -> str:
    return ".".join(part.replace("_", "-") for part in name.split("."))


def coerce_cli_value(value: Any, target: OptionTarget, option: str) -> Any:
    if target.is_bool:
        if isinstance(value, bool):
            return value
        normalized = str(value).lower()
        if normalized not in {"true", "false"}:
            raise BlueprintConfigError(
                f"Boolean option --{option} requires an explicit true or false value."
            )
        return normalized == "true"

    return coerce_string_value(value, target, f"Option --{option}")


def coerce_environment_value(value: str, target: OptionTarget, name: str) -> Any:
    """Coerce an environment value like a CLI value.

    Booleans keep pydantic's lax coercion and pure-str fields keep their raw
    string (a value starting with '[' or '{' must not become an error), so
    environment usage that predates the coercion keeps working.
    """
    scalars = target.scalar_types
    parse_json = scalars != {bool} and scalars != {str}
    return coerce_string_value(value, target, f"Environment variable {name}", parse_json=parse_json)


def coerce_string_value(
    value: Any, target: OptionTarget, label: str, *, parse_json: bool = True
) -> Any:
    if not isinstance(value, str):
        return value
    stripped = value.strip()
    if stripped == "null" and target.allows_none:
        return None
    if parse_json and stripped.startswith(("[", "{")):
        try:
            return json.loads(stripped)
        except json.JSONDecodeError as error:
            raise BlueprintConfigError(f"{label} contains invalid JSON: {error.msg}.") from error
    return value
