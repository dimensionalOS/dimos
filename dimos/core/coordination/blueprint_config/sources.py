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

"""Configuration sources: config file, environment, and GlobalConfig resolution."""

from __future__ import annotations

from collections.abc import Mapping
import json
import os
from pathlib import Path
from typing import Any

from dotenv import dotenv_values
from pydantic import AliasChoices, ValidationError

from dimos.core.coordination.blueprint_config.errors import (
    BlueprintConfigError,
    format_validation_error,
)
from dimos.core.coordination.blueprint_config.fields import leaf_fields, scalar_annotation_types
from dimos.core.coordination.blueprint_config.schema import (
    OptionTarget,
    coerce_cli_value,
    coerce_environment_value,
    normalize_option_name,
)
from dimos.core.coordination.blueprint_config.values import deep_set, plain_mapping
from dimos.core.global_config import GlobalConfig


def global_schema_defaults() -> dict[str, Any]:
    """Return GlobalConfig defaults without consulting environment sources."""
    defaults = GlobalConfig.model_construct().model_dump(mode="python")
    return plain_mapping(defaults)


def configuration_environment(
    environ: Mapping[str, str] | None,
) -> Mapping[str, str]:
    if environ is not None:
        return environ
    from_dotenv = {key: value for key, value in dotenv_values(".env").items() if value is not None}
    return {**from_dotenv, **os.environ}


def validate_global_values(values: Mapping[str, Any]) -> dict[str, Any]:
    try:
        model = GlobalConfig.model_validate(values)
    except ValidationError as error:
        raise BlueprintConfigError(format_validation_error("g", error)) from error
    return model.model_dump(mode="python")


def read_config_file(path: Path) -> Mapping[str, Any]:
    try:
        raw = path.read_text()
    except (FileNotFoundError, IsADirectoryError):
        return {}
    except OSError as error:
        raise BlueprintConfigError(f"Could not read config file {path}: {error}") from error
    try:
        values = json.loads(raw)
    except json.JSONDecodeError as error:
        raise BlueprintConfigError(
            f"Invalid JSON in config file {path}: {error.msg} "
            f"(line {error.lineno}, column {error.colno})"
        ) from error
    if not isinstance(values, Mapping):
        raise BlueprintConfigError(f"Config file {path} must contain a JSON object.")
    return values


def merge_global_environment(
    destination: dict[str, Any],
    environ: Mapping[str, str],
) -> None:
    global_env_names = global_environment_names()
    targets = _global_option_targets()

    def set_coerced(path: tuple[str, ...], raw_name: str, value: str) -> None:
        target = targets.get(normalize_option_name(".".join(path)))
        coerced = value if target is None else coerce_environment_value(value, target, raw_name)
        deep_set(destination, path, coerced)

    for raw_name, value in environ.items():
        global_field = global_env_names.get(raw_name.lower())
        if global_field is not None:
            set_coerced((global_field,), raw_name, value)
            continue
        parts = tuple(part.lower().replace("-", "_") for part in raw_name.split("__"))
        if len(parts) >= 2 and parts[0] == "g":
            set_coerced(parts[1:], raw_name, value)


def _global_option_targets() -> dict[str, OptionTarget]:
    targets: dict[str, OptionTarget] = {}
    for path, annotation in leaf_fields(GlobalConfig):
        target = OptionTarget(
            section="global",
            root="g",
            path=path,
            annotations=(annotation,),
        )
        targets[normalize_option_name(target.relative_name)] = target
        targets[normalize_option_name(target.qualified_name)] = target
    return targets


def merge_global_cli(
    destination: dict[str, Any],
    tokens: tuple[str, ...],
) -> None:
    targets = _global_option_targets()
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if not token.startswith("--"):
            index += 1
            continue

        option, separator, attached_value = token[2:].partition("=")
        normalized = normalize_option_name(option)
        target = targets.get(normalized)
        negated = False
        if target is None and normalized.startswith("no_"):
            target = targets.get(normalized.removeprefix("no_"))
            negated = target is not None and target.is_bool
        if target is None:
            index += 1
            continue

        if separator:
            raw_value: Any = attached_value
        elif negated:
            raw_value = False
        elif target.is_bool:
            if index + 1 < len(tokens) and not tokens[index + 1].startswith("--"):
                index += 1
                raw_value = tokens[index]
            else:
                raw_value = True
        else:
            if index + 1 >= len(tokens) or tokens[index + 1].startswith("--"):
                raise BlueprintConfigError(
                    f"Option --{option} requires a value. "
                    f"Use --{option}=VALUE when the value starts with '--'."
                )
            index += 1
            raw_value = tokens[index]

        if negated and separator:
            raise BlueprintConfigError(f"Negated option --{option} does not accept a value.")
        deep_set(destination, target.path, coerce_cli_value(raw_value, target, option))
        index += 1


def reserved_global_short_names() -> set[str]:
    reserved = set(GlobalConfig.model_fields)
    for name, info in GlobalConfig.model_fields.items():
        if scalar_annotation_types(info.annotation) == {bool}:
            reserved.add(f"no_{name}")
    return reserved


def global_environment_names() -> dict[str, str]:
    names: dict[str, str] = {}
    for field_name, info in GlobalConfig.model_fields.items():
        names[field_name.lower()] = field_name
        alias = info.validation_alias
        if isinstance(alias, str):
            names[alias.lower()] = field_name
        elif isinstance(alias, AliasChoices):
            for choice in alias.choices:
                if isinstance(choice, str):
                    names[choice.lower()] = field_name
    return names
