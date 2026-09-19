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

"""GlobalConfig as a configuration schema: its targets, names, defaults and validation."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from pydantic import AliasChoices, ValidationError

from dimos.core.coordination.blueprint_config.errors import (
    BlueprintConfigError,
    format_validation_error,
)
from dimos.core.coordination.blueprint_config.fields import leaf_fields, scalar_annotation_types
from dimos.core.coordination.blueprint_config.schema import OptionTarget, normalize_option_name
from dimos.core.coordination.blueprint_config.values import plain_mapping
from dimos.core.global_config import GlobalConfig


def global_schema_defaults() -> dict[str, Any]:
    """Return GlobalConfig defaults without consulting environment sources."""
    defaults = GlobalConfig.model_construct().model_dump(mode="python")
    return plain_mapping(defaults)


def validate_global_values(values: Mapping[str, Any]) -> dict[str, Any]:
    try:
        model = GlobalConfig.model_validate(values)
    except ValidationError as error:
        raise BlueprintConfigError(format_validation_error("g", error)) from error
    return model.model_dump(mode="python")


def global_option_targets() -> list[OptionTarget]:
    return [
        OptionTarget(section="global", root="g", path=path, annotations=(annotation,))
        for path, annotation in leaf_fields(GlobalConfig)
    ]


def global_targets_by_name(targets: list[OptionTarget]) -> dict[str, OptionTarget]:
    """Both spellings of every target, `--field` and `--g.field`."""
    by_name: dict[str, OptionTarget] = {}
    for target in targets:
        by_name[normalize_option_name(target.relative_name)] = target
        by_name[normalize_option_name(target.qualified_name)] = target
    return by_name


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
