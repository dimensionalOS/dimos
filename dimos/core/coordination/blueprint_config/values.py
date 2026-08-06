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

"""Plain-data helpers: copying, merging, and freezing configuration values."""

from __future__ import annotations

from collections.abc import Mapping
import copy
from types import MappingProxyType
from typing import Any, cast

from pydantic import BaseModel

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError


def normalize_mapping_keys(values: Mapping[str, Any]) -> dict[str, Any]:
    """Normalize schema field names without rewriting nested data mappings."""
    normalized: dict[str, Any] = {}
    for key, value in values.items():
        if not isinstance(key, str):
            raise BlueprintConfigError("Configuration object keys must be strings.")
        normalized_key = key.replace("-", "_")
        normalized[normalized_key] = plain(value)
    return normalized


def plain_mapping(values: Mapping[str, Any]) -> dict[str, Any]:
    return {key: plain(value) for key, value in values.items()}


def plain(value: Any) -> Any:
    if isinstance(value, BaseModel):
        return {key: plain(item) for key, item in value.model_dump(mode="python").items()}
    if isinstance(value, Mapping):
        return {_copy_opaque(key): plain(item) for key, item in value.items()}
    if isinstance(value, list):
        return [plain(item) for item in value]
    if isinstance(value, tuple):
        return tuple(plain(item) for item in value)
    if isinstance(value, set):
        return {plain(item) for item in value}
    return _copy_opaque(value)


def deep_merge(destination: dict[str, Any], incoming: Mapping[str, Any]) -> None:
    for key, value in incoming.items():
        if key in destination and isinstance(destination[key], dict) and isinstance(value, Mapping):
            deep_merge(destination[key], value)
        else:
            destination[key] = plain(value)


def deep_set(destination: dict[str, Any], path: tuple[str, ...], value: Any) -> None:
    current = destination
    for part in path[:-1]:
        child = current.setdefault(part, {})
        if not isinstance(child, dict):
            child = {}
            current[part] = child
        current = child
    current[path[-1]] = plain(value)


def extract_shape(values: Mapping[str, Any], shape: Mapping[str, Any]) -> dict[str, Any]:
    extracted: dict[str, Any] = {}
    for key, shape_value in shape.items():
        if key not in values:
            continue
        value = values[key]
        if isinstance(shape_value, Mapping) and isinstance(value, Mapping):
            extracted[key] = extract_shape(value, shape_value)
        else:
            extracted[key] = plain(value)
    return extracted


def read_only_view(value: Any) -> Any:
    """Return a detached, recursively read-only view of a snapshot."""
    if isinstance(value, Mapping):
        return MappingProxyType(
            {_copy_opaque(key): read_only_view(item) for key, item in value.items()}
        )
    if isinstance(value, (list, tuple)):
        return tuple(read_only_view(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(read_only_view(item) for item in value)
    if isinstance(value, BaseModel):
        return read_only_view(value.model_dump(mode="python"))
    return _copy_opaque(value)


def snapshot_mapping(value: Mapping[str, Any]) -> dict[str, Any]:
    """Copy a configuration mapping without changing nested Python types."""
    snapshot = _copy_opaque(dict(value))
    return cast("dict[str, Any]", snapshot)


def _copy_opaque(value: Any) -> Any:
    try:
        return copy.deepcopy(value)
    except Exception as error:
        raise BlueprintConfigError(
            f"Configuration value of type {type(value).__name__} cannot be copied safely: {error}"
        ) from error
