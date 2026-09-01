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

from collections.abc import Iterable
from dataclasses import dataclass
import importlib.metadata as importlib_metadata
import re

from packaging.utils import canonicalize_name

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import is_module_type

ENTRY_POINT_GROUP = "dimos.blueprints"
LOCAL_BLUEPRINT_NAME_PATTERN = re.compile(r"^[a-z0-9]+(-[a-z0-9]+)*$")


class ExternalBlueprintError(ValueError):
    """Base class for external blueprint discovery and resolution errors."""


def _invalid_external_blueprint_name_error(
    local_name: str, distribution_name: str
) -> ExternalBlueprintError:
    return ExternalBlueprintError(
        "Invalid external blueprint entry point name "
        f"{local_name!r} in distribution {distribution_name!r}. "
        "External local blueprint names must be lowercase kebab-case "
        "and match ^[a-z0-9]+(-[a-z0-9]+)*$."
    )


def _invalid_external_blueprint_request_name_error(local_name: str) -> ExternalBlueprintError:
    return ExternalBlueprintError(
        f"Invalid external blueprint local name {local_name!r}. "
        "External local blueprint names must be lowercase kebab-case "
        "and match ^[a-z0-9]+(-[a-z0-9]+)*$."
    )


def _external_blueprint_namespace_not_found_error(
    namespace: str, available_namespaces: Iterable[str]
) -> ExternalBlueprintError:
    msg = f"External blueprint namespace {namespace!r} was not discovered."
    available = sorted(set(available_namespaces))
    if available:
        msg += f" Available external namespaces: {', '.join(available)}."
    return ExternalBlueprintError(msg)


def _external_blueprint_local_name_not_found_error(
    namespace: str, local_name: str, available_local_names: Iterable[str]
) -> ExternalBlueprintError:
    msg = f"External blueprint namespace {namespace!r} has no local blueprint {local_name!r}."
    available = sorted(set(available_local_names))
    if available:
        msg += f" Available local blueprints: {', '.join(available)}."
    return ExternalBlueprintError(msg)


def _external_blueprint_load_error(
    name: str, target: str, cause: Exception
) -> ExternalBlueprintError:
    return ExternalBlueprintError(
        f"Failed to load external blueprint {name!r} from entry point {target!r}: "
        f"{type(cause).__name__}: {cause}"
    )


def _invalid_external_blueprint_target_error(name: str, target: object) -> ExternalBlueprintError:
    return ExternalBlueprintError(
        f"External blueprint {name!r} loaded unsupported target {target!r}. "
        "Entry point targets must be a Blueprint object or a DimOS Module class."
    )


@dataclass(frozen=True)
class ExternalBlueprintEntry:
    namespace: str
    local_name: str
    distribution_name: str
    entry_point: importlib_metadata.EntryPoint

    @property
    def qualified_name(self) -> str:
        return f"{self.namespace}.{self.local_name}"

    @property
    def target(self) -> str:
        return self.entry_point.value


@dataclass(frozen=True)
class InvalidExternalBlueprintEntry:
    namespace: str
    local_name: str
    distribution_name: str


@dataclass(frozen=True)
class _ExternalBlueprintCollection:
    entries_by_namespace: dict[str, list[ExternalBlueprintEntry]]
    invalid_entries_by_namespace: dict[str, list[InvalidExternalBlueprintEntry]]


def canonicalize_distribution_namespace(distribution_name: str) -> str:
    """Normalize a Python distribution name for use as an external blueprint namespace."""

    return str(canonicalize_name(distribution_name))


def is_valid_external_local_blueprint_name(name: str) -> bool:
    """Return whether a local external blueprint name uses DimOS-style kebab-case."""

    return LOCAL_BLUEPRINT_NAME_PATTERN.fullmatch(name) is not None


def is_namespaced_blueprint_name(name: str) -> bool:
    """Return whether a runnable blueprint name has an external namespace separator."""

    return "." in name


def list_external_blueprint_names() -> list[str]:
    """List namespaced external blueprint names from installed package metadata."""

    return sorted(entry.qualified_name for entry in list_external_blueprints())


def list_external_blueprints() -> list[ExternalBlueprintEntry]:
    """List external blueprint entry point metadata without loading targets."""

    namespace_entries = _collect_external_blueprints().entries_by_namespace
    return sorted(
        (entry for entries in namespace_entries.values() for entry in entries),
        key=lambda entry: entry.qualified_name,
    )


def resolve_external_blueprint_by_name(name: str) -> Blueprint:
    """Resolve a fully-qualified external blueprint name to a Blueprint."""

    namespace, sep, local_name = name.partition(".")
    if not sep:
        raise _external_blueprint_namespace_not_found_error(name, [])
    if not is_valid_external_local_blueprint_name(local_name):
        raise _invalid_external_blueprint_request_name_error(local_name)

    collection = _collect_external_blueprints()
    namespace_entries = collection.entries_by_namespace
    if namespace not in namespace_entries:
        invalid_entries = collection.invalid_entries_by_namespace.get(namespace)
        if invalid_entries:
            invalid_entry = invalid_entries[0]
            raise _invalid_external_blueprint_name_error(
                invalid_entry.local_name, invalid_entry.distribution_name
            )
        raise _external_blueprint_namespace_not_found_error(namespace, namespace_entries.keys())

    entries = namespace_entries[namespace]
    matches = [entry for entry in entries if entry.local_name == local_name]
    if not matches:
        raise _external_blueprint_local_name_not_found_error(
            namespace, local_name, (entry.local_name for entry in entries)
        )

    entry = matches[0]
    try:
        target = entry.entry_point.load()
    except Exception as exc:
        raise _external_blueprint_load_error(entry.qualified_name, entry.target, exc) from exc

    return _target_to_blueprint(entry.qualified_name, target)


def _target_to_blueprint(name: str, target: object) -> Blueprint:
    if isinstance(target, Blueprint):
        return target
    if is_module_type(target):
        return target.blueprint()
    raise _invalid_external_blueprint_target_error(name, target)


def _collect_external_blueprints() -> _ExternalBlueprintCollection:
    entries_by_namespace: dict[str, list[ExternalBlueprintEntry]] = {}
    invalid_entries_by_namespace: dict[str, list[InvalidExternalBlueprintEntry]] = {}

    for entry_point in importlib_metadata.entry_points(group=ENTRY_POINT_GROUP):
        distribution = entry_point.dist
        if distribution is None:
            continue
        distribution_name = distribution.metadata.get("Name")
        if not distribution_name:
            continue
        namespace = canonicalize_distribution_namespace(distribution_name)
        local_name = entry_point.name
        if not is_valid_external_local_blueprint_name(local_name):
            invalid_entries_by_namespace.setdefault(namespace, []).append(
                InvalidExternalBlueprintEntry(
                    namespace=namespace,
                    local_name=local_name,
                    distribution_name=distribution_name,
                )
            )
            continue
        entries_by_namespace.setdefault(namespace, []).append(
            ExternalBlueprintEntry(
                namespace=namespace,
                local_name=local_name,
                distribution_name=distribution_name,
                entry_point=entry_point,
            )
        )

    return _ExternalBlueprintCollection(
        entries_by_namespace=entries_by_namespace,
        invalid_entries_by_namespace=invalid_entries_by_namespace,
    )
