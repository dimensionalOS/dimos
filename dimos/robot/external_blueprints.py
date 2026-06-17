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
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import is_module_type

ENTRY_POINT_GROUP = "dimos.blueprints"
LOCAL_BLUEPRINT_NAME_PATTERN = re.compile(r"^[a-z0-9]+(-[a-z0-9]+)*$")
_DISTRIBUTION_SEPARATOR_PATTERN = re.compile(r"[-_.]+")


class ExternalBlueprintError(ValueError):
    """Base class for external blueprint discovery and resolution errors."""


class InvalidExternalBlueprintNameError(ExternalBlueprintError):
    def __init__(self, local_name: str, distribution_name: str) -> None:
        super().__init__(
            "Invalid external blueprint entry point name "
            f"{local_name!r} in distribution {distribution_name!r}. "
            "External local blueprint names must be lowercase kebab-case "
            "and match ^[a-z0-9]+(-[a-z0-9]+)*$."
        )


class AmbiguousExternalBlueprintNamespaceError(ExternalBlueprintError):
    def __init__(self, namespace: str, distribution_names: Iterable[str]) -> None:
        names = sorted(set(distribution_names))
        super().__init__(
            f"Ambiguous external blueprint namespace {namespace!r}; "
            f"multiple installed distributions normalize to it: {', '.join(names)}."
        )


class AmbiguousExternalBlueprintNameError(ExternalBlueprintError):
    def __init__(self, name: str) -> None:
        super().__init__(
            f"Ambiguous external blueprint name {name!r}; multiple entry points match it."
        )


class ExternalBlueprintNamespaceNotFoundError(ExternalBlueprintError):
    def __init__(self, namespace: str, available_namespaces: Iterable[str]) -> None:
        msg = f"External blueprint namespace {namespace!r} was not discovered."
        available = sorted(set(available_namespaces))
        if available:
            msg += f" Available external namespaces: {', '.join(available)}."
        super().__init__(msg)


class ExternalBlueprintLocalNameNotFoundError(ExternalBlueprintError):
    def __init__(
        self, namespace: str, local_name: str, available_local_names: Iterable[str]
    ) -> None:
        msg = f"External blueprint namespace {namespace!r} has no local blueprint {local_name!r}."
        available = sorted(set(available_local_names))
        if available:
            msg += f" Available local blueprints: {', '.join(available)}."
        super().__init__(msg)


class ExternalBlueprintLoadError(ExternalBlueprintError):
    def __init__(self, name: str, target: str, cause: Exception) -> None:
        super().__init__(
            f"Failed to load external blueprint {name!r} from entry point {target!r}: "
            f"{type(cause).__name__}: {cause}"
        )


class InvalidExternalBlueprintTargetError(ExternalBlueprintError):
    def __init__(self, name: str, target: Any) -> None:
        super().__init__(
            f"External blueprint {name!r} loaded unsupported target {target!r}. "
            "Entry point targets must be a Blueprint object or a DimOS Module class. "
            "Factory functions are not supported."
        )


@dataclass(frozen=True)
class ExternalBlueprintEntry:
    namespace: str
    local_name: str
    distribution_name: str
    entry_point: Any

    @property
    def qualified_name(self) -> str:
        return f"{self.namespace}.{self.local_name}"

    @property
    def target(self) -> str:
        return str(getattr(self.entry_point, "value", "<unknown>"))


def canonicalize_distribution_namespace(distribution_name: str) -> str:
    """Normalize a Python distribution name for use as an external blueprint namespace."""

    return _DISTRIBUTION_SEPARATOR_PATTERN.sub("-", distribution_name).lower()


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

    namespace_entries = _collect_external_blueprints()
    return sorted(
        (entry for entries in namespace_entries.values() for entry in entries),
        key=lambda entry: entry.qualified_name,
    )


def resolve_external_blueprint_by_name(name: str) -> Blueprint:
    """Resolve a fully-qualified external blueprint name to a Blueprint."""

    namespace, sep, local_name = name.partition(".")
    if not sep:
        raise ExternalBlueprintNamespaceNotFoundError(name, [])
    if not is_valid_external_local_blueprint_name(local_name):
        raise InvalidExternalBlueprintNameError(local_name, namespace)

    namespace_entries = _collect_external_blueprints()
    if namespace not in namespace_entries:
        raise ExternalBlueprintNamespaceNotFoundError(namespace, namespace_entries.keys())

    entries = namespace_entries[namespace]
    matches = [entry for entry in entries if entry.local_name == local_name]
    if not matches:
        raise ExternalBlueprintLocalNameNotFoundError(
            namespace, local_name, (entry.local_name for entry in entries)
        )
    if len(matches) > 1:
        raise AmbiguousExternalBlueprintNameError(name)

    entry = matches[0]
    try:
        target = entry.entry_point.load()
    except Exception as exc:
        raise ExternalBlueprintLoadError(entry.qualified_name, entry.target, exc) from exc

    return _target_to_blueprint(entry.qualified_name, target)


def _target_to_blueprint(name: str, target: Any) -> Blueprint:
    if isinstance(target, Blueprint):
        return target
    if is_module_type(target):
        return target.blueprint()  # type: ignore[no-any-return]
    raise InvalidExternalBlueprintTargetError(name, target)


def _collect_external_blueprints() -> dict[str, list[ExternalBlueprintEntry]]:
    entries_by_namespace: dict[str, list[ExternalBlueprintEntry]] = {}
    distribution_names_by_namespace: dict[str, set[str]] = {}

    for distribution in importlib_metadata.distributions():
        distribution_name = _distribution_name(distribution)
        if distribution_name is None:
            continue

        namespace = canonicalize_distribution_namespace(distribution_name)
        external_entry_points = [
            entry_point
            for entry_point in getattr(distribution, "entry_points", ())
            if getattr(entry_point, "group", None) == ENTRY_POINT_GROUP
        ]
        if not external_entry_points:
            continue

        distribution_names_by_namespace.setdefault(namespace, set()).add(distribution_name)
        for entry_point in external_entry_points:
            local_name = str(getattr(entry_point, "name", ""))
            if not is_valid_external_local_blueprint_name(local_name):
                raise InvalidExternalBlueprintNameError(local_name, distribution_name)
            entries_by_namespace.setdefault(namespace, []).append(
                ExternalBlueprintEntry(
                    namespace=namespace,
                    local_name=local_name,
                    distribution_name=distribution_name,
                    entry_point=entry_point,
                )
            )

    for namespace, distribution_names in distribution_names_by_namespace.items():
        if len(distribution_names) > 1 and namespace in entries_by_namespace:
            raise AmbiguousExternalBlueprintNamespaceError(namespace, distribution_names)

    return entries_by_namespace


def _distribution_name(distribution: Any) -> str | None:
    metadata = getattr(distribution, "metadata", None)
    if metadata is not None:
        name = metadata.get("Name")
        if name:
            return str(name)
    name = getattr(distribution, "name", None)
    if name:
        return str(name)
    return None
