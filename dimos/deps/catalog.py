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

"""Runtime reader of the generated blueprint catalog.

The catalog is static data shipped next to this module. Reading it needs no
part of the runtime framework, so the launcher can plan an environment
before importing any blueprint implementation. A plan is *complete* when the
catalog could vouch for everything the request runs; external blueprints and
selections the planner cannot read make it incomplete, and the launcher then
refuses to switch environments automatically.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from functools import lru_cache
import json
from pathlib import Path
from typing import Any

from dimos.deps.predicates import holds
from dimos.deps.selectors import SelectorInput, identifiers

CATALOG_PATH = Path(__file__).with_name("blueprint_catalog.json")
CATALOG_VERSION = 2
GLOBAL_INSTANCE = "g"
EXTERNAL_CATALOG_GROUP = "dimos.catalog"
"""Entry point group an external distribution may publish catalog entries in (not read yet)."""
MERGED_KEYS = ("backends", "native", "system", "tools")


class CatalogError(ValueError):
    """The catalog is missing, malformed or does not know a requested name."""


@dataclass(frozen=True)
class Plan:
    """Requirements of one run: blueprint presets plus active variants, rules and selections."""

    extras: frozenset[str] = frozenset()
    reasons: Mapping[str, frozenset[str]] = field(default_factory=dict)
    """Extra -> files (or ``rule:<name>``, ``backend:<name>``) that require it."""
    backends: Mapping[str, str] = field(default_factory=dict)
    """Backend -> accelerator it was resolved with (empty string when unresolved)."""
    native: frozenset[str] = frozenset()
    system: frozenset[str] = frozenset()
    tools: frozenset[str] = frozenset()
    external: tuple[str, ...] = ()
    """Requested namespaced (external) blueprint names, which the catalog cannot plan."""
    incomplete: tuple[str, ...] = ()
    """Why automatic planning cannot vouch for the whole run; empty when it can."""

    @property
    def complete(self) -> bool:
        return not self.incomplete

    @property
    def extras_argument(self) -> str:
        return ",".join(sorted(self.extras))


class Catalog:
    def __init__(self, data: Mapping[str, Any]) -> None:
        if data.get("version") != CATALOG_VERSION:
            raise CatalogError(f"unsupported catalog version {data.get('version')!r}")
        self._data = data
        self._entries: dict[str, Mapping[str, Any]] = {}
        for section in ("blueprints", "modules"):
            self._entries.update(data.get(section, {}))
        self._registries: Mapping[str, Mapping[str, Mapping[str, Any]]] = data.get("registries", {})

    @classmethod
    def load(cls, path: Path | None = None) -> Catalog:
        path = path or CATALOG_PATH
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            raise CatalogError(f"cannot read the blueprint catalog at {path}: {error}") from error
        return cls(data)

    @property
    def names(self) -> frozenset[str]:
        return frozenset(self._entries)

    @property
    def defaults(self) -> Mapping[str, object]:
        defaults: Mapping[str, object] = self._data.get("defaults", {})
        return defaults

    def entry(self, name: str) -> Mapping[str, Any] | None:
        return self._entries.get(name)

    def selector_fields(self) -> frozenset[str]:
        """Module configuration fields whose values select registry implementations."""
        fields: set[str] = set()
        for entry in self._entries.values():
            for table in (entry, *entry.get("variants", {}).values()):
                for instance, selectors in table.get("selectors", {}).items():
                    if instance != GLOBAL_INSTANCE:
                        fields.update(selectors)
        return frozenset(fields)

    def plan_for(
        self,
        names: Iterable[str],
        config: Mapping[str, object] | None = None,
        accelerator: str | None = None,
        inputs: Iterable[SelectorInput] = (),
    ) -> Plan:
        """Union of the requested built-in names under a configuration.

        Names containing a dot are external blueprints: they are listed in
        ``Plan.external`` and make the plan incomplete. Unknown built-in names
        raise :class:`CatalogError`. Global-keyed selectors resolve from
        ``config`` and module-keyed ones from ``inputs``; a selection the
        registries do not know makes the plan incomplete. With an
        ``accelerator`` (``cpu`` or ``cuda``) backends resolve to their extras;
        without one they stay listed in ``Plan.backends`` unresolved.
        """
        values = {**self.defaults, **(config or {})}
        merged: dict[str, Any] = {"extras": {}, **{key: set() for key in MERGED_KEYS}}
        selectors: dict[str, dict[str, str]] = {}
        external: list[str] = []
        incomplete: list[str] = []
        for name in names:
            if "." in name:
                external.append(name)
                continue
            entry = self.entry(name)
            if entry is None:
                raise CatalogError(f"the blueprint catalog has no entry for {name!r}")
            _merge(merged, selectors, entry)
            for scenario, delta in entry.get("variants", {}).items():
                if holds(self._data["scenarios"][scenario], values):
                    _merge(merged, selectors, delta)
        for rule in self._data.get("rules", {}).values():
            if holds(rule["when"], values):
                _merge(merged, selectors, rule)
        for field_name, family in sorted(selectors.get(GLOBAL_INSTANCE, {}).items()):
            value = values.get(field_name)
            if value:
                self._select(
                    merged, family, str(value).lower(), f"{field_name}={value!r}", incomplete
                )
        for item in inputs:
            self._select_input(merged, selectors, item, incomplete)
        for name in external:
            incomplete.append(
                f"external blueprint {name!r} publishes no dependency metadata (entry point "
                f"group {EXTERNAL_CATALOG_GROUP!r}); its requirements are unknown"
            )
        extras = {n: frozenset(c) for n, c in merged["extras"].items()}
        backends: dict[str, str] = {}
        for backend in sorted(merged["backends"]):
            backends[backend] = accelerator or ""
            if accelerator is not None:
                for extra in self._data["backends"].get(backend, {}).get(accelerator, []):
                    extras.setdefault(extra, frozenset())
                    extras[extra] = extras[extra] | {f"backend:{backend}"}
        return Plan(
            extras=frozenset(extras),
            reasons=extras,
            backends=backends,
            native=frozenset(merged["native"]),
            system=frozenset(merged["system"]),
            tools=frozenset(merged["tools"]),
            external=tuple(external),
            incomplete=tuple(incomplete),
        )

    def _select(
        self, merged: dict[str, Any], family: str, name: str, source: str, incomplete: list[str]
    ) -> None:
        table = self._registries.get(family, {})
        entry = table.get(name)
        if entry is None:
            known = ", ".join(sorted(table)) or "none"
            incomplete.append(
                f"{source} selects a {family} the planner does not know: {name!r} (known: {known})"
            )
            return
        _merge(merged, {}, entry)

    def _select_input(
        self,
        merged: dict[str, Any],
        selectors: Mapping[str, Mapping[str, str]],
        item: SelectorInput,
        incomplete: list[str],
    ) -> None:
        targets = sorted(
            instance
            for instance, table in selectors.items()
            if instance != GLOBAL_INSTANCE and item.field in table
        )
        if item.instance is not None:
            if item.instance not in targets:
                known = ", ".join(targets) or "none"
                incomplete.append(
                    f"{item.source} addresses a module instance the planner does not know "
                    f"(instances with a {item.field!r} selector: {known})"
                )
                return
            instance = item.instance
        elif len(targets) == 1:
            instance = targets[0]
        elif not targets:
            incomplete.append(
                f"{item.source} names a field no requested blueprint selects implementations with"
            )
            return
        else:
            incomplete.append(
                f"{item.source} is ambiguous ({', '.join(targets)}); use --<instance>.{item.field}"
            )
            return
        family = selectors[instance][item.field]
        try:
            names = identifiers(family, item.value)
        except ValueError as error:
            incomplete.append(f"{item.source}: {error}")
            return
        for name in names:
            self._select(merged, family, name, item.source, incomplete)


def _merge(
    target: dict[str, Any], selectors: dict[str, dict[str, str]], entry: Mapping[str, Any]
) -> None:
    for extra, files in entry.get("extras", {}).items():
        target["extras"].setdefault(extra, set()).update(files)
    for key in MERGED_KEYS:
        target[key].update(entry.get(key, []))
    for instance, table in entry.get("selectors", {}).items():
        selectors.setdefault(instance, {}).update(table)


@lru_cache(maxsize=1)
def default_catalog() -> Catalog:
    return Catalog.load()


def catalog_names() -> frozenset[str]:
    """Built-in blueprint and module names the shipped catalog can plan for."""
    return default_catalog().names
