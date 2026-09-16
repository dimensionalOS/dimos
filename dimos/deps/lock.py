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

"""What core and each extra declare directly, plus the lock's graph for diagnostics.

Ownership of an import is decided by direct declarations in ``pyproject.toml``:
a distribution dimOS imports must be listed by core or by the extra that
covers the importing file, even when another package would install it anyway.
``uv.lock`` is optional and only explains how an undeclared distribution is
reachable today, so a finding can say what to declare. Markers are ignored
on both sides; the installed environment is checked against markers elsewhere.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

from packaging.requirements import Requirement
from packaging.utils import canonicalize_name

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

PROJECT_NAME = "dimos"
CORE = "core"


@dataclass(frozen=True)
class ExtraInfo:
    name: str
    own_direct: frozenset[str]
    """Direct requirements declared on the extra itself (self-references excluded)."""
    own_edges: frozenset[str]
    """The same requirements as graph edges, keeping ``name[extra]`` for diagnostics."""
    includes: frozenset[str]
    """Other extras this extra pulls in through ``dimos[...]`` self-references."""

    @property
    def is_pure_aggregate(self) -> bool:
        return not self.own_direct and bool(self.includes)


class LockIndex:
    """Direct declarations of the project, and its locked graph when a lock is given."""

    def __init__(
        self,
        packages: Mapping[str, set[str]],
        optional: Mapping[str, Mapping[str, set[str]]],
        core_direct: Iterable[str],
        core_edges: Iterable[str],
        extras: Mapping[str, ExtraInfo],
    ) -> None:
        self._packages = {name: frozenset(deps) for name, deps in packages.items()}
        self._optional = {
            name: {extra: frozenset(deps) for extra, deps in table.items()}
            for name, table in optional.items()
        }
        self.core_direct = frozenset(core_direct)
        self._core_edges = frozenset(core_edges)
        self.extras = dict(extras)

    @property
    def packages(self) -> frozenset[str]:
        return frozenset(self._packages)

    @classmethod
    def load(cls, project_root: Path) -> LockIndex:
        """From ``pyproject.toml``, with ``uv.lock`` when the directory has one."""
        pyproject = tomllib.loads((project_root / "pyproject.toml").read_text(encoding="utf-8"))
        lock_path = project_root / "uv.lock"
        lock = tomllib.loads(lock_path.read_text(encoding="utf-8")) if lock_path.is_file() else None
        return cls.from_documents(pyproject, lock)

    @classmethod
    def from_documents(
        cls, pyproject: Mapping[str, Any], lock: Mapping[str, Any] | None = None
    ) -> LockIndex:
        packages: dict[str, set[str]] = {}
        optional: dict[str, dict[str, set[str]]] = {}
        for package in (lock or {}).get("package", []):
            name = canonicalize_name(package["name"])
            deps = packages.setdefault(name, set())
            deps.update(_edge_names(package.get("dependencies", [])))
            table = optional.setdefault(name, {})
            for extra, edges in package.get("optional-dependencies", {}).items():
                table.setdefault(canonicalize_name(extra), set()).update(_edge_names(edges))
        project = pyproject["project"]
        core_direct, core_edges, _includes = _declared(project.get("dependencies", []))
        extras: dict[str, ExtraInfo] = {}
        for extra, entries in project.get("optional-dependencies", {}).items():
            extra = canonicalize_name(extra)
            own_direct, own_edges, includes = _declared(entries)
            extras[extra] = ExtraInfo(
                name=extra,
                own_direct=frozenset(own_direct),
                own_edges=frozenset(own_edges),
                includes=frozenset(includes),
            )
        return cls(packages, optional, core_direct, core_edges, extras)

    def owner_of(self, distribution: str) -> str | None:
        """``core`` or the extra that declares ``distribution`` directly; ``None`` otherwise."""
        distribution = canonicalize_name(distribution)
        if distribution in self.core_direct:
            return CORE
        providers = self.providers(distribution)
        return providers[0] if providers else None

    def providers(self, distribution: str) -> tuple[str, ...]:
        """Extras that declare the distribution directly."""
        distribution = canonicalize_name(distribution)
        return tuple(
            sorted(name for name, info in self.extras.items() if distribution in info.own_direct)
        )

    def chain(
        self, distribution: str, prefer: Iterable[str] = ()
    ) -> tuple[str, tuple[str, ...]] | None:
        """How a distribution nobody declares is installed today: ``(owner, path)``.

        The owner is ``core`` or an extra whose direct requirements reach the
        distribution through the lock's graph; the path names each hop. Extras
        in ``prefer`` are tried first, so a hint can name the importing file's own.
        """
        target = canonicalize_name(distribution)
        preferred = [canonicalize_name(name) for name in prefer]
        roots: list[tuple[str, frozenset[str]]] = [
            (name, self.extras[name].own_edges) for name in preferred if name in self.extras
        ]
        roots.append((CORE, self._core_edges))
        roots += [
            (name, info.own_edges)
            for name, info in sorted(self.extras.items())
            if name not in preferred
        ]
        for owner, edges in roots:
            path = self._path(edges, target)
            if path is not None:
                return owner, path
        return None

    def _path(self, roots: Iterable[str], target: str) -> tuple[str, ...] | None:
        previous: dict[str, str | None] = {root: None for root in sorted(roots)}
        queue = deque(previous)
        while queue:
            name = queue.popleft()
            base, extra = _split_edge(name)
            if base == target:
                chain: list[str] = []
                current: str | None = name
                while current is not None:
                    chain.append(current)
                    current = previous[current]
                return tuple(reversed(chain))
            deps = (
                self._optional.get(base, {}).get(extra, frozenset())
                if extra is not None
                else self._packages.get(base, frozenset())
            )
            for dep in sorted(deps):
                if dep not in previous:
                    previous[dep] = name
                    queue.append(dep)
        return None


def _declared(entries: Iterable[str]) -> tuple[set[str], set[str], set[str]]:
    """Canonical names, graph edges and included extras of one requirement list."""
    names: set[str] = set()
    edges: set[str] = set()
    includes: set[str] = set()
    for entry in entries:
        requirement = Requirement(entry)
        name = canonicalize_name(requirement.name)
        if name == PROJECT_NAME:
            includes.update(canonicalize_name(extra) for extra in requirement.extras)
            continue
        names.add(name)
        if requirement.extras:
            edges.update(f"{name}[{canonicalize_name(extra)}]" for extra in requirement.extras)
        else:
            edges.add(name)
    return names, edges, includes


def _edge_names(edges: Iterable[Mapping[str, Any]]) -> set[str]:
    """Dependency edge targets; an edge with extras becomes ``name[extra]`` entries."""
    names: set[str] = set()
    for edge in edges:
        name = canonicalize_name(edge["name"])
        extras = edge.get("extra", [])
        if extras:
            names.update(f"{name}[{canonicalize_name(extra)}]" for extra in extras)
        else:
            names.add(name)
    return names


def _split_edge(name: str) -> tuple[str, str | None]:
    if name.endswith("]") and "[" in name:
        base, extra = name[:-1].split("[", 1)
        return base, extra
    return name, None
