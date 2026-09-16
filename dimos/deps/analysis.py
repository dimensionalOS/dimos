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

"""Import-closure analysis: from a blueprint's source file to its requirements.

The closure follows eager first-party imports (whose configuration predicate
holds), registry selections (``adapter_type="xarm"``) and the subprocesses a
file declares. Requirements come from the ``Requires(...)`` declarations of
the files in the closure, never from inference: moving an import from module
scope into a constructor changes nothing. The scanner audits every file's
third-party imports, eager and lazy, against its declarations, so an import a
file has not declared is a finding that fails catalog generation.
"""

from __future__ import annotations

import ast
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path

from dimos.deps.import_map import (
    BACKEND_DISTRIBUTIONS,
    IN_TREE_NATIVE,
    is_stdlib,
    provider_for,
    suggest,
)
from dimos.deps.imports import (
    Declaration,
    FileScan,
    ImportKind,
    ImportSite,
    resolve_internal,
    scan_file,
)
from dimos.deps.lock import LockIndex
from dimos.deps.predicates import holds
from dimos.deps.requires import Requires
from dimos.deps.rules import DEFAULTS, SCENARIOS, GlobalRule

MANIFEST_TABLES = {
    "ADAPTER_FACTORIES": "adapter",
    "TASK_FACTORIES": "task",
    "CONNECTION_FACTORIES": "connection",
    "SIM_MODULE_FACTORIES": "simulation",
}
GLOBAL_INSTANCE = "g"
"""Selector instance for fields keyed on global configuration (``g.<field>``)."""
Selectors = Mapping[str, Mapping[str, str]]


@dataclass(frozen=True)
class Requirements:
    extras: Mapping[str, frozenset[str]] = field(default_factory=dict)
    """Extra name -> files (module paths) that declare it."""
    backends: frozenset[str] = frozenset()
    native: frozenset[str] = frozenset()
    system: frozenset[str] = frozenset()
    tools: frozenset[str] = frozenset()
    selectors: Selectors = field(default_factory=dict)
    """Instance (``g`` or a module class name, lowercase) -> field -> registry family."""

    def union(self, other: Requirements) -> Requirements:
        extras = {name: set(files) for name, files in self.extras.items()}
        for name, files in other.extras.items():
            extras.setdefault(name, set()).update(files)
        selectors = {instance: dict(table) for instance, table in self.selectors.items()}
        for instance, table in other.selectors.items():
            selectors.setdefault(instance, {}).update(table)
        return Requirements(
            extras={name: frozenset(files) for name, files in extras.items()},
            backends=self.backends | other.backends,
            native=self.native | other.native,
            system=self.system | other.system,
            tools=self.tools | other.tools,
            selectors=selectors,
        )

    def added_over(self, base: Requirements) -> Requirements:
        """What this set requires beyond ``base``."""
        selectors: dict[str, dict[str, str]] = {}
        for instance, table in self.selectors.items():
            known = base.selectors.get(instance, {})
            added = {name: family for name, family in table.items() if known.get(name) != family}
            if added:
                selectors[instance] = added
        return Requirements(
            extras={n: c for n, c in self.extras.items() if n not in base.extras},
            backends=self.backends - base.backends,
            native=self.native - base.native,
            system=self.system - base.system,
            tools=self.tools - base.tools,
            selectors=selectors,
        )

    def is_empty(self) -> bool:
        return not (
            self.extras
            or self.backends
            or self.native
            or self.system
            or self.tools
            or self.selectors
        )

    def to_json(self) -> dict[str, object]:
        data: dict[str, object] = {}
        if self.extras:
            data["extras"] = {n: sorted(c) for n, c in sorted(self.extras.items())}
        for key in ("backends", "native", "system", "tools"):
            values: frozenset[str] = getattr(self, key)
            if values:
                data[key] = sorted(values)
        if self.selectors:
            data["selectors"] = {
                instance: dict(sorted(table.items()))
                for instance, table in sorted(self.selectors.items())
            }
        return data


@dataclass(frozen=True)
class Finding:
    path: str
    lineno: int
    import_name: str
    message: str
    hint: str

    def __str__(self) -> str:
        return f"dimos/{self.path}:{self.lineno}: `{self.import_name}`: {self.message}. {self.hint}"


@dataclass
class Closure:
    files: dict[Path, Path | None] = field(default_factory=dict)
    """File -> the file that imported it (``None`` for roots)."""
    reasons: dict[Path, str] = field(default_factory=dict)
    """Why a file entered the closure when not through a plain import."""
    native: set[str] = field(default_factory=set)

    def chain(self, target: Path) -> list[Path]:
        chain: list[Path] = []
        current: Path | None = target
        while current is not None:
            chain.append(current)
            current = self.files[current]
        return list(reversed(chain))


@dataclass(frozen=True)
class RootResult:
    name: str
    base: Requirements
    variants: Mapping[str, Requirements]
    findings: tuple[Finding, ...]
    warnings: tuple[str, ...]

    def to_json(self) -> dict[str, object]:
        data = self.base.to_json()
        variants = {n: v.to_json() for n, v in sorted(self.variants.items()) if not v.is_empty()}
        if variants:
            data["variants"] = variants
        return data


def target_file(project_root: Path, target: str) -> Path:
    """Source file of a registry target (``module:attr`` or ``module.Class``)."""
    module = target.split(":", 1)[0] if ":" in target else target.rsplit(".", 1)[0]
    return module_file(project_root, module)


def module_file(project_root: Path, module: str) -> Path:
    return project_root.joinpath(*module.split(".")).with_suffix(".py")


def merge_declarations(declarations: Iterable[Requires]) -> Requires:
    """One ``Requires`` with the union of several, keeping declaration order."""
    merged: dict[str, list[str]] = {
        name: [] for name in Requires.__dataclass_fields__ if name != "selectors"
    }
    selectors: dict[str, str] = {}
    for declaration in declarations:
        for name, values in merged.items():
            values.extend(v for v in getattr(declaration, name) if v not in values)
        selectors.update(declaration.selectors)
    return Requires(**{name: tuple(values) for name, values in merged.items()}, selectors=selectors)


class Analyzer:
    def __init__(
        self, project_root: Path, lock: LockIndex | None, *, package: str = "dimos"
    ) -> None:
        self.project_root = project_root.resolve()
        self.package = package
        self.package_dir = self.project_root / package
        self.lock = lock
        self._scans: dict[Path, FileScan] = {}
        self._manifests: dict[tuple[str, str], Path] | None = None
        self._manifest_conflicts: list[tuple[str, str, Path, Path]] = []
        self._resolved: dict[tuple[str, tuple[str, ...]], tuple[Path, ...]] = {}
        self._checked: dict[Path, tuple[Requirements, list[Finding], list[str]]] = {}

    def scan(self, path: Path) -> FileScan:
        path = path.resolve()
        scan = self._scans.get(path)
        if scan is None:
            scan = scan_file(path, project_root=self.project_root, package=self.package)
            self._scans[path] = scan
        return scan

    def relative(self, path: Path) -> str:
        return path.resolve().relative_to(self.package_dir).as_posix()

    def manifests(self) -> Mapping[tuple[str, str], Path]:
        """``(family, name)`` -> implementation source file, from every ``_registry.py``."""
        if self._manifests is None:
            found: dict[tuple[str, str], Path] = {}
            for manifest in sorted(self.package_dir.rglob("_registry.py")):
                tree = ast.parse(manifest.read_text(encoding="utf-8"))
                for node in tree.body:
                    if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                        continue
                    target = node.targets[0]
                    if not isinstance(target, ast.Name) or target.id not in MANIFEST_TABLES:
                        continue
                    family = MANIFEST_TABLES[target.id]
                    for name, factory in ast.literal_eval(node.value).items():
                        key = (family, str(name).lower())
                        file = target_file(self.project_root, str(factory))
                        if key in found and found[key] != file:
                            self._manifest_conflicts.append((family, key[1], found[key], file))
                        found[key] = file
            self._manifests = found
        return self._manifests

    def manifest_conflicts(self) -> list[tuple[str, str, Path, Path]]:
        """Names two manifests define with different implementations."""
        self.manifests()
        return list(self._manifest_conflicts)

    def families(self) -> frozenset[str]:
        return frozenset(family for family, _name in self.manifests())

    def closure(self, roots: Iterable[Path], config: Mapping[str, object]) -> Closure:
        closure = Closure()
        stack: list[tuple[Path, Path | None, str | None]] = [
            (root.resolve(), None, None) for root in roots
        ]
        while stack:
            path, parent, reason = stack.pop()
            if path in closure.files or not path.is_file():
                continue
            closure.files[path] = parent
            if reason is not None:
                closure.reasons[path] = reason
            scan = self.scan(path)
            for site in scan.imports:
                if site.kind is not ImportKind.EAGER or not holds(site.condition, config):
                    continue
                if site.top_level != self.package:
                    continue
                executed = self._resolve(site)
                for child in executed:
                    stack.append((child, path, None))
                if not executed:
                    closure.native.update(self._native_modules(site))
            for ref in sorted(scan.manifest_refs, key=lambda r: (r.family, r.name, r.lineno)):
                if not holds(ref.condition, config):
                    continue
                factory = self.manifests().get((ref.family, ref.name.lower()))
                if factory is not None:
                    stack.append((factory, path, f"{ref.family} registry manifest {ref.name!r}"))
            for declaration in scan.declarations:
                for module in declaration.requires.subprocesses:
                    stack.append(
                        (module_file(self.project_root, module), path, f"subprocess {module!r}")
                    )
        return closure

    def _resolve(self, site: ImportSite) -> tuple[Path, ...]:
        key = (site.module, site.names)
        executed = self._resolved.get(key)
        if executed is None:
            executed = resolve_internal(site, project_root=self.project_root, package=self.package)
            self._resolved[key] = executed
        return executed

    def _native_modules(self, site: ImportSite) -> set[str]:
        names = {site.module, *(f"{site.module}.{name}" for name in site.names)}
        return {name for name in names if name in IN_TREE_NATIVE}

    def requirements(self, closure: Closure) -> tuple[Requirements, list[Finding], list[str]]:
        """Requirements declared by the files of a closure, with the audit's findings."""
        total = Requirements(native=frozenset(closure.native))
        findings: list[Finding] = []
        warnings: list[str] = []
        for path in closure.files:
            checked = self._checked.get(path)
            if checked is None:
                checked = self.check_file(self.scan(path))
                self._checked[path] = checked
            requirements, file_findings, file_warnings = checked
            total = total.union(requirements)
            findings.extend(file_findings)
            warnings.extend(file_warnings)
        return total, findings, warnings

    def check_file(self, scan: FileScan) -> tuple[Requirements, list[Finding], list[str]]:
        """A file's declared requirements, after auditing its imports against them."""
        rel = self.relative(scan.path)
        reason = scan.module.removeprefix(f"{self.package}.")
        declared = merge_declarations(d.requires for d in scan.declarations)
        findings = [
            Finding(rel, lineno, "Requires", message, "see dimos/deps/requires.py")
            for lineno, message in scan.errors
        ]
        warnings: list[str] = []
        extras = list(declared.extras)
        if self.lock is not None:
            for extra in extras:
                info = self.lock.extras.get(extra)
                if info is None:
                    findings.append(
                        Finding(
                            rel,
                            0,
                            "Requires",
                            f"unknown extra {extra!r}",
                            "declare an extra from pyproject.toml",
                        )
                    )
                elif info.is_pure_aggregate:
                    findings.append(
                        Finding(
                            rel,
                            0,
                            "Requires",
                            f"extra {extra!r} is an aggregate",
                            f"declare the extra that provides the import: {', '.join(sorted(info.includes))}",
                        )
                    )
        lazy = {site.top_level for site in scan.imports if site.kind is ImportKind.LAZY}
        for name in declared.defers:
            if name not in lazy:
                findings.append(
                    Finding(
                        rel,
                        0,
                        name,
                        f"defers {name!r} but never imports it lazily",
                        "remove it from defers",
                    )
                )
        for site in scan.imports:
            if site.kind in (ImportKind.TYPE_ONLY, ImportKind.MAIN_ONLY):
                continue
            top = site.top_level
            if not top or top == self.package or is_stdlib(top):
                continue
            exempt = site.kind is ImportKind.OPTIONAL or (
                site.kind is ImportKind.LAZY and top in declared.defers
            )
            provider = provider_for(top)
            if provider is None:
                if not exempt:
                    findings.append(
                        Finding(
                            rel,
                            site.lineno,
                            top,
                            "unknown import name",
                            f"{suggest(top)}; add it to dimos/deps/import_map.py",
                        )
                    )
            elif provider.kind == "native":
                continue
            elif provider.kind == "system":
                if top not in declared.system and not exempt:
                    findings.append(
                        Finding(
                            rel,
                            site.lineno,
                            top,
                            "host-provided module not declared",
                            f"declare Requires(system=({top!r},)) in this file",
                        )
                    )
            elif provider.kind == "dev":
                if site.kind is ImportKind.EAGER:
                    findings.append(
                        Finding(
                            rel,
                            site.lineno,
                            top,
                            "development-only import in a production file",
                            "move the import into a test file or exclude the file",
                        )
                    )
                else:
                    warnings.append(
                        f"dimos/{rel}:{site.lineno}: `{top}` is a development-only import"
                    )
            elif provider.kind == "undeclared":
                warnings.append(f"dimos/{rel}:{site.lineno}: `{top}` is not provided by any extra")
            elif self.lock is not None and not exempt:
                self._check_distribution(rel, site, provider.names, declared, findings)
        requirements = Requirements(
            extras={extra: frozenset({reason}) for extra in extras},
            backends=frozenset(declared.backends),
            native=frozenset(declared.native) | frozenset(scan.native_executables),
            system=frozenset(declared.system),
            tools=frozenset(declared.tools),
            selectors=self._selectors(rel, scan.declarations, findings),
        )
        return requirements, findings, warnings

    def _check_distribution(
        self,
        rel: str,
        site: ImportSite,
        names: tuple[str, ...],
        declared: Requires,
        findings: list[Finding],
    ) -> None:
        lock = self.lock
        assert lock is not None
        if any(name in lock.core_direct for name in names):
            return
        backend = BACKEND_DISTRIBUTIONS.get(names[0])
        if backend is not None:
            if backend not in declared.backends:
                findings.append(
                    Finding(
                        rel,
                        site.lineno,
                        site.top_level,
                        f"backend {backend!r} not declared",
                        f"declare Requires(backends=({backend!r},)) in this file",
                    )
                )
            return
        for extra in declared.extras:
            info = lock.extras.get(extra)
            if info is not None and any(name in info.own_direct for name in names):
                return
        providers = sorted({provider for name in names for provider in lock.providers(name)})
        reachable = lock.chain(names[0], prefer=declared.extras)
        if providers:
            hint = (
                f"declare Requires(extras=({providers[0]!r},)) in this file"
                + (f" (providers: {', '.join(providers)})" if len(providers) > 1 else "")
                + f", add {site.top_level!r} to defers when the caller declares it, or move the import"
            )
        elif reachable is not None:
            owner, chain = reachable
            where = (
                "[project].dependencies"
                if owner == "core"
                else f"[project.optional-dependencies] {owner}"
            )
            hint = f"only reachable through {owner}: {' -> '.join(chain)}; declare it directly in {where} and run uv lock"
        else:
            hint = "no extra declares it; add it to an extra in pyproject.toml"
        declared_text = ", ".join(declared.extras) or "none"
        findings.append(
            Finding(
                rel,
                site.lineno,
                site.top_level,
                f"distribution {names[0]!r} is not provided by core or by this file's declarations (extras: {declared_text})",
                hint,
            )
        )

    def _selectors(
        self, rel: str, declarations: Iterable[Declaration], findings: list[Finding]
    ) -> dict[str, dict[str, str]]:
        families = self.families()
        selectors: dict[str, dict[str, str]] = {}
        for declaration in declarations:
            for key, family in declaration.requires.selectors.items():
                if family not in families:
                    findings.append(
                        Finding(
                            rel,
                            declaration.lineno,
                            key,
                            f"no registry manifest defines family {family!r}",
                            f"known families: {', '.join(sorted(families))}",
                        )
                    )
                elif key.startswith(f"{GLOBAL_INSTANCE}."):
                    selectors.setdefault(GLOBAL_INSTANCE, {})[key.split(".", 1)[1]] = family
                elif not declaration.owner:
                    findings.append(
                        Finding(
                            rel,
                            declaration.lineno,
                            key,
                            f"selector {key!r} must be declared on the module class that owns the field",
                            "move it into the class body as `requires = Requires(selectors=...)`",
                        )
                    )
                else:
                    selectors.setdefault(declaration.owner.lower(), {})[key] = family
        return selectors

    def analyze_root(self, name: str, target: str) -> RootResult:
        root = target_file(self.project_root, target)
        base_closure = self.closure([root], DEFAULTS)
        base, findings, warnings = self.requirements(base_closure)
        variants: dict[str, Requirements] = {}
        for scenario in SCENARIOS.values():
            config = {**DEFAULTS, **scenario.overrides}
            requirements, scenario_findings, scenario_warnings = self.requirements(
                self.closure([root], config)
            )
            findings.extend(f for f in scenario_findings if f not in findings)
            warnings.extend(w for w in scenario_warnings if w not in warnings)
            delta = requirements.added_over(base)
            if not delta.is_empty():
                variants[scenario.name] = delta
        return RootResult(
            name, base, variants, tuple(dict.fromkeys(findings)), tuple(dict.fromkeys(warnings))
        )

    def analyze_files(self, files: Iterable[Path]) -> tuple[Requirements, list[Finding], list[str]]:
        """Requirements of the closure of ``files`` under the default configuration."""
        return self.requirements(self.closure(files, DEFAULTS))

    def analyze_rule(self, rule: GlobalRule) -> tuple[Requirements, list[Finding], list[str]]:
        return self.analyze_files(self.package_dir / root for root in rule.roots)

    def why(
        self, roots: Iterable[Path], needle: str, config: Mapping[str, object]
    ) -> list[list[Path]]:
        """Import chains from the roots to files needing ``needle`` (an import, distribution or extra)."""
        closure = self.closure(roots, config)
        chains: list[list[Path]] = []
        for path in closure.files:
            if self._file_needs(self.scan(path), needle):
                chains.append(closure.chain(path))
        chains.sort(key=len)
        return chains

    def _file_needs(self, scan: FileScan, needle: str) -> bool:
        declared = merge_declarations(d.requires for d in scan.declarations)
        if needle in declared.extras or needle in declared.backends or needle in declared.tools:
            return True
        for site in scan.imports:
            if site.kind in (ImportKind.TYPE_ONLY, ImportKind.MAIN_ONLY):
                continue
            top = site.top_level
            if top == needle:
                return True
            provider = provider_for(top)
            if provider is not None and provider.kind == "dist" and needle in provider.names:
                return True
        return False
