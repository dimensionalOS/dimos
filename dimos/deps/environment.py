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

"""Does an installed environment satisfy a plan?

The direct requirements of ``dimos[<extras>]`` come from the installed
distribution metadata (``Requires-Dist``), self-referencing extras such as
``dimos[base,mapping]; extra == "unitree"`` are expanded to a fixpoint and
markers are evaluated for the host. Transitive dependencies are the
installer's responsibility and are not re-checked here.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import asdict, dataclass, field
import glob
import importlib
import importlib.metadata
from pathlib import Path
import shutil
import sys
from typing import Any, Literal, cast

from packaging.markers import default_environment
from packaging.requirements import Requirement
from packaging.utils import canonicalize_name

from dimos.constants import CACHE_DIR
from dimos.deps.catalog import Plan
from dimos.deps.import_map import IN_TREE_NATIVE

PROJECT_NAME = "dimos"
EXCLUSIVE_PROVIDERS: tuple[tuple[str, str], ...] = (
    ("onnxruntime", "onnxruntime-gpu"),
    ("opencv-python", "opencv-contrib-python"),
    ("opencv-python-headless", "opencv-contrib-python"),
    ("opencv-python", "opencv-python-headless"),
)
CHEAP_CHECKS: tuple[str, ...] = ("packages", "providers", "native", "system", "tools")
HEAVY_CHECKS: tuple[str, ...] = ("backends", "blueprints")
ALL_CHECKS: tuple[str, ...] = CHEAP_CHECKS + HEAVY_CHECKS
PREREQUISITE_LABELS = {
    "native": "Native module",
    "system": "Host module",
    "tools": "Tool",
    "backends": "Backend",
    "blueprints": "Blueprint",
}
Status = Literal["satisfied", "missing", "unchecked"]


@dataclass(frozen=True)
class RequirementIssue:
    requirement: str
    via_extra: str | None
    installed: str | None
    reason: Literal["missing", "mismatch"]

    def __str__(self) -> str:
        via = f" (extra {self.via_extra})" if self.via_extra else ""
        if self.reason == "missing":
            return f"{self.requirement}{via}: not installed"
        return f"{self.requirement}{via}: installed {self.installed}"


@dataclass(frozen=True)
class Outcome:
    """What a check established about one prerequisite."""

    status: Status
    detail: str | None = None


@dataclass
class EnvironmentReport:
    python: str
    prefix: str
    dimos_version: str | None
    checks: list[str] = field(default_factory=list)
    missing: list[RequirementIssue] = field(default_factory=list)
    mismatched: list[RequirementIssue] = field(default_factory=list)
    excluded: list[str] = field(default_factory=list)
    """Requirements skipped on this host because of their environment marker."""
    conflicts: list[tuple[str, str]] = field(default_factory=list)
    native: dict[str, Outcome] = field(default_factory=dict)
    system: dict[str, Outcome] = field(default_factory=dict)
    tools: dict[str, Outcome] = field(default_factory=dict)
    backends: dict[str, Outcome] = field(default_factory=dict)
    blueprints: dict[str, Outcome] = field(default_factory=dict)

    @property
    def satisfied_for_launch(self) -> bool:
        """Packages only: what selecting an environment needs to know."""
        return self.dimos_version is not None and not self.missing and not self.mismatched

    @property
    def prerequisites(self) -> dict[str, dict[str, Outcome]]:
        """Every prerequisite outcome by kind, in report order."""
        return {kind: getattr(self, kind) for kind in PREREQUISITE_LABELS}

    def _with_status(self, status: Status) -> list[tuple[str, str, Outcome]]:
        return [
            (kind, name, outcome)
            for kind, outcomes in self.prerequisites.items()
            for name, outcome in outcomes.items()
            if outcome.status == status
        ]

    @property
    def missing_prerequisites(self) -> list[tuple[str, str, Outcome]]:
        return self._with_status("missing")

    @property
    def unchecked(self) -> list[tuple[str, str, Outcome]]:
        return self._with_status("unchecked")

    @property
    def ok(self) -> bool:
        """Packages are satisfied and no checked prerequisite is missing.

        Everything in a plan is required, so a missing prerequisite fails;
        unchecked ones and package conflicts are warnings.
        """
        return self.satisfied_for_launch and not self.missing_prerequisites

    def to_json(self) -> dict[str, Any]:
        data = asdict(self)
        data["conflicts"] = [list(pair) for pair in self.conflicts]
        return data

    @classmethod
    def from_json(cls, data: Mapping[str, Any]) -> EnvironmentReport:
        outcomes = {
            kind: {name: Outcome(**entry) for name, entry in data.get(kind, {}).items()}
            for kind in PREREQUISITE_LABELS
        }
        return cls(
            python=data["python"],
            prefix=data["prefix"],
            dimos_version=data.get("dimos_version"),
            checks=list(data.get("checks", [])),
            missing=[RequirementIssue(**issue) for issue in data.get("missing", [])],
            mismatched=[RequirementIssue(**issue) for issue in data.get("mismatched", [])],
            excluded=list(data.get("excluded", [])),
            conflicts=[(a, b) for a, b in data.get("conflicts", [])],
            **outcomes,
        )


def display_requirement(requirement: Requirement) -> str:
    """``name[extras]specifier`` without the marker, as the report shows it."""
    extras = f"[{','.join(sorted(requirement.extras))}]" if requirement.extras else ""
    return f"{requirement.name}{extras}{requirement.specifier}"


def installed_versions() -> dict[str, str]:
    """Canonical distribution name -> installed version (first found wins)."""
    versions: dict[str, str] = {}
    for distribution in importlib.metadata.distributions():
        name = distribution.metadata.get("Name")
        if name:
            versions.setdefault(canonicalize_name(name), distribution.version)
    return versions


def requires_of(distribution: str) -> list[str] | None:
    try:
        return importlib.metadata.requires(distribution) or []
    except importlib.metadata.PackageNotFoundError:
        return None


def expand_extras(
    requires: Iterable[str], extras: Iterable[str], environment: Mapping[str, str]
) -> frozenset[str]:
    """Selected extras plus every extra they include through self-references."""
    selected = {canonicalize_name(extra) for extra in extras}
    references = [
        requirement
        for requirement in map(Requirement, requires)
        if canonicalize_name(requirement.name) == PROJECT_NAME
    ]
    changed = True
    while changed:
        changed = False
        for requirement in references:
            for extra in list(selected):
                if requirement.marker is None or requirement.marker.evaluate(
                    {**environment, "extra": extra}
                ):
                    included = {canonicalize_name(e) for e in requirement.extras}
                    if not included <= selected:
                        selected |= included
                        changed = True
    return frozenset(selected)


def direct_requirements(
    requires: Iterable[str], extras: Iterable[str], environment: Mapping[str, str]
) -> tuple[list[tuple[Requirement, str | None]], list[str]]:
    """Requirements active on this host for core plus the extras, and excluded ones."""
    requires = list(requires)
    selected = expand_extras(requires, extras, environment)
    active: list[tuple[Requirement, str | None]] = []
    excluded: list[str] = []
    seen: set[str] = set()
    for requirement in map(Requirement, requires):
        if canonicalize_name(requirement.name) == PROJECT_NAME:
            continue
        via: str | None = None
        matched = False
        if requirement.marker is None:
            matched = True
        else:
            for extra in ("", *sorted(selected)):
                if requirement.marker.evaluate({**environment, "extra": extra}):
                    matched, via = True, extra or None
                    break
        key = str(requirement)
        if matched:
            if key not in seen:
                seen.add(key)
                active.append((requirement, via))
        elif requirement.marker is not None and _mentions_selected_extra(
            str(requirement.marker), selected
        ):
            excluded.append(f"{display_requirement(requirement)} ({requirement.marker})")
    return active, excluded


def _mentions_selected_extra(marker: str, selected: frozenset[str]) -> bool:
    """Platform alternatives of core are expected; only selected extras are reported."""
    return any(f'extra == "{extra}"' in marker for extra in selected)


def check_packages(
    requirements: Iterable[tuple[Requirement, str | None]],
    versions: Mapping[str, str],
    *,
    requires_of: Callable[[str], list[str] | None],
    environment: Mapping[str, str],
) -> tuple[list[RequirementIssue], list[RequirementIssue]]:
    missing: list[RequirementIssue] = []
    mismatched: list[RequirementIssue] = []
    for requirement, via in requirements:
        name = canonicalize_name(requirement.name)
        installed = versions.get(name)
        if installed is None:
            missing.append(RequirementIssue(display_requirement(requirement), via, None, "missing"))
            continue
        if requirement.specifier and not requirement.specifier.contains(
            installed, prereleases=True
        ):
            mismatched.append(
                RequirementIssue(display_requirement(requirement), via, installed, "mismatch")
            )
            continue
        for extra in requirement.extras:
            nested = requires_of(name)
            if nested is None:
                continue
            for entry in map(Requirement, nested):
                if entry.marker is None or not entry.marker.evaluate(
                    {**environment, "extra": extra}
                ):
                    continue
                if "extra ==" not in str(entry.marker):
                    continue
                nested_name = canonicalize_name(entry.name)
                if nested_name not in versions:
                    missing.append(
                        RequirementIssue(
                            display_requirement(entry), f"{name}[{extra}]", None, "missing"
                        )
                    )
    return missing, mismatched


def find_conflicts(versions: Mapping[str, str]) -> list[tuple[str, str]]:
    return [pair for pair in EXCLUSIVE_PROVIDERS if pair[0] in versions and pair[1] in versions]


def _import_outcome(name: str) -> Outcome:
    try:
        importlib.import_module(name)
    except Exception as error:
        return Outcome("missing", f"{type(error).__name__}: {error}")
    return Outcome("satisfied")


def check_native(names: Iterable[str]) -> dict[str, Outcome]:
    """Import in-tree native modules; executables built separately stay unchecked."""
    results: dict[str, Outcome] = {}
    for name in sorted(names):
        if name in IN_TREE_NATIVE:
            results[name] = _import_outcome(name)
        else:
            results[name] = Outcome(
                "unchecked", "native executable built separately; not checked here"
            )
    return results


def check_system(names: Iterable[str]) -> dict[str, Outcome]:
    """Import host-provided modules such as ``rclpy``."""
    return {name: _import_outcome(name) for name in sorted(names)}


def check_tools(tools: Iterable[str]) -> dict[str, Outcome]:
    results: dict[str, Outcome] = {}
    for tool in sorted(tools):
        found = shutil.which(tool)
        if found is None and tool == "deno":
            cached = sorted(glob.glob(str(CACHE_DIR / "deno" / "*" / "deno")))
            found = cached[-1] if cached else None
        results[tool] = Outcome("satisfied", found) if found else Outcome("missing", "not on PATH")
    return results


def check_environment(
    plan: Plan,
    *,
    checks: Iterable[str] = CHEAP_CHECKS,
    environment: Mapping[str, str] | None = None,
    requires: Iterable[str] | None = None,
    versions: Mapping[str, str] | None = None,
) -> EnvironmentReport:
    """Cheap checks of the running interpreter against a plan."""
    checks = list(checks)
    environment = dict(
        environment if environment is not None else cast("Mapping[str, str]", default_environment())
    )
    versions = versions if versions is not None else installed_versions()
    report = EnvironmentReport(
        python=sys.version.split()[0],
        prefix=sys.prefix,
        dimos_version=versions.get(PROJECT_NAME),
        checks=checks,
    )
    if "packages" in checks:
        declared = list(requires) if requires is not None else requires_of(PROJECT_NAME)
        if declared is None:
            report.missing.append(RequirementIssue(PROJECT_NAME, None, None, "missing"))
        else:
            active, excluded = direct_requirements(declared, plan.extras, environment)
            report.missing, report.mismatched = check_packages(
                active, versions, requires_of=requires_of, environment=environment
            )
            report.excluded = excluded
    if "providers" in checks:
        report.conflicts = find_conflicts(versions)
    if "native" in checks:
        report.native = check_native(plan.native)
    if "system" in checks:
        report.system = check_system(plan.system)
    if "tools" in checks:
        report.tools = check_tools(plan.tools)
    return report


def format_report(report: EnvironmentReport, plan: Plan, *, checkout: bool) -> str:
    """Human-readable report with the fix recipe for the environment kind."""
    lines: list[str] = []
    location = f"{report.prefix} (Python {report.python}"
    location += (
        f", dimos {report.dimos_version})" if report.dimos_version else ", dimos not installed)"
    )
    lines.append(f"Environment: {location}")
    if report.missing or report.mismatched:
        lines.append("Missing or mismatched requirements:")
        lines.extend(f"  - {issue}" for issue in (*report.missing, *report.mismatched))
        extras = sorted(plan.extras)
        if extras:
            if checkout:
                flags = " ".join(f"--extra {extra}" for extra in extras)
                lines.append(f"Install into this checkout:  uv sync {flags} --inexact")
            else:
                lines.append(
                    f"Install into this environment:  pip install 'dimos[{','.join(extras)}]'"
                )
        lines.append("Or let dimos manage a runtime environment:  dimos prepare <blueprint>")
        if checkout:
            lines.append(
                "If the package is declared in pyproject.toml, uv.lock may be stale: run `uv lock`."
            )
    elif "packages" in report.checks:
        lines.append("Requirements: satisfied")
    for pair in report.conflicts:
        lines.append(
            f"Warning: both {pair[0]} and {pair[1]} are installed; they share files and "
            "only one should be present"
        )
    for name in report.excluded:
        lines.append(f"Warning: {name} is excluded on this platform by its marker")
    for kind, outcomes in report.prerequisites.items():
        for name, outcome in outcomes.items():
            detail = f" ({outcome.detail})" if outcome.detail else ""
            lines.append(f"{PREREQUISITE_LABELS[kind]} {name}: {outcome.status}{detail}")
    return "\n".join(lines)


def is_checkout(project_root: Path) -> bool:
    return (project_root / "pyproject.toml").is_file() and (project_root / "uv.lock").is_file()
