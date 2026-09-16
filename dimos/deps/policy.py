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

"""The repository's uv resolver policy, and the project that applies it to a release.

``[tool.uv]`` overrides, constraints, sources and indexes are not part of the
distribution metadata a wheel carries, yet a managed environment for an
installed release must resolve with them (the pytorch index, the opencv
override). The wheel ships a copy of ``pyproject.toml`` so this module can
read the same policy in both a checkout and an installation.

It also ships ``constraints.txt``, the tested resolution of the repository
lock exported by uv with markers for every supported platform. A managed
environment for a release constrains its own resolution to those versions,
so it installs what the release was tested with or fails loudly.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from typing import Any

from packaging.requirements import Requirement
from packaging.utils import canonicalize_name

from dimos.deps.profiles import Profile

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

PROJECT_NAME = "dimos"
MANAGED_PROJECT_NAME = "dimos-managed-env"
SHIPPED_PROJECT_FILE = Path(__file__).with_name("_project") / "pyproject.toml"
SHIPPED_CONSTRAINTS_FILE = Path(__file__).with_name("constraints.txt")


@dataclass(frozen=True)
class UvPolicy:
    required_version: str | None
    override_dependencies: tuple[str, ...]
    constraint_dependencies: tuple[str, ...]
    sources: Mapping[str, Any]
    indexes: tuple[Mapping[str, Any], ...]


def find_project_file(project_root: Path) -> Path | None:
    """The checkout's ``pyproject.toml``, else the copy shipped inside the wheel."""
    candidate = project_root / "pyproject.toml"
    if candidate.is_file():
        try:
            document = tomllib.loads(candidate.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            document = {}
        if document.get("project", {}).get("name") == PROJECT_NAME:
            return candidate
    if SHIPPED_PROJECT_FILE.is_file():
        return SHIPPED_PROJECT_FILE
    return None


def load_uv_policy(path: Path) -> UvPolicy:
    document = tomllib.loads(path.read_text(encoding="utf-8"))
    uv = document.get("tool", {}).get("uv", {})
    return UvPolicy(
        required_version=uv.get("required-version"),
        override_dependencies=tuple(uv.get("override-dependencies", [])),
        constraint_dependencies=tuple(uv.get("constraint-dependencies", [])),
        sources=dict(uv.get("sources", {})),
        indexes=tuple(uv.get("index", [])),
    )


def load_constraints(path: Path) -> tuple[str, ...]:
    """Requirement lines of the tested resolution, markers included."""
    lines: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if " @ " in line or line.startswith("-"):
            raise ValueError(f"{path.name}: unsupported constraint line {line!r}")
        lines.append(line)
    return tuple(lines)


def packages_outside(lock_path: Path, constraints: Iterable[str]) -> list[str]:
    """Packages a managed project resolved that the tested set does not contain."""
    document = tomllib.loads(lock_path.read_text(encoding="utf-8"))
    allowed: set[str] = {canonicalize_name(Requirement(line).name) for line in constraints}
    allowed |= {PROJECT_NAME, MANAGED_PROJECT_NAME}
    resolved: set[str] = {
        canonicalize_name(package["name"]) for package in document.get("package", [])
    }
    return sorted(resolved - allowed)


def render_wheel_project(
    version: str,
    extras: Iterable[str],
    python: str,
    profile: Profile,
    policy: UvPolicy,
    constraints: Iterable[str] = (),
) -> str:
    """A virtual uv project that installs ``dimos[extras]==version`` under the policy.

    Sources only bind direct dependencies, so every git or URL source that is not
    already an override becomes an override too; that is how the repository's own
    lock pins ``torch`` from the pytorch index although torch is transitive.
    ``constraints`` are the tested versions; they only apply to packages the
    resolution needs, and their markers keep other platforms' entries inert.
    """
    overrides = list(policy.override_dependencies)
    overridden = {canonicalize_name(_requirement_name(entry)) for entry in overrides}
    for name, source in policy.sources.items():
        if isinstance(source, dict) and canonicalize_name(name) not in overridden:
            url = _source_url(source)
            if url is not None:
                overrides.append(f"{name} @ {url}")
    extras = sorted(extras)
    requirement = (
        f"{PROJECT_NAME}[{','.join(extras)}]=={version}" if extras else f"{PROJECT_NAME}=={version}"
    )
    lines = [
        "# Generated by dimos; do not edit. Delete the environment to change it.",
        "[project]",
        f"name = {_toml(MANAGED_PROJECT_NAME)}",
        'version = "0"',
        f"requires-python = {_toml(f'=={python}.*')}",
        f"dependencies = [{_toml(requirement)}]",
        "",
        "[tool.uv]",
        "package = false",
    ]
    if policy.required_version:
        lines.append(f"required-version = {_toml(policy.required_version)}")
    lines.append(f"environments = [{_toml(profile.marker)}]")
    lines.append(f"override-dependencies = {_toml(overrides)}")
    constrained = [*policy.constraint_dependencies, *constraints]
    lines.append("constraint-dependencies = [")
    lines.extend(f"    {_toml(entry)}," for entry in constrained)
    lines.append("]")
    if policy.sources:
        lines.append("")
        lines.append("[tool.uv.sources]")
        for name, source in policy.sources.items():
            lines.append(f"{name} = {_toml(source)}")
    for index in policy.indexes:
        lines.append("")
        lines.append("[[tool.uv.index]]")
        for key, value in index.items():
            lines.append(f"{key} = {_toml(value)}")
    return "\n".join(lines) + "\n"


def _requirement_name(entry: str) -> str:
    for separator in (";", "@", "=", "<", ">", "!", "~", " ", "["):
        entry = entry.split(separator, 1)[0]
    return entry.strip()


def _source_url(source: Mapping[str, Any]) -> str | None:
    if "git" in source:
        url = f"git+{source['git']}"
        for key in ("rev", "tag", "branch"):
            if key in source:
                return f"{url}@{source[key]}"
        return url
    if "url" in source:
        return str(source["url"])
    return None


def _toml(value: Any) -> str:
    """Render the subset of TOML values a uv configuration uses."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    if isinstance(value, str):
        return json.dumps(value)
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_toml(item) for item in value) + "]"
    if isinstance(value, dict):
        return "{ " + ", ".join(f"{key} = {_toml(item)}" for key, item in value.items()) + " }"
    raise TypeError(f"cannot render {type(value).__name__} as TOML")
