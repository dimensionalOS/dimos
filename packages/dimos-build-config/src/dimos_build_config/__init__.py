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

"""Source-only defaults; no runtime imports, native commands or backend wrapper."""

from collections.abc import Mapping
from pathlib import Path
import re
import sys
from typing import Any

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

# These are distribution hygiene defaults, not a secret scanner. Authors must
# inspect artifacts and keep credentials out of the package's source tree.
EXCLUDES = [
    "**/.git/**",
    "**/.venv/**",
    "**/__pycache__/**",
    "**/*.pyc",
    "**/target/**",
    "**/build/**",
    "**/dist/**",
    "**/*.egg-info/**",
    "**/.env",
    "**/.env.*",
    "**/*.pem",
    "**/*.key",
    "**/.aws/**",
]


def read_project() -> tuple[dict[str, Any], dict[str, Any]]:
    """Read and validate declarations without importing package code."""
    with Path("pyproject.toml").open("rb") as stream:
        document = tomllib.load(stream)
    table = document.get("tool", {}).get("dimos", {})
    if not isinstance(table, dict):
        raise ValueError("tool.dimos must be a table")
    if not table:
        return document, table
    unknown = table.keys() - {"package", "blueprints"}
    if unknown:
        raise ValueError(f"Unknown tool.dimos fields: {sorted(unknown)}")
    package = table.get("package")
    if not isinstance(package, str) or not package:
        raise ValueError("tool.dimos.package must name a relative Python package directory")
    path = Path(package)
    if path.is_absolute() or ".." in path.parts or not path.name.isidentifier():
        raise ValueError("tool.dimos.package must be a relative, non-escaping package directory")
    if not path.is_dir() or not path.resolve().is_relative_to(Path.cwd().resolve()):
        raise ValueError(
            f"tool.dimos.package directory is missing or outside the project: {package}"
        )
    if path.is_symlink() or any(p.is_symlink() for p in path.rglob("*")):
        raise ValueError("tool.dimos.package must contain real files, not symlinks")
    blueprints = table.get("blueprints", {})
    if not isinstance(blueprints, dict):
        raise ValueError("tool.dimos.blueprints must be a name-to-target table")
    for name, target in blueprints.items():
        if not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", name):
            raise ValueError(f"Invalid blueprint name: {name!r}; use lowercase kebab-case")
        if not isinstance(target, str) or not re.fullmatch(
            r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*:[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*", target
        ):
            raise ValueError(f"Invalid blueprint target for {name!r}; expected module:object")
    project = document.get("project", {})
    if "dimos.blueprints" in project.get("entry-points", {}):
        raise ValueError("Declare dimos.blueprints only in tool.dimos.blueprints")
    providers = document.get("tool", {}).get("dynamic-metadata", [])
    dimos_providers = [p for p in providers if p.get("provider") == "dimos"]
    if blueprints and (
        "entry-points" not in project.get("dynamic", []) or len(dimos_providers) != 1
    ):
        raise ValueError(
            'Blueprints require project.dynamic = ["entry-points"] and one dimos metadata provider'
        )
    return document, table


def config(*, env: Mapping[str, str]) -> dict[str, Any]:
    """Supply defaults through scikit-build-core's public configuration hook."""
    document, table = read_project()
    if not table:
        return {}
    settings = document.get("tool", {}).get("scikit-build", {})
    for group, key in (("wheel", "cmake"), ("sdist", "cmake"), ("editable", "rebuild")):
        if settings.get(group, {}).get(key) or env.get(
            f"SKBUILD_{group}_{key}".upper(), ""
        ).lower() in {"1", "true", "on", "yes"}:
            raise ValueError(f"tool.dimos source-only packages cannot enable {group}.{key}")
    package = table["package"]
    return {
        "wheel": {
            "packages": [package],
            "cmake": False,
            "platlib": False,
            "py-api": "py3",
            "exclude": EXCLUDES,
        },
        # Include rules precede excludes. Negated include patterns let the
        # catch-all exclusion remove build products even inside the package.
        "sdist": {
            "cmake": False,
            "include": [
                "pyproject.toml",
                "README*",
                "LICENSE*",
                "COPYING*",
                f"{package}/**",
                *[f"!{pattern}" for pattern in EXCLUDES],
            ],
            "exclude": ["**"],
        },
        "editable": {"rebuild": False},
    }
