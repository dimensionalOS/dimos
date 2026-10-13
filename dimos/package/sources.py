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

"""Resolve repository member shorthand without importing package code."""

from __future__ import annotations

import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any
from urllib.parse import parse_qs, urlsplit

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def canonical_name(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def read_manifest(directory: Path) -> dict[str, Any]:
    with (directory / "pyproject.toml").open("rb") as stream:
        return tomllib.load(stream)


def workspace_member(root: Path, name: str) -> Path:
    """Find one declared project.name, never an import or blueprint target."""
    root = root.resolve()
    manifest = read_manifest(root)
    workspace = manifest.get("tool", {}).get("uv", {}).get("workspace", {})
    members = workspace.get("members", [])
    excludes = workspace.get("exclude", [])
    candidates = {root}
    for pattern in [*members, *excludes]:
        if (
            not isinstance(pattern, str)
            or Path(pattern).is_absolute()
            or ".." in Path(pattern).parts
        ):
            raise ValueError("Workspace patterns must stay inside the repository")
    excluded = {path.resolve() for pattern in excludes for path in root.glob(pattern)}
    for pattern in members:
        for path in root.glob(pattern):
            if path.is_symlink() or not path.resolve().is_relative_to(root):
                raise ValueError("Workspace members must stay inside the repository")
            if path.resolve() not in excluded:
                candidates.add(path.resolve())
    matches = [
        directory
        for directory in sorted(candidates)
        if canonical_name(read_manifest(directory).get("project", {}).get("name", ""))
        == canonical_name(name)
    ]
    if len(matches) != 1:
        raise ValueError(f"Expected one declared project named {name!r}; found {len(matches)}")
    return matches[0].relative_to(root)


def resolve_source(
    source: str,
    *,
    tag: str | None = None,
    branch: str | None = None,
    rev: str | None = None,
    subdirectory: str | None = None,
) -> str:
    """Translate github:OWNER/REPO#distribution to uv's Git subdirectory syntax."""
    if not source.startswith("github:"):
        if source.startswith((".", "/", "~")) or Path(source).is_dir():
            if subdirectory or tag or branch or rev:
                raise ValueError("Git selectors require a Git source")
            return str(Path(source).expanduser().resolve())
        if subdirectory:
            path = Path(subdirectory)
            if path.is_absolute() or ".." in path.parts:
                raise ValueError("Git subdirectory must stay inside the repository")
            if not source.startswith("git+") or "#" in source:
                raise ValueError("--subdirectory requires a Git URL without a fragment")
            source += f"#subdirectory={path.as_posix()}"
        if source.startswith("git+"):
            fragments = parse_qs(urlsplit(source.removeprefix("git+")).fragment)
            for fragment_path in fragments.get("subdirectory", []):
                if Path(fragment_path).is_absolute() or ".." in Path(fragment_path).parts:
                    raise ValueError("Git subdirectory must stay inside the repository")
        return source
    match = re.fullmatch(r"github:([A-Za-z0-9_.-]+)/([A-Za-z0-9_.-]+)#([A-Za-z0-9_.-]+)", source)
    if not match or subdirectory:
        raise ValueError("Use github:OWNER/REPO#distribution (without --subdirectory)")
    owner, repository, name = match.groups()
    selector = tag or branch or rev
    if selector and (selector.startswith("-") or not re.fullmatch(r"[A-Za-z0-9_./-]+", selector)):
        raise ValueError("Invalid Git reference")
    url = f"https://github.com/{owner}/{repository.removesuffix('.git')}.git"
    with tempfile.TemporaryDirectory(prefix="dimos-member-") as temporary:
        root = Path(temporary) / "repo"
        env = {**os.environ, "GIT_LFS_SKIP_SMUDGE": "1"}
        subprocess.run(
            ["git", "clone", "--filter=blob:none", "--no-checkout", "--", url, str(root)],
            check=True,
            env=env,
        )
        ref = f"refs/remotes/origin/{branch}" if branch else selector or "refs/remotes/origin/HEAD"
        commit = subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "--verify", f"{ref}^{{commit}}"], text=True
        ).strip()
        subprocess.run(
            ["git", "-C", str(root), "checkout", "--detach", commit], check=True, env=env
        )
        member = workspace_member(root, name)
    return "git+" + url + (f"#subdirectory={member.as_posix()}" if member.parts else "")
