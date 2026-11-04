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

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any

from dimos.constants import DIMOS_PROJECT_ROOT, STATE_DIR
from experimental.gateway import shared_config

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def gateway_dir() -> Path:
    return STATE_DIR / "gateway"


def logs_dir() -> Path:
    return gateway_dir() / "logs"


# the checkout the gateway serves (new_state), which may not be the gateway's own (Desktop's pinned copy)
_served: Path = DIMOS_PROJECT_ROOT


def serve(dimos_dir: Path) -> None:
    global _served
    _served = dimos_dir


def recordings_dir(dimos_dir: Path | None = None) -> Path:
    """The configured one (a relative one is under the checkout, where launches run), else the checkout's default."""
    dimos_dir = dimos_dir or _served
    configured = shared_config.setting("recordings", "dir")
    if configured:
        return dimos_dir / Path(str(configured)).expanduser()
    return dimos_dir / "recordings" if (dimos_dir / ".git").exists() else STATE_DIR / "recordings"


def shared_settings() -> dict[str, Any]:
    """`GET /config` and the `config` event: the settings read fresh on each use (socket and namespace are fixed at start: `GET /info`)."""
    return {"path": str(shared_config.config_file()), "recordingsDir": str(recordings_dir())}


def write_atomic(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w") as handle:
            handle.write(text)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def venv_dir(dimos_dir: Path) -> Path:
    return dimos_dir / ".venv"


def python_for(dimos_dir: Path) -> str:
    python = venv_dir(dimos_dir) / "bin" / "python"
    return str(python) if python.exists() else sys.executable


def dimos_bin(dimos_dir: Path) -> Path:
    return venv_dir(dimos_dir) / "bin" / "dimos"


def _stat_key(*paths: Path) -> tuple[tuple[int, int] | None, ...]:
    def one(path: Path) -> tuple[int, int] | None:
        try:
            stat = path.stat()
        except OSError:
            return None
        return stat.st_mtime_ns, stat.st_size

    return tuple(one(path) for path in paths)


# GET /info is polled: what it reads is redone only when the files behind it change
_versions: dict[Path, tuple[Any, tuple[bool, str | None]]] = {}
_git_ids: dict[Path, tuple[Any, tuple[str | None, str | None]]] = {}


def checkout_version(dimos_dir: Path) -> tuple[bool, str | None]:
    pyproject = dimos_dir / "pyproject.toml"
    key = _stat_key(pyproject)
    cached = _versions.get(dimos_dir)
    if cached is None or cached[0] != key:
        cached = (key, _read_version(pyproject))
        _versions[dimos_dir] = cached
    return cached[1]


def _read_version(pyproject: Path) -> tuple[bool, str | None]:
    try:
        project = tomllib.loads(pyproject.read_text()).get("project", {})
    except (OSError, tomllib.TOMLDecodeError):
        return False, None
    if project.get("name") != "dimos":
        return False, None
    version = project.get("version")
    return True, str(version) if version else None


def git(dimos_dir: Path, *args: str) -> str | None:
    try:
        done = subprocess.run(["git", "-C", str(dimos_dir), *args], capture_output=True, text=True)
    except OSError:
        return None
    return done.stdout.strip() or None if done.returncode == 0 else None


def _git_dirs(dimos_dir: Path) -> tuple[Path, Path]:
    """The checkout's git dir and the common one (a worktree's .git is a `gitdir:` file), without running git."""
    dot_git = dimos_dir / ".git"
    try:
        text = dot_git.read_text() if dot_git.is_file() else None
    except OSError:
        text = None
    git_dir = (
        (dimos_dir / text.split("gitdir:", 1)[1].strip()) if text and "gitdir:" in text else dot_git
    )
    try:
        common = git_dir / (git_dir / "commondir").read_text().strip()
    except OSError:
        common = git_dir
    return git_dir, common


def git_ids(dimos_dir: Path) -> tuple[str | None, str | None]:
    """HEAD's short commit and its desktop-gateway-v* tag, rerun only when HEAD (its reflog) or the tags move."""
    git_dir, common = _git_dirs(dimos_dir)
    key = _stat_key(
        git_dir / "HEAD",
        git_dir / "logs" / "HEAD",
        common / "packed-refs",
        common / "refs" / "tags",
    )
    cached = _git_ids.get(dimos_dir)
    if cached is None or cached[0] != key:
        ids = (
            git(dimos_dir, "rev-parse", "--short=10", "HEAD"),
            git(dimos_dir, "describe", "--tags", "--exact-match", "--match", "desktop-gateway-v*"),
        )
        cached = (key, ids)
        _git_ids[dimos_dir] = cached
    return cached[1]


def info(dimos_dir: Path) -> dict[str, Any]:
    from experimental.gateway.state import API_VERSION

    found, version = checkout_version(dimos_dir)
    commit, tag = git_ids(dimos_dir)
    return {
        "dir": str(dimos_dir),
        "found": found,
        "installed": found and dimos_bin(dimos_dir).exists(),
        "version": version,
        "api": API_VERSION,
        "commit": commit,
        "tag": tag,
    }
