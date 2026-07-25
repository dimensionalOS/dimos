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

"""Discover and remove disk caches owned by DimOS."""

from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
import shutil
import subprocess

from dimos.constants import (
    AMENT_PREFIX_CACHE_DIR,
    CACHE_DIR,
    DENO_CACHE_DIR,
    DRAKE_URDF_CACHE_DIR,
    ROBOT_ASSET_CACHE_DIR,
    VISER_URDF_CACHE_DIR,
)


@dataclass(frozen=True)
class CacheIssue:
    """A cache path that could not be removed."""

    path: Path
    reason: str


@dataclass
class CacheCleanResult:
    """Summary returned by :func:`clean_caches`."""

    cleaned: list[Path] = field(default_factory=list)
    skipped: list[CacheIssue] = field(default_factory=list)
    failed: list[CacheIssue] = field(default_factory=list)

    @property
    def complete(self) -> bool:
        """Whether every discovered cache path was removed."""
        return not self.skipped and not self.failed


def clean_caches(*, force: bool = False) -> CacheCleanResult:
    """Remove all known DimOS caches.

    Git checkouts used by robot assets are treated as disposable only when
    clean and fully represented by a remote. Pass ``force=True`` to remove
    checkouts containing local work.
    """
    result = CacheCleanResult()
    protected = {} if force else _protected_robot_checkouts()
    result.skipped.extend(CacheIssue(path, reason) for path, reason in protected.items())

    for target in _cache_targets():
        target = target.absolute()
        if not _lexists(target):
            continue
        removed_any = _remove_except(target, set(protected), result)
        if removed_any:
            result.cleaned.append(target)

    return result


def _cache_targets() -> tuple[Path, ...]:
    return (
        CACHE_DIR,
        DRAKE_URDF_CACHE_DIR,
        VISER_URDF_CACHE_DIR,
        AMENT_PREFIX_CACHE_DIR,
        DENO_CACHE_DIR,
    )


def _protected_robot_checkouts() -> dict[Path, str]:
    sources_root = (ROBOT_ASSET_CACHE_DIR / "sources").absolute()
    if not _lexists(sources_root) or sources_root.is_symlink() or not sources_root.is_dir():
        return {}

    protected: dict[Path, str] = {}

    def protect_unreadable(error: OSError) -> None:
        protected[sources_root] = f"could not inspect robot asset checkouts: {error}"

    for directory, dirnames, filenames in os.walk(
        sources_root,
        followlinks=False,
        onerror=protect_unreadable,
    ):
        if ".git" not in dirnames and ".git" not in filenames:
            continue

        checkout = Path(directory).absolute()
        reason = _git_protection_reason(checkout)
        if reason is not None:
            protected[checkout] = reason

        # A checkout may contain nested repositories as source data. They are
        # covered by the outer checkout's status and should not be inspected
        # independently.
        dirnames.clear()

    return protected


def _git_protection_reason(checkout: Path) -> str | None:
    status = _run_git(checkout, "status", "--porcelain", "--untracked-files=all")
    if status.returncode != 0:
        return f"could not inspect Git checkout: {_command_error(status)}"
    if status.stdout.strip():
        return "Git checkout has local changes"

    local_commits = _run_git(
        checkout,
        "rev-list",
        "HEAD",
        "--branches",
        "--not",
        "--remotes",
    )
    if local_commits.returncode != 0:
        return f"could not inspect Git history: {_command_error(local_commits)}"
    if local_commits.stdout.strip():
        return "Git checkout has local-only commits"
    return None


def _run_git(checkout: Path, *args: str) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            ["git", "-C", str(checkout), *args],
            capture_output=True,
            check=False,
            text=True,
        )
    except OSError as error:
        return subprocess.CompletedProcess(
            args=["git", "-C", str(checkout), *args],
            returncode=1,
            stdout="",
            stderr=str(error),
        )


def _command_error(result: subprocess.CompletedProcess[str]) -> str:
    return result.stderr.strip() or f"git exited with status {result.returncode}"


def _remove_except(
    path: Path,
    protected: set[Path],
    result: CacheCleanResult,
) -> bool:
    if path in protected:
        return False

    protected_below = {item for item in protected if _is_below(item, path)}
    if not protected_below:
        try:
            _remove_path(path)
        except OSError as error:
            result.failed.append(CacheIssue(path, str(error)))
            return False
        return True

    removed_any = False
    try:
        children = tuple(path.iterdir())
    except OSError as error:
        result.failed.append(CacheIssue(path, str(error)))
        return False

    for child in children:
        removed_any = _remove_except(child, protected_below, result) or removed_any
    return removed_any


def _remove_path(path: Path) -> None:
    if path.is_symlink() or not path.is_dir():
        path.unlink()
    else:
        shutil.rmtree(path)


def _is_below(path: Path, parent: Path) -> bool:
    return path != parent and parent in path.parents


def _lexists(path: Path) -> bool:
    return os.path.lexists(path)
