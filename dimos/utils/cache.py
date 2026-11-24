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

from collections.abc import Callable, Iterator
import contextlib
from dataclasses import dataclass, field
import functools
import os
from pathlib import Path
import shutil
from typing import ParamSpec, TypeVar
import uuid

from filelock import FileLock, Timeout

from dimos.constants import CACHE_DIR, STATE_DIR
from dimos.robot.assets.git_cache import git_checkout_protection_reason

_CACHE_LOCK_DIR = STATE_DIR / "cache-users"
_CACHE_GATE_PATH = STATE_DIR / "cache-clean.lock"

_P = ParamSpec("_P")
_R = TypeVar("_R")


class CacheInUseError(RuntimeError):
    """Raised when cleanup finds a process using DimOS caches."""


@dataclass(frozen=True)
class CacheIssue:
    """A cache path that was skipped or could not be removed."""

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
    """Remove files beneath the DimOS cache root.

    Robot asset Git checkouts containing local work are preserved unless
    ``force`` is true. Other cache entries are still removed.
    """
    result = CacheCleanResult()
    target = CACHE_DIR.absolute()
    if not _lexists(target):
        return result

    protected = {} if force else _protected_robot_checkouts(target / "robot_assets")
    result.skipped.extend(CacheIssue(path, reason) for path, reason in protected.items())
    if _remove_except(target, set(protected), result):
        result.cleaned.append(target)
    return result


def _protected_robot_checkouts(robot_cache_root: Path) -> dict[Path, str]:
    """Return cached robot checkouts that contain or may contain local work."""
    sources_root = (robot_cache_root / "sources").absolute()
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
        reason = git_checkout_protection_reason(checkout)
        if reason is not None:
            protected[checkout] = reason

        # Nested repositories are owned by the outer cached checkout.
        dirnames.clear()

    return protected


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

    try:
        children = tuple(path.iterdir())
    except OSError as error:
        result.failed.append(CacheIssue(path, str(error)))
        return False

    removed_any = False
    for child in children:
        removed_any = _remove_except(child, protected_below, result) or removed_any
    return removed_any


@contextlib.contextmanager
def cache_usage_guard() -> Iterator[None]:
    """Mark the current process as a cache user for the context lifetime."""
    _CACHE_LOCK_DIR.mkdir(parents=True, exist_ok=True)
    usage_path = _CACHE_LOCK_DIR / f"{os.getpid()}-{uuid.uuid4().hex}.lock"
    usage_lock = FileLock(usage_path)

    with FileLock(_CACHE_GATE_PATH):
        usage_lock.acquire()

    try:
        yield
    finally:
        usage_lock.release()
        usage_path.unlink(missing_ok=True)


def cache_usage_locked(function: Callable[_P, _R]) -> Callable[_P, _R]:
    """Run a callable while holding a cache usage marker."""

    @functools.wraps(function)
    def wrapped(*args: _P.args, **kwargs: _P.kwargs) -> _R:
        with cache_usage_guard():
            return function(*args, **kwargs)

    return wrapped


@contextlib.contextmanager
def cache_cleanup_guard() -> Iterator[None]:
    """Exclude run startup and fail if another process uses DimOS caches."""
    _CACHE_LOCK_DIR.mkdir(parents=True, exist_ok=True)

    with FileLock(_CACHE_GATE_PATH):
        for usage_path in _CACHE_LOCK_DIR.glob("*.lock"):
            usage_lock = FileLock(usage_path)
            try:
                usage_lock.acquire(timeout=0)
            except Timeout as error:
                raise CacheInUseError("DimOS caches are in use") from error
            else:
                usage_lock.release()
                usage_path.unlink(missing_ok=True)
        yield


def _remove_path(path: Path) -> None:
    if path.is_symlink() or not path.is_dir():
        path.unlink()
    else:
        shutil.rmtree(path)


def _is_below(path: Path, parent: Path) -> bool:
    return path != parent and parent in path.parents


def _lexists(path: Path) -> bool:
    return os.path.lexists(path)
