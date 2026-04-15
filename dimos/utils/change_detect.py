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

"""Change detection utility for file content hashing.

Tracks whether a set of files (by path, directory, or glob pattern) have
changed since the last check. Useful for skipping expensive rebuilds when
source files haven't been modified.

Path entries are type-dispatched:

- ``str`` / ``Path`` / ``LfsPath`` — treated as **literal** file or directory
  paths (no glob expansion, even if the path contains ``*``).
- ``Glob`` — expanded with :func:`glob.glob` to match filesystem patterns.
"""

from __future__ import annotations

from collections.abc import Sequence
import fcntl
import glob as glob_mod
import hashlib
import os
from pathlib import Path
import threading
from typing import Any, Union

import xxhash

from dimos.utils.data import LfsPath
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Glob(str):
    """A string that should be interpreted as a filesystem glob pattern.

    Wraps a plain ``str`` to signal that :func:`did_change` should expand it
    with :func:`glob.glob` rather than treating it as a literal path.

    Example::

        Glob("src/**/*.c")
    """


PathEntry = Union[str, Path, LfsPath, Glob]
"""A single entry in a change-detection path list."""


def _get_cache_dir() -> Path:
    """Return the directory used to store change-detection cache files.

    Uses ``<VIRTUAL_ENV>/dimos_cache/change_detect/`` when running inside a
    venv, otherwise falls back to ``~/.cache/dimos/change_detect/``.
    """
    venv = os.environ.get("VIRTUAL_ENV")
    if venv:
        return Path(venv) / "dimos_cache" / "change_detect"
    return Path.home() / ".cache" / "dimos" / "change_detect"


def _safe_filename(cache_name: str) -> str:
    """Convert an arbitrary cache name into a safe filename.

    If the cache name is already a simple identifier it is returned as-is.
    Otherwise a short SHA-256 prefix is appended so that names containing
    path separators or other special characters produce unique, safe filenames.
    """
    safe_chars = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")
    if all(c in safe_chars for c in cache_name) and len(cache_name) <= 200:
        return cache_name
    digest = hashlib.sha256(cache_name.encode()).hexdigest()[:16]
    return digest


def _add_path(files: set[Path], p: Path) -> None:
    """Add *p* (file or directory, walked recursively) to *files*."""
    if p.is_file():
        files.add(p.resolve())
    elif p.is_dir():
        for root, _dirs, filenames in os.walk(p):
            for fname in filenames:
                files.add(Path(root, fname).resolve())


def _resolve_paths(paths: Sequence[PathEntry], cwd: str | Path | None = None) -> list[Path]:
    """Resolve a mixed list of path entries into a sorted list of files.

    ``Glob`` entries are expanded via :func:`glob.glob`.  All other types
    (``str``, ``Path``, ``LfsPath``) are treated as literal paths — no
    wildcard expansion is performed.

    When *cwd* is provided, relative paths are resolved against it.
    When *cwd* is ``None``, relative paths raise :class:`ValueError`.
    """
    files: set[Path] = set()
    for entry in paths:
        if isinstance(entry, Glob):
            pattern = str(entry)
            if not Path(pattern).is_absolute():
                if cwd is None:
                    raise ValueError(
                        f"Relative path {pattern!r} passed to change detection without a cwd. "
                        "Either provide an absolute path or pass cwd= so relatives can be resolved."
                    )
                pattern = str(Path(cwd) / pattern)
            expanded = glob_mod.glob(pattern, recursive=True)
            if not expanded:
                logger.warning("Glob pattern matched no files", pattern=pattern)
                continue
            for match in expanded:
                _add_path(files, Path(match))
        else:
            # str, Path, LfsPath — literal path, no glob expansion
            path_str = str(entry)
            if not Path(path_str).is_absolute():
                if cwd is None:
                    raise ValueError(
                        f"Relative path {path_str!r} passed to change detection without a cwd. "
                        "Either provide an absolute path or pass cwd= so relatives can be resolved."
                    )
                path_str = str(Path(cwd) / path_str)
            p = Path(path_str)
            if not p.exists():
                logger.warning("Path does not exist", path=path_str)
                continue
            _add_path(files, p)
    return sorted(files)


_HASH_CHUNK_SIZE = 1 << 20  # 1 MiB


def _hash_files(files: list[Path], *, max_file_size: int | None = None) -> str:
    """Compute an aggregate xxhash digest over the sorted file list.

    Files are streamed in 1 MiB chunks so large files don't get loaded into
    memory all at once.  When *max_file_size* is set, any file whose size
    exceeds the threshold is fingerprinted by ``(size, mtime_ns)`` instead of
    by content — O(1) regardless of file size, at the cost of missing
    content changes that don't touch mtime (rare: in-place writes that
    preserve timestamps, or two different files hitting the same mtime).
    Good enough for "rebuild when a large dataset is swapped in", not good
    enough for correctness-critical integrity checking.

    The digest embeds a mode marker per file so that the same file switching
    between content-mode and stat-mode (e.g. because it grew past the
    threshold) produces a different aggregate digest.
    """
    h = xxhash.xxh64()
    for fpath in files:
        try:
            st = fpath.stat()
            # Include the path so additions/deletions/renames are detected.
            h.update(str(fpath).encode())
            if max_file_size is not None and st.st_size > max_file_size:
                # Fingerprint-only mode: stat fields, no content read.
                h.update(b"\x00stat\x00")
                h.update(str(st.st_size).encode())
                h.update(b"\x00")
                h.update(str(st.st_mtime_ns).encode())
                h.update(b"\x00")
            else:
                h.update(b"\x00content\x00")
                with open(fpath, "rb") as f:
                    while chunk := f.read(_HASH_CHUNK_SIZE):
                        h.update(chunk)
        except (OSError, PermissionError):
            logger.warning("Cannot read file for hashing", path=str(fpath))
    return h.hexdigest()


def hash_dict(data: dict[Any, Any], *, extra_hash: str | None = None) -> str:
    """Return a stable xxhash digest of a dict's keys and values.

    Keys are sorted (by their ``str`` form) so insertion order doesn't affect
    the result, and each key/value is serialized via ``str()`` — good enough
    for config dicts holding primitives, paths, and small nested structures.
    Not suitable for values whose ``str()`` isn't deterministic (e.g. objects
    that include memory addresses in their repr).
    """
    h = xxhash.xxh64()
    for key in sorted(data, key=str):
        h.update(str(key).encode())
        h.update(b"\x00")
        h.update(str(data[key]).encode())
        h.update(b"\x00")
    if extra_hash:
        h.update(extra_hash.encode())
    return h.hexdigest()


def hash_paths(
    paths: Sequence[PathEntry],
    cwd: str | Path | None = None,
    *,
    extra_hash: str | None = None,
    max_file_size: int | None = None,
) -> str | None:
    """Return a stable content hash of *paths*, or ``None`` if nothing resolves.

    Resolves a mixed list of files, directories, and :class:`Glob` patterns
    (see :func:`did_change` for path-entry semantics), then returns an xxhash
    digest of the sorted file contents.  If *extra_hash* is provided it is
    folded into the final digest, so callers can invalidate on non-file inputs
    (e.g. a build command, a processing version string).

    *max_file_size* (in bytes, default ``None``): files larger than this are
    fingerprinted by ``(size, mtime_ns)`` instead of full content.  See
    :func:`_hash_files` for the tradeoff — use it when sources may include
    the occasional large blob (datasets, binaries) you don't want to stream
    through xxhash every call.

    Use this directly when you want a content-addressed cache key without the
    full :func:`did_change` machinery (no cache file, no lock, no previous
    state).  :func:`did_change` and :func:`update_cache` both call this
    internally.

    Returns ``None`` when *paths* is empty or none of the entries resolve to
    existing files — callers decide what that means (skip, rebuild, error).
    """
    if not paths:
        return None
    files = _resolve_paths(paths, cwd=cwd)
    if not files:
        return None
    digest = _hash_files(files, max_file_size=max_file_size)
    if extra_hash:
        h = xxhash.xxh64()
        h.update(digest.encode())
        h.update(extra_hash.encode())
        digest = h.hexdigest()
    return digest


# Thread-level locks keyed by cache_name (flock only protects cross-process).
_thread_locks: dict[str, threading.Lock] = {}
_thread_locks_guard = threading.Lock()


def _get_thread_lock(cache_name: str) -> threading.Lock:
    with _thread_locks_guard:
        if cache_name not in _thread_locks:
            _thread_locks[cache_name] = threading.Lock()
        return _thread_locks[cache_name]


def did_change(
    cache_name: str,
    paths: Sequence[PathEntry],
    cwd: str | Path | None = None,
    *,
    update: bool = True,
    extra_hash: str | None = None,
    max_file_size: int | None = None,
) -> bool:
    """Check if any files/dirs matching the given paths have changed since last check.

    Examples::

        # Absolute paths — no cwd needed
        did_change("my_build", ["/src/main.cpp"])

        # Use Glob for wildcard patterns (str is always literal)
        did_change("c_sources", [Glob("/src/**/*.c"), Glob("/include/**/*.h")])

        # Relative paths — must pass cwd
        did_change("my_build", ["src/main.cpp"], cwd="/home/user/project")

        # Mix literal paths and globs
        did_change("config_check", ["config.yaml", Glob("templates/*.j2")], cwd="/project")

        # Track a whole directory (walked recursively)
        did_change("assets", ["/data/models/"])

        # Check without updating (dry run)
        did_change("my_build", ["/src/main.cpp"], update=False)

        # Second call with no file changes → False
        did_change("my_build", ["/src/main.cpp"])  # True  (first call, no cache)
        did_change("my_build", ["/src/main.cpp"])  # False (nothing changed)

        # After editing a file → True again
        Path("/src/main.cpp").write_text("// changed")
        did_change("my_build", ["/src/main.cpp"])  # True

        # Relative path without cwd → ValueError
        did_change("bad", ["src/main.cpp"])  # raises ValueError

    Args:
        cache_name: Unique identifier for this change-detection cache.
        paths: Files, directories, or :class:`Glob` patterns to monitor.
        cwd: Working directory for resolving relative paths.
        update: If ``True`` (default), update the cache with the current hash
            after checking.  Set to ``False`` to check without updating — this
            lets the caller decide whether to update (e.g. only after a
            successful build via :func:`update_cache`).
        extra_hash: Optional extra string folded into the hash (e.g. a build
            command), so changes to it trigger a rebuild even if source files
            are unchanged.
        max_file_size: If set, files larger than this (in bytes) are
            fingerprinted by ``(size, mtime_ns)`` instead of having their
            content streamed through xxhash.  Trades precision for constant-
            time handling of large blobs — see :func:`_hash_files`.

    Returns ``True`` on the first call (no previous cache), and on subsequent
    calls returns ``True`` only if file contents differ from the last check.
    When *update* is ``True`` the cache is updated, so two consecutive calls
    with no changes return ``True`` then ``False``.
    """
    current_hash = hash_paths(
        paths,
        cwd=cwd,
        extra_hash=extra_hash,
        max_file_size=max_file_size,
    )

    # If none of the monitored paths resolve to actual files (e.g. source
    # files don't exist on this branch or checkout), don't claim anything
    # changed — deleting a working binary because we can't find the sources
    # to compare against is destructive.
    if current_hash is None:
        logger.warning(
            "No source files found for change detection, skipping rebuild check",
            cache_name=cache_name,
        )
        return False

    cache_dir = _get_cache_dir()
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_file = cache_dir / f"{_safe_filename(cache_name)}.hash"
    lock_file = cache_dir / f"{_safe_filename(cache_name)}.lock"

    changed = True
    thread_lock = _get_thread_lock(cache_name)
    with thread_lock, open(lock_file, "w") as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        try:
            if cache_file.exists():
                previous_hash = cache_file.read_text().strip()
                changed = current_hash != previous_hash
            # Only update the cache when requested — allows callers to defer
            # the update until after a successful build so that a failed build
            # doesn't prevent future rebuild attempts.
            if update:
                cache_file.write_text(current_hash)
        finally:
            fcntl.flock(lf, fcntl.LOCK_UN)

    return changed


def update_cache(
    cache_name: str,
    paths: Sequence[PathEntry],
    cwd: str | Path | None = None,
    *,
    extra_hash: str | None = None,
    max_file_size: int | None = None,
) -> None:
    """Write the current file hash to the cache without checking for changes.

    Call this after a successful build to record the current state so that the
    next :func:`did_change` call returns ``False`` (unless files change again).
    Pass the same *max_file_size* you'll be passing to :func:`did_change`, or
    the two won't agree on the hash of any large files.

    Example::

        if did_change("my_build", sources, update=False, extra_hash=cmd):
            run_build()          # might fail
            update_cache("my_build", sources, extra_hash=cmd)  # only on success
    """
    current_hash = hash_paths(
        paths,
        cwd=cwd,
        extra_hash=extra_hash,
        max_file_size=max_file_size,
    )
    if current_hash is None:
        return

    cache_dir = _get_cache_dir()
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_file = cache_dir / f"{_safe_filename(cache_name)}.hash"
    lock_file = cache_dir / f"{_safe_filename(cache_name)}.lock"

    thread_lock = _get_thread_lock(cache_name)
    with thread_lock, open(lock_file, "w") as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        try:
            cache_file.write_text(current_hash)
        finally:
            fcntl.flock(lf, fcntl.LOCK_UN)


def clear_cache(cache_name: str) -> bool:
    """Remove the cached hash so the next ``did_change`` call returns ``True``.

    Example::

        clear_cache("my_build")
        did_change("my_build", ["/src/main.c"])  # always True after clear
    """
    cache_dir = _get_cache_dir()
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_file = cache_dir / f"{_safe_filename(cache_name)}.hash"
    lock_file = cache_dir / f"{_safe_filename(cache_name)}.lock"

    thread_lock = _get_thread_lock(cache_name)
    with thread_lock, open(lock_file, "w") as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        try:
            if cache_file.exists():
                cache_file.unlink()
                return True
            return False
        finally:
            fcntl.flock(lf, fcntl.LOCK_UN)
