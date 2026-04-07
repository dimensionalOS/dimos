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
from typing import Union

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
    """Convert an arbitrary cache name into a safe filename."""
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
    """Resolve a mixed list of path entries into a sorted list of files."""
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


def _hash_files(files: list[Path]) -> str:
    """Compute an aggregate xxhash digest over the sorted file list."""
    h = xxhash.xxh64()
    for fpath in files:
        try:
            # Include the path so additions/deletions/renames are detected
            h.update(str(fpath).encode())
            h.update(fpath.read_bytes())
        except (OSError, PermissionError):
            logger.warning("Cannot read file for hashing", path=str(fpath))
    return h.hexdigest()


def did_change(
    cache_name: str,
    paths: Sequence[PathEntry],
    cwd: str | Path | None = None,
    extra_hash: str | None = None,
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

        # Second call with no file changes → False
        did_change("my_build", ["/src/main.cpp"])  # True  (first call, no cache)
        did_change("my_build", ["/src/main.cpp"])  # False (nothing changed)

        # After editing a file → True again
        Path("/src/main.cpp").write_text("// changed")
        did_change("my_build", ["/src/main.cpp"])  # True

        # Relative path without cwd → ValueError
        did_change("bad", ["src/main.cpp"])  # raises ValueError

        # Include extra context (e.g. build command) in the hash
        did_change("my_build", ["/src/main.cpp"], extra_hash="nix build .#foo")

    Args:
        cache_name: Stable key for the cache file.
        paths: Files/dirs/globs to hash.
        cwd: Working directory for resolving relative paths.
        extra_hash: Optional extra string folded into the hash (e.g. a build
            command), so changes to it trigger a rebuild even if source files
            are unchanged.

    Returns ``True`` on the first call (no previous cache), and on subsequent
    calls returns ``True`` only if file contents differ from the last check.
    The cache is always updated, so two consecutive calls with no changes
    return ``True`` then ``False``.
    """
    if not paths:
        return False

    files = _resolve_paths(paths, cwd=cwd)

    if not files:
        logger.warning(
            "No source files found for change detection, skipping rebuild check",
            cache_name=cache_name,
        )
        return False

    current_hash = _hash_files(files)
    if extra_hash:
        h = xxhash.xxh64()
        h.update(current_hash.encode())
        h.update(extra_hash.encode())
        current_hash = h.hexdigest()

    cache_dir = _get_cache_dir()
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_file = cache_dir / f"{_safe_filename(cache_name)}.hash"
    lock_file = cache_dir / f"{_safe_filename(cache_name)}.lock"

    changed = True
    with open(lock_file, "w") as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        try:
            if cache_file.exists():
                previous_hash = cache_file.read_text().strip()
                changed = current_hash != previous_hash
            # Always update the cache with the current hash
            cache_file.write_text(current_hash)
        finally:
            fcntl.flock(lf, fcntl.LOCK_UN)

    return changed


def clear_cache(cache_name: str) -> bool:
    """Remove the cached hash so the next ``did_change`` call returns ``True``.

    Example::

        clear_cache("my_build")
        did_change("my_build", ["/src/main.c"])  # always True after clear
    """
    cache_file = _get_cache_dir() / f"{_safe_filename(cache_name)}.hash"
    if cache_file.exists():
        cache_file.unlink()
        return True
    return False
