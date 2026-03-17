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
"""

from __future__ import annotations

from collections.abc import Sequence
import glob as glob_mod
import os
from pathlib import Path

import xxhash

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _get_cache_dir() -> Path:
    """Return the directory used to store change-detection cache files.

    Uses ``<VIRTUAL_ENV>/dimos_cache/change_detect/`` when running inside a
    venv, otherwise falls back to ``~/.cache/dimos/change_detect/``.
    """
    venv = os.environ.get("VIRTUAL_ENV")
    if venv:
        return Path(venv) / "dimos_cache" / "change_detect"
    return Path.home() / ".cache" / "dimos" / "change_detect"


def _resolve_paths(paths: Sequence[str | Path], cwd: str | Path | None = None) -> list[Path]:
    """Expand globs/directories into a sorted list of individual file paths."""
    files: set[Path] = set()
    for entry in paths:
        entry_str = str(entry)
        # Resolve relative paths against cwd when provided
        if cwd is not None and not Path(entry_str).is_absolute():
            entry_str = str(Path(cwd) / entry_str)
        # Try glob expansion first (handles both glob patterns and plain paths)
        expanded = glob_mod.glob(entry_str, recursive=True)
        if not expanded:
            # Nothing matched — could be a non-existent path or empty glob
            if any(c in entry_str for c in ("*", "?", "[")):
                logger.warning("Glob pattern matched no files", pattern=entry_str)
            else:
                logger.warning("Path does not exist", path=entry_str)
            continue
        for match in expanded:
            p = Path(match)
            if p.is_file():
                files.add(p.resolve())
            elif p.is_dir():
                for root, _dirs, filenames in os.walk(p):
                    for fname in filenames:
                        files.add(Path(root, fname).resolve())
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
    paths: Sequence[str | Path],
    cwd: str | Path | None = None,
) -> bool:
    """Check if any files/dirs matching the given paths have changed since last check.

    Args:
        cache_name: Unique identifier for this cache (e.g. ``"mymodule_build_cache"``).
                    Different cache names track independently.
        paths: List of file paths, directory paths, or glob patterns.
               Directories are walked recursively.
               Globs are expanded with :func:`glob.glob`.
        cwd: Optional working directory for resolving relative paths.

    Returns:
        ``True`` if any file has changed (or if no previous cache exists).
        ``False`` if all files are identical to the cached state.
    """
    if not paths:
        return False

    files = _resolve_paths(paths, cwd=cwd)
    current_hash = _hash_files(files)

    cache_dir = _get_cache_dir()
    cache_file = cache_dir / f"{cache_name}.hash"

    changed = True
    if cache_file.exists():
        previous_hash = cache_file.read_text().strip()
        changed = current_hash != previous_hash

    # Always update the cache with the current hash
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_file.write_text(current_hash)

    return changed


def clear_cache(cache_name: str) -> bool:
    """Remove the cached hash for the given cache name.

    Returns:
        ``True`` if the cache file existed and was removed.
    """
    cache_file = _get_cache_dir() / f"{cache_name}.hash"
    if cache_file.exists():
        cache_file.unlink()
        return True
    return False
