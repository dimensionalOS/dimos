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

from dimos.constants import CACHE_DIR


@dataclass(frozen=True)
class CacheIssue:
    """A cache path that could not be removed."""

    path: Path
    reason: str


@dataclass
class CacheCleanResult:
    """Summary returned by :func:`clean_caches`."""

    cleaned: list[Path] = field(default_factory=list)
    failed: list[CacheIssue] = field(default_factory=list)

    @property
    def complete(self) -> bool:
        """Whether every discovered cache path was removed."""
        return not self.failed


def clean_caches() -> CacheCleanResult:
    """Remove all files beneath the DimOS cache root."""
    result = CacheCleanResult()
    target = CACHE_DIR.absolute()
    if not _lexists(target):
        return result

    try:
        _remove_path(target)
    except OSError as error:
        result.failed.append(CacheIssue(target, str(error)))
    else:
        result.cleaned.append(target)
    return result


def _remove_path(path: Path) -> None:
    if path.is_symlink() or not path.is_dir():
        path.unlink()
    else:
        shutil.rmtree(path)


def _lexists(path: Path) -> bool:
    return os.path.lexists(path)
