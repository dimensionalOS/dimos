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

"""Resolve a recording db path, falling back to an LFS pull when it is absent."""

from __future__ import annotations

from pathlib import Path

from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def resolve_db_path(db_path: str | Path) -> Path:
    """Return an existing db path, pulling it from LFS by name when missing.

    Recording dbs live in Git LFS as `<name>.tar.gz` archives (e.g.
    `china_office.db.tar.gz`). If the given path already exists on disk it is
    returned untouched; otherwise `get_data` pulls and decompresses the archive
    whose name matches the path's final component.
    """
    path = Path(db_path).expanduser()
    if path.exists():
        return path
    logger.info(f"db '{path}' not found locally, pulling '{path.name}' from LFS")
    return get_data(path.name)
