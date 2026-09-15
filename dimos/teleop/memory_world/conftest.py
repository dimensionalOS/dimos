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

"""Fixtures shared by this package's tests.

`memory_world` lived in test_query.py and was imported from there once a second file
needed it. That works, but it reads as a redefinition to any linter and makes one test
module depend on another for a fixture. pytest collects conftest fixtures for every test
beside it, which is what this is for.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.teleop.memory_world.module import MemoryWorldModule

if TYPE_CHECKING:
    from collections.abc import Iterator


def empty_store(path: Path) -> None:
    """An on-disk store with no streams: enough for a module to open and start."""
    store = SqliteStore(path=str(path))
    store.start()
    store.stop()


@pytest.fixture
def memory_world(tmp_path: Path) -> Iterator[MemoryWorldModule]:
    db_path = tmp_path / "recording.db"
    empty_store(db_path)
    module = MemoryWorldModule(store_path=str(db_path))
    yield module
    module.stop()
