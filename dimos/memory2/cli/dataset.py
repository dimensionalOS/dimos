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

"""Central dispatcher for opening a recorded memory2 dataset as a read store.

One entry point for every CLI that opens a recording. :func:`open_dataset`
resolves a dataset name/path (bare names look up the cwd / repo ``data/`` dir)
and picks the store by file extension: ``.db`` -> SqliteStore, ``.mcap`` ->
Go2McapStore. Use :func:`open_store` when the path is already resolved.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from dimos.utils.data import resolve_named_path

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store


def open_store(path: str | Path) -> Store:
    """Open an already-resolved dataset *path*, dispatching on its extension."""
    s = str(path)
    if s.endswith(".mcap"):
        from dimos.robot.unitree.go2.dds.store import Go2McapStore  # lazy: robot-layer codecs

        return Go2McapStore(path=s)
    if s.endswith(".db"):
        from dimos.memory2.store.sqlite import SqliteStore

        return SqliteStore(path=s, must_exist=True)
    raise ValueError(f"unsupported dataset {s!r}: expected a .db or .mcap path")


def resolve_dataset(dataset: str | Path) -> Path:
    """Resolve a dataset name/path to a file (bare names -> ``.db``, cwd / data/)."""
    return resolve_named_path(dataset, Path(dataset).suffix or ".db")


def open_dataset(dataset: str | Path) -> Store:
    """Resolve a dataset name/path (bare names -> ``.db``) and open it read-only."""
    return open_store(resolve_dataset(dataset))


def stream_payload_types(store: Store) -> dict[str, type]:
    """Map each stream name in *store* to its payload type (any backend)."""
    return {name: store.stream(name).data_type or object for name in store.list_streams()}
