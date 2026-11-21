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

"""Copy observation metadata and stored payloads between SQLite recordings."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    from dimos.memory.backend import Backend
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.memory.type.observation import Observation


@contextmanager
def new_recording(path: str | Path) -> Iterator[SqliteStore]:
    """Create the recording a copy is written to. *path* must not exist yet.

    A copy that fails part-way is removed: left behind, it would pass for a
    recording and block the retry, which refuses an existing destination.
    """
    from dimos.memory.store.sqlite import SqliteStore

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Claim the filename before opening SQLite so only its creator can roll back.
    destination.touch(mode=0o644, exist_ok=False)
    try:
        with SqliteStore(path=str(destination)) as store:
            yield store
    except BaseException:
        for suffix in ("-wal", "-shm", ""):
            Path(f"{destination}{suffix}").unlink(missing_ok=True)
        raise


def copy_recorded_stream(
    source: SqliteStore,
    name: str,
    destination: SqliteStore,
    output_name: str,
    rows: Iterable[Observation[Any]],
    *,
    verify: bool = False,
) -> int:
    """Copy selected rows, preserving inline values, original encoded bytes and embeddings.

    Rows may have updated metadata but must retain their source IDs and payloads.
    """
    src = cast("Backend[Any]", source.stream(name)._source)
    dst = cast(
        "Backend[Any]", destination.stream(output_name, src.data_type, codec=src.codec)._source
    )
    count = 0
    for obs in rows:
        # The SQLite observation reader returns numbers inline. Other values
        # live in the source blob store, which can be a separate file.
        blob = None
        if not isinstance(obs._data, (int, float)) and src.blob_store is not None:
            blob = src.blob_store.get(name, obs.id)
            if verify:
                src.codec.decode(blob)
        row_id = dst.metadata_store.insert(obs)
        if blob is not None:
            assert dst.blob_store is not None
            dst.blob_store.put(output_name, row_id, blob)
        if src.vector_store is not None:
            embedding = src.vector_store.get(name, obs.id)
            if embedding is not None:
                assert dst.vector_store is not None
                dst.vector_store.put(output_name, row_id, embedding)
        count += 1
    commit = getattr(dst.metadata_store, "commit", None)
    if commit is not None:
        commit()
    return count
