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

"""Print ``Store.summary()`` for a memory2 sqlite recording.

Usage:
    uv run dimos mem summary mid360
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import typer

if TYPE_CHECKING:
    from pathlib import Path

# Heavy dimos imports (memory2 store → codecs, msgs) are deferred into the
# function bodies so that `dimos --help` — which imports this module just to
# register the `mem summary` command — stays fast. See test_cli_startup.py.


def _stream_payload_types(db_path: Path) -> dict[str, type]:
    """Read each stream's registered payload type from the _streams table."""
    import json
    import sqlite3

    from dimos.memory2.codecs.base import _resolve_payload_type

    conn = sqlite3.connect(str(db_path))
    try:
        rows = conn.execute("SELECT name, config FROM _streams").fetchall()
    finally:
        conn.close()
    return {name: _resolve_payload_type(json.loads(cfg)["payload_module"]) for name, cfg in rows}


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
) -> None:
    """Print per-stream counts, time ranges, and payload sizes for a recorded SQLite dataset."""
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import resolve_named_path

    db_path = resolve_named_path(dataset, ".db")
    payload_types = _stream_payload_types(db_path)

    store = SqliteStore(path=str(db_path))
    with store:
        for name, ptype in payload_types.items():
            store.stream(name, ptype)
        print(store.summary())


if __name__ == "__main__":
    typer.run(main)
