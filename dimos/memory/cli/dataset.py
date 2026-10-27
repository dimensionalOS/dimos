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

"""Central dispatcher for opening a recorded memory dataset as a read store.

One entry point for every CLI that opens a recording. :func:`open_dataset`
resolves a dataset name/path (bare names look up the cwd / repo ``data/`` dir)
and picks the store by file extension: ``.db`` -> SqliteStore, ``.mcap`` ->
McapStore (with a legacy Go2 codec preset when needed). Use :func:`open_store`
when the path is already resolved.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from dimos.utils.data import resolve_named_path

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


def open_store(path: str | Path) -> Store:
    """Open an already-resolved dataset *path*, dispatching on its extension."""
    s = str(path)
    if s.endswith(".mcap"):
        from mcap.reader import make_reader

        from dimos.memory.store.mcap import McapStore

        with open(s, "rb") as recording:
            reader = make_reader(recording)
            summary = reader.get_summary()
            channels = list(summary.channels.values()) if summary else []
            legacy_topics = {
                channel.topic
                for channel in channels
                if (
                    channel.message_encoding == "cdr"
                    and (
                        channel.topic.startswith(("rt/utlidar/", "rt/frontvideo"))
                        or channel.topic in {"rt/lowstate", "rt/lowcmd", "rt/sportmodestate"}
                    )
                )
                or (
                    reader.get_header().profile != "dimos"
                    and channel.message_encoding == "json"
                    and channel.topic in {"telemetry", "control_log"}
                    and "dimos.payload_type" not in channel.metadata
                )
            }
        if legacy_topics:
            # Inject only legacy channels: a mixed file's native channels must
            # still select their own codec from encoding/type metadata.
            from dimos.robot.unitree.go2.dds.codec import GO2_CODECS
            from dimos.robot.unitree.go2.dds.store import STREAMS

            return McapStore(
                path=s,
                codecs={
                    topic: codec for topic, codec in GO2_CODECS.items() if topic in legacy_topics
                },
                streams={name: topic for name, topic in STREAMS.items() if topic in legacy_topics},
            )
        return McapStore(path=s)
    if s.endswith(".db"):
        from dimos.memory.store.sqlite import SqliteStore

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
    out: dict[str, type] = {}
    for name in store.list_streams():
        try:
            out[name] = store.stream(name).data_type or object
        except (ImportError, AttributeError) as e:
            print(f"  skip {name}: payload type unavailable ({e})")
    return out
