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
resolves a dataset name/path (including local recordings and cloud downloads)
and picks the store by file extension: ``.db`` -> SqliteStore, ``.mcap`` ->
McapStore (with a legacy Go2 codec preset when needed). Use :func:`open_store`
when the path is already resolved.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import TYPE_CHECKING

from dimos import constants
from dimos.utils import data

if TYPE_CHECKING:
    from dimos.memory.store.base import Store


class AmbiguousDatasetError(RuntimeError):
    """More than one local recording matches a dataset name."""


def open_store(path: str | Path, *, decode_metadata: bool = False) -> Store:
    """Open an already-resolved dataset *path*, dispatching on its extension.

    ``decode_metadata`` trusts an .mcap channel's ``dimos.payload_type`` (see ``McapStore``).
    """
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
                    channel.message_encoding == "json"
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
        return McapStore(path=s, decode_metadata=decode_metadata)
    if s.endswith(".db"):
        from dimos.memory.store.sqlite import SqliteStore

        return SqliteStore(path=s, must_exist=True)
    raise ValueError(f"unsupported dataset {s!r}: expected a .db or .mcap path")


def resolve_dataset(dataset: str | Path) -> Path:
    """Resolve a path, existing LFS name, recording folder, or downloaded dataset name."""
    name = str(dataset)
    try:
        return data.resolve_named_path(dataset, Path(dataset).suffix or ".db")
    except FileNotFoundError:
        if Path(name).name != name:
            raise

    matches = [
        p
        for p in (constants.RECORDINGS_DIR / name).glob("*")
        if p.is_file() and p.suffix in (".db", ".mcap")
    ]
    for p in constants.DOWNLOADS_DIR.glob("*"):
        if not p.is_file() or p.suffix not in (".db", ".mcap"):
            continue
        aliases = {p.name, p.stem}
        # cloud.data._tag prefixes the filename with an optional timestamp and an ID.
        tagged = re.fullmatch(r"((?:\d{8}-\d{6}-)?([^-]+))-(.+)", p.name)
        if tagged:
            tag, upload_id, filename = tagged.groups()
            aliases.update((tag, filename, Path(filename).stem))
            if upload_id.startswith(name):
                aliases.add(name)
        if name in aliases:
            matches.append(p)
    if len(matches) > 1:
        raise AmbiguousDatasetError(
            f"Ambiguous dataset {name!r}; use an explicit path: "
            + ", ".join(str(p) for p in sorted(matches))
        )
    if matches:
        return matches[0]
    raise FileNotFoundError(
        f"Dataset {name!r} not found. Searched {Path.cwd()}, {constants.DIMOS_PROJECT_ROOT}, "
        f"{data.get_data_dir()} (including LFS), {constants.RECORDINGS_DIR}, "
        f"and {constants.DOWNLOADS_DIR}."
    )


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
