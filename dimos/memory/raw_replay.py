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

"""Read a recording without the observation store.

The store writes observation tags as **JSONB** and reads them back with
``json(tags)``. JSONB arrived in SQLite 3.45. Anywhere older -- the R1 Pro's
Python carries 3.37.2 -- every query raises ``malformed JSON`` before it reaches
a single payload, so dimos cannot read a recording its own Rust recorder just
wrote, on the machine that wrote it.

Nothing is wrong with the file. `ts`, the payload blobs and the stream config
are all plain columns, and reading them directly works on any SQLite. This is
that direct path, for tools that only need to sample a recording: given a
timestamp, hand back the nearest message.

It is deliberately not a Store. No writing, no tags, no spatial queries, no
vectors -- if you want those, you want the real thing on a newer SQLite. The
narrowness is the point: it keeps a workaround for one SQLite version out of
the store that everything else depends on.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import json
from pathlib import Path
import sqlite3
from typing import Any

# Only a codec that is actually a self-describing byte string can be decoded
# here. `lcm` is what both recorders write.
SUPPORTED_CODEC = "lcm"


def _load_payload_type(module_path: str) -> type[Any]:
    """Turn ``a.b.C.C`` from the stream config into the class itself."""
    module_name, _, class_name = module_path.rpartition(".")
    if not module_name:
        raise ValueError(f"cannot read a payload type out of {module_path!r}")
    payload_type: type[Any] = getattr(importlib.import_module(module_name), class_name)
    return payload_type


@dataclass(frozen=True)
class RawStream:
    """One stream of a recording, read straight from its tables."""

    name: str
    connection: sqlite3.Connection
    payload_type: type[Any]
    stamps: tuple[float, ...]
    ids: tuple[int, ...]

    def __len__(self) -> int:
        return len(self.stamps)

    def first_ts(self) -> float | None:
        return self.stamps[0] if self.stamps else None

    def last_ts(self) -> float | None:
        return self.stamps[-1] if self.stamps else None

    def _decode(self, row_id: int) -> Any:
        row = self.connection.execute(
            f'SELECT data FROM "{self.name}_blob" WHERE id = ?', (row_id,)
        ).fetchone()
        if row is None or row[0] is None:
            return None
        return self.payload_type.lcm_decode(bytes(row[0]))

    def _index_near(self, ts: float) -> int | None:
        """Index of the stamp nearest *ts*, by binary search."""
        import bisect

        if not self.stamps:
            return None
        position = bisect.bisect_left(self.stamps, ts)
        candidates = [i for i in (position - 1, position) if 0 <= i < len(self.stamps)]
        if not candidates:
            return None
        return min(candidates, key=lambda i: abs(self.stamps[i] - ts))

    def find_closest(self, ts: float, tolerance: float = 1.0) -> Any:
        """The message nearest *ts*, or None if the nearest is further than that."""
        index = self._index_near(ts)
        if index is None or abs(self.stamps[index] - ts) > tolerance:
            return None
        return self._decode(self.ids[index])

    def first(self) -> Any:
        return self._decode(self.ids[0]) if self.ids else None

    def iterate(self, until_ts: float | None = None) -> Any:
        """Every message in order. Decodes one at a time: a recording's streams
        are gigabytes, and the callers here want a handful of samples.

        *until_ts* stops at that stamp. The cut is made on the stamp index
        rather than on the decoded message, so the rows past it are never
        decoded at all -- decoding is the cost here, and a caller reading the
        first three hours of a fourteen-hour recording should pay for three."""
        for stamp, row_id in zip(self.stamps, self.ids, strict=True):
            if until_ts is not None and stamp > until_ts:
                return
            message = self._decode(row_id)
            if message is not None:
                yield message


class RawRecording:
    """A recording opened read-only, bypassing the observation store."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(self.path)
        # Read-only, and not `immutable`: a recording being written right now is
        # exactly when you want to look at one, and immutable would hide every
        # row the writer has added since.
        self.connection = sqlite3.connect(f"file:{self.path}?mode=ro", uri=True)
        self._streams: dict[str, RawStream] = {}

    def close(self) -> None:
        self.connection.close()

    def __enter__(self) -> RawRecording:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def list_streams(self) -> list[str]:
        return [row[0] for row in self.connection.execute("SELECT name FROM _streams")]

    def stream(self, name: str) -> RawStream:
        if name in self._streams:
            return self._streams[name]
        row = self.connection.execute(
            "SELECT config FROM _streams WHERE name = ?", (name,)
        ).fetchone()
        if row is None:
            raise KeyError(f"{self.path.name} has no stream {name!r}")
        config = json.loads(row[0])
        codec = config.get("codec_id")
        if codec != SUPPORTED_CODEC:
            raise ValueError(
                f"stream {name!r} is encoded as {codec!r}; this reader only decodes "
                f"{SUPPORTED_CODEC!r}"
            )
        payload_type = _load_payload_type(config["payload_module"])
        rows = self.connection.execute(f'SELECT id, ts FROM "{name}" ORDER BY ts').fetchall()
        built = RawStream(
            name=name,
            connection=self.connection,
            payload_type=payload_type,
            stamps=tuple(float(r[1]) for r in rows),
            ids=tuple(int(r[0]) for r in rows),
        )
        self._streams[name] = built
        return built


def tf_buffer(
    recording: RawRecording,
    stream: str = "tf",
    buffer_size: float = 1e9,
    *,
    until_ts: float | None = None,
) -> Any:
    """Every transform in the recording, in a buffer that can be looked up.

    The default window is effectively unbounded, because this replays a whole
    recording at once rather than following a live robot: pruning to the last
    ten seconds would throw away the transforms for every sample but the last.

    *until_ts* stops reading there. A caller that is only scoring the first
    three hours of a fourteen-hour recording has no use for the other eleven,
    and the difference is not small: the R1 Pro recording carries ten million
    tf rows and holding all of them costs about 9 GB, which is a third of the
    robot's memory and enough that two of these at once leaves nothing for the
    stack that is actually driving the robot. A small margin past the last
    sample is the caller's business, since lookups interpolate.
    """
    from dimos.protocol.tf.tf import MultiTBuffer

    buffer = MultiTBuffer(buffer_size)
    for message in recording.stream(stream).iterate(until_ts=until_ts):
        buffer.receive_transform(*message.transforms)
    return buffer
