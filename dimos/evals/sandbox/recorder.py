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

"""Every zenoh sample that crosses the sandbox's robot router, in one sqlite file.

The recorder is a client of the router subscribed to ``**``, so it sees both what
the robot publishes and what the agent sends it. Payloads are stored as raw bytes;
:class:`Recording` reads them back and decodes dimOS messages from the type their
key carries (``dimos/odom/geometry_msgs.PoseStamped``).

    python -m dimos.evals.sandbox.recorder tcp/robot:7447 /out/zenoh.db
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import json
from pathlib import Path
import queue
import signal
import sqlite3
import sys
import threading
import time
from types import FrameType
from typing import Any

import zenoh

SCHEMA = """CREATE TABLE IF NOT EXISTS samples (
    ts REAL NOT NULL,      -- unix seconds when the recorder received it
    key TEXT NOT NULL,
    kind TEXT NOT NULL,    -- PUT or DELETE
    encoding TEXT NOT NULL,
    payload BLOB NOT NULL,
    attachment BLOB
)"""
# Seconds between commits; a stopped recorder commits what is left.
COMMIT_S = 0.5
# Printed once subscribed, then once per new key, so the runner can tell when the robot is up.
RECORDING = "recording"
NEW_KEY = "new key: "


def record(connect: str, path: Path, stop: threading.Event) -> None:
    """Record until *stop* is set. Prints each key the first time it is seen."""
    samples: queue.Queue[tuple[Any, ...]] = queue.Queue()

    def on_sample(sample: zenoh.Sample) -> None:
        attachment = sample.attachment.to_bytes() if sample.attachment is not None else None
        samples.put(
            (
                time.time(),
                str(sample.key_expr),
                "PUT" if sample.kind == zenoh.SampleKind.PUT else "DELETE",
                str(sample.encoding),
                sample.payload.to_bytes(),
                attachment,
            )
        )

    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([connect]))
    db = sqlite3.connect(path)
    db.execute(SCHEMA)
    seen: set[str] = set()
    with zenoh.open(config) as session:
        subscriber = session.declare_subscriber("**", on_sample)  # undeclared once dropped
        print(RECORDING, flush=True)
        while True:
            stopping = stop.wait(COMMIT_S)
            rows = []
            while not samples.empty():
                rows.append(samples.get_nowait())
            for key in {row[1] for row in rows} - seen:
                seen.add(key)
                print(NEW_KEY + key, flush=True)
            db.executemany("INSERT INTO samples VALUES (?, ?, ?, ?, ?, ?)", rows)
            db.commit()
            if stopping:
                break
        subscriber.undeclare()
    db.close()


@dataclass(frozen=True)
class Sample:
    ts: float
    key: str
    kind: str
    encoding: str
    payload: bytes
    attachment: bytes | None

    def decode(self) -> Any:
        """The dimOS message, when the key names its type; JSON payloads parsed; else bytes."""
        from dimos.protocol.pubsub.impl.zenohpubsub import _key_expr_to_topic

        lcm_type = _key_expr_to_topic(self.key).lcm_type
        if lcm_type is not None:
            return lcm_type.lcm_decode(self.payload)
        try:
            return json.loads(self.payload)
        except ValueError:
            return self.payload


class Recording:
    """A finished recording, read back for grading."""

    def __init__(self, path: Path) -> None:
        if not path.is_file():
            raise FileNotFoundError(path)
        self.path = path

    def keys(self) -> list[str]:
        with sqlite3.connect(self.path) as db:
            return [k for (k,) in db.execute("SELECT DISTINCT key FROM samples ORDER BY key")]

    def samples(self, key_expr: str = "**") -> Iterator[Sample]:
        """Samples whose key matches the zenoh key expression, oldest first."""
        wanted = zenoh.KeyExpr(key_expr)
        keys = [k for k in self.keys() if wanted.intersects(zenoh.KeyExpr(k))]
        if not keys:
            return
        marks = ",".join("?" * len(keys))
        with sqlite3.connect(self.path) as db:
            yield from (
                Sample(*row)
                for row in db.execute(
                    f"SELECT * FROM samples WHERE key IN ({marks}) ORDER BY ts", keys
                )
            )

    def last(self, key_expr: str) -> Sample | None:
        return next(iter(reversed(list(self.samples(key_expr)))), None)


if __name__ == "__main__":
    stop = threading.Event()

    def on_signal(signum: int, frame: FrameType | None) -> None:
        stop.set()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    record(sys.argv[1], Path(sys.argv[2]), stop)
