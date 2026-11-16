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

"""Memento: what localize has read, by label, read params and time.

A window is answered only from sightings inside it. A frame a label has
already detected under the same read params is never detected again: the
window takes its sightings from here, and the reader detects only its unread
frames.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
import sqlite3
import threading
from typing import Any, cast

import numpy as np

from dimos.msgs.geometry_msgs.Vector3 import Vector3

Span = tuple[float, float]  # closed [start, end], seconds
FrameSpan = tuple[str, float, float]  # (camera, start, end), every frame in it detected

_SCHEMA = """
CREATE TABLE IF NOT EXISTS sightings (
    id INTEGER PRIMARY KEY,
    label TEXT NOT NULL,
    key TEXT NOT NULL,
    ts REAL NOT NULL,
    camera TEXT NOT NULL,
    score REAL NOT NULL,
    vx REAL NOT NULL,
    vy REAL NOT NULL,
    vz REAL NOT NULL,
    points BLOB NOT NULL
);
CREATE INDEX IF NOT EXISTS sightings_in_time ON sightings (label, key, ts);
CREATE TABLE IF NOT EXISTS frame_spans (
    label TEXT NOT NULL,
    key TEXT NOT NULL,
    camera TEXT NOT NULL,
    start REAL NOT NULL,
    end REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS frame_spans_of_label ON frame_spans (label, key, camera);
"""


@dataclass
class Sighting:
    """One lifted detection of a label: its cloud, when, from where, and the detector's score."""

    ts: float
    camera: str
    score: float
    points: np.ndarray  # (n, 3) world frame, held as float32
    viewpoint: Vector3  # camera position, world frame
    id: int = -1  # row id, set once stored
    detection: Any = None  # the lifted detection; this process only, never stored

    def __post_init__(self) -> None:
        self.ts = float(self.ts)
        self.score = float(self.score)
        self.viewpoint = Vector3(self.viewpoint)
        self.points = np.ascontiguousarray(self.points, dtype=np.float32).reshape(-1, 3)


@dataclass(frozen=True)
class Gap:
    """The part of a window a label has not read: ``spans``, less the ``read`` frame spans."""

    spans: tuple[Span, ...]
    read: tuple[FrameSpan, ...]

    def __bool__(self) -> bool:
        return bool(self.spans)

    def __contains__(self, frame: object) -> bool:
        camera, ts = cast("tuple[str, float]", frame)
        t = float(ts)
        return any(a <= t <= b for a, b in self.spans) and not any(
            c == camera and a <= t <= b for c, a, b in self.read
        )


# per label: the frame spans it detected, and the sightings lifted from them
Reader = Callable[[dict[str, Gap]], Mapping[str, tuple[Iterable[FrameSpan], Iterable[Sighting]]]]


class Memento:
    """Sightings by label and read params in sqlite, with the frame spans each label has read.

    In memory by default; ``dump`` and ``load`` move it to and from a file. One
    lock serializes recalls, so two windows never detect the same frame twice.
    """

    def __init__(self, path: str | Path = ":memory:") -> None:
        self._db = sqlite3.connect(str(path), check_same_thread=False)
        self._db.executescript(_SCHEMA)
        self._lock = threading.Lock()
        self._held: dict[int, Sighting] = {}

    def recall(
        self, labels: list[str], key: str, window: Span, head: float, read: Reader
    ) -> tuple[dict[str, list[Sighting]], dict[str, Gap]]:
        """Each label's sightings inside ``window``, reading only its unread gap.

        ``read`` gets the gap of every label that has one, up to ``head``, the
        newest frame there is, and returns the frame spans it detected and the
        sightings it lifted from them. They are stored and the frame spans
        marked read in one transaction, so a failed read leaves nothing behind.
        """
        with self._lock:
            window = (float(window[0]), float(window[1]))
            end = min(window[1], float(head))
            gaps = {label: self._gap(label, key, (window[0], end)) for label in labels}
            todo = {label: gap for label, gap in gaps.items() if gap}
            done = read(todo) if todo else {}
            fresh: dict[str, list[Sighting]] = {}
            with self._db:
                for label, (spans, found) in done.items():
                    if label not in todo:
                        raise ValueError(f"{label!r} has no unread gap")
                    spans = [(str(c), float(a), float(b)) for c, a, b in spans]
                    fresh[label] = list(found)
                    for sighting in fresh[label]:
                        if (sighting.camera, sighting.ts) not in todo[label] or not any(
                            c == sighting.camera and a <= sighting.ts <= b for c, a, b in spans
                        ):
                            raise ValueError(
                                f"{label!r} sighting at {sighting.ts} lies outside its detected frames"
                            )
                        sighting.id = self._insert(label, key, sighting)
                    for camera, a, b in spans:
                        self._mark(label, key, camera, (a, b))
            for found in fresh.values():
                for sighting in found:
                    if sighting.detection is not None:
                        self._held[sighting.id] = sighting
            return {label: self._within(label, key, window) for label in labels}, gaps

    def forget(self, before: float) -> None:
        """Drop every sighting older than ``before``, and the read spans with them."""
        before = float(before)
        with self._lock, self._db:
            dropped = [
                row_id
                for (row_id,) in self._db.execute(
                    "SELECT id FROM sightings WHERE ts < ?", (before,)
                )
            ]
            self._db.execute("DELETE FROM sightings WHERE ts < ?", (before,))
            self._db.execute("DELETE FROM frame_spans WHERE end < ?", (before,))
            self._db.execute("UPDATE frame_spans SET start = ? WHERE start < ?", (before, before))
            for row_id in dropped:
                self._held.pop(row_id, None)

    def dump(self, path: str | Path) -> None:
        """Write everything to a sqlite file at ``path``."""
        target = sqlite3.connect(str(path))
        try:
            with self._lock:
                self._db.backup(target)
        finally:
            target.close()

    @classmethod
    def load(cls, path: str | Path) -> Memento:
        """An in-memory memento holding what ``dump`` wrote to ``path``."""
        memento = cls()
        uri = f"{Path(path).resolve().as_uri()}?mode=ro"
        source = sqlite3.connect(uri, uri=True)
        try:
            source.backup(memento._db)
        finally:
            source.close()
        return memento

    def close(self) -> None:
        with self._lock:
            self._held.clear()
            self._db.close()

    def _gap(self, label: str, key: str, window: Span) -> Gap:
        start, end = float(window[0]), float(window[1])
        if start > end:
            return Gap((), ())
        read = tuple(
            (str(c), float(a), float(b))
            for c, a, b in self._db.execute(
                "SELECT camera, start, end FROM frame_spans WHERE label = ? AND key = ? "
                "AND end >= ? AND start <= ? ORDER BY start",
                (label, key, start, end),
            )
        )
        return Gap(((start, end),), read)

    def _mark(self, label: str, key: str, camera: str, span: Span) -> None:
        start, end = span
        rows = list(
            self._db.execute(
                "SELECT rowid, start, end FROM frame_spans WHERE label = ? AND key = ? "
                "AND camera = ? AND end >= ? AND start <= ?",
                (label, key, camera, start, end),
            )
        )
        for _, a, b in rows:
            start, end = min(start, a), max(end, b)
        self._db.executemany(
            "DELETE FROM frame_spans WHERE rowid = ?", [(row,) for row, _, _ in rows]
        )
        self._db.execute(
            "INSERT INTO frame_spans (label, key, camera, start, end) VALUES (?, ?, ?, ?, ?)",
            (label, key, camera, start, end),
        )

    def _insert(self, label: str, key: str, sighting: Sighting) -> int:
        vx, vy, vz = sighting.viewpoint.to_tuple()
        cursor = self._db.execute(
            "INSERT INTO sightings (label, key, ts, camera, score, vx, vy, vz, points) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (
                label,
                key,
                sighting.ts,
                sighting.camera,
                sighting.score,
                vx,
                vy,
                vz,
                sighting.points.tobytes(),
            ),
        )
        assert cursor.lastrowid is not None
        return cursor.lastrowid

    def _within(self, label: str, key: str, window: Span) -> list[Sighting]:
        rows = self._db.execute(
            "SELECT id, ts, camera, score, vx, vy, vz, points FROM sightings "
            "WHERE label = ? AND key = ? AND ts BETWEEN ? AND ? ORDER BY ts, id",
            (label, key, *window),
        )
        return [
            self._held.get(row_id)
            or Sighting(
                ts=ts,
                camera=camera,
                score=score,
                points=np.frombuffer(blob, dtype=np.float32),
                viewpoint=Vector3(vx, vy, vz),
                id=row_id,
            )
            for row_id, ts, camera, score, vx, vy, vz, blob in rows
        ]
