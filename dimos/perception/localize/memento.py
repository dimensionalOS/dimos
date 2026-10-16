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

A window is answered only from sightings inside it. A span a label has
already read under the same read params is never read again: the window takes
its sightings from here, and only the window's unread gap goes to the reader.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
import sqlite3
import threading
from typing import Any

import numpy as np

Span = tuple[float, float]  # closed [start, end], seconds

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
CREATE TABLE IF NOT EXISTS spans (
    label TEXT NOT NULL,
    key TEXT NOT NULL,
    start REAL NOT NULL,
    end REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS spans_of_label ON spans (label, key);
"""


@dataclass
class Sighting:
    """One lifted detection of a label: its cloud, when, from where, and the detector's score."""

    ts: float
    camera: str
    score: float
    points: np.ndarray  # (n, 3) world frame, held as float32
    viewpoint: tuple[float, float, float]  # camera position, world frame
    id: int = -1  # row id, set once stored
    detection: Any = None  # the lifted detection; this process only, never stored

    def __post_init__(self) -> None:
        self.points = np.ascontiguousarray(self.points, dtype=np.float32).reshape(-1, 3)


@dataclass(frozen=True)
class Gap:
    """The part of a window a label has not read: ``spans``, less the closed ``read`` spans."""

    spans: tuple[Span, ...]
    read: tuple[Span, ...]

    def __bool__(self) -> bool:
        return bool(self.spans)

    def __contains__(self, ts: object) -> bool:
        t = float(ts)  # type: ignore[arg-type]
        return any(a <= t <= b for a, b in self.spans) and not any(
            a <= t <= b for a, b in self.read
        )

    @property
    def seconds(self) -> float:
        return sum(b - a for a, b in self.spans)


Reader = Callable[[dict[str, Gap]], Mapping[str, Iterable[Sighting]]]


class Memento:
    """Sightings by label and read params in sqlite, with the spans each label has read.

    In memory by default; ``dump`` and ``load`` move it to and from a file. One
    lock serializes recalls, so two windows never read the same span twice.
    """

    def __init__(self, path: str | Path = ":memory:") -> None:
        self._db = sqlite3.connect(str(path), check_same_thread=False)
        self._db.executescript(_SCHEMA)
        self._lock = threading.Lock()

    def recall(
        self, labels: list[str], key: str, window: Span, head: float, read: Reader
    ) -> tuple[dict[str, list[Sighting]], dict[str, Gap]]:
        """Each label's sightings inside ``window``, reading only its unread gap.

        ``read`` gets the gap of every label that has one, up to ``head``, the
        newest frame there is, and returns the sightings it lifted, each inside
        its label's gap. They are stored and the window up to ``head`` marked
        read in one transaction, so a failed read leaves nothing behind and a
        frame that arrives later is read by a later call.
        """
        with self._lock:
            end = min(window[1], head)
            gaps = {label: self._gap(label, key, (window[0], end)) for label in labels}
            todo = {label: gap for label, gap in gaps.items() if gap}
            fresh = {label: list(found) for label, found in read(todo).items()} if todo else {}
            with self._db:
                for label, found in fresh.items():
                    for sighting in found:
                        if label not in todo or sighting.ts not in todo[label]:
                            raise ValueError(
                                f"{label!r} sighting at {sighting.ts} lies outside its unread gap"
                            )
                        sighting.id = self._insert(label, key, sighting)
                if window[0] <= end:
                    for label in todo:
                        self._mark(label, key, (window[0], end))
            held = {sighting.id: sighting for found in fresh.values() for sighting in found}
            return {label: self._within(label, key, window, held) for label in labels}, gaps

    def forget(self, before: float) -> None:
        """Drop every sighting older than ``before``, and the read spans with them."""
        with self._lock, self._db:
            self._db.execute("DELETE FROM sightings WHERE ts < ?", (before,))
            self._db.execute("DELETE FROM spans WHERE end < ?", (before,))
            self._db.execute("UPDATE spans SET start = ? WHERE start < ?", (before, before))

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
        source = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        try:
            source.backup(memento._db)
        finally:
            source.close()
        return memento

    def close(self) -> None:
        self._db.close()

    def _gap(self, label: str, key: str, window: Span) -> Gap:
        start, end = window
        read = tuple(
            (float(a), float(b))
            for a, b in self._db.execute(
                "SELECT start, end FROM spans WHERE label = ? AND key = ? AND end >= ? "
                "AND start <= ? ORDER BY start",
                (label, key, start, end),
            )
        )
        if start > end:
            return Gap((), read)
        if not read:
            return Gap(((start, end),), read)
        spans: list[Span] = []
        cursor = start
        for a, b in read:
            if a > cursor:
                spans.append((cursor, min(a, end)))
            cursor = max(cursor, b)
        if cursor < end:
            spans.append((cursor, end))
        return Gap(tuple(spans), read)

    def _mark(self, label: str, key: str, span: Span) -> None:
        start, end = span
        rows = list(
            self._db.execute(
                "SELECT rowid, start, end FROM spans WHERE label = ? AND key = ? AND end >= ? "
                "AND start <= ?",
                (label, key, start, end),
            )
        )
        for _, a, b in rows:
            start, end = min(start, a), max(end, b)
        self._db.executemany("DELETE FROM spans WHERE rowid = ?", [(row,) for row, _, _ in rows])
        self._db.execute(
            "INSERT INTO spans (label, key, start, end) VALUES (?, ?, ?, ?)",
            (label, key, start, end),
        )

    def _insert(self, label: str, key: str, sighting: Sighting) -> int:
        vx, vy, vz = sighting.viewpoint
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

    def _within(
        self, label: str, key: str, window: Span, held: dict[int, Sighting]
    ) -> list[Sighting]:
        rows = self._db.execute(
            "SELECT id, ts, camera, score, vx, vy, vz, points FROM sightings "
            "WHERE label = ? AND key = ? AND ts BETWEEN ? AND ? ORDER BY ts, id",
            (label, key, *window),
        )
        return [
            held.get(row_id)
            or Sighting(
                ts=ts,
                camera=camera,
                score=score,
                points=np.frombuffer(blob, dtype=np.float32),
                viewpoint=(vx, vy, vz),
                id=row_id,
            )
            for row_id, ts, camera, score, vx, vy, vz, blob in rows
        ]
