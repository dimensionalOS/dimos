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

"""The patch index held in memory, because sqlite is what a query spends its time on.

Measured on sf_office's so400m index, 886,032 patches: the vector search took 3.2 s and
reading the winning vectors back another 2.6 s, while the arithmetic those five seconds
exist to perform is 58 ms. The index is not big -- it is only *far away*. Held in an
array it answers exactly, with no top-k cap and no approximation, in the time the
maths takes.

The cost is moved rather than removed: reading a model's vectors out of vec0 is minutes,
because a virtual table hands back one row at a time. That is paid once, at startup or
as a recording is ingested, which is the trade this module exists to make.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

# Rows pulled from the vector table per round trip. Only bounds peak memory during the
# load; the table is read start to finish either way.
READ_CHUNK = 50_000

# Rows promoted to single precision at a time when scoring. Big enough that the matrix
# multiply is worth starting, small enough that the promotion is half a gigabyte.
SCORE_CHUNK = 100_000

# What the vectors are held as. Single precision, because that is what the machine
# multiplies: half precision halves the index -- grocery's three models are 29 GB
# against 14 -- but CPUs have no half-precision arithmetic and BLAS no half-precision
# path, so every block has to be promoted before it can be multiplied. Measured on
# grocery, that promotion is about a second a query. Memory is the cheaper side of
# that trade here; on a machine where it is not, this is the line to change.
HELD_AS = np.float32


@dataclass
class ResidentPatches:
    """One model's patches: the vectors, and the little that placing them needs.

    Everything is parallel by row, so a patch is an index into all of them at once and
    scoring is one matrix multiply over the whole index.
    """

    tag: str
    stream: str
    vectors: NDArray[Any]
    camera_frames: list[str]
    frame_of: NDArray[np.int32]
    ts: NDArray[np.float64]
    cell: NDArray[np.int32]
    grid: NDArray[np.int16]
    ray: NDArray[np.float32]
    depth: NDArray[np.float32]
    # Live, the arrays are over-allocated and only the first `count` rows are patches.
    # -1 means "the whole array", which is what a read of a finished recording gives.
    count: int = -1
    # The last row id taken from the stream, so a later read can ask for what is new
    # rather than for everything again.
    last_id: int = 0

    @property
    def rows(self) -> int:
        return int(self.vectors.shape[0]) if self.count < 0 else self.count

    @property
    def capacity(self) -> int:
        return int(self.vectors.shape[0])

    def extend(self, block: ResidentPatches) -> int:
        """Append another read's rows. Returns how many landed.

        Capacity doubles rather than growing to fit, because a live index is appended
        to several times a second and copying the whole array each time is quadratic:
        a seven gigabyte index would spend its life memcpying itself.
        """
        added = block.rows
        if not added:
            return 0
        if self.count < 0:
            self.count = int(self.vectors.shape[0])
        wanted = self.count + added
        if wanted > self.capacity:
            self._reserve(max(wanted, max(self.capacity * 2, 1024)))
        # Two reads name their camera frames independently, so the block's frame ids
        # are re-pointed at this index's table rather than trusted.
        mapping = {}
        for at, name in enumerate(block.camera_frames):
            if name not in self.camera_frames:
                self.camera_frames.append(name)
            mapping[at] = self.camera_frames.index(name)
        end = self.count + added
        self.vectors[self.count : end] = block.vectors[:added]
        self.frame_of[self.count : end] = [mapping[int(v)] for v in block.frame_of[:added]]
        self.ts[self.count : end] = block.ts[:added]
        self.cell[self.count : end] = block.cell[:added]
        self.grid[self.count : end] = block.grid[:added]
        self.ray[self.count : end] = block.ray[:added]
        self.depth[self.count : end] = block.depth[:added]
        self.count = end
        self.last_id = max(self.last_id, block.last_id)
        return added

    def _reserve(self, capacity: int) -> None:
        def grown(array: NDArray[Any]) -> NDArray[Any]:
            shape = (capacity, *array.shape[1:])
            out = np.empty(shape, dtype=array.dtype)
            out[: self.count] = array[: self.count]
            return out

        self.vectors = grown(self.vectors)
        self.frame_of = grown(self.frame_of)
        self.ts = grown(self.ts)
        self.cell = grown(self.cell)
        self.grid = grown(self.grid)
        self.ray = grown(self.ray)
        self.depth = grown(self.depth)

    @property
    def width(self) -> int:
        return int(self.vectors.shape[1])

    @property
    def megabytes(self) -> float:
        return float(self.vectors.nbytes) / 1e6

    def scores(
        self,
        query: NDArray[np.float32],
        background: NDArray[np.float32],
        rows: NDArray[np.intp] | None = None,
    ) -> NDArray[np.float32]:
        """Every patch's contrast against the query: how much better than generic room.

        Exact, over every patch, rather than over whatever an approximate index would
        have returned.

        *rows* narrows that to a subset, and the answer is then as long as *rows* rather
        than as long as the index. The cost of a search is reading the vectors, not the
        handful of dot products per row, so scoring a tenth of the rows costs about a
        tenth as much -- which is the whole point of `rank_with` in `hot_frames`.

        Done a block at a time, and promoted to single precision if it is not already
        stored that way. Numpy has no fast half-precision matrix multiply -- it falls
        back to something thirty times slower, measured, which once turned a 0.2 s
        search into 18 s -- so a half-precision index has to be promoted to be usable
        at all, and blocks keep that from costing the whole index in memory at once.
        At single precision the block is a view and the loop costs nothing.

        Passing no background rows turns the contrast off and leaves the plain
        similarity to the query. That is worth having as a comparison -- the contrast is
        a correction for CLIP scoring almost everything somewhat highly, and what it
        subtracts is a guess at what "generic" looks like -- but it is on by default
        because without it a wall answers most questions moderately well.
        """
        # The query and the backgrounds go through together: one pass over the index
        # rather than one for the words and another for the room.
        background = np.asarray(background, dtype=np.float32).reshape(-1, len(query))
        texts = np.vstack([np.asarray(query, dtype=np.float32)[None, :], background])
        count = self.rows if rows is None else len(rows)
        out = np.empty(count, dtype=np.float32)
        at = 0
        for first, last in _spans(rows, self.rows):
            # SLICES, not a fancy index, and the difference is not small. A gather reads
            # one row at a time wherever they happen to be; a slice streams. MEASURED on
            # the same query, same index, two machines: gathering made CudaLaptop's search
            # go 4.11 s -> 17.20 s, four times SLOWER than reading everything, while the
            # Mac barely noticed (0.81 s -> 0.83 s) because its memory hides random
            # access. The rows wanted here are whole frames' worth of patches, written
            # consecutively at ingest, so they come in long runs and slicing gets the
            # sequential read back.
            for start in range(first, last, SCORE_CHUNK):
                stop = min(start + SCORE_CHUNK, last)
                block = np.asarray(self.vectors[start:stop], dtype=np.float32)
                against = block @ texts.T
                taken = (
                    against[:, 0] - against[:, 1:].max(axis=1) if len(background) else against[:, 0]
                )
                out[at : at + len(taken)] = taken
                at += len(taken)
        return out[:at] if rows is not None else out

    def hot(
        self,
        query: NDArray[np.float32],
        background: NDArray[np.float32],
        *,
        threshold: float,
        limit: int | None = None,
        rows: NDArray[np.intp] | None = None,
    ) -> tuple[NDArray[np.intp], NDArray[np.float32]]:
        """Rows scoring above *threshold*, strongest first, at most *limit* of them.

        *rows* narrows the search to a subset. The indices handed back are into the
        index as a whole either way -- a caller that got back positions within its own
        subset would have to map them home, and that is exactly the kind of bookkeeping
        that puts a patch on the wrong frame.
        """
        found = self.scores(query, background, rows)
        picked = np.flatnonzero(found > threshold)
        order = np.argsort(-found[picked])
        if limit is not None and len(order) > limit:
            order = order[:limit]
        picked = picked[order]
        return (picked if rows is None else rows[picked]), found[picked]


def _spans(rows: NDArray[np.intp] | None, total: int) -> list[tuple[int, int]]:
    """*rows* as half-open contiguous ranges, or the whole index when it is None.

    The row numbers arrive sorted and mostly consecutive -- they are every patch of a
    handful of frames, and a frame's patches were written together. Turning them back
    into runs is what lets the scorer slice instead of gather.
    """
    if rows is None:
        return [(0, total)]
    if not len(rows):
        return []
    breaks = np.flatnonzero(np.diff(rows) != 1)
    starts = np.concatenate(([0], breaks + 1))
    ends = np.concatenate((breaks + 1, [len(rows)]))
    return [(int(rows[a]), int(rows[b - 1]) + 1) for a, b in zip(starts, ends, strict=True)]


def _warn_if_it_will_not_fit(tag: str, rows: int, width: int) -> None:
    """Say so before spending minutes on a read that ends in a swap storm.

    Not a refusal: how much of a machine an index may have is the caller's business,
    and a model that only just fits is a normal thing to want. But finding out by
    watching the machine die is not, so the number goes in the log first.
    """
    wanted = rows * width * HELD_AS().itemsize
    try:
        import psutil

        free = int(psutil.virtual_memory().available)
    except Exception:
        # Whatever went wrong asking, it is not a reason to refuse to load an index.
        return
    if wanted > free * 0.8:
        logger.warning(
            f"hyperspace: {tag} wants {wanted / 1e9:.1f} GB resident and this machine has "
            f"{free / 1e9:.1f} GB free -- expect swapping, or pass --no-resident"
        )


def _from_chunks(conn: Any, stream: str, width: int, rows: int) -> NDArray[Any] | None:
    """Read the vectors out of vec0's chunk storage, or None if that is not possible.

    Fifty times faster than the virtual table -- 0.7 s against 34 s for half a gigabyte
    -- because a chunk is thousands of vectors in one blob where the table hands back
    one row per round trip.

    It is sqlite-vec's own storage and not an interface, so nothing here trusts it:
    the tables have to exist, and the rowid-to-slot map has to say the rows sit in
    order, or this returns None and the slow read stands. A version of sqlite-vec that
    lays them out differently makes this refuse rather than lie.
    """
    chunks = f"{stream}_vec_vector_chunks00"
    for table in (chunks, f"{stream}_vec_chunks", f"{stream}_vec_rowids"):
        found = conn.execute(
            "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?", (table,)
        ).fetchone()
        if not found:
            return None

    sizes = dict(conn.execute(f'SELECT chunk_id, size FROM "{stream}_vec_chunks"'))
    start_of: dict[int, int] = {}
    at = 0
    for chunk_id in sorted(sizes):
        start_of[chunk_id] = at
        at += int(sizes[chunk_id])
    slots = [
        start_of[chunk_id] + offset
        for chunk_id, offset in conn.execute(
            f'SELECT chunk_id, chunk_offset FROM "{stream}_vec_rowids" ORDER BY rowid'
        )
    ]
    if len(slots) != rows or slots != list(range(rows)):
        return None

    out = np.empty((rows, width), dtype=HELD_AS)
    at = 0
    # One chunk at a time, cast as it lands: the whole table in single precision is
    # sixteen gigabytes for a big model, and the point of this is to hold half that.
    for (blob,) in conn.execute(f'SELECT vectors FROM "{chunks}" ORDER BY rowid'):
        block = np.frombuffer(blob, dtype=np.float32).reshape(-1, width)
        taken = min(len(block), rows - at)
        if taken <= 0:
            break
        out[at : at + taken] = block[:taken]
        at += taken
    return out[:at] if at == rows else None


def _vectors_of(conn: Any, stream: str, width: int, rows: int) -> NDArray[Any]:
    """Read a whole vec0 table into one array, in rowid order.

    One row per round trip through the virtual table, which is minutes for a big
    model. Only reached when the chunk storage could not be read.
    """
    out = np.empty((rows, width), dtype=HELD_AS)
    cursor = conn.execute(f'SELECT embedding FROM "{stream}_vec" ORDER BY rowid')
    at = 0
    while at < rows:
        block = cursor.fetchmany(READ_CHUNK)
        if not block:
            break
        taken = min(len(block), rows - at)
        out[at : at + taken] = np.frombuffer(
            b"".join(row[0] for row in block[:taken]), dtype=np.float32
        ).reshape(taken, width)
        at += taken
    return out[:at]


def since(store: Any, tag: str, stream: str, last_id: int) -> ResidentPatches | None:
    """The patches written to this stream after `last_id`, or None if there are none.

    The live counterpart of `load`. An ingest is still writing while queries are being
    asked, so the index has to pick up what has landed since it last looked -- and
    nothing here re-reads what it already holds, which is the whole point.

    The vectors come one row at a time: vec0's chunk storage is only readable whole, and
    a poll's worth of rows is small enough (about six microseconds each) that the fast
    path would be the slow one here.
    """
    backend = store.stream(stream, dict)._source
    blobs, codec = backend.blob_store, backend.codec
    conn = store._registry_conn

    ids = [
        row[0]
        for row in conn.execute(f'SELECT id FROM "{stream}" WHERE id > ? ORDER BY id', (last_id,))
    ]
    if not ids:
        return None
    probe = conn.execute(f'SELECT embedding FROM "{stream}_vec" LIMIT 1').fetchone()
    width = len(np.frombuffer(probe[0], dtype=np.float32))
    # `id` is the stream's row id and `rowid` is the vector table's; they are the same
    # sequence, assigned by the same insert, which is what lets the vectors be asked for
    # by id rather than by position.
    marks = ",".join("?" * len(ids))
    vectors = np.empty((len(ids), width), dtype=HELD_AS)
    cursor = conn.execute(
        f'SELECT embedding FROM "{stream}_vec" WHERE rowid IN ({marks}) ORDER BY rowid', ids
    )
    at = 0
    while at < len(ids):
        block = cursor.fetchmany(READ_CHUNK)
        if not block:
            break
        vectors[at : at + len(block)] = np.frombuffer(
            b"".join(row[0] for row in block), dtype=np.float32
        ).reshape(len(block), width)
        at += len(block)
    if at != len(ids):
        # A patch row without its vector means the two tables disagree, and scoring a
        # half-written block would place patches by the wrong rays.
        logger.warning(f"hyperspace: {tag} has {len(ids)} new rows but {at} new vectors; waiting")
        return None

    patches = _placements(stream, blobs, codec, ids)
    return ResidentPatches(
        tag=tag, stream=stream, vectors=vectors, count=len(ids), last_id=ids[-1], **patches
    )


def _placements(stream: str, blobs: Any, codec: Any, ids: Sequence[int]) -> dict[str, Any]:
    """The little that placing a patch needs, read off the rows' payloads."""
    rows = len(ids)
    names: dict[str, int] = {}
    frame_of = np.empty(rows, dtype=np.int32)
    stamps = np.empty(rows, dtype=np.float64)
    cells = np.empty(rows, dtype=np.int32)
    grids = np.empty((rows, 2), dtype=np.int16)
    rays = np.empty((rows, 2), dtype=np.float32)
    depths = np.empty(rows, dtype=np.float32)
    for at, row_id in enumerate(ids):
        payload = codec.decode(blobs.get(stream, row_id))
        name = str(payload["camera_frame"])
        if name not in names:
            names[name] = len(names)
        frame_of[at] = names[name]
        stamps[at] = float(payload["ts"])
        cells[at] = int(payload["cell"])
        grids[at] = payload["grid"]
        rays[at] = payload["ray"]
        depths[at] = float(payload["depth"])
    return {
        "camera_frames": [name for name, _ in sorted(names.items(), key=lambda kv: kv[1])],
        "frame_of": frame_of,
        "ts": stamps,
        "cell": cells,
        "grid": grids,
        "ray": rays,
        "depth": depths,
    }


def load(store: Any, tag: str, stream: str, stride: int = 1) -> ResidentPatches:
    """Pull one model's patch index into memory, optionally every `stride`-th row.

    Minutes for a big model, and that is the point of doing it once. Vectors come from
    the vector table; the rest comes from the same rows' payloads, which are cheap
    (about six microseconds each) next to the vectors.

    `stride` > 1 keeps every Nth patch. **It buys MEMORY and payload-parsing time, not
    query time** -- measured on roscon, a query is `detect=113 index=0 search=0`, so the
    whole of it is the detector and the search over these vectors is already under a
    second. What it does buy is real on a machine that is swapping: 1.9M patches at
    768-wide fp32 is 5.9 GB across two members, and halving that took this Mac off swap.
    The rows are read either way -- the vector table is read in chunks and there is no
    cheap way to ask it for every second row -- so the read time is unchanged.
    """
    # Reaching past the Stream for the backend and the connection. There is no public
    # way to read a whole vector table or to fetch a payload by id -- `Stream.filter` is
    # a python predicate over everything, which is worse than what this replaces -- and
    # `stored_vectors` in frames.py already does the same. Worth a public accessor if
    # anything else comes to want one.
    backend = store.stream(stream, dict)._source
    blobs, codec = backend.blob_store, backend.codec
    conn = store._registry_conn

    rows = int(conn.execute(f'SELECT COUNT(*) FROM "{stream}"').fetchone()[0])
    if not rows:
        raise ValueError(f"{stream!r} holds no patches")
    stride = max(1, int(stride))
    ids = [row[0] for row in conn.execute(f'SELECT id FROM "{stream}" ORDER BY id')]

    started = time.monotonic()
    probe = conn.execute(f'SELECT embedding FROM "{stream}_vec" LIMIT 1').fetchone()
    width = len(np.frombuffer(probe[0], dtype=np.float32))

    _warn_if_it_will_not_fit(tag, rows, width)
    vectors = _from_chunks(conn, stream, width, rows)
    if vectors is None:
        logger.warning(
            f"hyperspace: {tag} has no readable vec0 chunk storage, falling back to the "
            "row-by-row read -- minutes rather than seconds"
        )
        vectors = _vectors_of(conn, stream, width, rows)
    read = time.monotonic() - started

    # Thinned AFTER the read and BEFORE the payloads: the vectors arrive in chunks
    # whatever we do, but `_placements` decodes one blob per row and is the half that
    # shrinks with the stride. `last_id` stays the true last row so that `grow` picks up
    # from where the stream really ends, not from the last one we chose to keep.
    last_id = ids[-1]
    if stride > 1:
        kept = len(ids[::stride])
        logger.info(
            f"hyperspace: {tag} at 1/{stride} scale -- keeping {kept} of {len(ids)} patches"
        )
        vectors = np.ascontiguousarray(vectors[::stride])
        ids = ids[::stride]

    started = time.monotonic()
    placements = _placements(stream, blobs, codec, ids)
    meta = time.monotonic() - started

    logger.info(
        f"hyperspace: {tag} resident -- {len(vectors)} x {width} "
        f"({vectors.nbytes / 1e6:.0f} MB) in {read:.1f}s, payloads in {meta:.1f}s"
    )
    return ResidentPatches(tag=tag, stream=stream, vectors=vectors, last_id=last_id, **placements)


class ResidentIndex:
    """The models a process is holding, loaded once and asked many times.

    Keyed by stream rather than by store so that two handles on one recording share the
    array instead of each paying the read.
    """

    def __init__(self, stride: int = 1) -> None:
        self._held: dict[str, ResidentPatches] = {}
        # Every load through this holder keeps every Nth patch; see `load`.
        self.stride = max(1, int(stride))

    def __contains__(self, stream: str) -> bool:
        return stream in self._held

    def get(self, stream: str) -> ResidentPatches | None:
        return self._held.get(stream)

    def of(self, store: Any, tag: str, stream: str) -> ResidentPatches:
        held = self._held.get(stream)
        if held is None:
            held = self._held[stream] = load(store, tag, stream, self.stride)
        return held

    def warm(self, store: Any, members: Sequence[tuple[str, str]]) -> float:
        """Load every named model. Returns the seconds spent, for a caller to report."""
        started = time.monotonic()
        for tag, stream in members:
            self.of(store, tag, stream)
        return time.monotonic() - started

    def grow(self, store: Any, members: Sequence[tuple[str, str]]) -> int:
        """Take in whatever an ingest has written since the last look. Returns the rows.

        A live index starts empty and fills while queries are being asked, so "load it
        once at startup" is not available: a query that has not caught up cannot see the
        thing the robot drove past a second ago. Cheap when nothing has landed -- one
        indexed count per model -- so it is safe to call before every query.
        """
        added = 0
        for tag, stream in members:
            held = self._held.get(stream)
            if held is None:
                # Nothing held yet, and `load` refuses an empty stream, so wait for the
                # first patches rather than treating "not written yet" as an error.
                try:
                    self._held[stream] = load(store, tag, stream, self.stride)
                except (ValueError, KeyError, TypeError):
                    continue
                added += self._held[stream].rows
                continue
            block = since(store, tag, stream, held.last_id)
            if block is not None:
                added += held.extend(block)
        return added

    def drop(self, stream: str) -> None:
        self._held.pop(stream, None)


# One per process. A query path looks here before it reaches for sqlite.
RESIDENT = ResidentIndex()
