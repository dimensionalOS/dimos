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

"""A recording on disk: opening it, its embedding index, and the pass that fills one.

Shared by ``dimos map embed`` and ``dimos map fill-depth``. Nothing here asks a
question -- the answering lives in `live.py` and is reached through the `Hyperspace`
module, so an offline tool and the robot cannot drift apart.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import time
from typing import TYPE_CHECKING

import typer

from dimos.mapping.hyperspace.ingest import (
    COMPLETE_STREAM,
    KEYFRAME_STREAM,
    IngestConfig,
    PatchIngestor,
    index_slug,
    index_specs,
    indexes_in,
    patch_stream_for,
    stream_names,
    thumbnail_stream_for,
)
from dimos.mapping.hyperspace.siglip_embedder import PatchEnsemble

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

TIMELINE = "ts"


def open_store(path: Path, *, must_exist: bool = True) -> Store:
    """Open a recording, picking the store from the file extension."""
    if path.suffix == ".mcap":
        from dimos.memory.store.mcap import McapStore

        store = McapStore(path=str(path))
    elif path.suffix == ".db":
        from dimos.memory.store.sqlite import SqliteStore

        store = SqliteStore(path=str(path), must_exist=must_exist)
    else:
        raise typer.BadParameter(
            f"expected a .db or .mcap recording, got {path.suffix or path.name!r}"
        )
    store.start()
    return store


# Folding the log back takes the database exclusively for as long as the copy
# runs, which on a sixty gigabyte file is tens of seconds. Five seconds is the
# default patience and it is not enough: a fold from the janitor killed a pass
# with "database is locked".
WAL_PRAGMAS = ("PRAGMA wal_autocheckpoint=0", "PRAGMA busy_timeout=600000")


def hold_the_wal(store: Store) -> None:
    """Stop every commit from re-walking the write-ahead log.

    SQLite checkpoints the WAL on commit once it passes a thousand pages, and a
    checkpoint in passive mode never shrinks the file, so the log kept growing
    and every commit rescanned all of it: at four gigabytes that was 92% of the
    wall time and the rate had fallen from 16 to 0.7 images a second. Folding is
    left to :func:`fold_the_wal` at the end of the run, and to whatever watches
    the file size in between.
    """
    open_connection = store._open_connection  # type: ignore[attr-defined]

    def without_autocheckpoint() -> Any:
        conn = open_connection()
        for pragma in WAL_PRAGMAS:
            conn.execute(pragma)
        return conn

    store._open_connection = without_autocheckpoint  # type: ignore[attr-defined]
    for pragma in WAL_PRAGMAS:
        store._registry_conn.execute(pragma)  # type: ignore[attr-defined]


WAL_CAP_BYTES = 4_000_000_000


def wal_bytes(store: Store) -> int:
    log = Path(f"{getattr(store.config, 'path', '')}-wal")
    return log.stat().st_size if log.exists() else 0


def fold_the_wal(store: Store) -> None:
    """Copy the write-ahead log back into the database and truncate it.

    Only the process doing the writing may call this. A fold from outside waits
    for the writer's transaction while the writer waits for the fold, and the
    pair sit there until one of them gives up -- which cost a pass, twice.
    """
    started = time.monotonic()
    conn = store._registry_conn  # type: ignore[attr-defined]
    busy, copied, checkpointed = conn.execute("PRAGMA wal_checkpoint(TRUNCATE)").fetchone()
    took = time.monotonic() - started
    if busy:
        typer.echo(f"wal: still held open, {copied} of {checkpointed} pages folded ({took:.0f}s)")
    else:
        typer.echo(f"wal: folded {checkpointed} pages back ({took:.0f}s)")


def memory_db_for(recording: Path) -> Path:
    """Where a recording's keyframes and patches live: the recording itself.

    One recording is one file. Only an .mcap, which cannot be written to, needs a
    companion db beside it.
    """
    return recording if recording.suffix == ".db" else recording.with_suffix(".hyperspace.db")


def index_is_finished(memory: Store, slug: str = "") -> bool:
    """True when this index has keyframes to reuse.

    Jeff's call (2026-09-12): no completeness marker. A half-written index therefore
    reads as a whole one, and the cost of that is a rerun with --no-reuse.
    """
    # Keyframes only: an ensemble index has no patch stream at all, because nothing
    # would read it (see PatchIngestor.write_patch_vectors).
    # The flat layout has no keyframe stream: a per-model patch stream with rows in it
    # is what makes an index answerable.
    names = set(memory.list_streams())
    keyframes, patches = stream_names(slug)
    if keyframes in names and memory.stream(keyframes, dict).count() > 0:
        return True
    prefix = patch_stream_for(slug, "x").rsplit("_x", 1)[0]
    return any(memory.stream(name, dict).count() > 0 for name in names if name.startswith(prefix))


def pick_index(memory: Store, specs: Sequence[str]) -> str:
    """Which index these checkpoints own: "" for the canonical one, else a slug.

    One recording, several indexes. The first model to be ingested takes the bare
    stream names; a different model later gets its own pair rather than overwriting
    an index someone may still be comparing against.
    """
    wanted = index_slug(specs)
    for slug in indexes_in(memory):
        if index_slug(index_specs(memory, slug) or []) == wanted:
            return slug
    return "" if KEYFRAME_STREAM not in memory.list_streams() else wanted


def refuse_unless_readable(recording: Store, names: Sequence[str]) -> None:
    """Every stream the ingest reads, checked BEFORE anything is deleted.

    The ingest drops the old index so a rerun does not append a second copy of every
    keyframe. If it then refused on an empty stream, the index it was about to replace
    would already be gone and nothing would take its place. Derive this list from what
    `ingest` actually reads, not from memory: today that is the colour and depth images,
    both camera_infos and tf.
    """
    for name in names:
        if next(iter(recording.streams[name].order_by(TIMELINE)), None) is None:
            raise typer.BadParameter(f"stream {name!r} is empty; nothing was changed")


def drop_index(memory: Store, slug: str = "") -> None:
    """Clear one index so a rerun replaces it instead of appending a second copy.

    Only this model's streams: the other indexes in the recording are someone else's
    answer to the same question and are what the comparison is for. COMPLETE_STREAM is
    not written any more but is still dropped with the canonical index, since a db
    indexed before that changed carries one and it would vouch for keyframes that are
    no longer there.
    """
    keyframes, patches = stream_names(slug)
    # Every per-model vec0 stream too (`<patches>__m_<member>`), or a re-ingest would
    # append a second copy of every vector into indexes the drop had missed.
    prefix = patch_stream_for(slug, "x").rsplit("_x", 1)[0]
    members = [name for name in memory.list_streams() if name.startswith(prefix)]
    names = [keyframes, patches, thumbnail_stream_for(slug), *members] + (
        [COMPLETE_STREAM] if not slug else []
    )
    for name in names:
        if name in memory.list_streams():
            memory.delete_stream(name)


def pick_stream(store: Store, wanted: str | None, *keywords: str) -> str:
    """Name the stream to use: the one asked for, else the best keyword match."""
    names = store.list_streams()
    if wanted:
        if wanted not in names:
            raise typer.BadParameter(f"no stream {wanted!r} in the recording; have {sorted(names)}")
        return wanted
    matches = [name for name in names if all(word in name.lower() for word in keywords)]
    if not matches:
        raise typer.BadParameter(
            f"no stream matching {'+'.join(keywords)} in the recording; have {sorted(names)}"
        )
    # Prefer the shortest match: "color_image" over "color_image_camera_info".
    return min(matches, key=len)


def ingest(
    recording: Store,
    memory: Store,
    model: PatchEnsemble,
    *,
    color_stream: str,
    depth_stream: str,
    color_info_stream: str,
    depth_info_stream: str,
    tf_stream: str,
    hz: float,
    max_seconds: float,
    config: IngestConfig,
    slug: str = "",
) -> dict[str, int]:
    """Run the live module's ingest over a recording. Returns its stats."""
    recorded_tf = StreamTF.from_store(recording, tf_stream)

    def lookup(target: str, source: str, ts: float) -> NDArray[np.float64] | None:
        if recorded_tf is None:
            return None
        transform = recorded_tf.get(target, source, ts, warn=False)
        return None if transform is None else transform_to_matrix(transform)

    ingestor = PatchIngestor(
        memory, model, config, lookup=lookup, slug=slug, copy_tf=memory is not recording
    )
    for name in (color_info_stream, depth_info_stream):
        first = next(iter(recording.streams[name].order_by(TIMELINE)), None)
        if first is None:
            raise typer.BadParameter(f"stream {name!r} is empty")
        ingestor.add_camera_info(first.data)
    colors = recording.streams[color_stream].order_by(TIMELINE)
    depths = recording.streams[depth_stream].order_by(TIMELINE)
    start_ts = float(colors.first().ts)
    # Copy tf into the index only when the index is a SEPARATE file -- an .mcap cannot
    # hold one, so its companion db carries its own copy of the transforms. When the
    # index lives in the recording the tf is already there, and copying it appends a
    # second set of the recording's own transforms to itself: grocery.db reached 85,302
    # rows over 16,048 distinct stamps, 5.3x duplicated, before this was caught.
    transforms = 0
    if ingestor.copy_tf:
        # Only the tf the slice can use; a whole recording's tf is hundreds of
        # thousands of messages the query side would otherwise decode.
        for observation in recording.streams[tf_stream].order_by(TIMELINE):
            stamp = float(observation.ts)
            if stamp < start_ts - 5.0 or stamp > start_ts + max_seconds + 5.0:
                continue
            ingestor.add_tf(observation.data, ts=stamp)
            transforms += 1
    typer.echo(f"tf: {transforms} copied ({'in place' if memory is recording else 'companion db'})")

    # --hz IS the embed rate. It used to be max(1/hz, the config default), which meant
    # it could only ever slow embedding down: --hz 15 against a 0.2 default did nothing.
    ingestor.config.min_frame_interval_s = 1.0 / hz if hz > 0 else 0.0
    ingestor.buffer.config.min_interval = min(
        ingestor.buffer.config.min_interval or 0.0, ingestor.config.min_frame_interval_s
    )
    hold_the_wal(memory)
    first_ts: float | None = None
    started = time.monotonic()
    resume_from: float | None = None
    while True:
        # Read in windows. The log can only be folded while no read is open, and the
        # pairing iterator holds one for as long as it lives -- with it open the fold
        # waits on a lock this same process holds and the pass stops dead. So the
        # window ends, the iterator is closed, the log is folded, and the next window
        # picks up from the stamp we stopped at.
        source = colors if resume_from is None else colors.after(resume_from)
        other = depths if resume_from is None else depths.after(resume_from - config.depth_max_dt)
        pairs = iter(source.align(other, tolerance=config.depth_max_dt))
        stop_at: float | None = None
        for pair in pairs:
            color_obs, depth_obs = pair.data[0], pair.data[1]
            stamp = float(color_obs.ts)
            if first_ts is None:
                first_ts = stamp
            if stamp - first_ts > max_seconds:
                stop_at = None
                break
            ingestor.add_depth(depth_obs.data)
            ingestor.add_image(color_obs.data)
            if ingestor.stats["images"] % 200 == 0:
                size = wal_bytes(memory)
                typer.echo(
                    f"{stamp - first_ts:.0f}s: {ingestor.stats['embedded']} embedded, "
                    f"{ingestor.stats['kept']} kept ({time.monotonic() - started:.0f}s, "
                    f"wal {size / 1e9:.1f} GB)"
                )
                if size > WAL_CAP_BYTES:
                    stop_at = stamp
                    break
        else:
            pairs.close()
            break
        pairs.close()
        del pairs
        if stop_at is None:
            break
        fold_the_wal(memory)
        resume_from = stop_at
    ingestor.flush()
    fold_the_wal(memory)
    return dict(ingestor.stats)
