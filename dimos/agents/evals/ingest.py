# Copyright 2025-2026 Dimensional Inc.
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

"""One-time replay -> ``SpatialMemory`` ingest: builds the store the eval queries.

The agent under test answers "where is X" out of its own spatial memory, so the
eval needs that memory populated before any question is asked. Doing it inside
the test would mean replaying 292 s of recording per run against a module that
throttles on wall-clock; instead this script fills a store once, offline, and
every later run attaches to it read-only with ``new_memory=False``.

Three details are load-bearing:

* **Sampling happens here, not in the module.** ``SpatialMemory``'s distance and
  time throttles are measured on wall-clock, which during a no-clock replay is
  meaningless. Both are pinned to ``0`` and this script decides which frames to
  keep, using each observation's *recorded* timestamp.
* **The frames are pushed through the module's real path.** ``process_stream``
  wants a plain dict of ``frame``/``position``/``rotation``, so each observation
  is adapted rather than the embeddings being written directly — the store the
  eval queries is built by the same code that builds it on a robot.
* **The saved pickle is backed up read-only.** ``SpatialMemory.stop()`` saves
  *then* clears, so a teardown that fires twice re-saves an empty memory over a
  populated file. Answering runs therefore point ``visual_memory_path`` at a
  throwaway path; if one ever does clobber the ingested file, restore it from
  the backup written here instead of re-ingesting.

Attach to the result with the same ``db_path`` and ``collection_name`` this run
prints, plus ``new_memory=False`` — the default is ``True``, which deletes the
collection on construction.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from pathlib import Path
import shutil
import stat
import sys
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.stream import Stream

DEFAULT_DATASET = "go2_bigoffice"
DEFAULT_SAMPLE_HZ = 1.0

#: Name of the pickle SpatialMemory keeps its stored frames in, and of the
#: read-only copy taken right after the ingest succeeds.
VISUAL_MEMORY_NAME = "visual_memory.pkl"
VISUAL_MEMORY_BACKUP_NAME = "visual_memory.pkl.bak"


class SampleCounts:
    """Frames seen and kept while streaming, filled in as the ingest consumes."""

    def __init__(self) -> None:
        self.seen = 0
        self.sampled = 0
        self.without_pose = 0
        self.first_ts: float | None = None
        self.last_ts: float | None = None

    @property
    def span_s(self) -> float:
        if self.first_ts is None or self.last_ts is None:
            return 0.0
        return self.last_ts - self.first_ts


def sample_frames(
    store: Store, min_gap_s: float, counts: SampleCounts, progress: bool = True
) -> Iterator[dict[str, Any]]:
    """Yield ``process_stream`` payloads for frames spaced *min_gap_s* apart.

    Spacing is measured on the recorded timestamp, so the same replay always
    produces the same frames regardless of how fast the machine reads it.
    Observations without a pose are skipped: a frame that cannot be placed in
    the map frame is not a spatial memory.
    """
    last_kept: float | None = None
    images: Stream[Any] = store.stream("color_image")
    for obs in images:
        counts.seen += 1
        if counts.first_ts is None:
            counts.first_ts = obs.ts
        counts.last_ts = obs.ts
        if last_kept is not None and (obs.ts - last_kept) < min_gap_s:
            continue
        pose = obs.pose
        if pose is None:
            counts.without_pose += 1
            continue
        last_kept = obs.ts
        counts.sampled += 1
        if progress and counts.sampled % 50 == 0:
            print(f"  ingested {counts.sampled} frames", flush=True)
        yield {
            "frame": obs.data.to_opencv(),  # BGR, as the module expects
            "position": pose.position,
            "rotation": pose.orientation.to_euler(),
        }


def backup_visual_memory(path: Path) -> Path | None:
    """Copy the freshly saved pickle next to itself and make the copy read-only.

    Returns the backup path, or ``None`` if the pickle was never written.
    """
    if not path.exists():
        return None
    backup = path.with_name(VISUAL_MEMORY_BACKUP_NAME)
    if backup.exists():
        backup.chmod(stat.S_IWUSR | stat.S_IRUSR)  # let shutil overwrite it
    shutil.copy2(path, backup)
    backup.chmod(stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)
    return backup


def ingest(
    dataset: str, out_dir: Path, collection: str, sample_hz: float, progress: bool = True
) -> int:
    """Fill a SpatialMemory store from *dataset* and return the frames stored."""
    import reactivex as rx

    from dimos.memory2.cli.dataset import open_dataset
    from dimos.perception.spatial_perception import SpatialMemory

    started = time.monotonic()
    counts = SampleCounts()
    errors: list[BaseException] = []
    stored: list[Any] = []

    memory = SpatialMemory(
        collection_name=collection,
        new_memory=True,  # this run *is* the ingest; later runs attach with False
        db_path=str(out_dir / "chroma"),
        visual_memory_path=str(out_dir / VISUAL_MEMORY_NAME),
        output_dir=str(out_dir / "images"),
        # Sampling is done above, on recorded timestamps; see the module docstring.
        min_distance_threshold=0.0,
        min_time_threshold=0.0,
    )
    constructed = time.monotonic()

    try:
        store = open_dataset(dataset)
        with store:
            frames = sample_frames(store, 1.0 / sample_hz, counts, progress=progress)
            memory.process_stream(rx.from_iterable(frames)).subscribe(
                on_next=stored.append, on_error=errors.append
            )
        if errors:
            raise RuntimeError(f"{len(errors)} frame(s) failed to ingest: {errors[0]!r}")
        if not stored:
            raise RuntimeError(f"nothing was stored from {dataset!r}: is the replay data present?")

        saved = memory.save()
        backup = backup_visual_memory(out_dir / VISUAL_MEMORY_NAME) if saved else None
        finished = time.monotonic()

        print(
            f"dataset:    {dataset} ({counts.seen} color_image observations, "
            f"{counts.span_s:.1f}s recorded)\n"
            f"sampled:    {counts.sampled} frames at {sample_hz}Hz on recorded ts "
            f"({counts.without_pose} skipped for having no pose)\n"
            f"stored:     {len(stored)} frames, chroma collection {collection!r} holds "
            f"{memory.vector_db.image_collection.count()}\n"
            f"stats:      {memory.get_stats()}\n"
            f"saved:      {out_dir / VISUAL_MEMORY_NAME} ({'ok' if saved else 'FAILED'})\n"
            f"backup:     {backup} (read-only)\n"
            f"timing:     {finished - started:.1f}s total, "
            f"{constructed - started:.1f}s of it constructing SpatialMemory\n"
            f"attach via: SpatialMemory(collection_name={collection!r}, "
            f"db_path={str(out_dir / 'chroma')!r}, new_memory=False)"
        )
    finally:
        # Not optional: constructing SpatialMemory starts ~18 non-daemon threads
        # behind the chroma client, and the interpreter will not exit while they
        # live. stop() releases them. It also saves once more before clearing —
        # harmless here because the memory is still populated, and precisely the
        # behavior that makes the read-only backup above worth having.
        memory.stop()
    return len(stored)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m dimos.agents.evals.ingest",
        description=(
            "Populate a SpatialMemory store from a recorded replay, once, so the "
            "evals can attach to it read-only."
        ),
    )
    parser.add_argument("--dataset", default=DEFAULT_DATASET, help="dataset name or .db/.mcap path")
    parser.add_argument(
        "--out-dir",
        required=True,
        help="directory for the chroma db, the visual-memory pickle and its backup",
    )
    parser.add_argument(
        "--collection",
        default="",
        help="chroma collection name (default: the dataset name); the eval must pin the same one",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=DEFAULT_SAMPLE_HZ,
        help="frames kept per second of recording, measured on the recorded timestamp",
    )
    parser.add_argument("--quiet", action="store_true", help="suppress per-frame progress lines")
    return parser


def main(argv: list[str]) -> int:
    args = _build_parser().parse_args(argv)
    if args.sample_hz <= 0:
        raise SystemExit("--sample-hz must be positive")
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    collection = args.collection or Path(args.dataset).stem
    ingest(args.dataset, out_dir, collection, args.sample_hz, progress=not args.quiet)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
