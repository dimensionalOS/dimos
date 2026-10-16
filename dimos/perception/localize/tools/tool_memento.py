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

"""Check that a window answered from Memento's cached reads matches a fresh read of it.

Run: uv run python -m dimos.perception.localize.tools.tool_memento [query ...]
         [--dataset <db>] [--from <s>] [--duration <s>] [--params <name|json>] [--step <s>]
         [--whole-lidar]

The window is embedded once and answered three times on that index and the
same models, each from an empty memory and support-plane cache: fresh; fresh
again, the run-to-run floor; and composed, after reading the window as
consecutive ``--step`` second windows, the way a caller polling a rolling
window fills the memory, so the composed answer itself reads nothing.
Retrieval picks peaks and verification frames per window, so composed and
fresh can differ. Every answer is printed beside its match by position.

On a lidar rig each read lifts through its own window's scans, as the module's
calls do; ``--whole-lidar`` lifts every read through the whole window's scans,
so what still differs comes from retrieval alone.
"""

import argparse
from dataclasses import replace
from pathlib import Path
import sys
from typing import Any

import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.localize.dandetect import DanDetector
from dimos.perception.localize.localize import embed_index, localize
from dimos.perception.localize.localize_params import XARM_PARAMS, parse_params
from dimos.perception.localize.memento import Memento
from dimos.perception.localize.rig import Rig, window_tf
from dimos.perception.localize.types import Localization, LocalizeParams
from dimos.utils.data import get_data

XARM_DATASET = (
    "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
    "xarm6_worldbelief_20260729_203624_161992.db"
)


class Counted:
    """The detector, counting the frames it scores."""

    def __init__(self, detector: Any) -> None:
        self.detector = detector
        self.frames = 0

    def query_score_rows(
        self, image: Image, queries: list[str], threshold: float
    ) -> tuple[np.ndarray, np.ndarray]:
        self.frames += 1
        rows: tuple[np.ndarray, np.ndarray] = self.detector.query_score_rows(
            image, queries, threshold
        )
        return rows

    def stop(self) -> None:
        pass


def describe(hit: Localization, lo: float) -> str:
    x, y, z = hit.position_world_xyz
    return (
        f"({x:.3f}, {y:.3f}, {z:.3f}) seen {hit.first_seen_timestamp - lo:.1f}-"
        f"{hit.last_seen_timestamp - lo:.1f}s views {hit.n_views} score {hit.semantic_score:.2f}"
    )


def compare(
    name: str, base: list[Localization], other: list[Localization], radius: float, lo: float
) -> int:
    """Print ``other``'s answers beside ``base``'s, matched by position; count differences."""
    pending = list(range(len(other)))
    differences = 0
    for hit in base:
        here = np.asarray(hit.position_world_xyz)
        near = [
            (float(np.linalg.norm(np.asarray(other[j].position_world_xyz) - here)), j)
            for j in pending
        ]
        near = [(d, j) for d, j in near if d <= radius]
        if not near:
            print(f"    only fresh: {describe(hit, lo)}")
            differences += 1
            continue
        d, j = min(near)
        pending.remove(j)
        twin = other[j]
        same = (
            d == 0.0
            and twin.first_seen_timestamp == hit.first_seen_timestamp
            and twin.last_seen_timestamp == hit.last_seen_timestamp
            and twin.n_views == hit.n_views
            and twin.semantic_score == hit.semantic_score
        )
        if same:
            print(f"    same: {describe(hit, lo)}")
            continue
        differences += 1
        print(f"    fresh: {describe(hit, lo)}")
        print(f"    {name}: {describe(twin, lo)}, {1000 * d:.1f} mm apart")
    for j in pending:
        print(f"    only {name}: {describe(other[j], lo)}")
        differences += 1
    return differences


def answer(
    queries: list[str],
    windows: list[tuple[float, float]],
    *,
    index: Any,
    rig: Rig,
    dan: DanDetector,
    params: LocalizeParams,
    head: float,
    whole_lidar: bool,
) -> tuple[list[list[Localization]], int]:
    """Read ``windows`` in order into one empty memory; the last one's answer and the frames scored.

    Each read lifts through its own window's lidar scans, as the module's calls do.
    """
    rig.plane_cache.clear()
    memento, counted = Memento(), Counted(dan.detector)
    results: list[list[Localization]] = []
    for lo, hi in windows:
        read_rig = (
            rig
            if rig.lidar is None or whole_lidar
            else replace(rig, lidar=rig.lidar.time_range(lo, hi))
        )
        results = localize(
            queries,
            index=index.time_range(lo, hi),
            window=(lo, hi),
            head=head,
            rig=read_rig,
            embedder=dan.embedder,
            detector=counted,
            segmenter=dan.segmenter,
            params=params,
            memento=memento,
        )
    return results, counted.frames


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("queries", nargs="+", metavar="query")
    parser.add_argument("--dataset", type=Path, help="memory recording database")
    parser.add_argument("--from", dest="start", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--params", default="xarm")
    parser.add_argument("--step", type=float, default=5.0, help="length of each composing read (s)")
    parser.add_argument(
        "--whole-lidar", action="store_true", help="lift every read through the window's scans"
    )
    args = parser.parse_args()

    store = SqliteStore(path=args.dataset or get_data(XARM_DATASET))
    rig = Rig.from_store(store)
    lo, hi = rig.time_range()
    after = lo + args.start
    before = lo + args.start + args.duration if args.duration is not None else hi
    rig.tf = window_tf(store, after, before)
    if rig.lidar is not None:
        rig.lidar = rig.lidar.after(after).before(before)
    detector = DanDetector()
    params = detector.shifted(*parse_params(args.params, XARM_PARAMS))
    steps = [
        (float(t), float(min(t + args.step, before))) for t in np.arange(after, before, args.step)
    ]

    with detector as dan:
        index = embed_index(rig, dan.embedder, after, before)
        run = {
            "queries": args.queries,
            "index": index,
            "rig": rig,
            "dan": dan,
            "params": params,
            "whole_lidar": args.whole_lidar,
        }
        fresh, fresh_frames = answer(windows=[(after, before)], head=hi, **run)
        again, again_frames = answer(windows=[(after, before)], head=hi, **run)
        composed, composed_frames = answer(windows=[*steps, (after, before)], head=hi, **run)

    print(
        f"\nframes scored: fresh {fresh_frames}, fresh again {again_frames}, "
        f"composed {composed_frames} over {len(steps)} reads of {args.step:.0f}s"
    )
    totals = {"fresh again": 0, "composed": 0}
    for query, base, repeat, built in zip(args.queries, fresh, again, composed, strict=True):
        print(f"\n{query!r}: fresh {len(base)}, fresh again {len(repeat)}, composed {len(built)}")
        print("  fresh again:")
        totals["fresh again"] += compare("again", base, repeat, params.cluster_radius_m, lo)
        print("  composed:")
        totals["composed"] += compare("composed", base, built, params.cluster_radius_m, lo)
    print(
        f"\nanswers differing from fresh: fresh again {totals['fresh again']}, "
        f"composed {totals['composed']}"
    )
    store.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
