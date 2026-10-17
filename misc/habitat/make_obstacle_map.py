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

"""Reduce 3D ground-truth files (``<scene_id>.json``) to obstacle maps (``<scene_id>.obstacles.json``).

The obstacle map is a top-down ``Detection2DArray`` like ``<scene_id>.top_down.json``,
but unlabeled and reduced to what blocks a robot: only boxes that cross the slice
height count (carpets and mats are below it, ceiling lamps, windows, pictures and
tabletop items above it); object footprints that overlap or touch are wrapped into
one rectangle labeled ``obstacle`` while it stays at least half solid (a dining
table with its chairs, a bed with its nightstand, not a whole room); wall pieces
merge into one ``wall`` only while the union stays a thin strip, so corners never
chain and doorways remain open. It reads the 3D file, not
the 2D one, because the slice needs each box's height.

    uv run python misc/habitat/make_obstacle_map.py misc/habitat/ground_truth/hssd/*[0-9].json
"""

from __future__ import annotations

import argparse
from collections import Counter
from collections.abc import Sequence
import itertools
from pathlib import Path

from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.simulation.object_detections import (
    GroundTruthBox,
    boxes_to_detection3d_array,
    read_detection3d_json,
    top_down,
    write_detection2d_json,
)

Rect = tuple[float, float, float, float]  # (min_x, min_y, max_x, max_y) in meters
Piece = tuple[Rect, float, float]  # footprint plus the box's min and max z

SLICE_HEIGHT_M = 0.2  # robot height of interest
MIN_FILL = 0.5  # keep wrapping only while the rectangle stays at least this solid
TOUCH_M = 0.05  # footprints this close count as overlapping
WALL_LABEL = "wall"
OBSTACLE_LABEL = "obstacle"
WALL_STRIP_MAX_M = 0.6  # wall pieces merge only while the union stays this thin


def area(r: Rect) -> float:
    return max(0.0, r[2] - r[0]) * max(0.0, r[3] - r[1])


def overlaps(a: Rect, b: Rect) -> bool:
    """True when the footprints overlap or come within TOUCH_M of each other."""
    return max(a[0], b[0]) - TOUCH_M < min(a[2], b[2]) and max(a[1], b[1]) - TOUCH_M < min(
        a[3], b[3]
    )


def union(a: Rect, b: Rect) -> Rect:
    return (min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3]))


def union_area(rects: Sequence[Rect]) -> float:
    """Exact area covered by axis-aligned rectangles, by coordinate compression."""
    xs = sorted({x for r in rects for x in (r[0], r[2])})
    ys = sorted({y for r in rects for y in (r[1], r[3])})
    total = 0.0
    for x0, x1 in itertools.pairwise(xs):
        for y0, y1 in itertools.pairwise(ys):
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if any(r[0] <= cx <= r[2] and r[1] <= cy <= r[3] for r in rects):
                total += (x1 - x0) * (y1 - y0)
    return total


def pieces(
    detections: Sequence[Detection3D], slice_height: float
) -> tuple[list[Piece], list[Piece], Counter[str]]:
    """Object and wall pieces crossing the slice height, plus what was dropped by label."""
    objects: list[Piece] = []
    walls: list[Piece] = []
    dropped: Counter[str] = Counter()
    for d in detections:
        c, s = d.bbox.center.position, d.bbox.size
        lo_z, hi_z = c.z - s.z / 2, c.z + s.z / 2
        label = d.results[0].hypothesis.class_id
        if lo_z > slice_height or hi_z < slice_height:
            dropped[label] += 1
            continue
        piece = ((c.x - s.x / 2, c.y - s.y / 2, c.x + s.x / 2, c.y + s.y / 2), lo_z, hi_z)
        (walls if label == WALL_LABEL else objects).append(piece)
    return objects, walls, dropped


def _merge(a: Piece, b: Piece) -> Piece:
    return union(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2])


def wrap(objects: Sequence[Piece], min_fill: float) -> list[Piece]:
    """Wrap overlapping or touching footprints into one piece covering all of them."""
    clusters: list[tuple[Piece, list[Rect]]] = []
    for piece in objects:
        members = [piece[0]]
        merged = True
        while merged:
            merged = False
            for k, (other, other_members) in enumerate(clusters):
                if not overlaps(piece[0], other[0]):
                    continue
                joined = _merge(piece, other)
                if (
                    min_fill > 0
                    and union_area(members + other_members) / area(joined[0]) < min_fill
                ):
                    continue
                piece, members = joined, members + other_members
                del clusters[k]
                merged = True
                break
        clusters.append((piece, members))
    return [piece for piece, _ in clusters]


def wrap_walls(walls: Sequence[Piece], strip_max: float) -> list[Piece]:
    """Merge overlapping wall pieces only while the union is still a strip.

    Collinear pieces and stacked window frames fold into their wall; an L or T
    junction would make a thick union and stays as two boxes, so corners never
    chain around the building.
    """
    clusters: list[Piece] = []
    for piece in walls:
        merged = True
        while merged:
            merged = False
            for k, other in enumerate(clusters):
                if not overlaps(piece[0], other[0]):
                    continue
                joined = _merge(piece, other)
                if min(joined[0][2] - joined[0][0], joined[0][3] - joined[0][1]) > strip_max:
                    continue
                piece = joined
                del clusters[k]
                merged = True
                break
        clusters.append(piece)
    return clusters


def boxes(label: str, merged: Sequence[Piece]) -> list[GroundTruthBox]:
    """One box per piece, numbered by footprint position so regenerated files are stable."""
    ordered = sorted(merged, key=lambda p: (round(p[0][0], 3), round(p[0][1], 3)))
    return [
        GroundTruthBox(
            id=f"{label}_{k:03d}",
            labels=(label,),
            min=(rect[0], rect[1], lo_z),
            max=(rect[2], rect[3], hi_z),
        )
        for k, (rect, lo_z, hi_z) in enumerate(ordered)
    ]


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("paths", nargs="+", type=Path, help="3D ground-truth JSON files")
    parser.add_argument("--out", type=Path, help="output directory (default: next to each input)")
    parser.add_argument("--slice-height", type=float, default=SLICE_HEIGHT_M, help="meters")
    parser.add_argument(
        "--wall-strip-max",
        type=float,
        default=WALL_STRIP_MAX_M,
        help="merge wall pieces only while their union stays this thin (m)",
    )
    parser.add_argument(
        "--min-fill",
        type=float,
        default=MIN_FILL,
        help="only wrap while the rectangle stays at least this solid (0 = always wrap)",
    )
    args = parser.parse_args(argv)
    for path in args.paths:
        if path.name.endswith((".top_down.json", ".obstacles.json")):
            continue
        detections, provenance = read_detection3d_json(path)
        source = detections.detections[: detections.detections_length]
        objects, walls, dropped = pieces(source, args.slice_height)
        wrapped = wrap(objects, args.min_fill)
        strips = wrap_walls(walls, args.wall_strip_max)
        reduced = boxes_to_detection3d_array(
            boxes(OBSTACLE_LABEL, wrapped) + boxes(WALL_LABEL, strips),
            to_ros=lambda x, y, z: (x, y, z),
            frame_id=detections.frame_id,
            ts=detections.ts,
        )
        out_dir = args.out or path.parent
        out_dir.mkdir(parents=True, exist_ok=True)
        out = write_detection2d_json(
            top_down(reduced, covering_fraction=None),
            out_dir / f"{path.name[: -len('.json')]}.obstacles.json",
            provenance={**provenance, "slice_height_m": args.slice_height},
        )
        print(
            f"{path.name}: {len(source)} boxes, {sum(dropped.values())} off the slice, "
            f"{len(objects)} objects -> {len(wrapped)} obstacles, "
            f"{len(walls)} wall pieces -> {len(strips)} walls; wrote {out}"
        )


if __name__ == "__main__":
    main()
