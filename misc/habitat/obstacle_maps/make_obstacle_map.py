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

"""Obstacle map for one scene: unlabeled rectangles from its 3D ground-truth JSON.

Keeps only boxes a robot meets at the slice height (the box spans z = 0.2 m), so
carpets, mats and anything mounted above the robot are free space. Object
footprints that overlap or touch are wrapped into one rectangle covering all of
them; wall pieces merge only while their union is still a thin strip, so corners
never chain and doorways remain open.

    uv run python misc/habitat/obstacle_maps/make_obstacle_map.py \\
        misc/habitat/ground_truth/hssd/102344193.json --out misc/habitat/obstacle_maps
"""

from __future__ import annotations

import argparse
from collections import Counter
import itertools
import json
from pathlib import Path

Rect = list[float]  # [min_x, min_y, max_x, max_y] in meters

SLICE_HEIGHT_M = 0.2  # robot height of interest
MIN_FILL = 0.0  # optional: require a wrapped rectangle to be at least this solid
TOUCH_M = 0.05  # footprints this close count as overlapping
WALL_LABEL = "wall"
WALL_STRIP_MAX_M = 0.6  # wall pieces merge only while the union stays this thin
PADDING_M = 1.0


def area(r: Rect) -> float:
    return max(0.0, r[2] - r[0]) * max(0.0, r[3] - r[1])


def overlaps(a: Rect, b: Rect) -> bool:
    """True when the footprints overlap or come within TOUCH_M of each other."""
    return max(a[0], b[0]) - TOUCH_M < min(a[2], b[2]) and max(a[1], b[1]) - TOUCH_M < min(
        a[3], b[3]
    )


def union(a: Rect, b: Rect) -> Rect:
    return [min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3])]


def union_area(rects: list[Rect]) -> float:
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


def footprints(view: dict, slice_height: float) -> tuple[list[Rect], list[Rect], Counter[str]]:
    """Object and wall footprints crossing the slice height, plus what was dropped."""
    objects: list[Rect] = []
    walls: list[Rect] = []
    dropped: Counter[str] = Counter()
    for d in view["detections"]:
        c, s = d["center_xyz"], d["size_xyz"]
        if c[2] - s[2] / 2 > slice_height or c[2] + s[2] / 2 < slice_height:
            dropped[d["label"]] += 1
            continue
        rect = [c[0] - s[0] / 2, c[1] - s[1] / 2, c[0] + s[0] / 2, c[1] + s[1] / 2]
        (walls if d["label"] == WALL_LABEL else objects).append(rect)
    return objects, walls, dropped


def wrap(objects: list[Rect], min_fill: float) -> list[tuple[Rect, int]]:
    """Wrap overlapping or touching footprints into one rectangle covering all of them."""
    clusters: list[tuple[Rect, list[Rect]]] = []
    for r in objects:
        box, members = list(r), [r]
        merged = True
        while merged:
            merged = False
            for k, (cb, cm) in enumerate(clusters):
                if not overlaps(box, cb):
                    continue
                ub = union(box, cb)
                if min_fill > 0 and union_area(members + cm) / area(ub) < min_fill:
                    continue
                box, members = ub, members + cm
                del clusters[k]
                merged = True
                break
        clusters.append((box, members))
    return [(b, len(m)) for b, m in clusters]


def wrap_walls(walls: list[Rect], strip_max: float) -> list[Rect]:
    """Merge overlapping wall pieces only while the union is still a strip.

    Collinear pieces and stacked window frames fold into their wall; an L or T
    junction would make a thick union and stays as two boxes, so corners never
    chain around the building.
    """
    clusters: list[Rect] = []
    for r in walls:
        box = list(r)
        merged = True
        while merged:
            merged = False
            for k, cb in enumerate(clusters):
                if not overlaps(box, cb):
                    continue
                ub = union(box, cb)
                if min(ub[2] - ub[0], ub[3] - ub[1]) > strip_max:
                    continue
                box = ub
                del clusters[k]
                merged = True
                break
        clusters.append(box)
    return clusters


def svg(rects: list[Rect], width_px: int = 1000) -> str:
    lo_x, lo_y = min(r[0] for r in rects) - PADDING_M, min(r[1] for r in rects) - PADDING_M
    hi_x, hi_y = max(r[2] for r in rects) + PADDING_M, max(r[3] for r in rects) + PADDING_M
    ex, ey = hi_x - lo_x, hi_y - lo_y
    flip = -1.0  # SVG y grows downward; world y grows upward
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" '
        f'height="{width_px * ey / ex:.0f}" viewBox="{lo_x:.3f} {flip * hi_y:.3f} {ex:.3f} {ey:.3f}">',
        f'<rect x="{lo_x:.3f}" y="{flip * hi_y:.3f}" width="{ex:.3f}" height="{ey:.3f}" fill="#fcfcfb"/>',
    ]
    grid = 'stroke="#e6e5e0" stroke-width="1" vector-effect="non-scaling-stroke"'
    for g in range(int(lo_x) - 1, int(hi_x) + 2):
        parts.append(
            f'<line x1="{g}" y1="{flip * hi_y:.3f}" x2="{g}" y2="{flip * lo_y:.3f}" {grid}/>'
        )
    for g in range(int(lo_y) - 1, int(hi_y) + 2):
        parts.append(
            f'<line x1="{lo_x:.3f}" y1="{flip * g}" x2="{hi_x:.3f}" y2="{flip * g}" {grid}/>'
        )
    for r in rects:
        parts.append(
            f'<rect x="{r[0]:.3f}" y="{flip * r[3]:.3f}" width="{r[2] - r[0]:.3f}" '
            f'height="{r[3] - r[1]:.3f}" fill="#2a78d6" fill-opacity="0.18" stroke="#2a78d6" '
            'stroke-width="1.5" vector-effect="non-scaling-stroke"/>'
        )
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("source", type=Path, help="3D ground-truth JSON of one scene")
    parser.add_argument("--out", type=Path, default=Path(__file__).parent, help="output directory")
    parser.add_argument("--slice-height", type=float, default=SLICE_HEIGHT_M)
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

    view = json.loads(args.source.read_text())
    objects, walls, dropped = footprints(view, args.slice_height)
    wrapped = wrap(objects, args.min_fill)
    wall_boxes = wrap_walls(walls, args.wall_strip_max)
    rects = sorted(
        [b for b, _ in wrapped] + wall_boxes, key=lambda b: (round(b[0], 3), round(b[1], 3))
    )
    out = {
        "dataset": view["dataset"],
        "scene_id": view["scene_id"],
        "frame_id": view["frame_id"],
        "slice_height_m": args.slice_height,
        "units": "m",
        "count": len(rects),
        "obstacles": [
            {"min_xy": [round(b[0], 3), round(b[1], 3)], "max_xy": [round(b[2], 3), round(b[3], 3)]}
            for b in rects
        ],
    }
    args.out.mkdir(parents=True, exist_ok=True)
    stem = args.out / view["scene_id"]
    stem.with_suffix(".json").write_text(json.dumps(out, indent=2) + "\n")
    stem.with_suffix(".svg").write_text(svg(rects))
    groups = sorted((n for _, n in wrapped if n > 1), reverse=True)
    print(
        f"{view['scene_id']}: {len(view['detections'])} boxes, {sum(dropped.values())} below or above "
        f"{args.slice_height} m, {len(objects)} objects wrapped into {len(wrapped)} "
        f"(groups {groups}), {len(walls)} wall pieces -> {len(wall_boxes)}; "
        f"{len(rects)} obstacles; wrote {stem}.json/.svg"
    )


if __name__ == "__main__":
    main()
