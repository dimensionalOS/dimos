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

"""Draw a top-down ground-truth file (``*.top_down.json``) as a reference SVG.

Walls are filled grey, objects are outlined rectangles with their category
written inside when the footprint is large enough to hold text. Coordinates are
the file's ROS ``world`` meters: x to the right, y up.

    uv run python misc/habitat/render_top_down.py misc/habitat/ground_truth/hssd/*.top_down.json
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from xml.sax.saxutils import escape

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e6e5e0"
WALL = "#52514e"
OBJECT = "#2a78d6"
PADDING_M = 1.0


@dataclass(frozen=True)
class Rect:
    id: str
    label: str
    cx: float
    cy: float
    w: float
    h: float

    @property
    def is_wall(self) -> bool:
        return self.label == "wall"


def load(path: Path) -> tuple[dict[str, object], list[Rect]]:
    view = json.loads(path.read_text())
    rects = [
        Rect(
            d["id"],
            d["label"],
            d["center_xy"][0],
            d["center_xy"][1],
            d["size_xy"][0],
            d["size_xy"][1],
        )
        for d in view["detections"]
    ]
    return view, rects


def render(view: dict[str, object], rects: list[Rect], *, width_px: int, min_label_m: float) -> str:
    lo_x = min(r.cx - r.w / 2 for r in rects) - PADDING_M
    hi_x = max(r.cx + r.w / 2 for r in rects) + PADDING_M
    lo_y = min(r.cy - r.h / 2 for r in rects) - PADDING_M
    hi_y = max(r.cy + r.h / 2 for r in rects) + PADDING_M
    extent_x, extent_y = hi_x - lo_x, hi_y - lo_y
    # SVG y grows downward; world y grows upward.
    y = lambda wy: -wy  # noqa: E731
    font = max(0.14, max(extent_x, extent_y) / 90)
    walls = [r for r in rects if r.is_wall]
    objects = [r for r in rects if not r.is_wall]

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" '
        f'height="{width_px * extent_y / extent_x:.0f}" '
        f'viewBox="{lo_x:.3f} {y(hi_y):.3f} {extent_x:.3f} {extent_y:.3f}" font-family="sans-serif">',
        f'<rect x="{lo_x:.3f}" y="{y(hi_y):.3f}" width="{extent_x:.3f}" height="{extent_y:.3f}" fill="{SURFACE}"/>',
    ]
    # Recessive 1 m grid with labels every 2 m.
    for gx in range(int(lo_x) - 1, int(hi_x) + 2):
        parts.append(
            f'<line x1="{gx}" y1="{y(hi_y):.3f}" x2="{gx}" y2="{y(lo_y):.3f}" stroke="{GRID}" '
            'stroke-width="1" vector-effect="non-scaling-stroke"/>'
        )
        if gx % 2 == 0:
            parts.append(
                f'<text x="{gx}" y="{y(lo_y) - font * 0.4:.3f}" font-size="{font:.3f}" fill="{INK_SECONDARY}" '
                f'text-anchor="middle">{gx}</text>'
            )
    for gy in range(int(lo_y) - 1, int(hi_y) + 2):
        parts.append(
            f'<line x1="{lo_x:.3f}" y1="{y(gy)}" x2="{hi_x:.3f}" y2="{y(gy)}" stroke="{GRID}" '
            'stroke-width="1" vector-effect="non-scaling-stroke"/>'
        )
        if gy % 2 == 0:
            parts.append(
                f'<text x="{lo_x + font * 0.4:.3f}" y="{y(gy) + font * 0.35:.3f}" font-size="{font:.3f}" '
                f'fill="{INK_SECONDARY}">{gy}</text>'
            )
    for r in walls:
        parts.append(
            f'<rect x="{r.cx - r.w / 2:.3f}" y="{y(r.cy + r.h / 2):.3f}" width="{r.w:.3f}" '
            f'height="{r.h:.3f}" fill="{WALL}"><title>{escape(r.id)}</title></rect>'
        )
    for r in objects:
        parts.append(
            f'<rect x="{r.cx - r.w / 2:.3f}" y="{y(r.cy + r.h / 2):.3f}" width="{r.w:.3f}" '
            f'height="{r.h:.3f}" fill="{OBJECT}" fill-opacity="0.08" stroke="{OBJECT}" '
            f'stroke-width="1.5" vector-effect="non-scaling-stroke"><title>{escape(r.label)}</title></rect>'
        )
    for r in objects:
        if min(r.w, r.h) < min_label_m:
            continue
        size = min(font, r.h * 0.6, r.w / max(len(r.label), 1) * 1.8)
        parts.append(
            f'<text x="{r.cx:.3f}" y="{y(r.cy):.3f}" font-size="{size:.3f}" fill="{INK}" '
            f'text-anchor="middle" dominant-baseline="middle" paint-order="stroke" stroke="{SURFACE}" '
            f'stroke-width="{size * 0.25:.3f}">{escape(r.label)}</text>'
        )
    title = (
        f"{view.get('dataset', '')} {view.get('scene_id', '')}  "
        f"{len(objects)} objects, {len(walls)} walls  (meters, ROS world frame: x right, y up)"
    )
    parts.append(
        f'<text x="{lo_x + font:.3f}" y="{y(hi_y) + font * 1.6:.3f}" font-size="{font * 1.3:.3f}" '
        f'fill="{INK}">{escape(title)}</text>'
    )
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("paths", nargs="+", type=Path, help="*.top_down.json files")
    parser.add_argument("--out", type=Path, help="output directory (default: next to each input)")
    parser.add_argument("--width-px", type=int, default=1600)
    parser.add_argument(
        "--min-label-m", type=float, default=0.35, help="smallest footprint side that gets a label"
    )
    args = parser.parse_args(argv)
    for path in args.paths:
        view, rects = load(path)
        out_dir = args.out or path.parent
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / path.name.replace(".json", ".svg")
        out.write_text(render(view, rects, width_px=args.width_px, min_label_m=args.min_label_m))
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
