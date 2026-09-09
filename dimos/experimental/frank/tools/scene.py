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

"""What the camera sees, with the map drawn on it: one photo, eight rays, the frontier edges.

    uv run python dimos/experimental/frank/tools/scene.py see scene.jpg   # then Read the JPEG
    uv run python dimos/experimental/frank/tools/scene.py see --n 4        # more frontier edges
    uv run python dimos/experimental/frank/tools/scene.py see --to 2.0 -1.5  # also draw the corridor you would walk to x=2 y=-1.5

Each ray from `grid.py look` and each edge from `grid.py frontier` that falls inside the camera's
view is drawn as a vertical line at the pixel column its bearing lands on, so "ahead-left 2.3 m
(unseen)" points at the actual gap in the picture. Edges behind the camera are listed at the
bottom with their direction word. The text printed alongside says the same thing in words.
With `--to x y` the robot-wide corridor to that point is drawn on the floor in red, so you can
check with your own eyes that nothing the lidar misses (a chair leg, a cable, a bag, glass) is
inside it before you walk. Read-only; the picture is stale the moment you turn.
"""

from __future__ import annotations

import argparse
import math
import sys

import cv2
import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import grid

from dimos.porcelain.dimos import Dimos
from dimos.robot.unitree.go2.connection import _camera_info_static

CAMERA_HEIGHT = 0.30  # metres above the floor, standing level
RAY = (60, 200, 255)  # BGR: rays in yellow
PATH = (60, 60, 230)  # the corridor in red
EDGE = (80, 220, 80)  # frontier edges in green
TEXT = (255, 255, 255)


def load(timeout: float = 20.0) -> tuple[grid.Space, np.ndarray]:
    app = Dimos.connect(timeout=10.0)
    try:
        og = app.peek_stream("global_costmap", timeout=timeout)
        od = app.peek_stream("odom", timeout=timeout)
        img = app.peek_stream("color_image", timeout=timeout)
        if og is None or od is None or img is None:
            missing = "global_costmap" if og is None else "odom" if od is None else "color_image"
            raise SystemExit(f"nothing on the {missing} stream yet")
    finally:
        app.stop()
    yaw = float(od.yaw)
    yaw = math.degrees(yaw) if abs(yaw) < 7 else yaw
    return grid.Space(og, float(od.position.x), float(od.position.y), yaw), img.to_opencv()


def column(bearing_deg: float, width: int) -> int | None:
    """Pixel column for a bearing left of the nose (positive = left), or None if out of view."""
    info = _camera_info_static()
    scale = width / info.width
    fx, cx = info.K[0] * scale, info.K[2] * scale
    half = math.degrees(math.atan2(cx, fx))
    if abs(bearing_deg) >= half:
        return None
    return round(cx - fx * math.tan(math.radians(bearing_deg)))


def ground_pixel(
    ahead: float, left: float, width: int, height: int, pitch_deg: float = 0.0
) -> tuple[int, int] | None:
    """Pixel of a floor point `ahead` m in front of the nose and `left` m to its left."""
    info = _camera_info_static()
    scale = width / info.width
    fx, fy, cx, cy = info.K[0] * scale, info.K[4] * scale, info.K[2] * scale, info.K[5] * scale
    xc, yc, zc = ahead - 0.3, left, -CAMERA_HEIGHT  # camera_link: x forward, y left, z up
    p = math.radians(pitch_deg)  # nose-up pitch tilts the view up (odom pitch is negative nose-up)
    xc, zc = xc * math.cos(p) - zc * math.sin(p), xc * math.sin(p) + zc * math.cos(p)
    if xc < 0.15:
        return None
    u = cx + fx * (-yc) / xc
    v = cy + fy * (-zc) / xc
    if not (0 <= u < width and 0 <= v < height):
        return None
    return int(u), int(v)


def draw_corridor(space: grid.Space, out: np.ndarray, x: float, y: float) -> str:
    """The robot-wide strip of floor from here to (x, y), in red. Returns one line of text."""
    h, w = out.shape[:2]
    length = math.hypot(x - space.x, y - space.y)
    rel = math.degrees(math.atan2(y - space.y, x - space.x)) - space.yaw
    rel = (rel + 180.0) % 360.0 - 180.0
    if abs(rel) > 60:
        return f"corridor to x={x:.1f} y={y:.1f} is {rel:+.0f} deg off the nose: turn to face it before you can see it"
    ca, sa = math.cos(math.radians(rel)), math.sin(math.radians(rel))
    for side in (grid.ROBOT_RADIUS, -grid.ROBOT_RADIUS):
        pts = []
        d = 0.3
        while d <= length:
            px = ground_pixel(d * ca - side * sa, d * sa + side * ca, w, h)
            if px is not None:
                pts.append(px)
            d += 0.1
        if len(pts) > 1:
            cv2.polylines(out, [np.array(pts, dtype=np.int32)], False, PATH, 2)
    end = ground_pixel(length * ca, length * sa, w, h)
    if end is not None:
        cv2.circle(out, end, 8, PATH, 2)
        cv2.putText(
            out,
            f"goal {length:.1f}m",
            (end[0] + 10, end[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            PATH,
            2,
        )
    return f"corridor to x={x:.1f} y={y:.1f} ({length:.1f} m, {rel:+.0f} deg off the nose) drawn in red; anything inside it is in your way"


def annotate(
    space: grid.Space, frame: np.ndarray, n_edges: int, to: tuple[float, float] | None = None
) -> tuple[np.ndarray, list[str]]:
    h, w = frame.shape[:2]
    out = frame.copy()
    lines: list[str] = []
    if to is not None:
        lines.append(draw_corridor(space, out, *to))
    for name, deg in grid.DIRECTIONS:
        dist, why = space.ray(deg, relative=True)
        rel = (deg + 180.0) % 360.0 - 180.0
        col = column(rel, w)
        if col is None:
            continue
        cv2.line(out, (col, 0), (col, h - 1), RAY, 1)
        label = f"{name} {dist:.1f}m {why}"
        cv2.putText(out, label, (max(2, col - 60), 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, RAY, 1)
        lines.append(f"{label}: in the photo at column {col} of {w}")
    edges = space.frontiers(n_edges)
    behind = []
    for i, (x, y, width, length, where) in enumerate(edges):
        rel = math.degrees(math.atan2(y - space.y, x - space.x)) - space.yaw
        rel = (rel + 180.0) % 360.0 - 180.0
        dist = math.hypot(x - space.x, y - space.y)
        text = f"edge {i + 1}: x={x:.1f} y={y:.1f}, {width:.1f} m wide, {length:.1f} m of path, {where}"
        col = column(rel, w)
        if col is None:
            behind.append(text)
            lines.append(text + " (not in the photo)")
            continue
        cv2.line(out, (col, h // 3), (col, h - 1), EDGE, 2)
        cv2.putText(
            out,
            f"edge {i + 1} {dist:.1f}m",
            (max(2, col - 40), h // 3 - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            EDGE,
            2,
        )
        lines.append(text + f" (in the photo at column {col}, {dist:.1f} m straight-line)")
    for j, text in enumerate(behind):
        cv2.putText(
            out,
            "not in view: " + text,
            (8, h - 10 - 18 * j),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            EDGE,
            1,
        )
    cv2.putText(
        out,
        f"x={space.x:.1f} y={space.y:.1f} yaw={space.yaw:.0f}",
        (8, h - 10 - 18 * len(behind) - 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        TEXT,
        1,
    )
    return out, lines


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("see")
    s.add_argument("out", nargs="?", default="scene.jpg")
    s.add_argument("--n", type=int, default=3, help="frontier edges to draw")
    s.add_argument(
        "--to",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        help="draw the corridor to this world point",
    )
    a = p.parse_args(argv)
    space, frame = load()
    out, lines = annotate(space, frame, a.n, tuple(a.to) if a.to else None)
    cv2.imwrite(a.out, out)
    print(
        f"{a.out} - camera view from x={space.x:.1f} y={space.y:.1f} yaw={space.yaw:.0f}, rays yellow, frontier edges green, corridor red"
    )
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
