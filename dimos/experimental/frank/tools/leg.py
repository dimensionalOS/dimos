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

"""One snapshot, every check: is the leg to (x, y) safe, and what is around me right now.

    uv run python dimos/experimental/frank/tools/leg.py check 2.0 -1.5          # text + leg.jpg + leg_map.png, then Read both
    uv run python dimos/experimental/frank/tools/leg.py check 2.0 -1.5 --json   # {"text": ..., "image": path, "map": path}
    uv run python dimos/experimental/frank/tools/leg.py check                   # no leg: just pose, rays, edges, the two pictures
    uv run python dimos/experimental/frank/tools/leg.py check 2.0 -1.5 --no-turn  # do not turn to face the leg first

With x, y the robot first turns in place to face the leg when it is more than 60 degrees off the
nose, so the photo covers it; that is the only thing here that moves anything.

Two pictures decide a leg: the camera frame with the corridor drawn on the floor (what the lidar
misses: chair legs, cables, bags, glass) and a crop of the map around the robot with the same leg
drawn at the robot's width (what the camera cannot see: walls behind it, how much room there is).
Bundles `grid.py path`, `grid.py look`, `grid.py frontier`, `grid.py crop`, and `scene.py see --to`
from a single read of the map, pose, and camera. In the Pi harness this is the `check_leg` tool
(pi_tools.ts), which returns the text and both pictures together.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import cv2

sys.path.insert(0, str(Path(__file__).parent))
import robot
import scene

CACHE = Path(__file__).resolve().parents[1] / "cache"


FACE_WITHIN = 60.0  # degrees off the nose the camera still covers the leg


def face(x: float, y: float) -> str | None:
    """Turn in place so the camera covers the leg to (x, y). Returns a line of text if it turned."""
    try:
        px, py, yaw = robot.pose()
        off = (math.degrees(math.atan2(y - py, x - px)) - yaw + 180.0) % 360.0 - 180.0
        if abs(off) <= FACE_WITHIN:
            return None
        if robot.MOTION_OFF.exists():
            return f"the leg is {off:+.0f} deg off the nose and motion is off, so this photo does not cover it"
        return robot.face(at=(x, y)) + " (to face the leg first)"
    finally:
        robot.close()


def check(
    x: float | None,
    y: float | None,
    out: str,
    n_edges: int = 3,
    around: float = 3.0,
    turn: bool = True,
) -> tuple[str, str, str]:
    """Returns (text, photo path, map crop path)."""
    to = (x, y) if x is not None and y is not None else None
    turned = face(x, y) if to is not None and turn else None
    space, frame = scene.load()
    picture, photo_lines = scene.annotate(space, frame, n_edges, to)
    cv2.imwrite(out, picture)
    crop = space.crop(str(Path(out).with_name(Path(out).stem + "_map.png")), around, to)

    lines = [f"you: x={space.x:.1f} y={space.y:.1f} yaw={space.yaw:.0f} deg (world frame)"]
    if turned:
        lines.append(turned)
    if to is not None:
        verdict, length, tightest, stop, why = space.path(x, y)
        ok, plan_len = space.reachable(x, y)
        if stop is None:
            leg = f"{verdict}: straight line to x={x:.1f} y={y:.1f}, {length:.1f} m, tightest gap {tightest:.2f} m ({why})"
        else:
            leg = f"{verdict}: straight line to x={x:.1f} y={y:.1f}, {length:.1f} m, stops at {stop:.1f} m ({why})"
        planner = (
            f"planner: {'a path of about ' + f'{plan_len:.1f}' + ' m exists' if ok else 'no path'}"
        )
        lines.append(f"map says: {leg}. {planner}.")
        lines.append(
            "decide from the two pictures. blocked = the lidar saw a wall inside your width: no. "
            "unseen = nobody has scanned that floor yet: fine if the photo shows plain floor the whole "
            "way and the leg is short. clear or tight: walk it unless the photo shows something in the corridor."
        )
    lines.append(
        "rays: " + ", ".join(f"{name} {dist:.1f} m ({why})" for name, dist, why in space.look())
    )
    edges = space.frontiers(n_edges)
    if edges:
        lines.append(
            "edges into unseen space: "
            + "; ".join(
                f"x={ex:.1f} y={ey:.1f} {w:.1f} m wide, {length:.1f} m of path, {where}"
                for ex, ey, w, length, where in edges
            )
        )
    else:
        lines.append("edges into unseen space: none reachable")
    lines.append(
        f"photo {out}: rays yellow, edges green" + (", corridor red" if to is not None else "")
    )
    lines += photo_lines
    lines.append(
        f"map {crop}: {around:.0f} m around you, north up; white floor, black wall, grey unseen, "
        "pink floor you do not fit on, orange you, yellow wedge = what the photo covers"
        + (", red = the leg at your width" if to is not None else "")
    )
    return "\n".join(lines), out, crop


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check")
    c.add_argument("x", type=float, nargs="?")
    c.add_argument("y", type=float, nargs="?")
    c.add_argument("--out", default=str(CACHE / "leg.jpg"))
    c.add_argument("--n", type=int, default=3)
    c.add_argument("--around", type=float, default=3.0, help="metres of map around you in the crop")
    c.add_argument("--no-turn", action="store_true", help="do not turn to face the leg first")
    c.add_argument("--json", action="store_true")
    a = p.parse_args(argv)
    if (a.x is None) != (a.y is None):
        p.error("give both x and y, or neither")
    CACHE.mkdir(exist_ok=True)
    text, image, crop = check(a.x, a.y, a.out, a.n, a.around, not a.no_turn)
    if a.json:
        print(json.dumps({"text": text, "image": image, "map": crop}))
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
