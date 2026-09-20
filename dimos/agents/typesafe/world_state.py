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
"""Fold the latest perception messages into the JSON state the model reads.

Everything is relative to the robot: no world coordinates or headings reach the model.
"""

from __future__ import annotations

from collections import deque
from itertools import pairwise
import math
import re
from typing import TYPE_CHECKING, Any, TypedDict

import numpy as np
from typing_extensions import NotRequired

from dimos.msgs.sensor_msgs.PointCloud2 import SECTOR_NAMES
from dimos.msgs.vision_msgs.Detection2DArray import BBoxJson

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
    from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray

MAX_OBJECTS = 20  # 2D
MAX_OBSTACLES = 5
MAX_LABEL = 24  # catalogue titles run to a sentence; the target keeps its full label
OBSTACLE_RANGE_M = 4.0
# ponytail: pose z above the floor (habitat, Go2 base_link); config if a robot differs.
BASE_ABOVE_FLOOR_M = 0.15
BODY_BAND_M = (0.1, 0.9)  # above the floor: lower is driven over, higher is driven under
MIN_FOOTPRINT_M = 0.2
_WALL = re.compile(r"\bwall\b", re.I)
WALL_WORD = "wall"
_OVERHEAD = re.compile(r"\b(ceiling|floor)\b", re.I)  # a room's own surfaces are not obstacles
RECENT_S = 8.0  # how far back `robot.recent` looks
STILL_S = 1.5  # standing this long from the start already reads still
SAME_WAY_DEG = 60.0  # a way past that moved less than this between ticks is the same one
FORGET_SIDE_S = 3.0  # the line clear this long: the side gone around by is forgotten
# ponytail: one body radius for every robot; config if a wider one drives this.
ROBOT_RADIUS_M = 0.25
OPEN_M = 1.5  # a direction is open when nothing solid lies within this range
CORNER_JUMP_M = 1.0  # neighbouring rays this different in range: a corner with space behind it
ADJACENT_M = 0.3  # furniture this close to the target's box belongs to it
STOPPED_BY_M = 0.5  # such furniture this close on the line has stopped the robot ...
AT_TARGET_M = 0.6  # ... and is in the way when the target's edge is still further than this
AHEAD_DEG = 15.0
WALL_RUN_M = 1.0  # a wall box at least this long (and twice its thickness) is a run of wall
DOORWAY_MIN_M = 0.7  # a gap in a run of wall the robot fits through ...
DOORWAY_MAX_M = 3.0  # ... and wider than this it is open room, not an opening in a wall
JAMB_M = 0.45  # a wide opening is passed no closer than this to its jambs
THROUGH_M = 0.5  # the points in front of and beyond an opening that bearings are taken to
BLOCKED_M = 0.5  # nearer than this reads blocked
TRAIL_STEP_M = 0.5  # the robot's own trail is kept at this spacing ...
TRAIL_AGE_S = 10.0  # ... and counts as `been_there` once it is this old
BEEN_M = 0.75  # a way whose far point is this close to the trail has been driven already
PROBE_M = 2.0  # how far along an open direction its far point is taken
SCAN_CELL_M = 0.1  # what the scan saw is remembered on this grid ...
SCAN_KEEP_S = 30.0  # ... this long ...
SCAN_KEEP_M = 2.5  # ... from returns no further than this
STUCK_S = 3.0  # driving picks for this long ...
STUCK_M = 0.15  # ... with less than this moved: stuck
_RAY_STEP_DEG = 5
Box = tuple[float, float, float, float]  # xmin, ymin, xmax, ymax, world frame
_BEARINGS_2D = ("far_left", "left", "center", "right", "far_right")
_SIZES_2D = ((0.4, "filling_view"), (0.15, "large"), (0.03, "medium"), (0.0, "small"))
_GOAL_XY = re.compile(r"\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")
_GOAL_AT = re.compile(r"\s+at\s+\([^)]*\)")
_DISTANCES = ((0.5, "touching"), (1.5, "near"), (4.0, "mid"), (math.inf, "far"))


class ObjectState(TypedDict):
    label: str
    bearing: str
    target: NotRequired[bool]
    distance: NotRequired[str]  # 3D only
    distance_m: NotRequired[float]
    bearing_deg: NotRequired[float]
    width_m: NotRequired[float]
    size: NotRequired[str]  # 2D only
    bbox: NotRequired[BBoxJson]
    score: NotRequired[float]  # 2D only


class RobotState(TypedDict, total=False):
    motion: str
    last_drive: dict[str, str]
    recent: dict[str, Any]


class WorldState(TypedDict):
    task: NotRequired[str]
    goal: str
    robot: RobotState
    objects: list[ObjectState]
    way_to_target: NotRequired[dict[str, Any]]
    free_space: NotRequired[dict[str, dict[str, Any]]]
    unavailable: NotRequired[list[str]]


def bearing_word(rel_angle: float) -> str:
    """8-way word for an angle in the robot frame (0 = ahead, +pi/2 = left). `ahead` is only
    +-AHEAD_DEG, so steering until a bearing reads ahead lines the robot up with it; behind
    always says on which hand, so it asks for one turn, not either."""
    deg = _wrap_deg(rel_angle)
    if abs(deg) <= AHEAD_DEG:
        return "ahead"
    word = SECTOR_NAMES[round(rel_angle / (math.pi / 4)) % 8]
    if word in ("ahead", "behind"):
        return f"{word}_left" if deg > 0 else f"{word}_right"
    return word


def _wrap_deg(angle: float) -> float:
    return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))


def distance_word(d: float) -> str:
    return next(word for limit, word in _DISTANCES if d < limit)


def _objects_3d(
    dets: Detection3DArray, pose: PoseStamped, goal: str
) -> tuple[list[ObjectState], list[tuple[str, Box]], Box | None]:
    """The target first, then the nearest things a ground robot on this floor can run into;
    every solid footprint on this floor (walls included); the target's footprint."""
    floor = pose.z - BASE_ABOVE_FLOOR_M
    raw = dets.to_json()
    named = [d for d in raw if d["label"] and d["label"].lower() in goal.lower()]
    at = _GOAL_XY.search(goal)
    if at and named:  # a goal with coordinates names one object
        gx, gy = float(at.group(1)), float(at.group(2))
        named = [
            min(named, key=lambda d: math.hypot(d["position"]["x"] - gx, d["position"]["y"] - gy))
        ]
    # Same-label others would be taken for the one the coordinates single out.
    twin_label = named[0]["label"] if at and named else None
    out: list[ObjectState] = []
    solids: list[tuple[str, Box]] = []
    target_box: Box | None = None
    for d in raw:
        is_target = any(d is n for n in named)
        p, s = d["position"], d["size"]
        box = (p["x"] - s["x"] / 2, p["y"] - s["y"] / 2, p["x"] + s["x"] / 2, p["y"] + s["y"] / 2)
        if not is_target and (
            p["z"] - s["z"] / 2 > floor + BODY_BAND_M[1]
            or p["z"] + s["z"] / 2 < floor + BODY_BAND_M[0]
            or max(s["x"], s["y"]) < MIN_FOOTPRINT_M
            or d["label"] == twin_label
            or _OVERHEAD.search(d["label"])
        ):
            continue
        dx, dy = p["x"] - pose.x, p["y"] - pose.y
        # To the box's nearest edge: the centre of a large object is never reachable.
        dist = math.hypot(max(0.0, abs(dx) - s["x"] / 2), max(0.0, abs(dy) - s["y"] / 2))
        # Standing inside a box means it is not solid there (rug, region, loose bounding box).
        if not is_target and dist == 0.0:
            continue
        if is_target:
            target_box = target_box or box
        else:
            short = d["label"][:MAX_LABEL]  # unless that cuts the word that makes it a wall
            cut = bool(_WALL.search(short)) == bool(_WALL.search(d["label"]))
            solids.append((short if cut else d["label"], box))
        if (not is_target and _WALL.search(d["label"])) or (
            not is_target and dist > OBSTACLE_RANGE_M
        ):
            continue  # walls crowd the list: they are named in way_to_target and free_space
        rel = math.atan2(dy, dx) - pose.yaw
        obj: ObjectState = {
            "label": d["label"] if is_target else d["label"][:MAX_LABEL],
            "bearing": bearing_word(rel),
            "distance_m": round(dist, 2),
            "width_m": round(max(s["x"], s["y"]), 1),
        }
        if is_target:  # only the target is steered by: the rest are told in fewer words
            obj = {
                "target": True,
                **obj,
                "bearing_deg": round(_wrap_deg(rel)),
                "distance": distance_word(dist),
            }
        out.append(obj)
    out.sort(key=lambda o: (not o.get("target", False), o["distance_m"]))
    n_targets = sum(1 for o in out if o.get("target"))
    return out[: n_targets + MAX_OBSTACLES], solids, target_box


def ray_cast(
    boxes: list[Box], x: float, y: float, angles: np.ndarray, grow: float
) -> tuple[np.ndarray, np.ndarray]:
    """First hit of each world-frame ray on the boxes grown by `grow`: (range, box index or -1).
    A box the origin is inside is not grown; if the origin is inside even then, it is ignored."""
    if not boxes:
        return np.full(len(angles), np.inf), np.full(len(angles), -1)
    b = np.array(boxes, dtype=float)
    lo, hi, o = b[:, :2] - grow, b[:, 2:] + grow, np.array([x, y])
    inside = ((lo < o) & (o < hi)).all(axis=1)
    lo[inside], hi[inside] = b[inside, :2], b[inside, 2:]
    skip = ((lo < o) & (o < hi)).all(axis=1)
    d = np.column_stack((np.cos(angles), np.sin(angles)))[:, None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo - o) / d, (hi - o) / d  # rays x boxes x 2
    near = np.nanmax(np.minimum(t1, t2), axis=2)
    far = np.nanmin(np.maximum(t1, t2), axis=2)
    t = np.where((far >= np.maximum(near, 0.0)) & ~skip, np.maximum(near, 0.0), np.inf)
    idx = t.argmin(axis=1)
    rng = t[np.arange(len(angles)), idx]
    return rng, np.where(np.isfinite(rng), idx, -1)


class Memory:
    """What the robot itself did lately, kept by the agent between ticks: poses for
    `robot.recent`, and the open side its own picks have been steering to."""

    def __init__(self) -> None:
        self._goal = ""
        self._reset()

    def _reset(self) -> None:
        self._poses: deque[tuple[float, float, float, float, float]] = deque()
        self._trail: list[tuple[float, float, float]] = []  # t, x, y: everywhere it has driven
        self._scan: dict[tuple[int, int], float] = {}  # cell -> when the scan last hit it
        self._pushing_since: float | None = None  # picks have driven (x or y) since then
        self._offered: dict[str, float] = {}  # side -> bearing_deg, as read on the last tick
        self._world: dict[str, float] = {}  # the same, as world-frame degrees
        self._side: str | None = None
        self._blocker = ""
        self._since = 0.0
        self._clear_since: float | None = None

    def seen_lately(
        self, now: float, pose: PoseStamped, bearing: np.ndarray, r: np.ndarray
    ) -> np.ndarray:
        """`scan_reach` bins of what the scan hit lately near the robot, whichever way it faces
        now; `bearing`, `r`: this tick's solid returns (robot frame), remembered from here on."""
        self._scan = {c: t for c, t in self._scan.items() if now - t <= SCAN_KEEP_S}
        keep = r < SCAN_KEEP_M
        wx = pose.x + r[keep] * np.cos(bearing[keep] + pose.yaw)
        wy = pose.y + r[keep] * np.sin(bearing[keep] + pose.yaw)
        cells = np.unique(np.round(np.column_stack((wx, wy)) / SCAN_CELL_M).astype(int), axis=0)
        self._scan.update({(int(i), int(j)): now for i, j in cells})
        out = np.full(360 // _RAY_STEP_DEG, np.inf)
        if self._scan:
            c = np.array(list(self._scan)) * SCAN_CELL_M - np.array([pose.x, pose.y])
            d = np.hypot(c[:, 0], c[:, 1])
            near = (d > SCAN_CELL_M) & (d < 1.0)  # only the near field decides anything
            rel = np.degrees(np.arctan2(c[near, 1], c[near, 0]) - pose.yaw)
            np.minimum.at(out, np.round(rel / _RAY_STEP_DEG).astype(int) % len(out), d[near])
        return out

    def recent(
        self,
        now: float,
        goal: str,
        pose: PoseStamped,
        target_m: float,
        last_drive: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        if goal != self._goal:
            self._reset()
            self._goal = goal
        self._poses.append((now, pose.x, pose.y, pose.yaw, target_m))
        if (
            not self._trail
            or math.hypot(pose.x - self._trail[-1][1], pose.y - self._trail[-1][2]) >= TRAIL_STEP_M
        ):
            self._trail.append((now, pose.x, pose.y))
        while now - self._poses[0][0] > RECENT_S:
            self._poses.popleft()
        p = list(self._poses)
        turned = sum(abs(_wrap_deg(b[3] - a[3])) for a, b in pairwise(p))
        net = math.hypot(p[-1][1] - p[0][1], p[-1][2] - p[0][2])
        closer = p[0][4] - p[-1][4]
        still = net < 0.3 and turned < 45
        last = last_drive or {}  # backing away is the way out of stuck, not more of it
        if last.get("x") == "forward" or last.get("y", "none") != "none":
            self._pushing_since = now if self._pushing_since is None else self._pushing_since
        else:
            self._pushing_since = None
        was = next((q for q in reversed(p) if now - q[0] >= STUCK_S), None)
        stuck = (
            was is not None
            and self._pushing_since is not None
            and now - self._pushing_since >= STUCK_S
            and math.hypot(p[-1][1] - was[1], p[-1][2] - was[2]) < STUCK_M
        )
        pattern = (
            "starting" if now - p[0][0] < (STILL_S if still else RECENT_S / 2)
            else "advancing" if closer >= 0.5
            else "stuck" if stuck
            else "still" if still
            else "turning_on_the_spot" if net < 0.5 and turned >= 90
            else "moving_without_getting_closer"
        )  # fmt: skip
        return {
            "moved_m": round(net, 1),
            "turned_deg": round(turned),
            "target_closer_m": round(closer, 1),
            "pattern": pattern,
        }

    def going_around(
        self, now: float, way: dict[str, Any], last_drive: dict[str, str] | None
    ) -> dict[str, Any] | None:
        """The side the last pick steered to: the one open side that pick fits (it turned
        toward it, or drove at it when it was ahead). Kept while that side stays listed."""
        if last_drive and self._offered:
            want = {"turn_left": 1, "turn_right": -1}.get(last_drive.get("yaw", ""), 0)
            fits = [
                side
                for side, deg in self._offered.items()
                if (want and want * deg > AHEAD_DEG)
                or (not want and last_drive.get("x") == "forward" and abs(deg) <= AHEAD_DEG)
            ]
            if len(fits) == 1 and self._side is None:  # kept until it closes, never re-read
                self._side, self._since = fits[0], now
        blocker = way.pop("_blocker", self._blocker)
        entries = {o["side"]: o for o in way.get("open_sides", [])}
        for o in entries.values():
            fx, fy = o.pop("_far", (math.inf, math.inf))
            if any(
                now - t > TRAIL_AGE_S and math.hypot(fx - x, fy - y) < BEEN_M
                for t, x, y in self._trail
            ):
                o["been_there"] = True
        offered = {side: o["bearing_deg"] for side, o in entries.items()}
        yaw = math.degrees(self._poses[-1][3]) if self._poses else 0.0
        world = {side: deg + yaw for side, deg in offered.items()}
        if blocker != self._blocker and self._side is not None:
            # Something else blocks now. Two boxes of one obstacle leave the way past where it
            # was; a way past that jumped elsewhere is a new situation and a new choice.
            was, is_ = self._world.get(self._side), world.get(self._side)
            if was is None or is_ is None or abs((is_ - was + 180) % 360 - 180) > SAME_WAY_DEG:
                self._side = None
        self._blocker, self._offered, self._world = blocker, offered, world
        if way["state"] == "clear":
            self._clear_since = self._clear_since if self._clear_since is not None else now
            if now - self._clear_since > FORGET_SIDE_S:
                self._side = None
            return None
        self._clear_since = None
        if self._side not in self._offered:
            self._side = None  # that side closed
            return None
        return {"side": self._side, "for_s": round(now - self._since)}


def doorways(
    walls: list[Box], pose: PoseStamped, toward: tuple[float, float]
) -> list[dict[str, Any]]:
    """Gaps in the straight runs of wall that the robot can see into: the bearing of each
    (of the spot just in front of it, or just beyond it when that line is free; standing in
    it, through it the way the robot faces), its range and width, and whether `toward` lies
    beyond that wall. Single walls as seen from the robot; nothing is searched or chained."""
    lines: list[tuple[int, float, float]] = []  # axis, strip lo, strip hi
    for b in walls:
        ax = 0 if b[2] - b[0] >= b[3] - b[1] else 1
        length, thick = b[ax + 2] - b[ax], b[3 - ax] - b[1 - ax]
        if length < max(WALL_RUN_M, 2 * thick):
            continue  # a pillar or a stub is not a run of wall
        lo, hi = b[1 - ax], b[3 - ax]
        if not any(a == ax and min(hi, h) - max(lo, l) > 0.05 for a, l, h in lines):
            lines.append((ax, lo, hi))
    me = (pose.x, pose.y)
    out: list[dict[str, Any]] = []
    for ax, lo, hi in lines:
        c, half = (lo + hi) / 2, (hi - lo) / 2
        inside = abs(me[1 - ax] - c) < half + ROBOT_RADIUS_M  # in one of this wall's openings
        # The near point is on the robot's side of the wall; standing in the opening, the far
        # point is on the side the robot faces.
        facing = math.sin(pose.yaw) if ax == 0 else math.cos(pose.yaw)
        side = (1.0 if me[1 - ax] > c else -1.0) if not inside else (-1.0 if facing > 0 else 1.0)
        spans = sorted(
            (w[ax], w[ax + 2]) for w in walls if min(hi, w[3 - ax]) - max(lo, w[1 - ax]) > 0.02
        )
        runs = [list(spans[0])]
        for a, b_ in spans[1:]:
            if a <= runs[-1][1] + 0.05:
                runs[-1][1] = max(runs[-1][1], b_)
            else:
                runs.append([a, b_])
        beyond = (toward[1 - ax] - c) * side < 0
        # Where the line to `toward` crosses this wall, else abreast of the robot.
        d = toward[1 - ax] - me[1 - ax]
        k = (c - me[1 - ax]) / d if beyond and d else 0.0
        ref = me[ax] + k * (toward[ax] - me[ax])
        for (_, g0), (g1, _) in pairwise(runs):
            width = g1 - g0
            if not DOORWAY_MIN_M <= width <= DOORWAY_MAX_M or (inside and not g0 < me[ax] < g1):
                continue
            u = (g0 + g1) / 2 if width < 2 * JAMB_M else min(max(ref, g0 + JAMB_M), g1 - JAMB_M)
            pts = []
            for off in (side * (half + THROUGH_M), -side * (half + THROUGH_M)):
                p = [0.0, 0.0]
                p[ax], p[1 - ax] = u, c + off
                pts.append((p[0] - me[0], p[1] - me[1]))
            (mx, my), (fx, fy) = pts
            if not inside:
                seen, _ = ray_cast(
                    walls, me[0], me[1], np.array([math.atan2(my, mx)]), ROBOT_RADIUS_M
                )
                if seen[0] < math.hypot(mx, my) - 0.05:
                    continue  # no body-wide straight line to it past the walls
            thru, _ = ray_cast(walls, me[0], me[1], np.array([math.atan2(fy, fx)]), ROBOT_RADIUS_M)
            straight = inside or thru[0] >= math.hypot(fx, fy) - 0.05
            ax_, ay_ = (fx, fy) if straight else (mx, my)
            angle = math.atan2(ay_, ax_)
            if any(abs(_wrap_deg(angle - float(o["angle"]))) < 5.0 for o in out):
                continue  # the same opening, from a second box of the same wall
            out.append(
                {
                    "angle": angle,
                    "aim_m": math.hypot(ax_, ay_),
                    "range_m": round(math.hypot(u - me[ax], c - me[1 - ax]), 1),
                    "width_m": round(width, 1),
                    "target_beyond": bool(beyond),
                    "far": (me[0] + fx, me[1] + fy),
                }
            )
    return out


def way_to_target(
    solids: list[tuple[str, Box]],
    target: Box,
    pose: PoseStamped,
    scan: np.ndarray | None = None,
) -> dict[str, Any]:
    """Is the straight line to the target free, what stands on it, and on each side of it the
    nearest opening in a wall (when a wall is on the line) or else the nearest direction that
    is open or grazes a corner with space behind it. Single rays and single walls as seen
    from the robot: a description of what surrounds it, not a route. `scan` (`scan_reach`)
    is what the depth scan reads: nothing it reads blocked is listed as a way."""

    def seen(angles: np.ndarray) -> np.ndarray:
        """The scan's nearest return along world-frame angles, a body (three bins) wide."""
        if scan is None:
            return np.full(len(angles), np.inf)
        k = np.round(np.degrees(angles - pose.yaw) / _RAY_STEP_DEG).astype(int)
        n = len(scan)
        near: np.ndarray = np.minimum(scan[k % n], np.minimum(scan[(k - 1) % n], scan[(k + 1) % n]))
        return near

    tx0, ty0, tx1, ty1 = target
    to_target = math.atan2((ty0 + ty1) / 2 - pose.y, (tx0 + tx1) / 2 - pose.x)
    line = np.array([to_target])
    reach, _ = ray_cast([target], pose.x, pose.y, line, 0.0)
    edge = math.hypot(max(tx0 - pose.x, 0.0, pose.x - tx1), max(ty0 - pose.y, 0.0, pose.y - ty1))
    if edge < _DISTANCES[0][0]:
        return {"state": "clear", "room": "blocked"}  # touching it: no way left to describe
    keep = solids
    rng, idx = ray_cast([b for _, b in keep], pose.x, pose.y, line, ROBOT_RADIUS_M)
    if not (rng[0] <= STOPPED_BY_M and edge - rng[0] > AT_TARGET_M):
        # Furniture standing at the target is in the way only once it has stopped the robot
        # short of the target; from further off it is part of where the robot is going.
        keep = [
            (label, b)
            for label, b in solids
            if _WALL.search(label)
            or max(b[0] - tx1, tx0 - b[2], b[1] - ty1, ty0 - b[3]) > ADJACENT_M
        ]
        rng, idx = ray_cast([b for _, b in keep], pose.x, pose.y, line, ROBOT_RADIUS_M)
    boxes = [b for _, b in keep]
    bare, _ = ray_cast(boxes, pose.x, pose.y, line, 0.0)
    looked = float(seen(line)[0])
    by, at, blocker = "", 0.0, ""
    # What the target stands against (its wall) is grown to in front of its face: not a blocker.
    if not (rng[0] < reach[0] - ROBOT_RADIUS_M or bare[0] < reach[0]):
        ahead, _ = ray_cast([b for _, b in solids], pose.x, pose.y, line, ROBOT_RADIUS_M)
        if not (looked < BLOCKED_M and edge - looked > AT_TARGET_M):
            room = float(min(ahead[0], reach[0], looked))
            return {"state": "clear", "room": _sector(room)["state"]}
        # Only the scan sees it, well short of the target: a blocker like any other.
        by, at, blocker = "obstacle", looked, "scan"
    elif bare[0] >= reach[0] and edge < OPEN_M:
        # The last stretch: nothing stands on the line, something beside it narrows it.
        b = boxes[idx[0]]
        dx, dy = (b[0] + b[2]) / 2 - pose.x, (b[1] + b[3]) / 2 - pose.y
        left = math.cos(to_target) * dy - math.sin(to_target) * dx > 0
        return {
            "state": "clear",
            "room": _sector(float(min(rng[0], looked)))["state"],
            "narrowed_on": "left" if left else "right",
        }
    doors: list[dict[str, Any]] = []
    walls = [b for label, b in solids if _WALL.search(label)]
    wall_at, wall_idx = ray_cast(walls, pose.x, pose.y, line, 0.0)
    if not by:
        by, at, blocker = keep[idx[0]][0], float(rng[0]), f"{keep[idx[0]]}"
    if wall_at[0] < reach[0]:
        # Whatever else stands in front of it, a wall on the line is passed by its openings.
        by, at = WALL_WORD, max(0.0, float(wall_at[0]) - ROBOT_RADIUS_M)
        blocker = f"{by}{walls[wall_idx[0]]}"
        doors = doorways(walls, pose, ((tx0 + tx1) / 2, (ty0 + ty1) / 2))
        if doors:  # only those with a free straight line to them past the furniture as well
            to_doors = np.array([d["angle"] for d in doors])
            along, _ = ray_cast(boxes, pose.x, pose.y, to_doors, ROBOT_RADIUS_M)
            doors = [
                d
                for d, r, v in zip(doors, along, seen(to_doors), strict=True)
                if r >= d["aim_m"] - 0.1 and v >= BLOCKED_M
            ]
    # Open means: leads past the first thing on the line (which may stand before a wall).
    need = max(OPEN_M, (looked if by == "obstacle" else float(rng[0])) + 1.0)
    offsets = np.radians(np.arange(0, 180 + 2 * _RAY_STEP_DEG, _RAY_STEP_DEG))
    sides: list[dict[str, Any]] = []
    spare: list[dict[str, Any]] = []  # per side the longest free direction, if nothing is open

    def ray_entry(side: str, sign: int, k: int, free: np.ndarray, kind: str) -> dict[str, Any]:
        angle = to_target + sign * offsets[k]
        far = min(float(free[k]), PROBE_M)
        return {
            "side": side,
            "kind": kind,
            "bearing": bearing_word(angle - pose.yaw),
            "bearing_deg": round(_wrap_deg(angle - pose.yaw)),
            "detour_deg": round(math.degrees(offsets[k])),
            "clear_m": round(float(free[k]), 1),
            "_far": (pose.x + far * math.cos(angle), pose.y + far * math.sin(angle)),
        }

    for side, sign in (("left", 1), ("right", -1)):
        # A wall is passed by its openings: on this side of the line, the one nearest the line.
        mine = [
            (off, d)
            for d in doors
            if 0.0 < (off := sign * _wrap_deg(d["angle"] - to_target)) <= 150.0
        ]
        if mine:
            off, d = min(mine, key=lambda m: m[0])
            rel = d["angle"] - pose.yaw
            sides.append(
                {
                    "side": side,
                    "kind": "doorway",
                    "bearing": bearing_word(rel),
                    "bearing_deg": round(_wrap_deg(rel)),
                    "detour_deg": round(off),
                    "range_m": d["range_m"],
                    "width_m": d["width_m"],
                    "target_beyond": d["target_beyond"],
                    "_far": d["far"],
                }
            )
            continue
        rays = to_target + sign * offsets
        free, _ = ray_cast(boxes, pose.x, pose.y, rays, ROBOT_RADIUS_M)
        free = np.where(seen(rays) < BLOCKED_M, 0.0, np.minimum(free, 9.9))
        for k in range(1, len(offsets) - 1):
            corner = (
                free[k] >= 1.0
                and max(free[k] - free[k - 1], free[k] - free[k + 1]) >= CORNER_JUMP_M
            )
            if free[k] >= need or corner:
                sides.append(
                    ray_entry(side, sign, k, free, "open" if free[k] >= need else "corner")
                )
                break
        else:
            k = 1 + int(free[1:-1].argmax())
            if free[k] >= BLOCKED_M:
                spare.append(ray_entry(side, sign, k, free, "free"))  # not a way past: just room
    return {
        "state": "blocked",
        "_blocker": blocker,
        "blocked_by": by,
        "blocked_at_m": round(at, 1),
        # Fixed order, left then right: choosing is the model's. Never empty while anything
        # around is free: a robot with nothing to steer by stands still for good.
        "open_sides": sides or spare,
    }


def map_free_space(
    solids: list[tuple[str, Box]], pose: PoseStamped, max_range: float
) -> dict[str, tuple[float, str]]:
    """Nearest footprint per 45 deg sector, all around: (range, label)."""
    rel = np.radians(np.arange(-180, 180, _RAY_STEP_DEG))
    rng, idx = ray_cast([b for _, b in solids], pose.x, pose.y, rel + pose.yaw, 0.0)
    sector = np.round(rel / (np.pi / 4)).astype(int) % 8
    out: dict[str, tuple[float, str]] = {}
    for i, name in enumerate(SECTOR_NAMES):
        k = np.flatnonzero(sector == i)
        j = k[rng[k].argmin()]
        if rng[j] < max_range:
            out[name] = (float(rng[j]), solids[idx[j]][0])
    return out


def _objects_2d(dets: Detection2DArray, image_size: tuple[int, int]) -> list[ObjectState]:
    w, h = image_size
    out: list[ObjectState] = []
    for d in dets.to_json():
        area = d["bbox"]["w"] * d["bbox"]["h"] / (w * h)
        out.append(
            {
                "label": d["label"],
                "score": d["score"],
                "bbox": d["bbox"],
                "bearing": _BEARINGS_2D[min(4, int(5 * d["bbox"]["cx"] / w))],
                "size": next(word for limit, word in _SIZES_2D if area > limit),
            }
        )
    return sorted(out, key=lambda o: -o["bbox"]["w"] * o["bbox"]["h"])[:MAX_OBJECTS]


def _sector(clear_m: float, by: str | None = None) -> dict[str, Any]:
    state = "blocked" if clear_m < BLOCKED_M else "tight" if clear_m < 1.0 else "clear"
    out: dict[str, Any] = {"clear_m": round(clear_m, 1), "state": state}
    return {**out, "by": by} if by else out


def _scan_polar(
    lidar: PointCloud2, origin: PoseStamped | None, z_min: float, z_max: float, max_range: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """The scan in the robot frame: bearing and range of every point, which are returns at
    all, and which of those are solid (in the body's height band, within range)."""
    pts = lidar.points_f32().astype(np.float64)
    if origin is not None:
        pts = pts - np.array([origin.x, origin.y, origin.z])
        c, s = np.cos(-origin.yaw), np.sin(-origin.yaw)
        pts = np.column_stack(
            (c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1], pts[:, 2])
        )
    r = np.hypot(pts[:, 0], pts[:, 1])
    seen = r > 0.05
    solid = seen & (pts[:, 2] > z_min) & (pts[:, 2] < z_max) & (r < max_range)
    return np.arctan2(pts[:, 1], pts[:, 0]), r, seen, solid


def scan_reach(
    lidar: PointCloud2, origin: PoseStamped | None, z_min: float, z_max: float, max_range: float
) -> np.ndarray:
    """Nearest solid return per _RAY_STEP_DEG of robot-frame bearing; inf where the scan has
    none (not looked at, or nothing within range): there the footprints stand alone."""
    bearing, r, _, solid = _scan_polar(lidar, origin, z_min, z_max, max_range)
    return _bins(bearing[solid], r[solid])


def _bins(bearing: np.ndarray, r: np.ndarray) -> np.ndarray:
    out = np.full(360 // _RAY_STEP_DEG, np.inf)
    idx = np.round(np.degrees(bearing) / _RAY_STEP_DEG).astype(int) % len(out)
    np.minimum.at(out, idx, r)
    return out


def free_space(
    lidar: PointCloud2 | None,
    origin: PoseStamped | None,
    z_min: float,
    z_max: float,
    max_range: float,
    known: dict[str, tuple[float, str]] | None = None,
) -> dict[str, dict[str, Any]]:
    """Nearest obstacle per 45 deg sector from the scan and the `known` footprints. Without
    footprints a sector the scan has no return from at all (floor included) was not looked
    at and reads `unseen`, never clear."""
    if lidar is None:
        return {n: _sector(*(known or {}).get(n, (max_range, None))) for n in SECTOR_NAMES}
    bearing, r, seen, solid = _scan_polar(lidar, origin, z_min, z_max, max_range)
    sector = np.round(bearing / (np.pi / 4)).astype(int) % 8
    out: dict[str, dict[str, Any]] = {}
    for i, name in enumerate(SECTOR_NAMES):
        map_m, by = (known or {}).get(name, (max_range, None))
        if not (seen & (sector == i)).any():
            out[name] = {"state": "unseen"} if known is None else _sector(map_m, by)
            continue
        hits = r[solid & (sector == i)]
        clear_m = min(float(hits.min()) if hits.size else max_range, map_m)
        # The name belongs to the reading only when the footprint is about where the scan hit.
        out[name] = _sector(clear_m, by if map_m <= clear_m + 0.5 else None)
    return out


def build_world_state(
    goal: str,
    pose: PoseStamped,
    *,
    task: str | None = None,
    detections_3d: Detection3DArray | None,
    detections_2d: Detection2DArray | None,
    lidar: PointCloud2 | None,
    robot: RobotState,
    image_size: tuple[int, int] = (1280, 720),
    lidar_band: tuple[float, float, float] = (-0.2, 0.8, 5.0),
    memory: Memory | None = None,
    now: float = 0.0,
) -> WorldState:
    known: dict[str, tuple[float, str]] | None = None
    way: dict[str, Any] | None = None
    # A scan in the robot's own frame (habitat `lidar`, base_link) needs no transform.
    origin = pose if lidar is None or lidar.frame_id == pose.frame_id else None
    if detections_3d is not None:
        objects, solids, target_box = _objects_3d(detections_3d, pose, goal)
        everything = solids
        if target_box is not None:
            scan = None
            if lidar is not None:
                bearing, r, _, solid = _scan_polar(lidar, origin, *lidar_band)
                scan = _bins(bearing[solid], r[solid])
                if memory is not None:  # what it saw lately counts when it looks away
                    lately = memory.seen_lately(now, pose, bearing[solid], r[solid])
                    scan = np.minimum(scan, lately)
            way = way_to_target(solids, target_box, pose, scan)
            everything = [*solids, (objects[0]["label"], target_box)]
            if memory is not None:
                robot = {
                    **robot,
                    "recent": memory.recent(
                        now, goal, pose, objects[0]["distance_m"], robot.get("last_drive")
                    ),
                }
                going = memory.going_around(now, way, robot.get("last_drive"))
                if going:
                    way = {**way, "going_around": going}
        known = map_free_space(everything, pose, lidar_band[2])
    elif detections_2d is not None:
        objects = _objects_2d(detections_2d, image_size)
    else:
        objects = []
    state: WorldState = {"goal": _GOAL_AT.sub("", goal), "robot": {**robot}, "objects": objects}
    if task:
        state = {"task": task, **state}
    if way is not None:
        way.pop("_blocker", None)
        for o in way.get("open_sides", []):
            o.pop("_far", None)
        state["way_to_target"] = way
    if lidar is not None or known is not None:
        state["free_space"] = free_space(lidar, origin, *lidar_band, known=known)
    else:
        state["unavailable"] = ["free_space"]
    return state
