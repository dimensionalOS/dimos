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

"""What the space around FRANK looks like: the 2D occupancy grid, in plain answers.

Reads the `global_costmap` and `odom` streams of the running DimOS instance. Never moves anything.

    uv run python dimos/experimental/frank/tools/grid.py render map.png    # plan view, then Read it
    uv run python dimos/experimental/frank/tools/grid.py crop leg_map.png [--around 3] [--to 2.0 -1.5]  # the map right around you, big, with a leg drawn
    uv run python dimos/experimental/frank/tools/grid.py look              # eight rays around the robot
    uv run python dimos/experimental/frank/tools/grid.py ray 0 [--relative]
    uv run python dimos/experimental/frank/tools/grid.py at 2.0 -1.5
    uv run python dimos/experimental/frank/tools/grid.py nearest-free 2.0 -1.5
    uv run python dimos/experimental/frank/tools/grid.py reachable 2.0 -1.5
    uv run python dimos/experimental/frank/tools/grid.py open-spots [--n 3]
    uv run python dimos/experimental/frank/tools/grid.py frontier [--n 3]     # edges of the map, where to go to see more
    uv run python dimos/experimental/frank/tools/grid.py path 2.0 -1.5        # is the straight line there safe: clear | tight | blocked | unseen
    uv run python dimos/experimental/frank/tools/grid.py front 2.0 -1.5 90 1.5

Or import it:

    import sys; sys.path.insert(0, "dimos/experimental/frank/tools"); import grid
    s = grid.load()            # one snapshot of the map and the pose
    s.look(); s.ray(0); s.reachable(2.0, -1.5); s.render("map.png")

All coordinates are the world frame `robot.py move x y` uses: +x east, +y north, metres,
heading 0 = +x, 90 = +y. Unknown space is never traversable, but it is reported as "unknown"
rather than "wall" so you can go and look instead of giving up.
"""

from __future__ import annotations

import argparse
import heapq
import math
import sys
from typing import Any

import cv2
import numpy as np
from scipy import ndimage

from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.porcelain.dimos import Dimos

OCCUPIED_COST = 50  # same cut the nav stack uses (navigation/replanning_a_star/goal_validator.py)
ROBOT_RADIUS = 0.35  # Go2 half-width, metres
DIRECTIONS = [
    ("ahead", 0),
    ("ahead-left", 45),
    ("left", 90),
    ("behind-left", 135),
    ("behind", 180),
    ("behind-right", 225),
    ("right", 270),
    ("ahead-right", 315),
]


def load(timeout: float = 20.0) -> Space:
    """One snapshot of the costmap and the robot pose from the running instance."""
    app = Dimos.connect(timeout=10.0)
    try:
        og = app.peek_stream("global_costmap", timeout=timeout)
        od = app.peek_stream("odom", timeout=timeout)
        if og is None or od is None:
            missing = "global_costmap" if og is None else "odom"
            raise SystemExit(
                f"nothing on the {missing} stream yet - the map takes a minute to build"
            )
        yaw = float(od.yaw)
        yaw = math.degrees(yaw) if abs(yaw) < 7 else yaw
        return Space(og, float(od.position.x), float(od.position.y), yaw)
    finally:
        app.stop()


class Space:
    """A costmap snapshot plus the robot pose, with the questions FRANK actually asks."""

    def __init__(self, og: OccupancyGrid, x: float, y: float, yaw_deg: float) -> None:
        self.og = og
        self.x, self.y, self.yaw = x, y, yaw_deg
        self.res = og.resolution
        self.ox = og.origin.position.x
        self.oy = og.origin.position.y
        g = og.grid
        self.occupied = g >= OCCUPIED_COST
        self.unknown = g == CostValues.UNKNOWN
        # The lidar cannot see under the robot, so its own footprint usually reads unknown.
        # The robot is standing there: that patch is floor.
        self.unknown &= ~self._disk(x, y, ROBOT_RADIUS)
        self.free = ~self.occupied & ~self.unknown
        # For planning: grow obstacles by the robot's half-width, and never cross unknown.
        grown = _dilate(self.occupied, ROBOT_RADIUS / self.res)
        self.open = self.free & ~grown
        # Metres from every cell to the nearest wall (unknown does not count as a wall here).
        self.clearance = ndimage.distance_transform_edt(~self.occupied) * self.res

    # --- geometry ---------------------------------------------------------------

    def cell(self, x: float, y: float) -> tuple[int, int]:
        """Grid (row, col) for a world point. May be outside the map."""
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def world(self, row: int, col: int) -> tuple[float, float]:
        """World centre of a cell."""
        return self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res

    def inside(self, row: int, col: int) -> bool:
        return 0 <= row < self.og.height and 0 <= col < self.og.width

    def _disk(self, x: float, y: float, radius: float) -> np.ndarray:
        """Mask of cells within radius metres of a world point."""
        rows, cols = np.ogrid[: self.og.height, : self.og.width]
        wx = self.ox + (cols + 0.5) * self.res
        wy = self.oy + (rows + 0.5) * self.res
        return (wx - x) ** 2 + (wy - y) ** 2 <= radius**2

    # --- questions --------------------------------------------------------------

    def at(self, x: float, y: float) -> str:
        """ "free" | "occupied" | "unknown" | "outside map" at a world point."""
        r, c = self.cell(x, y)
        if not self.inside(r, c):
            return "outside map"
        if self.occupied[r, c]:
            return "occupied"
        if self.unknown[r, c]:
            return "unknown"
        return "free"

    def ray(self, heading_deg: float, relative: bool = False) -> tuple[float, str]:
        """How far the robot can travel on a heading before something stops it.

        Returns (metres, reason) where reason is "wall", "unknown" or "edge of map".
        With relative=True the heading is measured from where the robot is facing.
        """
        head = heading_deg + (self.yaw if relative else 0.0)
        dx, dy = math.cos(math.radians(head)), math.sin(math.radians(head))
        step = self.res / 2
        d = 0.0
        while True:
            d += step
            r, c = self.cell(self.x + dx * d, self.y + dy * d)
            if not self.inside(r, c):
                return round(d - step, 1), "edge of map"
            if self.occupied[r, c]:
                return round(d - step, 1), "wall"
            if self.unknown[r, c]:
                return round(d - step, 1), "unknown"

    def look(self) -> list[tuple[str, float, str]]:
        """Eight rays around the robot, starting straight ahead: (label, metres, reason)."""
        return [(name, *self.ray(deg, relative=True)) for name, deg in DIRECTIONS]

    def nearest_free(self, x: float, y: float) -> tuple[float, float, float] | None:
        """Closest cell the robot fits in, to a world point: (x, y, clearance in m)."""
        rows, cols = np.nonzero(self.open)
        if len(rows) == 0:
            return None
        wx = self.ox + (cols + 0.5) * self.res
        wy = self.oy + (rows + 0.5) * self.res
        i = int(np.argmin((wx - x) ** 2 + (wy - y) ** 2))
        return float(wx[i]), float(wy[i]), float(self.clearance[rows[i], cols[i]])

    def reachable(self, x: float, y: float) -> tuple[bool, float]:
        """Is there a path over open cells from the robot to there? (yes/no, rough metres)."""
        goal = self.nearest_free(x, y)
        if goal is None:
            return False, 0.0
        start = self._start_cell()
        if start is None:
            return False, 0.0
        target = self.cell(goal[0], goal[1])
        length = self._path_length(start, target)
        if length is None:
            return False, 0.0
        return True, round(length, 1)

    def open_spots(self, n: int = 3) -> list[tuple[float, float, float, float]]:
        """A few roomy places: (x, y, radius, distance from robot), nearest first."""
        room = np.where(self.open, self.clearance, 0.0)
        spots = []
        for _ in range(n):
            r, c = np.unravel_index(int(np.argmax(room)), room.shape)
            radius = float(room[r, c])
            if radius < ROBOT_RADIUS:
                break
            sx, sy = self.world(int(r), int(c))
            spots.append((sx, sy, round(radius, 1), round(math.hypot(sx - self.x, sy - self.y), 1)))
            _blank(room, int(r), int(c), radius / self.res)
        return sorted(spots, key=lambda s: s[3])

    def path(self, x: float, y: float) -> tuple[str, float, float, float | None, str]:
        """Is the straight line from the robot to (x, y) safe to walk?

        (verdict, length in m, tightest clearance in m, distance at which it stops, why).
        verdict: "clear" (the robot fits the whole way with room), "tight" (fits, but
        closer than 0.5 m to something), "blocked" (a wall inside the robot's width) or
        "unseen" (crosses space nobody has scanned). Straight legs only; the planner
        bends around what the map knows, but this is the line you asked for.
        """
        length = math.hypot(x - self.x, y - self.y)
        if length < 1e-6:
            return (
                "clear",
                0.0,
                float(self.clearance[self.cell(self.x, self.y)])
                if self.inside(*self.cell(self.x, self.y))
                else 0.0,
                None,
                "already there",
            )
        dx, dy = (x - self.x) / length, (y - self.y) / length
        step = self.res / 2
        tightest = math.inf
        d = 0.0
        while d <= length:
            r, c = self.cell(self.x + dx * d, self.y + dy * d)
            if not self.inside(r, c):
                return (
                    "unseen",
                    round(length, 1),
                    round(min(tightest, 9.9), 2),
                    round(d, 1),
                    "edge of map",
                )
            if self.unknown[r, c]:
                return (
                    "unseen",
                    round(length, 1),
                    round(min(tightest, 9.9), 2),
                    round(d, 1),
                    "unscanned space",
                )
            clear = float(self.clearance[r, c])
            if clear < ROBOT_RADIUS:
                return (
                    "blocked",
                    round(length, 1),
                    round(clear, 2),
                    round(d, 1),
                    "wall inside the robot's width",
                )
            tightest = min(tightest, clear)
            d += step
        verdict = "clear" if tightest >= ROBOT_RADIUS + 0.15 else "tight"
        return verdict, round(length, 1), round(tightest, 2), None, "fits the whole way"

    def frontiers(self, n: int = 3) -> list[tuple[float, float, float, float, str]]:
        """Edges of the map: open places next to space nobody has seen yet.

        (x, y, width in m, path length in m, direction word), shortest path first. Only
        edges the robot can walk to and stand at. Where a person says they are but the map
        shows nothing, this is the list to pick from.
        """
        edge = self.free & _dilate(self.unknown, 1)
        labels, count = ndimage.label(edge, structure=np.ones((3, 3)))
        start = self._start_cell()
        if count == 0 or start is None:
            return []
        out = []
        for k in range(1, count + 1):
            part = labels == k
            if not np.any(part & self.open):
                continue  # a crack the robot cannot stand in
            rows, cols = np.nonzero(part)
            width = float(max(np.ptp(rows), np.ptp(cols)) + 1) * self.res
            if width < 2 * ROBOT_RADIUS:
                continue  # narrower than the robot: not a way through
            cx, cy = self.world(int(np.median(rows)), int(np.median(cols)))
            stand = self.nearest_free(cx, cy)
            if stand is None:
                continue
            length = self._path_length(start, self.cell(stand[0], stand[1]))
            if length is None:
                continue
            out.append(
                (
                    stand[0],
                    stand[1],
                    round(width, 1),
                    round(length, 1),
                    self.direction(stand[0], stand[1]),
                )
            )
        return sorted(out, key=lambda f: f[3])[:n]

    def direction(self, x: float, y: float) -> str:
        """Which way a world point is from the robot's nose: ahead, left, behind-right, ..."""
        bearing = math.degrees(math.atan2(y - self.y, x - self.x)) - self.yaw
        bearing = (bearing + 180.0) % 360.0 - 180.0
        return min(DIRECTIONS, key=lambda d: abs(((d[1] - bearing + 180.0) % 360.0) - 180.0))[0]

    def front(self, x: float, y: float, yaw_deg: float, d: float) -> tuple[float, float]:
        """The world point d metres in front of a pose facing yaw_deg."""
        return x + d * math.cos(math.radians(yaw_deg)), y + d * math.sin(math.radians(yaw_deg))

    # --- render -----------------------------------------------------------------

    def crop(
        self, path: str = "leg_map.png", around: float = 3.0, to: tuple[float, float] | None = None
    ) -> str:
        """The map right around the robot, north up, big enough to read: white floor, black
        wall, grey unseen, pink floor too close to a wall for the robot's width. The robot is
        the orange dot with a nose, the camera's view is the yellow wedge (that is the photo),
        and the leg to `to` is drawn in red at the robot's width. Returns the path."""
        ppm = 800.0 / (2 * around)  # pixels per metre
        size = int(2 * around * ppm)
        left, top = self.x - around, self.y + around

        # colour every cell, then resample the window around the robot
        img = np.full((*self.og.grid.shape, 3), 255, dtype=np.uint8)
        img[self.free & ~self.open] = (205, 205, 255)
        img[self.unknown] = (165, 165, 165)
        img[self.occupied] = (20, 20, 20)
        r0, c0 = self.cell(left, self.y - around)
        r1, c1 = self.cell(self.x + around, top)
        cells = round(2 * around / self.res)
        window = np.full((cells, cells, 3), 165, dtype=np.uint8)
        rs, cs = (
            slice(max(r0, 0), min(r1, self.og.height)),
            slice(max(c0, 0), min(c1, self.og.width)),
        )
        window[rs.start - r0 : rs.stop - r0, cs.start - c0 : cs.stop - c0] = img[rs, cs]
        out = cv2.resize(window[::-1], (size, size), interpolation=cv2.INTER_NEAREST)  # north up

        def px(wx: float, wy: float) -> tuple[int, int]:
            return int((wx - left) * ppm), int((top - wy) * ppm)

        for m in range(math.ceil(left), math.floor(self.x + around) + 1):  # 1 m grid, world axes
            u, _ = px(m, 0)
            cv2.line(out, (u, 0), (u, size - 1), (200, 160, 120), 1)
            cv2.putText(
                out, f"x={m}", (u + 3, size - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 90, 40), 1
            )
        for m in range(math.ceil(self.y - around), math.floor(top) + 1):
            _, v = px(0, m)
            cv2.line(out, (0, v), (size - 1, v), (200, 160, 120), 1)
            cv2.putText(out, f"y={m}", (3, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 90, 40), 1)

        if to is not None:  # the leg, robot-wide, in red
            tx, ty = to
            length = math.hypot(tx - self.x, ty - self.y)
            if length > 1e-6:
                nx, ny = (
                    -(ty - self.y) / length * ROBOT_RADIUS,
                    (tx - self.x) / length * ROBOT_RADIUS,
                )
                for sx, sy in ((nx, ny), (-nx, -ny)):
                    cv2.line(
                        out, px(self.x + sx, self.y + sy), px(tx + sx, ty + sy), (60, 60, 230), 2
                    )
            cv2.circle(out, px(tx, ty), 7, (60, 60, 230), 2)
            cv2.putText(
                out,
                f"x={tx:.1f} y={ty:.1f}",
                (px(tx, ty)[0] + 10, px(tx, ty)[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (60, 60, 230),
                2,
            )

        centre = px(self.x, self.y)
        for sign in (1, -1):  # camera wedge: what the photo covers
            a = math.radians(self.yaw + sign * 39)
            cv2.line(
                out,
                centre,
                px(self.x + around * math.cos(a), self.y + around * math.sin(a)),
                (60, 200, 255),
                2,
            )
        cv2.circle(out, centre, int(ROBOT_RADIUS * ppm), (0, 120, 240), -1)
        nose = math.radians(self.yaw)
        cv2.arrowedLine(
            out,
            centre,
            px(self.x + 0.6 * math.cos(nose), self.y + 0.6 * math.sin(nose)),
            (0, 120, 240),
            4,
            tipLength=0.35,
        )
        cv2.rectangle(out, (0, 0), (size - 1, 42), (245, 245, 245), -1)
        cv2.putText(
            out,
            "north up, 1 m squares. white floor, black wall, grey unseen, pink too near a wall for you",
            (8, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            1,
        )
        cv2.putText(
            out,
            "orange you, yellow wedge = the photo's view, red = the leg at your width",
            (8, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            1,
        )
        cv2.imwrite(path, out)
        return path

    def render(self, path: str = "map.png", trail: list[tuple[float, float]] | None = None) -> str:
        """Plan view of the map, north up, with the robot and its eight rays. Returns the path.
        `trail` is a list of world (x, y) points to draw as the path walked, oldest first."""
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        picture = np.full((*self.og.grid.shape, 3), 0.65)  # unknown: grey
        picture[self.free] = 1.0  # free: white
        picture[self.occupied] = 0.15  # occupied: near-black
        left, right = self.ox, self.ox + self.og.width * self.res
        bottom, top = self.oy, self.oy + self.og.height * self.res

        fig, ax = plt.subplots(figsize=(8, 8), dpi=130)
        ax.imshow(
            picture, origin="lower", extent=(left, right, bottom, top), interpolation="nearest"
        )
        ax.set_xticks(np.arange(math.ceil(left), right, 1.0))
        ax.set_yticks(np.arange(math.ceil(bottom), top, 1.0))
        ax.grid(color="#4488cc", lw=0.4, alpha=0.4)
        ax.set_xlabel("world x (m, east)")
        ax.set_ylabel("world y (m, north)")

        for name, dist, why in self.look():
            head = math.radians(dict(DIRECTIONS)[name] + self.yaw)
            ax.plot(
                [self.x, self.x + dist * math.cos(head)],
                [self.y, self.y + dist * math.sin(head)],
                color="#cc2222" if why == "wall" else "#2266cc",
                lw=0.8,
                alpha=0.45,
            )
        if trail:
            ax.plot(
                [t[0] for t in trail], [t[1] for t in trail], color="#2266cc", lw=1.4, alpha=0.8
            )
            ax.plot(trail[0][0], trail[0][1], "o", color="#2266cc", ms=5)
        nose = math.radians(self.yaw)
        ax.arrow(
            self.x,
            self.y,
            0.5 * math.cos(nose),
            0.5 * math.sin(nose),
            width=0.08,
            color="#ee7700",
            length_includes_head=True,
            zorder=5,
        )
        ax.set_title(
            f"robot at x={self.x:.1f} y={self.y:.1f} yaw={self.yaw:.0f}deg, "
            f"{self.res * 100:.0f} cm cells, grid lines 1 m"
        )
        fig.savefig(path, bbox_inches="tight")
        plt.close(fig)
        return path

    # --- internals --------------------------------------------------------------

    def _start_cell(self) -> tuple[int, int] | None:
        """The robot's own cell, or the nearest open one if it reads as blocked."""
        r, c = self.cell(self.x, self.y)
        if self.inside(r, c) and self.open[r, c]:
            return r, c
        near = self.nearest_free(self.x, self.y)
        if near is None:
            return None
        r, c = self.cell(near[0], near[1])
        return (r, c) if self.inside(r, c) else None

    def _path_length(self, start: tuple[int, int], goal: tuple[int, int]) -> float | None:
        """Shortest way over open cells, in metres, or None if the goal is cut off."""
        best = np.full(self.open.shape, np.inf)
        best[start] = 0.0
        queue = [(0.0, start)]
        while queue:
            dist, (r, c) = heapq.heappop(queue)
            if (r, c) == goal:
                return dist
            if dist > best[r, c]:
                continue
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if (dr or dc) and self.inside(nr, nc) and self.open[nr, nc]:
                        step = self.res * math.hypot(dr, dc)
                        if dist + step < best[nr, nc]:
                            best[nr, nc] = dist + step
                            heapq.heappush(queue, (dist + step, (nr, nc)))
        return None


def _dilate(mask: np.ndarray, radius_cells: float) -> np.ndarray:
    """Grow a mask by a disc of the given radius, in cells."""
    n = max(1, round(radius_cells))
    yy, xx = np.mgrid[-n : n + 1, -n : n + 1]
    disc = yy**2 + xx**2 <= n**2
    return ndimage.binary_dilation(mask, structure=disc)


def _blank(room: np.ndarray, r: int, c: int, radius_cells: float) -> None:
    """Zero a disc around a spot so the next pick is somewhere else."""
    n = max(1, int(radius_cells))
    room[max(0, r - n) : r + n + 1, max(0, c - n) : c + n + 1] = 0.0


# --- module-level convenience, one snapshot shared by all of them -------------------

_space: Space | None = None


def _current() -> Space:
    global _space
    if _space is None:
        _space = load()
    return _space


def at(x: float, y: float) -> str:
    return _current().at(x, y)


def ray(heading_deg: float, relative: bool = False) -> tuple[float, str]:
    return _current().ray(heading_deg, relative)


def look() -> list[tuple[str, float, str]]:
    return _current().look()


def nearest_free(x: float, y: float) -> tuple[float, float, float] | None:
    return _current().nearest_free(x, y)


def reachable(x: float, y: float) -> tuple[bool, float]:
    return _current().reachable(x, y)


def open_spots(n: int = 3) -> list[tuple[float, float, float, float]]:
    return _current().open_spots(n)


def frontiers(n: int = 3) -> list[tuple[float, float, float, float, str]]:
    return _current().frontiers(n)


def path(x: float, y: float) -> tuple[str, float, float, float | None, str]:
    return _current().path(x, y)


def front(x: float, y: float, yaw_deg: float, d: float) -> tuple[float, float]:
    return _current().front(x, y, yaw_deg, d)


def render(path: str = "map.png") -> str:
    return _current().render(path)


def crop(
    path: str = "leg_map.png", around: float = 3.0, to: tuple[float, float] | None = None
) -> str:
    return _current().crop(path, around, to)


# --- cli ---------------------------------------------------------------------------

INFLATED = (
    f"(obstacles grown by the robot's {ROBOT_RADIUS} m half-width; unknown counts as blocked)"
)


def _print(cmd: str, a: Any, s: Space) -> None:
    if cmd == "render":
        path = s.render(a.out)
        print(f"{path} - plan view, north up, robot arrow at x={s.x:.1f} y={s.y:.1f}")
    elif cmd == "crop":
        to = tuple(a.to) if a.to else None
        path = s.crop(a.out, a.around, to)
        print(
            f"{path} - {a.around:.0f} m around you at x={s.x:.1f} y={s.y:.1f}, north up; white floor, "
            "black wall, grey unseen, pink floor you do not fit on, orange you, yellow wedge = camera view"
            + (f", red = the leg to x={to[0]:.1f} y={to[1]:.1f}" if to else "")
        )
    elif cmd == "at":
        print(f"{s.at(a.x, a.y)} at world x={a.x:.1f} y={a.y:.1f}")
    elif cmd == "ray":
        dist, why = s.ray(a.heading, a.relative)
        frame = "from straight ahead" if a.relative else "world heading"
        print(f"{dist:.1f} m free at {a.heading:.0f} deg ({frame}), stopped by {why}")
    elif cmd == "look":
        print(
            f"from x={s.x:.1f} y={s.y:.1f} yaw={s.yaw:.0f} deg (world frame), relative to the nose:"
        )
        print(", ".join(f"{name} {dist:.1f} m ({why})" for name, dist, why in s.look()))
    elif cmd == "nearest-free":
        spot = s.nearest_free(a.x, a.y)
        if spot is None:
            print("no free space in the map at all")
        else:
            print(
                f"x={spot[0]:.1f} y={spot[1]:.1f} world, {spot[2]:.1f} m clearance, "
                f"{math.hypot(spot[0] - a.x, spot[1] - a.y):.1f} m from the point asked. {INFLATED}"
            )
    elif cmd == "reachable":
        ok, length = s.reachable(a.x, a.y)
        where = f"x={a.x:.1f} y={a.y:.1f} world"
        print(
            f"yes, about {length:.1f} m of path to {where}. {INFLATED}"
            if ok
            else f"no path to {where}. {INFLATED}"
        )
    elif cmd == "open-spots":
        spots = s.open_spots(a.n)
        if not spots:
            print(f"no open area big enough for the robot. {INFLATED}")
        else:
            for sx, sy, radius, dist in spots:
                print(
                    f"x={sx:.1f} y={sy:.1f} world, {radius:.1f} m radius of room, {dist:.1f} m away"
                )
            print(INFLATED)
    elif cmd == "path":
        verdict, length, tightest, stop, why = s.path(a.x, a.y)
        where = f"straight line to x={a.x:.1f} y={a.y:.1f}, {length:.1f} m"
        if stop is None:
            print(f"{verdict}: {where}, tightest gap {tightest:.2f} m from a wall ({why}).")
        else:
            print(f"{verdict}: {where}, stops at {stop:.1f} m ({why}). Do not walk this line.")
        print(
            "(clear = walk it; tight = walk slowly, check the photo; blocked/unseen = pick another leg)"
        )
    elif cmd == "frontier":
        edges = s.frontiers(a.n)
        if not edges:
            print("no reachable edge into unseen space; the map is closed or you are boxed in")
        else:
            for fx, fy, width, length, where in edges:
                print(
                    f"x={fx:.1f} y={fy:.1f} world, {width:.1f} m wide edge into unseen space, "
                    f"{where}, {length:.1f} m of path"
                )
            print(
                "(unseen means nobody has looked there yet; stand at the edge and look, then ask again)"
            )
    elif cmd == "front":
        fx, fy = s.front(a.x, a.y, a.yaw, a.d)
        print(f"x={fx:.1f} y={fy:.1f} world, {s.at(fx, fy)} there")


def _main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("render")
    r.add_argument("out", nargs="?", default="map.png")
    cp = sub.add_parser("crop")
    cp.add_argument("out", nargs="?", default="leg_map.png")
    cp.add_argument("--around", type=float, default=3.0, help="metres around the robot")
    cp.add_argument(
        "--to", type=float, nargs=2, metavar=("X", "Y"), help="draw the leg to this world point"
    )
    q = sub.add_parser("at")
    q.add_argument("x", type=float)
    q.add_argument("y", type=float)
    ry = sub.add_parser("ray")
    ry.add_argument("heading", type=float, help="degrees, 0 = +x world")
    ry.add_argument("--relative", action="store_true", help="0 = straight ahead instead")
    sub.add_parser("look")
    nf = sub.add_parser("nearest-free")
    nf.add_argument("x", type=float)
    nf.add_argument("y", type=float)
    rc = sub.add_parser("reachable")
    rc.add_argument("x", type=float)
    rc.add_argument("y", type=float)
    os_ = sub.add_parser("open-spots")
    os_.add_argument("--n", type=int, default=3)
    pt = sub.add_parser("path")
    pt.add_argument("x", type=float)
    pt.add_argument("y", type=float)
    ft = sub.add_parser("frontier")
    ft.add_argument("--n", type=int, default=3)
    fr = sub.add_parser("front")
    fr.add_argument("x", type=float)
    fr.add_argument("y", type=float)
    fr.add_argument("yaw", type=float)
    fr.add_argument("d", type=float)
    a = p.parse_args(argv)

    _print(a.cmd, a, _current())
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
