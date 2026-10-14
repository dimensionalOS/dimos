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

"""Seeded procedural scenes built from boxes."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import hashlib
import itertools
import json
import math
from typing import Literal

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy import ndimage

from dimos.msgs.sim_msgs.Contacts import Kind

Vec3 = tuple[float, float, float]
Rect = tuple[float, float, float, float]
Point = tuple[float, float]
Family = Literal[
    "office",
    "room",
    "doorway",
    "table",
    "block",
    "overhang",
    "corridor",
    "corridor_turn",
    "scattered",
    "small_object",
    "hidden_object",
    "hidden_clutter",
]

SLAB_THICKNESS = 0.15
WALL_THICKNESS = 0.1
WALL_HEIGHT = 2.6
DOOR_HEIGHT = 2.0
DOOR_WIDTH = (0.8, 1.2)
TABLE_TOP_THICKNESS = 0.04
TABLE_LEG = 0.04
TABLE_SIZE = (1.4, 0.8)
TABLE_HEIGHT = 0.75
BLOCK_SIZE = 0.3
OVERHANG_SIZE = (0.6, 2.0)
OVERHANG_LEG = 0.1
START_CLEARANCE = 1.0
DOOR_CLEARANCE = 0.8
DOOR_SIDE_MARGIN = 0.3
DOOR_CLUTTER_SIZE = (0.3, 0.3, 0.5)
MIN_ROOM_WIDTH = 3.0
PLACEMENT_TRIES = 50
CORRIDOR_WIDTH = (0.8, 1.2)
CORRIDOR_LENGTH = (3.5, 4.5)
CORRIDOR_MOUTH_INSET = 1.0
WINDING_LENGTH = (4.0, 10.0)
WINDING_SEGMENT = (1.5, 3.0)
WINDING_TURN = (math.pi / 6, math.pi / 2)
WINDING_SPAN = 12.0
WINDING_WIDTH_MAX = 1.2
SCATTER_AREA = 64.0
SCATTER_SIDE = (0.15, 0.6)
SCATTER_HEIGHT = (0.05, 1.2)
SCATTER_LANE = 0.4
SCATTER_CELL = 0.05
SCATTER_COUNT = 40
SMALL_OBJECT_LENGTH = (0.08, 0.5)
SMALL_OBJECT_ASPECT = 0.4
SMALL_OBJECT_HEIGHT = 0.1
HIDDEN_CORRIDOR_WIDTH = 1.4
HIDDEN_ALONG = (0.4, 0.7)
SIGHT_SAMPLES = 60
SIGHT_ACROSS = 7
HIDDEN_ROOM_ALONG = 0.5
HIDDEN_CLUTTER_CORRIDOR = 1.0
HIDDEN_CLUTTER_COUNT = 4
HIDDEN_CLUTTER_ASTRIDE = 0.7
HIDDEN_ROOM_DEPTH = (7.0, 9.0)
HIDDEN_ROOM_SPAN = 16.0


@dataclass(frozen=True)
class Box:
    center: Vec3
    half: Vec3
    kind: Kind
    yaw: float = 0.0

    def local(self, points: NDArray[np.float64]) -> NDArray[np.float64]:
        """Points in the box's own frame, whose axes the half sizes lie along."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        turn = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])[
            : points.shape[-1], : points.shape[-1]
        ]
        moved: NDArray[np.float64] = (points - np.array(self.center)[: points.shape[-1]]) @ turn.T
        return moved

    def gap(self, xy: NDArray[np.float64]) -> NDArray[np.float64]:
        """Per point, the horizontal distance to the box's footprint."""
        outside = np.maximum(np.abs(self.local(xy)) - np.array(self.half[:2]), 0.0)
        away: NDArray[np.float64] = np.hypot(outside[..., 0], outside[..., 1])
        return away

    def reach(self) -> Vec3:
        """Half the box's extent along each world axis."""
        c, s = abs(math.cos(self.yaw)), abs(math.sin(self.yaw))
        return (
            c * self.half[0] + s * self.half[1],
            s * self.half[0] + c * self.half[1],
            self.half[2],
        )


@dataclass(frozen=True)
class Door:
    """A doorway in a wall across the given axis, with the opening along the other axis."""

    axis: int
    at: float
    start: float
    width: float


@dataclass
class Scene:
    name: str
    params: dict[str, float] = field(default_factory=dict)
    boxes: list[Box] = field(default_factory=list)
    doors: list[Door] = field(default_factory=list)
    start: Vec3 = (0.0, 0.0, 0.0)

    def add(self, lo: ArrayLike, hi: ArrayLike, kind: Kind) -> None:
        low, high = np.asarray(lo, float), np.asarray(hi, float)
        if np.any(high - low <= 1e-6):
            return
        center, half = (low + high) / 2, (high - low) / 2
        self.boxes.append(
            Box(
                (float(center[0]), float(center[1]), float(center[2])),
                (float(half[0]), float(half[1]), float(half[2])),
                kind,
            )
        )

    def bounds(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        centers = np.array([box.center for box in self.boxes])
        halves = np.array([box.reach() for box in self.boxes])
        return (centers - halves).min(0), (centers + halves).max(0)

    def digest(self) -> str:
        """A hash of the geometry and start, for detecting generator drift."""
        record = {
            "boxes": [
                (box.center, box.half, box.kind, *([box.yaw] if box.yaw else []))
                for box in self.boxes
            ],
            "start": self.start,
        }
        return hashlib.sha256(json.dumps(record, sort_keys=True).encode()).hexdigest()[:16]


def _uniform(rng: np.random.Generator, low: float, high: float) -> float:
    """A uniform draw with the arithmetic done in Python, so every platform rounds it the same way."""
    return low + (high - low) * float(rng.random())


def _given(drawn: float, given: float | None) -> float:
    """The given value over the drawn one. The draw still happens, so the seed's later draws do not shift."""
    return drawn if given is None else given


def _walls(scene: Scene, x0: float, y0: float, x1: float, y1: float, z: float, top: float) -> None:
    t = WALL_THICKNESS
    scene.add((x0 - t, y0 - t, z), (x0, y1 + t, top), "wall")
    scene.add((x1, y0 - t, z), (x1 + t, y1 + t, top), "wall")
    scene.add((x0, y0 - t, z), (x1, y0, top), "wall")
    scene.add((x0, y1, z), (x1, y1 + t, top), "wall")


def _shell(scene: Scene, width: float, length: float, z0: float) -> float:
    """The floor and outer walls of one rectangular floor. Returns the height the walls reach."""
    top = z0 + WALL_HEIGHT
    scene.add((0.0, 0.0, z0 - SLAB_THICKNESS), (width, length, z0), "floor")
    _walls(scene, 0, 0, width, length, z0, top)
    return top


def _wall_with_doors(
    scene: Scene,
    axis: int,
    at: float,
    lo: float,
    hi: float,
    z: float,
    top: float,
    doors: list[Door],
) -> None:
    """A wall across the given axis with door gaps and lintels."""
    t = WALL_THICKNESS
    edges = [lo]
    for door in sorted(doors, key=lambda d: d.start):
        edges += [door.start, door.start + door.width]
    edges.append(hi)
    for i in range(0, len(edges), 2):
        a, b = edges[i], edges[i + 1]
        if axis == 0:
            scene.add((at, a, z), (at + t, b, top), "wall")
        else:
            scene.add((a, at, z), (b, at + t, top), "wall")
    for door in doors:
        lintel = z + DOOR_HEIGHT
        if axis == 0:
            scene.add((at, door.start, lintel), (at + t, door.start + door.width, top), "wall")
        else:
            scene.add((door.start, at, lintel), (door.start + door.width, at + t, top), "wall")
    scene.doors += doors


def _door_zone(door: Door) -> Rect:
    """The passage through a door: the opening plus DOOR_CLEARANCE on both sides of the wall."""
    lo, hi = door.start - DOOR_SIDE_MARGIN, door.start + door.width + DOOR_SIDE_MARGIN
    near, far = door.at - DOOR_CLEARANCE, door.at + WALL_THICKNESS + DOOR_CLEARANCE
    return (near, lo, far, hi) if door.axis == 0 else (lo, near, hi, far)


def _clear(x0: float, y0: float, x1: float, y1: float, keep_clear: list[Rect]) -> bool:
    """Whether a footprint overlaps none of the zones."""
    return not any(
        x0 < kx1 and x1 > kx0 and y0 < ky1 and y1 > ky0 for kx0, ky0, kx1, ky1 in keep_clear
    )


def _joined(free: NDArray[np.bool_], a: tuple[int, int], b: tuple[int, int]) -> bool:
    """Whether two cells lie in one connected patch of free cells."""
    patches, _ = ndimage.label(free)
    return bool(patches[a] and patches[a] == patches[b])


def _clutter(
    scene: Scene,
    rng: np.random.Generator,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    z: float,
    n: int,
    keep_clear: list[Rect],
    scale: float = 1.0,
    height: float = 1.0,
) -> None:
    for _ in range(n):
        for _ in range(PLACEMENT_TRIES):
            hx, hy = scale * _uniform(rng, 0.15, 0.5), scale * _uniform(rng, 0.15, 0.5)
            cx, cy = _uniform(rng, x0 + hx, x1 - hx), _uniform(rng, y0 + hy, y1 - hy)
            fits = 2 * hx < x1 - x0 and 2 * hy < y1 - y0
            if fits and _clear(cx - hx, cy - hy, cx + hx, cy + hy, keep_clear):
                scene.add(
                    (cx - hx, cy - hy, z),
                    (cx + hx, cy + hy, z + height * _uniform(rng, 0.2, 1.0)),
                    "clutter",
                )
                break


def _table(
    scene: Scene,
    rng: np.random.Generator,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    z: float,
    keep_clear: list[Rect],
) -> None:
    """A table on four legs, with its top anywhere from below to well above the robot's height."""
    lx, ly = _uniform(rng, 1.0, 1.8), _uniform(rng, 0.6, 0.9)
    for _ in range(PLACEMENT_TRIES):
        tx, ty = _uniform(rng, x0, x1 - lx), _uniform(rng, y0, y1 - ly)
        if _clear(tx, ty, tx + lx, ty + ly, keep_clear):
            break
    else:
        return
    _table_at(scene, tx, ty, lx, ly, z, z + _uniform(rng, 0.45, 0.8), TABLE_LEG)


def _table_at(
    scene: Scene, tx: float, ty: float, lx: float, ly: float, z: float, top: float, leg: float
) -> None:
    """A table top with a square leg inside each corner."""
    scene.add((tx, ty, top - TABLE_TOP_THICKNESS), (tx + lx, ty + ly, top), "clutter")
    for px in (tx, tx + lx - leg):
        for py in (ty, ty + ly - leg):
            scene.add((px, py, z), (px + leg, py + leg, top - TABLE_TOP_THICKNESS), "clutter")


def _solid(
    scene: Scene, width: float, length: float, z: float, top: float, hollow: list[Rect]
) -> None:
    """Wall filling one rectangular floor everywhere outside the hollows."""
    xs = sorted({0.0, width, *(x for x0, _, x1, _ in hollow for x in (x0, x1))})
    ys = sorted({0.0, length, *(y for _, y0, _, y1 in hollow for y in (y0, y1))})
    for x0, x1 in itertools.pairwise(xs):
        filled = [_clear(x0, y0, x1, y1, hollow) for y0, y1 in itertools.pairwise(ys)]
        for solid, run in itertools.groupby(range(len(filled)), filled.__getitem__):
            if solid:
                strips = list(run)
                scene.add((x0, ys[strips[0]], z), (x1, ys[strips[-1] + 1], top), "wall")


def office(
    seed: int,
    door_width: float | None = None,
    door_clutter: bool = False,
    size: float = 1.0,
    clutter_scale: float = 1.0,
    clutter_height: float = 1.0,
) -> Scene:
    """One floor of rooms joined by doorways, with clutter and tables kept out of the start and the doorways."""
    rng = np.random.default_rng(seed)
    width, length = size * float(_uniform(rng, 12, 18)), size * float(_uniform(rng, 9, 13))
    z0 = float(_uniform(rng, 0.0, 0.08))
    scene = Scene(f"office_{seed}", {"width": width, "length": length, "z0": z0})
    top = _shell(scene, width, length, z0)
    wy = _uniform(rng, 0.4, 0.6) * length
    splits = int(rng.integers(1, 3))
    for _ in range(PLACEMENT_TRIES):
        xs = sorted(_uniform(rng, 0.25, 0.75) * width for _ in range(splits))
        if all(b - a >= MIN_ROOM_WIDTH for a, b in itertools.pairwise(xs)):
            break
    scene.start = (1.0, 1.0, z0)
    sx, sy, _ = scene.start
    keep_clear = [
        (sx - START_CLEARANCE, sy - START_CLEARANCE, sx + START_CLEARANCE, sy + START_CLEARANCE)
    ]
    doors_mid = []
    for a, b in zip([0.0, *xs], [*xs, width], strict=True):
        w = _given(_uniform(rng, *DOOR_WIDTH), door_width)
        if b - a > w + 1.0:
            doors_mid.append(Door(1, wy, _uniform(rng, a + 0.4, b - w - 0.4), w))
    _wall_with_doors(scene, 1, wy, 0, width, z0, top, doors_mid)
    for x in xs:
        for a, b in ((0.0, wy), (wy + WALL_THICKNESS, length)):
            w = _given(_uniform(rng, *DOOR_WIDTH), door_width)
            door = Door(0, x, _uniform(rng, a + 0.3, b - w - 0.3), w)
            _wall_with_doors(scene, 0, x, a, b, z0, top, [door])
    keep_clear += [_door_zone(door) for door in scene.doors]
    clutter = int(rng.integers(8, 16))
    _clutter(
        scene,
        rng,
        0.2,
        0.2,
        width - 0.2,
        length - 0.2,
        z0,
        clutter,
        keep_clear,
        clutter_scale,
        clutter_height,
    )
    tables = int(rng.integers(1, 4))
    for _ in range(tables):
        _table(scene, rng, 0.3, 0.3, width - 0.3, length - 0.3, z0, keep_clear)
    if door_clutter:
        for door in scene.doors:
            _door_clutter(scene, door, z0)
    scene.params.update({"rooms": 2 * (len(xs) + 1), "clutter": clutter, "tables": tables})
    return scene


def _door_clutter(scene: Scene, door: Door, z: float) -> None:
    sx, sy, sz = DOOR_CLUTTER_SIZE
    along = door.at + WALL_THICKNESS + 0.1
    across = door.start
    if door.axis == 0:
        scene.add((along, across, z), (along + sx, across + sy, z + sz), "clutter")
    else:
        scene.add((across, along, z), (across + sx, along + sy, z + sz), "clutter")


def room(seed: int, width: float | None = None, length: float | None = None) -> Scene:
    """One empty walled room. The size is drawn, so the walls fall anywhere against a voxel grid."""
    rng = np.random.default_rng(seed)
    width = _given(_uniform(rng, 6, 8), width)
    length = _given(_uniform(rng, 5, 7), length)
    z0 = _uniform(rng, 0.0, 0.08)
    scene = Scene(f"room_{seed}", {"width": width, "length": length, "z0": z0})
    _shell(scene, width, length, z0)
    scene.start = (1.0, 1.0, z0)
    return scene


def doorway(seed: int, door_width: float | None = None) -> Scene:
    """Two empty rooms joined by one doorway, the only way from the start to the far room."""
    rng = np.random.default_rng(seed)
    width, length = _uniform(rng, 8, 10), _uniform(rng, 5, 7)
    z0 = _uniform(rng, 0.0, 0.08)
    scene = Scene(f"doorway_{seed}", {"width": width, "length": length, "z0": z0})
    top = _shell(scene, width, length, z0)
    at = _uniform(rng, 0.45, 0.55) * width
    along = float(rng.random())
    w = _given(_uniform(rng, *DOOR_WIDTH), door_width)
    door = Door(0, at, 1.0 + along * (length - w - 2.0), w)
    _wall_with_doors(scene, 0, at, 0, length, z0, top, [door])
    scene.start = (1.0, 1.0, z0)
    return scene


def table(seed: int, leg_width: float | None = None) -> Scene:
    """One empty room with a table standing in open floor, taller than the robot."""
    rng = np.random.default_rng(seed)
    width, length = _uniform(rng, 10, 12), _uniform(rng, 8, 10)
    z0 = _uniform(rng, 0.0, 0.08)
    lx, ly = TABLE_SIZE
    tx = _uniform(rng, 0.35, 0.45) * width - lx / 2
    ty = _uniform(rng, 0.35, 0.5) * length - ly / 2
    scene = Scene(
        f"table_{seed}",
        {"width": width, "length": length, "z0": z0, "table_x": tx, "table_y": ty},
    )
    _shell(scene, width, length, z0)
    _table_at(scene, tx, ty, lx, ly, z0, z0 + TABLE_HEIGHT, _given(TABLE_LEG, leg_width))
    scene.start = (1.0, 1.0, z0)
    return scene


def block(seed: int, block_height: float | None = None) -> Scene:
    """One empty room with a square block standing in open floor."""
    rng = np.random.default_rng(seed)
    width, length = _uniform(rng, 10, 12), _uniform(rng, 8, 10)
    z0 = _uniform(rng, 0.0, 0.08)
    x, y = _uniform(rng, 0.35, 0.45) * width, _uniform(rng, 0.35, 0.5) * length
    height = _given(_uniform(rng, 0.1, 0.4), block_height)
    scene = Scene(
        f"block_{seed}", {"width": width, "length": length, "z0": z0, "block_x": x, "block_y": y}
    )
    _shell(scene, width, length, z0)
    half = BLOCK_SIZE / 2
    scene.add((x - half, y - half, z0), (x + half, y + half, z0 + height), "clutter")
    scene.start = (1.0, 1.0, z0)
    return scene


def overhang(seed: int, clearance: float | None = None) -> Scene:
    """One empty room with a long slab on four posts across the open floor, its underside at the clearance."""
    rng = np.random.default_rng(seed)
    width, length = _uniform(rng, 10, 12), _uniform(rng, 8, 10)
    z0 = _uniform(rng, 0.0, 0.08)
    x, y = _uniform(rng, 0.35, 0.45) * width, _uniform(rng, 0.35, 0.5) * length
    clearance = _given(_uniform(rng, 0.3, 0.9), clearance)
    scene = Scene(
        f"overhang_{seed}", {"width": width, "length": length, "z0": z0, "slab_x": x, "slab_y": y}
    )
    _shell(scene, width, length, z0)
    if clearance > 0:
        lx, ly = OVERHANG_SIZE
        top = z0 + clearance + TABLE_TOP_THICKNESS
        _table_at(scene, x - lx / 2, y - ly / 2, lx, ly, z0, top, OVERHANG_LEG)
    scene.start = (1.0, 1.0, z0)
    return scene


def corridor(seed: int, corridor_width: float | None = None) -> Scene:
    """Two empty rooms joined by one straight corridor, the only way from the start to the far room."""
    rng = np.random.default_rng(seed)
    near, run, far = _uniform(rng, 3, 4), _uniform(rng, *CORRIDOR_LENGTH), _uniform(rng, 4, 5)
    width, length = near + run + far, _uniform(rng, 5, 7)
    z0 = _uniform(rng, 0.0, 0.08)
    y = _uniform(rng, CORRIDOR_MOUTH_INSET, length - CORRIDOR_MOUTH_INSET)
    half = _given(_uniform(rng, *CORRIDOR_WIDTH), corridor_width) / 2
    scene = Scene(
        f"corridor_{seed}",
        {
            "width": width,
            "length": length,
            "z0": z0,
            "corridor_x": near,
            "corridor_y": y,
            "room_x": near + run,
            "room_y": 0.0,
        },
    )
    top = _shell(scene, width, length, z0)
    hollow = [
        (0.0, 0.0, near, length),
        (near, y - half, near + run, y + half),
        (near + run, 0.0, width, length),
    ]
    _solid(scene, width, length, z0, top, hollow)
    scene.start = (1.0, 1.0, z0)
    return scene


def _slab(a: Point, b: Point, half: float, back: float = 0.0, on: float = 0.0) -> Box:
    """A flat box from one point to another, this far to each side and past each end."""
    run, yaw = math.dist(a, b), math.atan2(b[1] - a[1], b[0] - a[0])
    mid = (run + on - back) / 2
    center = (a[0] + mid * math.cos(yaw), a[1] + mid * math.sin(yaw), 0.0)
    return Box(center, ((run + on + back) / 2, half, 0.0), "wall", yaw)


def _overlap(a: Box, b: Box) -> bool:
    """Whether two flat boxes share any floor."""
    for box, other in ((a, b), (b, a)):
        x, y = other.half[0], other.half[1]
        turn = other.yaw - box.yaw
        corners = np.array([[x, y], [x, -y], [-x, y], [-x, -y]]) @ np.array(
            [[math.cos(turn), math.sin(turn)], [-math.sin(turn), math.cos(turn)]]
        )
        reach = np.abs(corners).max(axis=0)
        if np.any(np.abs(box.local(np.array(other.center[:2]))) >= reach + np.array(box.half[:2])):
            return False
    return True


def _winding(rng: np.random.Generator) -> tuple[list[Point], float]:
    """A corridor's centerline from the origin heading along x, and the heading it ends on.

    Its length, how many turns it takes and each turn's side and angle are drawn.
    """
    length = _uniform(rng, *WINDING_LENGTH)
    runs: list[float] = []
    while length - sum(runs) >= 2 * WINDING_SEGMENT[0]:
        runs.append(min(_uniform(rng, *WINDING_SEGMENT), length - sum(runs) - WINDING_SEGMENT[0]))
    runs.append(length - sum(runs))
    points, heading = [(0.0, 0.0)], 0.0
    for i, run in enumerate(runs):
        if i:
            heading += _uniform(rng, *WINDING_TURN) * (1 if rng.random() < 0.5 else -1)
        x, y = points[-1]
        points.append((x + run * math.cos(heading), y + run * math.sin(heading)))
    return points, heading


def _edge(points: list[Point], offset: float) -> list[Point]:
    """A centerline moved sideways, with the corners kept sharp."""
    normals = [
        (-(y1 - y0) / math.dist((x0, y0), (x1, y1)), (x1 - x0) / math.dist((x0, y0), (x1, y1)))
        for (x0, y0), (x1, y1) in itertools.pairwise(points)
    ]
    moved = []
    for i, (x, y) in enumerate(points):
        (ax, ay), (bx, by) = normals[max(i - 1, 0)], normals[min(i, len(normals) - 1)]
        scale = offset / (1 + ax * bx + ay * by)
        moved.append((x + scale * (ax + bx), y + scale * (ay + by)))
    return moved


def _sees(points: list[Point], half: float, along: float) -> bool:
    """Whether any of the corridor's mouth has a clear line to its width this far along its last run."""
    floor = [
        _slab(a, b, half, half if i else 0.0, half if i < len(points) - 2 else 0.0)
        for i, (a, b) in enumerate(itertools.pairwise(points))
    ]
    (ax, ay), (bx, by) = points[-2:]
    run = math.dist((ax, ay), (bx, by))
    x, y = ax + along * (bx - ax), ay + along * (by - ay)
    sides = np.linspace(-0.9 * half, 0.9 * half, SIGHT_ACROSS)
    for near in sides:
        for far in sides:
            spot = (x - far * (by - ay) / run, y + far * (bx - ax) / run)
            line = np.linspace((0.0, near), spot, SIGHT_SAMPLES)
            if np.all(np.min([part.gap(line) for part in floor], axis=0) == 0.0):
                return True
    return False


def corridor_turn(seed: int, corridor_width: float | None = None) -> Scene:
    """Two empty rooms joined by one winding corridor, the only way between them.

    The corridor's length, its number of turns and each turn's side and angle are drawn.
    """
    rng = np.random.default_rng(seed)
    return _wound(f"corridor_turn_{seed}", rng, corridor_width, None)[0]


def _wound(
    name: str,
    rng: np.random.Generator,
    corridor_width: float | None,
    hidden: float | None,
    deep: tuple[float, float] = (4.0, 5.0),
    span: float = WINDING_SPAN,
) -> tuple[Scene, Point, Point]:
    """A winding corridor scene and the ends of its last run.

    With a hidden distance, no part of the mouth sees that far along the last run.
    """
    near = _uniform(rng, 3, 4)
    z0 = _uniform(rng, 0.0, 0.08)
    below, above = _uniform(rng, 1.5, 3.0), _uniform(rng, 1.5, 3.0)
    half = _given(_uniform(rng, *CORRIDOR_WIDTH), corridor_width) / 2
    t = WALL_THICKNESS
    for _ in range(PLACEMENT_TRIES):
        points, heading = _winding(rng)
        far, depth = _uniform(rng, 4, 5), _uniform(rng, *deep)
        left = _uniform(rng, CORRIDOR_MOUTH_INSET, far - CORRIDOR_MOUTH_INSET)
        ahead, across = (
            (math.cos(heading), math.sin(heading)),
            (-math.sin(heading), math.cos(heading)),
        )
        end = points[-1]
        shift = left - far / 2
        room_a = (end[0] + shift * across[0], end[1] + shift * across[1])
        room_b = (room_a[0] + (depth + 2 * t) * ahead[0], room_a[1] + (depth + 2 * t) * ahead[1])
        room = _slab(room_a, room_b, far / 2 + t)
        reach_out = max(WINDING_WIDTH_MAX / 2, half) + 2 * t
        widest = [_slab(a, b, reach_out) for a, b in itertools.pairwise(points)]
        parts = [*widest, room]
        apart = all(not _overlap(a, b) for i, a in enumerate(parts) for b in parts[i + 2 :])
        reach = [
            (p.center[0] - p.reach()[0], p.center[1] - p.reach()[1], p.center[1] + p.reach()[1])
            for p in parts
        ]
        low, high = min(-below, *(r[1] for r in reach)), max(above, *(r[2] for r in reach))
        fits = apart and all(r[0] > 2 * t for r in reach[1:]) and high - low <= span
        if fits and not (hidden is not None and _sees(points, half, hidden)):
            break
    else:
        raise ValueError(f"{name} found no corridor that keeps clear of itself")
    width = near + max(p.center[0] + p.reach()[0] for p in parts)
    length = high - low
    top = z0 + WALL_HEIGHT

    def placed(point: Point) -> Point:
        return near + point[0], point[1] - low

    scene = Scene(
        name,
        {
            "width": width,
            "length": length,
            "z0": z0,
            "corridor_x": near,
            "corridor_y": -low,
            "room_x": placed(end)[0],
            "room_y": placed(end)[1],
            "room_yaw": heading,
            "room_depth": depth,
            "room_left": left,
            "room_right": far - left,
            "turns": len(points) - 2,
            "corridor_length": sum(math.dist(a, b) for a, b in itertools.pairwise(points)),
        },
    )
    _shell(scene, width, length, z0)
    y = -low
    scene.add((near, 0.0, z0), (near + t, y - half, top), "wall")
    scene.add((near, y + half, z0), (near + t, length, top), "wall")

    def wall(a: Point, b: Point, back: float = 0.0, on: float = 0.0) -> None:
        box = _slab(placed(a), placed(b), t / 2, back, on)
        scene.boxes.append(
            Box(
                (box.center[0], box.center[1], (z0 + top) / 2),
                (box.half[0], t / 2, (top - z0) / 2),
                "wall",
                box.yaw,
            )
        )

    for side in (1, -1):
        edge = _edge(points, side * (half + t / 2))
        for a, b in itertools.pairwise(edge):
            wall(a, b, on=t if b == edge[-1] else 0.0)

    def into(u: float, v: float) -> Point:
        return end[0] + u * ahead[0] + v * across[0], end[1] + u * ahead[1] + v * across[1]

    wall(into(t / 2, half), into(t / 2, left + t))
    wall(into(t / 2, -half), into(t / 2, left - far - t))
    wall(into(depth + 1.5 * t, left + t), into(depth + 1.5 * t, left - far - t))
    wall(into(0.0, left + t / 2), into(depth + 2 * t, left + t / 2))
    wall(into(0.0, left - far - t / 2), into(depth + 2 * t, left - far - t / 2))
    scene.start = (1.0, 1.0, z0)
    return scene, placed(points[-2]), placed(points[-1])


def hidden_object(seed: int, object_length: float | None = None) -> Scene:
    """A winding corridor with a low shoe-shaped box on its last run, out of sight of the first room."""
    rng = np.random.default_rng(seed)
    scene, (ax, ay), (bx, by) = _wound(
        f"hidden_object_{seed}", rng, HIDDEN_CORRIDOR_WIDTH, HIDDEN_ALONG[0]
    )
    along = _uniform(rng, *HIDDEN_ALONG)
    yaw = _uniform(rng, -math.pi / 2, math.pi / 2)
    long = _given(_uniform(rng, *SMALL_OBJECT_LENGTH), object_length)
    x, y, z0 = ax + along * (bx - ax), ay + along * (by - ay), scene.params["z0"]
    scene.params.update(object_x=x, object_y=y)
    if long > 0:
        half = (long / 2, SMALL_OBJECT_ASPECT * long / 2, SMALL_OBJECT_HEIGHT / 2)
        scene.boxes.append(Box((x, y, z0 + half[2]), half, "clutter", yaw))
    return scene


def _scatter(
    rng: np.random.Generator,
    count: int,
    width: float,
    length: float,
    a: Point,
    b: Point,
    astride: float | None = None,
) -> list[tuple[Rect, float]]:
    """Footprints and heights of boxes of drawn sizes over a floor, clear of two points.

    A lane a body can walk stays open between the two points. With a distance, every box
    lies within it of the straight line between them.
    """
    c, lane = START_CLEARANCE, SCATTER_LANE / 2
    keep_clear = [(x - c, y - c, x + c, y + c) for x, y in (a, b)]
    xs = (np.arange(int(width / SCATTER_CELL)) + 0.5) * SCATTER_CELL
    ys = (np.arange(int(length / SCATTER_CELL)) + 0.5) * SCATTER_CELL
    lanes = np.outer(
        np.abs(xs - width / 2) < width / 2 - lane, np.abs(ys - length / 2) < length / 2 - lane
    )
    ends = [(int(x / SCATTER_CELL), int(y / SCATTER_CELL)) for x, y in (a, b)]
    boxes = []
    for _ in range(count):
        half_x, half_y = _uniform(rng, *SCATTER_SIDE) / 2, _uniform(rng, *SCATTER_SIDE) / 2
        height = _uniform(rng, *SCATTER_HEIGHT)
        for _ in range(PLACEMENT_TRIES):
            if astride is None:
                x, y = _uniform(rng, half_x, width - half_x), _uniform(rng, half_y, length - half_y)
            else:
                on, off = _uniform(rng, 0.0, 1.0), _uniform(rng, -astride, astride)
                run = math.dist(a, b)
                x = a[0] + on * (b[0] - a[0]) - off * (b[1] - a[1]) / run
                y = a[1] + on * (b[1] - a[1]) + off * (b[0] - a[0]) / run
                if not (half_x <= x <= width - half_x and half_y <= y <= length - half_y):
                    continue
            spot = (x - half_x, y - half_y, x + half_x, y + half_y)
            left = lanes.copy()
            left[np.ix_(np.abs(xs - x) < half_x + lane, np.abs(ys - y) < half_y + lane)] = False
            if _clear(*spot, keep_clear) and _joined(left, *ends):
                boxes.append((spot, height))
                keep_clear.append(spot)
                lanes = left
                break
    return boxes


def scattered(seed: int, clutter_count: int = SCATTER_COUNT) -> Scene:
    """One room with boxes of drawn sizes scattered over the floor, clear of the start and the goal."""
    rng = np.random.default_rng(seed)
    width = _uniform(rng, 8, 10)
    length = SCATTER_AREA / width
    z0 = _uniform(rng, 0.0, 0.08)
    gx, gy = width - _uniform(rng, 1.5, 2.5), _uniform(rng, 1.5, length - 1.5)
    scene = Scene(
        f"scattered_{seed}",
        {"width": width, "length": length, "z0": z0, "goal_x": gx, "goal_y": gy},
    )
    _shell(scene, width, length, z0)
    scene.start = (1.0, 1.0, z0)
    for spot, height in _scatter(rng, int(clutter_count), width, length, scene.start[:2], (gx, gy)):
        scene.add((spot[0], spot[1], z0), (spot[2], spot[3], z0 + height), "clutter")
    return scene


def hidden_clutter(seed: int, clutter_count: int = HIDDEN_CLUTTER_COUNT) -> Scene:
    """A winding corridor into a long far room with boxes across the way to the goal.

    None of them is seen from the first room.
    """
    rng = np.random.default_rng(seed)
    scene, _, _ = _wound(
        f"hidden_clutter_{seed}",
        rng,
        HIDDEN_CLUTTER_CORRIDOR,
        HIDDEN_ROOM_ALONG,
        HIDDEN_ROOM_DEPTH,
        HIDDEN_ROOM_SPAN,
    )
    params = scene.params
    depth, right, z0 = params["room_depth"], params["room_right"], params["z0"]
    wide = params["room_left"] + right
    c, s = math.cos(params["room_yaw"]), math.sin(params["room_yaw"])

    def placed(x: float, y: float) -> Point:
        ahead, across = WALL_THICKNESS + x, y - right
        return params["room_x"] + ahead * c - across * s, params["room_y"] + ahead * s + across * c

    goal = (depth - _uniform(rng, 1.0, 1.5), _uniform(rng, 1.0, wide - 1.0))
    gx, gy = placed(*goal)
    params.update(goal_x=gx, goal_y=gy)
    boxes = _scatter(
        rng, int(clutter_count), depth, wide, (0.5, right), goal, HIDDEN_CLUTTER_ASTRIDE
    )
    for spot, height in boxes:
        x, y = placed((spot[0] + spot[2]) / 2, (spot[1] + spot[3]) / 2)
        half = ((spot[2] - spot[0]) / 2, (spot[3] - spot[1]) / 2, height / 2)
        scene.boxes.append(Box((x, y, z0 + half[2]), half, "clutter", params["room_yaw"]))
    return scene


def small_object(seed: int, object_length: float | None = None) -> Scene:
    """One empty room with a low shoe-shaped box lying in open floor at a drawn angle."""
    rng = np.random.default_rng(seed)
    width, length = _uniform(rng, 10, 12), _uniform(rng, 8, 10)
    z0 = _uniform(rng, 0.0, 0.08)
    x, y = _uniform(rng, 0.35, 0.45) * width, _uniform(rng, 0.35, 0.5) * length
    yaw = _uniform(rng, -math.pi / 2, math.pi / 2)
    long = _given(_uniform(rng, *SMALL_OBJECT_LENGTH), object_length)
    scene = Scene(
        f"small_object_{seed}",
        {"width": width, "length": length, "z0": z0, "object_x": x, "object_y": y},
    )
    _shell(scene, width, length, z0)
    if long > 0:
        half = (long / 2, SMALL_OBJECT_ASPECT * long / 2, SMALL_OBJECT_HEIGHT / 2)
        scene.boxes.append(Box((x, y, z0 + half[2]), half, "clutter", yaw))
    scene.start = (1.0, 1.0, z0)
    return scene


Params = dict[str, bool | int | float]
FAMILIES: dict[Family, Callable[..., Scene]] = {
    "office": office,
    "room": room,
    "doorway": doorway,
    "table": table,
    "block": block,
    "overhang": overhang,
    "corridor": corridor,
    "corridor_turn": corridor_turn,
    "scattered": scattered,
    "small_object": small_object,
    "hidden_object": hidden_object,
    "hidden_clutter": hidden_clutter,
}


def generate(family: Family, seed: int, **params: bool | int | float) -> Scene:
    """The family's scene for the seed, with any of its named parameters overridden."""
    return FAMILIES[family](seed, **params)
