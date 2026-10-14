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

"""Frozen navigation cases: a scene, a start, a goal, and how hard the route between them is."""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
from typing import Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import TypeAdapter

from dimos.navigation.bench.ground_truth import (
    CLUTTER_NEAR,
    GO2,
    Difficulty,
    GroundTruth,
    Route,
    boxes_near,
)
from dimos.simulation.scenes.procedural import (
    BLOCK_SIZE,
    OVERHANG_SIZE,
    PLACEMENT_TRIES,
    SMALL_OBJECT_ASPECT,
    SMALL_OBJECT_LENGTH,
    START_CLEARANCE,
    TABLE_SIZE,
    TABLE_TOP_THICKNESS,
    WALL_THICKNESS,
    Box,
    Family,
    Params,
    Scene,
    generate,
)

Split = Literal["dev", "held_out"]
Pose = tuple[float, float, float]
Point = tuple[float, float, float]

MINED = "mined"
NARROW_DOOR_M = 2 * GO2.radius + 0.12
START_CLEARANCE_MIN = 0.4
AGAINST_WALL_M = GO2.radius + 0.08
DOOR_BINS = (1, 2)
CLEARANCE_BINS = (0.35, 0.6)
STRADDLE_M = (1.2, 1.5, 2.0)
OBSTRUCTED = 1.05
DEAD_END_DETOUR = 1.4
SIDE_WALL_M = 1.5
PAST_M = (2.3, 3.0)
APPROACH_M = 1.5
UNDER_SLAB_RAD = math.pi / 6


@dataclass(frozen=True)
class Rules:
    """How an episode is scored. Changing any value is a new version."""

    version: int = 3
    goal_xy_m: float = 0.5
    goal_z_m: float = 0.3
    stand_height_m: float = GO2.stand
    timeout_s_per_m: float = 6.0
    timeout_base_s: float = 20.0
    fall_rad: float = 1.0
    fall_trunk_floor_s: float = 0.5
    stuck_s: float = 10.0
    stuck_progress_m: float = 0.3
    stuck_moving_fraction: float = 0.8
    dither_path_m: float = 0.7
    stalled_s: float = 10.0
    moving_cmd: float = 0.05
    reroute_m: float = 0.5
    yaw_reversal_rad_s: float = 0.2

    def timeout_s(self, route_length: float | None) -> float | None:
        if route_length is None:
            return None
        return self.timeout_base_s + self.timeout_s_per_m * route_length


@dataclass(frozen=True)
class Candidate:
    family: Family
    seed: int
    params: Params
    start: Pose
    goal: Point
    tag: str

    def scene(self) -> Scene:
        return generate(self.family, self.seed, **self.params)


@dataclass(frozen=True)
class Level:
    """Where a probe case sits on the variable its probe sweeps. A control case has the feature removed."""

    variable: str
    value: float
    control: bool = False


@dataclass(frozen=True)
class Case(Candidate):
    id: str
    split: Split
    difficulty: Difficulty
    route_length: float
    scene_digest: str
    level: Level | None = None
    feature: tuple[float, float] | None = None
    premap_objects: bool = True


@dataclass
class Manifest:
    suite: str
    rules: Rules
    sampling_seed: int
    cases: list[Case]

    def save(self, path: Path) -> None:
        path.write_text(_MANIFEST.dump_json(self, indent=2).decode() + "\n")

    @classmethod
    def load(cls, path: Path) -> Manifest:
        return _MANIFEST.validate_json(path.read_text())

    def case(self, case_id: str) -> Case:
        return next(c for c in self.cases if c.id == case_id)

    def check_drift(self) -> None:
        """Fail if any case's scene no longer generates the geometry it was frozen with."""
        drifted = sorted({c.id for c in self.cases if c.scene().digest() != c.scene_digest})
        if drifted:
            raise RuntimeError(f"scene generator drifted for cases: {', '.join(drifted)}")


_MANIFEST = TypeAdapter(Manifest)


@dataclass(frozen=True)
class Measured(Difficulty):
    """A route's difficulty plus the measures only the templates select on."""

    route: Route
    tables_near: int
    goal_clearance: float


def _table_tops(scene: Scene) -> list[Box]:
    return [
        box
        for box in scene.boxes
        if box.kind == "clutter" and abs(2 * box.half[2] - TABLE_TOP_THICKNESS) < 1e-9
    ]


def measure(gt: GroundTruth, route: Route) -> Measured:
    return Measured(
        **vars(gt.difficulty(route)),
        route=route,
        tables_near=len(boxes_near(route, _table_tops(gt.scene), CLUTTER_NEAR)),
        goal_clearance=float(route.clearance[-1]),
    )


Pair = tuple[NDArray[np.float64], NDArray[np.float64]]
Pairs = Callable[[GroundTruth, np.random.Generator, int], list[Pair]]


def from_scene_start(gt: GroundTruth, rng: np.random.Generator, n: int) -> list[Pair]:
    """The scene's start paired with sampled walkable cells."""
    start = np.array(gt.scene.start, dtype=np.float64)
    return [(start, gt.center(ix, iy)) for ix, iy in _walkable_cells(gt, rng, n)]


def _straddling(boxes: list[Box]) -> Pairs:
    """Starts and goals on opposite sides of a box, so the straight line runs through it."""

    def pairs(gt: GroundTruth, rng: np.random.Generator, n: int) -> list[Pair]:
        found: list[Pair] = []
        for i in rng.permutation(len(boxes)):
            box = boxes[i]
            for axis in (0, 1):
                for d in STRADDLE_M:
                    offset = np.zeros(3)
                    offset[axis] = box.half[axis] + d
                    center = np.array(box.center, dtype=np.float64)
                    a, b = center - offset, center + offset
                    if _placeable(gt, a) and _placeable(gt, b):
                        found.append((gt.center(*gt.index(a)), gt.center(*gt.index(b))))
                    if len(found) >= n:
                        return found
        return found

    return pairs


def _placeable(gt: GroundTruth, point: NDArray[np.float64]) -> bool:
    """Walkable with room to put the robot down."""
    try:
        return gt.stands(point) and bool(gt.clearance[gt.index(point)] >= START_CLEARANCE_MIN)
    except ValueError:
        return False


def across_clutter(gt: GroundTruth, rng: np.random.Generator, n: int) -> list[Pair]:
    tops = _table_tops(gt.scene)
    boxes = [box for box in gt.scene.boxes if box.kind == "clutter" and box not in tops]
    return _straddling(boxes)(gt, rng, n)


def across_tables(gt: GroundTruth, rng: np.random.Generator, n: int) -> list[Pair]:
    return _straddling(_table_tops(gt.scene))(gt, rng, n)


@dataclass(frozen=True)
class Stressor:
    """A template for one failure mode."""

    name: str
    params: Params
    want: Callable[[Measured], bool]
    prefer: Callable[[Measured], float]
    pairs: Pairs = from_scene_start


STRESSORS = [
    Stressor(
        "narrow_door",
        {"door_width": NARROW_DOOR_M},
        lambda m: m.doors >= 1,
        lambda m: m.route.length,
    ),
    Stressor(
        "doorway_clutter",
        {"door_clutter": True},
        lambda m: m.doors >= 1,
        lambda m: m.route.length,
    ),
    Stressor(
        "behind_clutter",
        {},
        lambda m: m.doors == 0 and m.detour >= OBSTRUCTED and m.clutter_near >= 1,
        lambda m: -m.detour,
        across_clutter,
    ),
    Stressor(
        "dead_end",
        {},
        lambda m: m.doors >= 1 and m.detour >= DEAD_END_DETOUR,
        lambda m: -m.detour,
    ),
    Stressor(
        "three_doors",
        {},
        lambda m: m.doors >= 3,
        lambda m: m.route.length,
    ),
    Stressor(
        "against_wall",
        {},
        lambda m: m.doors >= 1 and m.goal_clearance <= AGAINST_WALL_M,
        lambda m: m.route.length,
    ),
    Stressor(
        "around_table",
        {},
        lambda m: m.doors == 0 and m.tables_near >= 1 and m.detour >= OBSTRUCTED,
        lambda m: -m.detour,
        across_tables,
    ),
]


@dataclass(frozen=True)
class FreezeConfig:
    suite: str = "v1"
    family: Family = "office"
    seeds: tuple[int, ...] = tuple(range(1, 21))
    sampling_seed: int = 0
    stressor_samples: int = 300
    samples_per_scene: int = 60
    cases_per_bin: int = 3
    min_route_m: float = 3.0
    held_out_fraction: float = 1 / 3
    rules: Rules = Rules()


Tick = Callable[[int, int, str], None]


@dataclass
class _Mined:
    """One scene's share of the suite before the mined pool is spread over bins."""

    cases: list[Case]
    pool: list[Case]


def freeze(config: FreezeConfig, tick: Tick | None = None) -> Manifest:
    """Generate, validate and bin the suite's cases, one process per scene. Deterministic in the config."""
    total = len(config.seeds) + 1
    if tick is not None:
        tick(0, total, "scenes")
    mined: dict[int, _Mined] = {}
    workers = max(1, min(len(config.seeds), os.cpu_count() or 1))
    with ProcessPoolExecutor(workers, mp_context=multiprocessing.get_context("forkserver")) as pool:
        futures = {pool.submit(_freeze_scene, config, seed): seed for seed in config.seeds}
        for future in as_completed(futures):
            mined[futures[future]] = future.result()
            if tick is not None:
                tick(len(mined), total, f"scene {futures[future]} done")
    cases: list[Case] = []
    pooled: list[Case] = []
    for seed in config.seeds:
        cases += mined[seed].cases
        pooled += mined[seed].pool
    rng = np.random.default_rng(config.sampling_seed)
    cases += _spread_over_bins(pooled, rng, config.cases_per_bin)
    if tick is not None:
        tick(total, total, "done")
    return Manifest(
        suite=config.suite,
        rules=config.rules,
        sampling_seed=config.sampling_seed,
        cases=cases,
    )


def _freeze_scene(config: FreezeConfig, seed: int) -> _Mined:
    """Every stressor placed in one scene and its mined candidates validated, from the scene's own random stream."""
    rng = np.random.default_rng([config.sampling_seed, seed])
    truths: dict[str, GroundTruth] = {}

    def truth(params: Params) -> GroundTruth:
        key = json.dumps(params, sort_keys=True)
        if key not in truths:
            truths[key] = GroundTruth(generate(config.family, seed, **params))
        return truths[key]

    found = _Mined([], [])
    for stressor in STRESSORS:
        gt = truth(stressor.params)
        candidate = _stressor_candidate(gt, stressor, seed, rng, config)
        if candidate is None:
            continue
        case = _validate(gt, candidate, rng, config)
        if case is not None:
            found.cases.append(case)
    gt = truth({})
    for candidate in _mined_candidates(gt, seed, rng, config):
        case = _validate(gt, candidate, rng, config)
        if case is not None:
            found.pool.append(case)
    return found


def _yaw(rng: np.random.Generator) -> float:
    return float(-math.pi + 2 * math.pi * rng.random())


def _walkable_cells(
    gt: GroundTruth, rng: np.random.Generator, n: int, min_clearance: float = 0.0
) -> list[tuple[int, int]]:
    flat = np.flatnonzero(gt.walkable & (gt.clearance >= min_clearance))
    picked = rng.choice(flat, size=min(n, len(flat)), replace=False)
    return [(int(ix), int(iy)) for ix, iy in zip(*np.unravel_index(picked, gt.shape), strict=True)]


def _stressor_candidate(
    gt: GroundTruth, stressor: Stressor, seed: int, rng: np.random.Generator, config: FreezeConfig
) -> Candidate | None:
    """The best of the template's pairs whose route fits it."""
    best: tuple[float, Pair] | None = None
    for start, goal in stressor.pairs(gt, rng, config.stressor_samples):
        route = gt.route(start, goal)
        if route is None or route.length < config.min_route_m:
            continue
        measured = measure(gt, route)
        if not stressor.want(measured):
            continue
        score = stressor.prefer(measured)
        if best is None or score < best[0]:
            best = (score, (start, goal))
    if best is None:
        return None
    start, goal = best[1]
    pose = (float(start[0]), float(start[1]), _yaw(rng))
    return Candidate(config.family, seed, stressor.params, pose, _point(goal), stressor.name)


def _mined_candidates(
    gt: GroundTruth, seed: int, rng: np.random.Generator, config: FreezeConfig
) -> list[Candidate]:
    """Random start and goal pairs over the walkable ground."""
    starts = _walkable_cells(gt, rng, config.samples_per_scene, START_CLEARANCE_MIN)
    goals = _walkable_cells(gt, rng, config.samples_per_scene)
    candidates = []
    for (sx, sy), (gx, gy) in zip(starts, goals, strict=False):
        start, goal = gt.center(sx, sy), gt.center(gx, gy)
        candidates.append(
            Candidate(
                config.family,
                seed,
                {},
                (float(start[0]), float(start[1]), _yaw(rng)),
                _point(goal),
                MINED,
            )
        )
    return candidates


def _point(p: NDArray[np.float64]) -> Point:
    return float(p[0]), float(p[1]), float(p[2])


def _validate(
    gt: GroundTruth, candidate: Candidate, rng: np.random.Generator, config: FreezeConfig
) -> Case | None:
    """The case for a candidate, or None when its start, goal or route does not qualify."""
    try:
        start_index = gt.index(candidate.start)
        goal_stands = gt.stands(candidate.goal)
    except ValueError:
        return None
    if not gt.walkable[start_index] or gt.clearance[start_index] < START_CLEARANCE_MIN:
        return None
    if not goal_stands:
        return None
    route = gt.route(candidate.start, candidate.goal)
    if route is None or route.length < config.min_route_m:
        return None
    split: Split = "held_out" if rng.random() < config.held_out_fraction else "dev"
    return Case(
        **vars(candidate),
        id=_case_id(candidate),
        split=split,
        difficulty=gt.difficulty(route),
        route_length=route.length,
        scene_digest=gt.scene.digest(),
    )


@dataclass(frozen=True)
class Placed:
    """A probe case's start and goal, and the feature the robot could pass without meeting."""

    start: Pose
    goal: Point
    feature: tuple[float, float] | None = None


Placement = Callable[[GroundTruth, float, np.random.Generator], Placed]


@dataclass(frozen=True)
class Probe:
    """One feature in an otherwise empty scene, swept over fixed levels with one case per level and seed.

    The level is handed to the placement, and to the scene as scene_param when that is named.
    A seed's placement draws are the same at every level, so only the level differs between its cases.
    control is the level that removes the feature, run once per seed to see whether the robot's own
    line crosses it. premap_objects False leaves the clutter out of every case's premap.
    """

    name: str
    family: Family
    variable: str
    levels: tuple[float, ...]
    place: Placement
    scene_param: str | None = None
    control: float | None = None
    seeds: tuple[int, ...] = tuple(range(1, 11))
    premap_objects: bool = True


def _start_before(scene: Scene, x: float, rng: np.random.Generator) -> Pose:
    """A drawn pose short of the given x and clear of the walls."""
    length = scene.params["length"]
    sx = START_CLEARANCE + (x - START_CLEARANCE) * float(rng.random())
    sy = START_CLEARANCE + (length - 2 * START_CLEARANCE) * float(rng.random())
    return sx, sy, _yaw(rng)


def goal_by_wall(gt: GroundTruth, clearance: float, rng: np.random.Generator) -> Placed:
    """A drawn start well short of the far wall, and a goal this far from it and well clear of the side walls."""
    width, length = gt.scene.params["width"], gt.scene.params["length"]
    start = _start_before(gt.scene, width - APPROACH_M, rng)
    across = SIDE_WALL_M + (length - 2 * SIDE_WALL_M) * float(rng.random())
    return Placed(start, (width - clearance, across, gt.scene.start[2]))


GOAL_CLEARANCE = Probe(
    "goal_clearance",
    "room",
    "goal_clearance",
    (0.20, 0.23, 0.26, 0.29, 0.32, 0.35, 0.40, 0.50, 0.60),
    goal_by_wall,
)


def past_the_door(gt: GroundTruth, width: float, rng: np.random.Generator) -> Placed:
    """A drawn start well short of the door's wall, and a goal in the far room well clear of the door and every wall."""
    (door,) = gt.scene.doors
    start = _start_before(gt.scene, door.at - APPROACH_M, rng)
    near = door.at + WALL_THICKNESS + SIDE_WALL_M
    x = near + (gt.scene.params["width"] - SIDE_WALL_M - near) * float(rng.random())
    y = SIDE_WALL_M + (gt.scene.params["length"] - 2 * SIDE_WALL_M) * float(rng.random())
    return Placed(start, (x, y, gt.scene.start[2]))


DOORWAY = Probe(
    "door_width",
    "doorway",
    "door_width",
    (0.45, 0.48, 0.51, 0.54, 0.57, 0.60, 0.65, 0.75, 0.90),
    past_the_door,
    scene_param="door_width",
)


def table_leg(scene: Scene, leg_width: float, corner: int) -> tuple[float, float]:
    """The center of one of the table's legs, counted from the corner nearest the origin."""
    lx, ly = TABLE_SIZE
    x = scene.params["table_x"] + (leg_width / 2 if corner % 2 == 0 else lx - leg_width / 2)
    y = scene.params["table_y"] + (leg_width / 2 if corner < 2 else ly - leg_width / 2)
    return x, y


def _start_facing(
    scene: Scene,
    before: float,
    x: float,
    y: float,
    rng: np.random.Generator,
    spread: float = math.pi / 2,
) -> Pose:
    """A drawn start clear of the walls and of an object that begins at the given x.

    Its line through the point slants no more than the spread and ends well clear of the walls.
    """
    width, length = scene.params["width"], scene.params["length"]
    for _ in range(PLACEMENT_TRIES):
        reach = APPROACH_M + (x - START_CLEARANCE - APPROACH_M) * float(rng.random())
        slant = spread * (2 * float(rng.random()) - 1)
        yaw = _yaw(rng)
        sx, sy = x - reach * math.cos(slant), y - reach * math.sin(slant)
        far_x, far_y = x + PAST_M[1] * math.cos(slant), y + PAST_M[1] * math.sin(slant)
        if (
            sx <= before - START_CLEARANCE
            and START_CLEARANCE <= sy <= length - START_CLEARANCE
            and far_x <= width - SIDE_WALL_M
            and SIDE_WALL_M <= far_y <= length - SIDE_WALL_M
        ):
            return sx, sy, yaw
    raise ValueError(f"{scene.name} has no start facing ({x}, {y})")


def _straight_past(
    scene: Scene, start: Pose, x: float, y: float, half_width: float, rng: np.random.Generator
) -> Placed:
    """A goal along a line that passes the point closely enough for the body to meet an object this wide.

    The feature is where that line comes nearest the point.
    """
    sx, sy, _ = start
    reach = math.hypot(x - sx, y - sy)
    offset = (half_width + GO2.radius) * (2 * float(rng.random()) - 1)
    heading = math.atan2(y - sy, x - sx) + math.asin(offset / reach)
    meet = math.sqrt(reach**2 - offset**2)
    low, high = PAST_M
    past = meet + low + (high - low) * float(rng.random())
    goal = (sx + past * math.cos(heading), sy + past * math.sin(heading), scene.start[2])
    return Placed(start, goal, (sx + meet * math.cos(heading), sy + meet * math.sin(heading)))


def through_a_leg(gt: GroundTruth, leg_width: float, rng: np.random.Generator) -> Placed:
    """A goal straight past one table leg, so the body walking the direct line meets it."""
    corner = int(rng.integers(4))
    start = _start_facing(
        gt.scene, gt.scene.params["table_x"], *table_leg(gt.scene, 0.0, corner), rng
    )
    x, y = table_leg(gt.scene, leg_width, corner)
    return _straight_past(gt.scene, start, x, y, leg_width / 2, rng)


def through_the_block(gt: GroundTruth, height: float, rng: np.random.Generator) -> Placed:
    """A goal straight past the block, so the body walking the direct line meets it."""
    x, y = gt.scene.params["block_x"], gt.scene.params["block_y"]
    start = _start_facing(gt.scene, x - BLOCK_SIZE / 2, x, y, rng)
    return _straight_past(gt.scene, start, x, y, BLOCK_SIZE / 2, rng)


LEG_WIDTH = Probe(
    "leg_width",
    "table",
    "leg_width",
    (0.01, 0.02, 0.03, 0.04, 0.06, 0.08, 0.10, 0.15, 0.20),
    through_a_leg,
    scene_param="leg_width",
    control=0.0,
)


def under_the_slab(gt: GroundTruth, clearance: float, rng: np.random.Generator) -> Placed:
    """A goal straight past the slab, so the direct line runs under it between the posts."""
    x, y = gt.scene.params["slab_x"], gt.scene.params["slab_y"]
    across = min(OVERHANG_SIZE) / 2
    start = _start_facing(gt.scene, x - across, x, y, rng, UNDER_SLAB_RAD)
    return _straight_past(gt.scene, start, x, y, across - GO2.radius, rng)


OVERHANG = Probe(
    "overhang_clearance",
    "overhang",
    "overhang_clearance",
    (0.30, 0.36, 0.40, 0.44, 0.48, 0.52, 0.56, 0.64, 0.80),
    under_the_slab,
    scene_param="clearance",
    control=0.0,
)
BLOCK_HEIGHT = Probe(
    "block_height",
    "block",
    "block_height",
    (0.04, 0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32, 0.40),
    through_the_block,
    scene_param="block_height",
    control=0.0,
)


def over_the_small_object(gt: GroundTruth, length: float, rng: np.random.Generator) -> Placed:
    """A goal straight past the object, so the body walking the direct line meets even the smallest."""
    x, y = gt.scene.params["object_x"], gt.scene.params["object_y"]
    start = _start_facing(gt.scene, x - SMALL_OBJECT_LENGTH[1] / 2, x, y, rng)
    least = SMALL_OBJECT_ASPECT * SMALL_OBJECT_LENGTH[0] / 2
    return _straight_past(gt.scene, start, x, y, least, rng)


SMALL_OBJECT = Probe(
    "small_object",
    "small_object",
    "object_length",
    (0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.34, 0.42, 0.50),
    over_the_small_object,
    scene_param="object_length",
    control=0.0,
)


def past_the_corridor(gt: GroundTruth, width: float, rng: np.random.Generator) -> Placed:
    """A drawn start well short of the corridor, and a goal in the far room well clear of every wall."""
    params = gt.scene.params
    start = _start_before(gt.scene, params["corridor_x"] - APPROACH_M, rng)
    near_x, near_y = params["room_x"] + SIDE_WALL_M, params["room_y"] + SIDE_WALL_M
    x = near_x + (params["width"] - SIDE_WALL_M - near_x) * float(rng.random())
    y = near_y + (params["length"] - SIDE_WALL_M - near_y) * float(rng.random())
    return Placed(start, (x, y, gt.scene.start[2]))


def past_the_winding_corridor(gt: GroundTruth, width: float, rng: np.random.Generator) -> Placed:
    """A drawn start well short of the corridor, and a goal in the turned far room well clear of its walls."""
    params = gt.scene.params
    start = _start_before(gt.scene, params["corridor_x"] - APPROACH_M, rng)
    ahead = WALL_THICKNESS + SIDE_WALL_M
    ahead += (params["room_depth"] - 2 * SIDE_WALL_M) * float(rng.random())
    span = params["room_left"] + params["room_right"] - 2 * SIDE_WALL_M
    across = SIDE_WALL_M - params["room_right"] + span * float(rng.random())
    c, s = math.cos(params["room_yaw"]), math.sin(params["room_yaw"])
    x, y = params["room_x"] + ahead * c - across * s, params["room_y"] + ahead * s + across * c
    return Placed(start, (x, y, gt.scene.start[2]))


def past_the_hidden_object(gt: GroundTruth, length: float, rng: np.random.Generator) -> Placed:
    """Through the winding corridor to the far room, with the object on the way as the feature."""
    placed = past_the_winding_corridor(gt, length, rng)
    return Placed(
        placed.start, placed.goal, (gt.scene.params["object_x"], gt.scene.params["object_y"])
    )


CORRIDOR = Probe(
    "corridor_width",
    "corridor",
    "corridor_width",
    (0.45, 0.48, 0.51, 0.54, 0.57, 0.60, 0.65, 0.75, 0.90, 1.00, 1.20),
    past_the_corridor,
    scene_param="corridor_width",
)
CORRIDOR_TURN = Probe(
    "corridor_turn",
    "corridor_turn",
    "corridor_width",
    CORRIDOR.levels,
    past_the_winding_corridor,
    scene_param="corridor_width",
)


def across_the_boxes(gt: GroundTruth, count: float, rng: np.random.Generator) -> Placed:
    """The scene's start at a drawn yaw, and the goal the scene keeps clear across the room."""
    sx, sy, z = gt.scene.start
    return Placed((sx, sy, _yaw(rng)), (gt.scene.params["goal_x"], gt.scene.params["goal_y"], z))


CLUTTER = Probe(
    "clutter_count",
    "scattered",
    "clutter_count",
    (0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0),
    across_the_boxes,
    scene_param="clutter_count",
)


def into_the_hidden_clutter(gt: GroundTruth, count: float, rng: np.random.Generator) -> Placed:
    """A drawn start well short of the corridor, and the goal the far room keeps clear."""
    params = gt.scene.params
    start = _start_before(gt.scene, params["corridor_x"] - APPROACH_M, rng)
    return Placed(start, (params["goal_x"], params["goal_y"], gt.scene.start[2]))


HIDDEN_CLUTTER = Probe(
    "hidden_clutter",
    "hidden_clutter",
    "clutter_count",
    (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0),
    into_the_hidden_clutter,
    scene_param="clutter_count",
    premap_objects=False,
)
HIDDEN_OBJECT = Probe(
    "hidden_object",
    "hidden_object",
    "object_length",
    SMALL_OBJECT.levels,
    past_the_hidden_object,
    scene_param="object_length",
    control=0.0,
    premap_objects=False,
)
PROBES = {
    probe.name: probe
    for probe in (
        GOAL_CLEARANCE,
        DOORWAY,
        LEG_WIDTH,
        BLOCK_HEIGHT,
        OVERHANG,
        CORRIDOR,
        CORRIDOR_TURN,
        CLUTTER,
        SMALL_OBJECT,
        HIDDEN_OBJECT,
        HIDDEN_CLUTTER,
    )
}


def freeze_probe(probe: Probe, sampling_seed: int = 0, rules: Rules = Rules()) -> Manifest:
    """The probe's cases, every level in every seed's scene. Deterministic in its arguments."""
    cases = []
    for seed in probe.seeds:
        gt = None
        levels = probe.levels if probe.control is None else (probe.control, *probe.levels)
        for value in levels:
            rng = np.random.default_rng([sampling_seed, seed])
            params: Params = {} if probe.scene_param is None else {probe.scene_param: value}
            if gt is None or params:
                gt = GroundTruth(generate(probe.family, seed, **params))
            placed = probe.place(gt, value, rng)
            candidate = Candidate(probe.family, seed, params, placed.start, placed.goal, probe.name)
            route = gt.route(placed.start, placed.goal)
            if route is None:
                raise ValueError(f"{probe.name} at {value} has no route in seed {seed}")
            cases.append(
                Case(
                    **vars(candidate),
                    id=_case_id(candidate),
                    split="dev",
                    difficulty=gt.difficulty(route),
                    route_length=route.length,
                    scene_digest=gt.scene.digest(),
                    level=Level(probe.variable, value, control=value == probe.control),
                    feature=placed.feature,
                    premap_objects=probe.premap_objects,
                )
            )
    return Manifest(suite=probe.name, rules=rules, sampling_seed=sampling_seed, cases=cases)


def _case_id(candidate: Candidate) -> str:
    record = json.dumps(vars(candidate), sort_keys=True)
    digest = hashlib.sha256(record.encode()).hexdigest()[:6]
    return f"{candidate.tag}-s{candidate.seed}-{digest}"


def _bin(case: Case) -> tuple[int, int]:
    doors = sum(case.difficulty.doors >= edge for edge in DOOR_BINS)
    clearance = sum(case.difficulty.min_clearance >= edge for edge in CLEARANCE_BINS)
    return doors, clearance


def _spread_over_bins(pool: list[Case], rng: np.random.Generator, per_bin: int) -> list[Case]:
    """Up to per_bin cases from each doors-by-clearance bin, in a random order."""
    order = rng.permutation(len(pool))
    taken: Counter[tuple[int, int]] = Counter()
    chosen = []
    for i in order:
        case = pool[i]
        if taken[_bin(case)] < per_bin:
            taken[_bin(case)] += 1
            chosen.append(case)
    return sorted(chosen, key=lambda c: c.id)
