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

"""Frozen navigation cases: a scene, a start, a goal, and how hard the route between them is.

A suite is frozen once from a sampling seed and saved as a manifest. Stressor templates
provoke one failure mode each. Mined cases are random pairs spread evenly over
difficulty bins. Every candidate that fails validation is kept as a rejection.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import TypeAdapter

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.navigation.sim_eval.ground_truth import (
    CLUTTER_NEAR,
    GO2,
    Difficulty,
    GroundTruth,
    Route,
    boxes_near,
    detour,
    doors_crossed,
)
from dimos.simulation.scenes.procedural import TABLE_TOP_THICKNESS, Box, Family, Scene, generate

Split = Literal["dev", "held_out"]
Params = dict[str, bool | int | float]
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


@dataclass(frozen=True)
class Rules:
    """How an episode is scored. Changing any value is a new version."""

    version: int = 1
    premapped: bool = True
    goal_xy_m: float = 0.5
    goal_z_m: float = 0.3
    stand_height_m: float = 0.3
    timeout_s_per_m: float = 6.0
    timeout_base_s: float = 20.0
    fall_rad: float = 1.0
    stuck_s: float = 10.0
    stuck_progress_m: float = 0.3
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
class Case(Candidate):
    id: str
    split: Split
    difficulty: Difficulty
    route_length: float
    scene_digest: str


@dataclass(frozen=True)
class Rejection:
    family: Family
    seed: int
    params: Params
    tag: str
    reason: str
    start: Pose | None = None
    goal: Point | None = None


@dataclass
class Manifest:
    suite: str
    rules: Rules
    sampling_seed: int
    git_sha: str | None
    git_dirty: bool
    cases: list[Case]
    rejections: list[Rejection]
    mined_rejected: dict[str, int] = field(default_factory=dict)

    def save(self, path: Path) -> None:
        path.write_text(_MANIFEST.dump_json(self, indent=2).decode() + "\n")

    @classmethod
    def load(cls, path: Path) -> Manifest:
        return _MANIFEST.validate_json(path.read_text())

    def check_drift(self) -> None:
        """Fail if any case's scene no longer generates the geometry it was frozen with."""
        drifted = sorted({c.id for c in self.cases if c.scene().digest() != c.scene_digest})
        if drifted:
            raise RuntimeError(f"scene generator drifted for cases: {', '.join(drifted)}")


_MANIFEST = TypeAdapter(Manifest)


@dataclass(frozen=True)
class Measured:
    """What a route looks like, by the measures the templates select on."""

    route: Route
    doors: int
    detour: float
    clutter_near: int
    tables_near: int
    goal_clearance: float


def _table_tops(scene: Scene) -> list[Box]:
    return [
        box
        for box in scene.boxes
        if box.kind == "clutter" and abs(2 * box.half[2] - TABLE_TOP_THICKNESS) < 1e-9
    ]


def measure(gt: GroundTruth, route: Route) -> Measured:
    boxes = [box for box in gt.scene.boxes if box.kind == "clutter"]
    tops = _table_tops(gt.scene)
    return Measured(
        route=route,
        doors=doors_crossed(gt.scene, route),
        detour=detour(route),
        clutter_near=len(boxes_near(route, boxes, CLUTTER_NEAR)),
        tables_near=len(boxes_near(route, tops, CLUTTER_NEAR)),
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
    """A template for one failure mode: scene overrides, where to look, which routes qualify, which is best."""

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
        lambda m: m.doors >= 1 and m.detour >= 1.4,
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


def freeze(config: FreezeConfig) -> Manifest:
    """Generate, validate and bin the suite's cases. Deterministic in the config."""
    rng = np.random.default_rng(config.sampling_seed)
    truths = _Truths()
    cases: list[Case] = []
    rejections: list[Rejection] = []
    for seed in config.seeds:
        for stressor in STRESSORS:
            gt = truths.get(config.family, seed, stressor.params)
            candidate = _stressor_candidate(gt, stressor, seed, rng, config)
            if candidate is None:
                rejections.append(
                    Rejection(
                        config.family, seed, stressor.params, stressor.name, "no goal matched"
                    )
                )
                continue
            case = _validate(gt, candidate, rng, config)
            if isinstance(case, Case):
                cases.append(case)
            else:
                rejections.append(_rejection(candidate, case))
    pool: list[Case] = []
    tally: Counter[str] = Counter()
    for seed in config.seeds:
        gt = truths.get(config.family, seed, {})
        for candidate in _mined_candidates(gt, seed, rng, config):
            case = _validate(gt, candidate, rng, config)
            if isinstance(case, Case):
                pool.append(case)
            else:
                tally[case] += 1
    cases += _spread_over_bins(pool, rng, config.cases_per_bin)
    git_sha, git_dirty = _git_state()
    return Manifest(
        suite=config.suite,
        rules=config.rules,
        sampling_seed=config.sampling_seed,
        git_sha=git_sha,
        git_dirty=git_dirty,
        cases=cases,
        rejections=rejections,
        mined_rejected=dict(sorted(tally.items())),
    )


class _Truths:
    """Ground truth per scene, built once."""

    def __init__(self) -> None:
        self._cache: dict[str, GroundTruth] = {}

    def get(self, family: Family, seed: int, params: Params) -> GroundTruth:
        key = json.dumps([family, seed, params], sort_keys=True)
        if key not in self._cache:
            self._cache[key] = GroundTruth(generate(family, seed, **params))
        return self._cache[key]


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
) -> Case | str:
    """The case for a candidate, or the reason it is rejected."""
    try:
        start_index = gt.index(candidate.start)
    except ValueError:
        return "start outside the scene"
    if not gt.walkable[start_index]:
        return "start not walkable"
    if gt.clearance[start_index] < START_CLEARANCE_MIN:
        return "start too close to an obstacle"
    try:
        if not gt.stands(candidate.goal):
            return "goal not walkable"
    except ValueError:
        return "goal outside the scene"
    route = gt.route(candidate.start, candidate.goal)
    if route is None:
        return "no route"
    if route.length < config.min_route_m:
        return "route too short"
    split: Split = "held_out" if rng.random() < config.held_out_fraction else "dev"
    return Case(
        **vars(candidate),
        id=_case_id(candidate),
        split=split,
        difficulty=gt.difficulty(route),
        route_length=route.length,
        scene_digest=gt.scene.digest(),
    )


def _case_id(candidate: Candidate) -> str:
    record = json.dumps(vars(candidate), sort_keys=True)
    digest = hashlib.sha256(record.encode()).hexdigest()[:6]
    return f"{candidate.tag}-s{candidate.seed}-{digest}"


def _rejection(candidate: Candidate, reason: str) -> Rejection:
    return Rejection(
        candidate.family,
        candidate.seed,
        candidate.params,
        candidate.tag,
        reason,
        candidate.start,
        candidate.goal,
    )


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


def _git_state() -> tuple[str | None, bool]:
    def git(*args: str) -> str:
        return subprocess.run(
            ["git", *args], cwd=DIMOS_PROJECT_ROOT, capture_output=True, text=True, timeout=5
        ).stdout.strip()

    try:
        return git("rev-parse", "HEAD") or None, bool(git("status", "--porcelain"))
    except (OSError, subprocess.SubprocessError):
        return None, False
