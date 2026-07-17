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

from __future__ import annotations

from dataclasses import replace
import itertools
import json
from pathlib import Path
from typing import TYPE_CHECKING, cast

import numpy as np
import pytest

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.navigation.nav_3d.evaluator import metrics, tripwire
from dimos.navigation.nav_3d.evaluator.cases import Case, Suite, load_suite, save_suite
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.final_map import (
    FinalMap,
    MapCheckpoints,
    encode_deltas,
    key_centers,
    keys_contain,
    replay_frames,
    voxel_keys,
)
from dimos.navigation.nav_3d.evaluator.generate import (
    Candidate,
    GenerationParams,
    _select_diverse,
    generate_cases,
    snap_to_surface,
)
from dimos.navigation.nav_3d.evaluator.recording import Frame, Trajectory
from dimos.navigation.nav_3d.evaluator.runner import (
    CaseResult,
    DatasetResult,
    Report,
    _no_plan,
    _run_plan,
    score_negative,
)

if TYPE_CHECKING:
    from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner

VOXEL = 0.1


def test_voxel_key_roundtrip() -> None:
    pts = np.array([[0.05, 0.05, 0.05], [-3.21, 4.7, -0.09], [80.0, -80.0, 12.3]], dtype=np.float32)
    centers = key_centers(voxel_keys(pts, VOXEL), VOXEL)
    assert np.all(np.abs(centers - pts) <= VOXEL / 2 + 1e-5)


def test_keys_contain() -> None:
    keys = np.sort(voxel_keys(np.array([[0, 0, 0], [1, 1, 1]], dtype=np.float32), VOXEL))
    query = voxel_keys(np.array([[0, 0, 0], [5, 5, 5]], dtype=np.float32), VOXEL)
    assert keys_contain(keys, query).tolist() == [True, False]
    assert keys_contain(np.array([], dtype=np.int64), query).tolist() == [False, False]


def _wall(x: float) -> np.ndarray:
    ys, zs = np.meshgrid(np.arange(-1, 1, VOXEL), np.arange(0.05, 1.5, VOXEL))
    return np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1, dtype=np.float32)


def _gate(waypoints: np.ndarray, obstacles: np.ndarray) -> metrics.GateResult:
    keys = np.unique(voxel_keys(obstacles, VOXEL))
    return metrics.check_path(
        waypoints, keys, VOXEL, robot_radius=0.16, ground_margin=0.25, body_clearance=0.45
    )


def test_gate_blocks_wall_crossing() -> None:
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    result = _gate(path, _wall(0.0))
    assert not result.valid
    assert len(result.collision_points) > 0
    assert np.all(np.abs(result.collision_points[:, 0]) < 0.3)


def test_gate_passes_clear_path() -> None:
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    assert _gate(path, _wall(5.0)).valid


def test_gate_ignores_ground() -> None:
    xs, ys = np.meshgrid(np.arange(-2, 2, VOXEL), np.arange(-2, 2, VOXEL))
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.05)], axis=1, dtype=np.float32)
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    assert _gate(path, floor).valid


def test_gate_tolerates_stair_slope() -> None:
    """Terrain rising at stair slope inside the disc must not trigger the gate."""
    xs, ys = np.meshgrid(np.arange(-1, 2, VOXEL), np.arange(-1, 1, VOXEL))
    slope = np.stack([xs.ravel(), ys.ravel(), xs.ravel() * 0.7 - 0.05], axis=1, dtype=np.float32)
    path = np.stack(
        [np.arange(-0.5, 1.5, 0.1), np.zeros(20), np.arange(-0.5, 1.5, 0.1) * 0.7], axis=1
    ).astype(np.float32)
    assert _gate(path, slope).valid


def test_gate_reports_clearance_margin() -> None:
    wall = _wall(10.0)
    graze = np.array([[9.7, -0.5, 0], [9.7, 0.5, 0]], dtype=np.float32)
    result = _gate(graze, wall)
    assert result.valid
    assert result.min_clearance_m == pytest.approx(0.35 - 0.16, abs=0.02)
    crossing = _gate(np.array([[9, 0, 0], [11, 0, 0]], dtype=np.float32), wall)
    assert not crossing.valid
    assert crossing.min_clearance_m < 0
    far = _gate(np.array([[2, 0, 0], [3, 0, 0]], dtype=np.float32), wall)
    assert far.min_clearance_m == metrics.MARGIN_CAP_M


def test_spl() -> None:
    assert metrics.spl(False, 10.0, 10.0) == 0.0
    assert metrics.spl(True, 10.0, 10.0) == 1.0
    assert metrics.spl(True, 10.0, 20.0) == pytest.approx(0.5)
    assert metrics.spl(True, 10.0, 5.0) == 1.0


def test_reference_length_snaps_to_trajectory() -> None:
    positions = np.stack(
        [np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1
    ).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 10, 101), positions=positions)
    # Walking toward a never-yet-visited goal is not causal.
    ref = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), robot_height=0.3)
    assert ref.snapped
    assert ref.length == pytest.approx(10.0, abs=0.01)
    assert not ref.causal
    assert ref.start_ts == float("inf")
    # Returning to the walk's origin is causal.
    ref = metrics.reference_length(traj, (10, 0, 0), (0, 0, 0), robot_height=0.3)
    assert ref.causal
    assert 9.0 <= ref.start_ts <= 10.0
    ref = metrics.reference_length(traj, (0, 5, 0), (10, 0, 0), robot_height=0.3)
    assert not ref.snapped
    assert ref.start_ts == float("inf")


def test_reference_length_uses_shortest_revisit() -> None:
    """An out-and-back trajectory must not inflate the reference with the loop."""
    out = np.stack([np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1)
    detour = np.stack([np.full(101, 10.0), np.linspace(0, 30, 101), np.full(101, 0.3)], axis=1)
    back = detour[::-1]
    positions = np.concatenate([out, detour, back]).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 30, len(positions)), positions=positions)
    ref = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), robot_height=0.3)
    assert ref.snapped
    assert ref.length == pytest.approx(10.0, abs=0.2)


def test_path_length_and_goal() -> None:
    path = np.array([[0, 0, 0], [3, 4, 0]], dtype=np.float32)
    assert metrics.path_length(path) == pytest.approx(5.0)
    assert metrics.goal_reached(path, (3, 4, 0.2), tolerance=0.5)
    assert not metrics.goal_reached(path, (3, 4, 1.0), tolerance=0.5)


def test_generate_cases_around_wall() -> None:
    """A U-shaped walk around a wall must yield non-trivial cases spanning it."""
    wall_pts = _wall(10.0)
    wall_keys = np.unique(voxel_keys(wall_pts, VOXEL))
    final = FinalMap(
        voxel_size=VOXEL,
        occupied=wall_pts,
        occupied_keys=wall_keys,
        frames=1,
        add_frame_ms={"p50": 0.0, "p95": 0.0, "max": 0.0},
        build_ms=0.0,
    )
    xs, ys = np.meshgrid(np.arange(0, 20, VOXEL), np.arange(-3, 6, VOXEL))
    surface = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1, dtype=np.float32)

    legs = [
        np.stack([np.linspace(2, 8, 40), np.zeros(40)], axis=1),
        np.stack([np.full(40, 8.0), np.linspace(0, 4, 40)], axis=1),
        np.stack([np.linspace(8, 12, 40), np.full(40, 4.0)], axis=1),
        np.stack([np.full(40, 12.0), np.linspace(4, 0, 40)], axis=1),
        np.stack([np.linspace(12, 18, 40), np.zeros(40)], axis=1),
    ]
    xy = np.concatenate(legs)
    positions = np.column_stack([xy, np.full(len(xy), 0.3)]).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 60, len(positions)), positions=positions)

    cfg = EvalConfig(voxel_size=VOXEL)
    cases = generate_cases(traj, final, surface, cfg, GenerationParams(max_cases=10))
    assert cases
    assert len({c.id for c in cases}) == len(cases)
    spans_wall = [c for c in cases if (c.start[0] - 10) * (c.goal[0] - 10) < 0]
    assert spans_wall
    for c in cases:
        assert abs(c.start[2]) < 1e-5 and abs(c.goal[2]) < 1e-5
        assert "flat" in c.tags


def test_select_diverse_backfills_to_min_cases() -> None:
    """Sector caps must not starve a dataset below the case floor."""
    candidates = [
        Candidate(
            start=(float(x), 0.0, 0.0),
            goal=(float(x), 20.0, 0.0),
            walked_m=30.0,
            detour_ratio=1.5,
            dz=0.0,
        )
        for x in np.arange(0.0, 16.0, 2.0)
    ]
    strict = _select_diverse(candidates, GenerationParams(min_cases=0), max_cases=12)
    assert len(strict) == 4
    backfilled = _select_diverse(candidates, GenerationParams(min_cases=10), max_cases=12)
    assert len(backfilled) == 8
    assert len({(c.start, c.goal) for c in backfilled}) == 8


def test_snap_to_surface() -> None:
    xs, ys = np.meshgrid(np.arange(0, 2, VOXEL), np.arange(0, 2, VOXEL))
    surface = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1, dtype=np.float32)
    snapped = snap_to_surface(np.array([1.0, 1.0, 0.4], dtype=np.float32), surface, 1.0)
    assert snapped is not None
    assert abs(snapped[2]) < 1e-6
    assert np.linalg.norm(snapped[:2] - [1.0, 1.0]) < VOXEL
    assert snap_to_surface(np.array([9.0, 9.0, 0.0], dtype=np.float32), surface, 1.0) is None
    assert snap_to_surface(np.array([1.0, 1.0, 5.0], dtype=np.float32), surface, 1.0) is None


def test_checkpoint_deltas_roundtrip() -> None:
    snapshots = [
        np.array([1, 2, 3], dtype=np.int64),
        np.array([2, 3, 4, 5], dtype=np.int64),
        np.array([4, 5], dtype=np.int64),
    ]
    observed = [
        np.array([1, 2, 3], dtype=np.int64),
        np.array([1, 2, 3, 4, 5], dtype=np.int64),
        np.array([1, 2, 3, 4, 5, 9], dtype=np.int64),
    ]
    added, removed = encode_deltas(snapshots)
    observed_added, _ = encode_deltas(observed)
    ckpt = MapCheckpoints(
        times=np.arange(3, dtype=np.float64),
        added=added,
        removed=removed,
        observed_added=observed_added,
    )
    seen = np.array([], dtype=np.int64)
    for (orig_keys, orig_obs), (keys, obs_new) in zip(
        zip(snapshots, observed, strict=True), ckpt.iter_snapshots(), strict=True
    ):
        assert np.array_equal(orig_keys, keys)
        seen = np.union1d(seen, obs_new)
        assert np.array_equal(orig_obs, seen)


def test_replay_frames_snapshots_grow_with_time() -> None:
    """Each checkpoint must contain exactly the frames seen up to its time."""
    mapper = VoxelRayMapper(voxel_size=VOXEL, max_range=30.0, support_min=1)

    def frame_at(ts: float, x: float) -> Frame:
        return Frame(ts=ts, points=_wall(x), origin=(x - 2.0, 0.0, 0.5))

    # A voxel needs a second observation to persist, so hit each wall twice.
    frames = [
        frame_at(0.0, 5.0),
        frame_at(0.1, 5.0),
        frame_at(1.0, 8.0),
        frame_at(1.1, 8.0),
        frame_at(2.0, 11.0),
        frame_at(2.1, 11.0),
    ]
    times = np.array([0.5, 1.5, np.inf])
    final, snapshots, observed = replay_frames(frames, mapper, VOXEL, times)
    assert final.frames == 6
    sizes = [len(s) for s in snapshots]
    assert 0 < sizes[0] < sizes[1] < sizes[2]
    assert np.array_equal(snapshots[2], final.occupied_keys)
    for earlier, later in itertools.pairwise(snapshots):
        assert keys_contain(later, earlier).all()
    # The observed set holds raw returns causally: wall 1 by the first
    # checkpoint, wall 3 only at the end.
    wall1 = np.unique(voxel_keys(_wall(5.0), VOXEL))
    wall3 = np.unique(voxel_keys(_wall(11.0), VOXEL))
    assert keys_contain(observed[0], wall1).all()
    assert not keys_contain(observed[0], wall3).any()
    assert keys_contain(observed[2], wall3).all()


class _StubPlanner:
    """Returns a fixed path regardless of the map, for gaming the scorer."""

    def __init__(self, waypoints: np.ndarray | None) -> None:
        self._waypoints = waypoints

    def plan(
        self, start: tuple[float, float, float], goal: tuple[float, float, float]
    ) -> np.ndarray | None:
        return self._waypoints


def _stub(waypoints: np.ndarray | None) -> MLSPlanner:
    return cast("MLSPlanner", _StubPlanner(waypoints))


def _floor(x_lo: float = 0.0, x_hi: float = 20.0) -> np.ndarray:
    xs, ys = np.meshgrid(np.arange(x_lo, x_hi, VOXEL), np.arange(-2, 6, VOXEL))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.05)], axis=1, dtype=np.float32)


def _meta_scene() -> tuple[np.ndarray, EvalConfig, Case]:
    """A floored corridor with a wall at x=10 between x=2 and x=18."""
    scene = np.concatenate([_floor(), _wall(10.0)])
    keys = np.unique(voxel_keys(scene, VOXEL))
    cfg = EvalConfig(voxel_size=VOXEL)
    case = Case(id="meta", start=(2.0, 0.0, 0.0), goal=(18.0, 0.0, 0.0))
    return keys, cfg, case


def _u_route() -> np.ndarray:
    return np.array(
        [[2, 0, 0], [2, 4, 0], [18, 4, 0], [18, 0, 0]],
        dtype=np.float32,
    )


def test_meta_straight_line_cheat_scores_zero() -> None:
    """A planner that ignores the map and beelines must not score."""
    keys, cfg, case = _meta_scene()
    line = np.array([case.start, case.goal], dtype=np.float32)
    out, _ = _run_plan(_stub(line), case, 24.0, keys, keys, cfg)
    assert out.planned and out.reached and out.supported
    assert not out.valid
    assert out.spl == 0.0
    assert out.collisions
    assert out.min_clearance is not None and out.min_clearance < 0


def test_meta_no_path_scores_zero_with_miss() -> None:
    keys, cfg, case = _meta_scene()
    out, _ = _run_plan(_stub(None), case, 24.0, keys, keys, cfg)
    assert not out.planned
    assert out.spl == 0.0
    assert out.goal_miss == pytest.approx(16.0)
    assert out.min_clearance is None


def test_meta_demonstrated_route_scores_full() -> None:
    """The route the robot actually walked must earn full SPL."""
    keys, cfg, case = _meta_scene()
    route = _u_route()
    l_ref = metrics.path_length(route)
    out, _ = _run_plan(_stub(route), case, l_ref, keys, keys, cfg)
    assert out.success
    assert out.spl == pytest.approx(1.0)
    assert out.goal_miss == 0.0
    assert out.min_clearance == metrics.MARGIN_CAP_M


def test_meta_everything_occupied_fails_even_good_routes() -> None:
    """An all-occupied map must collapse the score, not inflate it."""
    xs, ys, zs = np.meshgrid(
        np.arange(0, 20, VOXEL), np.arange(-2, 6, VOXEL), np.arange(0.3, 0.5, VOXEL)
    )
    everything = np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1, dtype=np.float32)
    keys = np.unique(voxel_keys(np.concatenate([_floor(), everything]), VOXEL))
    _, cfg, case = _meta_scene()
    out, _ = _run_plan(_stub(_u_route()), case, 24.0, keys, keys, cfg)
    assert out.planned and out.reached
    assert not out.valid
    assert out.spl == 0.0


def test_meta_floating_bridge_fails_support() -> None:
    """A path across a floor gap collides with nothing but must still fail."""
    gapped = np.concatenate([_floor(0.0, 6.0), _floor(14.0, 20.0)])
    keys = np.unique(voxel_keys(gapped, VOXEL))
    _, cfg, case = _meta_scene()
    line = np.array([case.start, case.goal], dtype=np.float32)
    out, _ = _run_plan(_stub(line), case, 24.0, keys, keys, cfg)
    assert out.planned and out.reached and out.valid
    assert not out.supported
    assert out.spl == 0.0
    assert out.unsupported
    gap_x = np.asarray(out.unsupported, dtype=np.float32)[:, 0]
    assert gap_x.min() > 5.5 and gap_x.max() < 14.5


def test_meta_negative_case_scoring() -> None:
    """A certified-infeasible case scores 1.0 for refusal, 0.0 for any claim."""
    keys, cfg, case = _meta_scene()
    refused, _ = _run_plan(_stub(None), case, 16.0, keys, keys, cfg)
    out = score_negative(refused)
    assert out.success
    assert out.spl == 1.0
    claimed, _ = _run_plan(_stub(_u_route()), case, 16.0, keys, keys, cfg)
    out = score_negative(claimed)
    assert not out.success
    assert out.spl == 0.0
    # A path that wanders but never reaches the goal is still a refusal.
    wander = np.array([case.start, [4.0, 2.0, 0.0]], dtype=np.float32)
    partial, _ = _run_plan(_stub(wander), case, 16.0, keys, keys, cfg)
    assert score_negative(partial).success


def test_check_kinematics_rejects_cliff_jumps() -> None:
    stairs = np.array([[0, 0, 0], [0.4, 0, 0.16], [0.8, 0, 0.32]], dtype=np.float32)
    assert metrics.check_kinematics(stairs, max_slope=1.0, max_step_m=0.2, window_m=0.5).valid
    riser = np.array([[0, 0, 0], [0.08, 0, 0.16]], dtype=np.float32)
    assert metrics.check_kinematics(riser, max_slope=1.0, max_step_m=0.2, window_m=0.5).valid
    # A double riser between adjacent cells is quantization, not a cliff.
    quantized = np.array(
        [[0, 0, 0], [0.4, 0, 0.08], [0.56, 0, 0.4], [0.96, 0, 0.48]], dtype=np.float32
    )
    assert metrics.check_kinematics(quantized, max_slope=1.0, max_step_m=0.2, window_m=0.5).valid
    cliff = np.array([[0, 0, 0], [0.2, 0, 0.9], [1, 0, 0.9]], dtype=np.float32)
    result = metrics.check_kinematics(cliff, max_slope=1.0, max_step_m=0.2, window_m=0.5)
    assert not result.valid
    assert len(result.violation_points) >= 1


def test_pick_along_ray() -> None:
    from dimos.navigation.nav_3d.evaluator.picker import pick_along_ray

    wall = _wall(10.0)
    origin = np.array([0.0, 0.0, 0.5])
    target = np.array([10.0, 0.35, 0.75])
    direction = target - origin
    direction /= np.linalg.norm(direction)
    picked = pick_along_ray(wall, origin, direction)
    assert picked is not None
    assert np.linalg.norm(picked - target) < 0.15
    # The nearest surface along the ray wins over one behind it.
    two_walls = np.concatenate([_wall(10.0), _wall(15.0)])
    picked = pick_along_ray(two_walls, origin, direction)
    assert picked is not None
    assert abs(picked[0] - 10.0) < 0.2
    # A ray into empty space picks nothing.
    up = np.array([0.0, 0.0, 1.0])
    assert pick_along_ray(wall, origin, up) is None


def test_save_suite_roundtrip(tmp_path: Path) -> None:
    suite = Suite(
        dataset="demo",
        cases=[
            Case(id="a", start=(0.0, 0.0, 0.0), goal=(1.0, 2.0, 3.0), weight=2.0, tags=["x"]),
            Case(id="neg", start=(0.0, 0.0, 0.0), goal=(5.0, 5.0, 5.0), expect_fail=True),
        ],
        lidar_stream="other_lidar",
        db="~/recordings/demo.db",
    )
    path = save_suite(suite, tmp_path / "demo.yaml")
    loaded = load_suite(path)
    assert loaded.dataset == "demo"
    assert loaded.lidar_stream == "other_lidar"
    assert loaded.odom_stream == "pointlio_odometry"
    assert loaded.db == "~/recordings/demo.db"
    assert loaded.db_path() == Path.home() / "recordings/demo.db"
    assert loaded.cases[0].goal == (1.0, 2.0, 3.0)
    assert loaded.cases[0].tags == ["x"]
    assert not loaded.cases[0].expect_fail
    assert loaded.cases[1].expect_fail


def test_load_suite(tmp_path: Path) -> None:
    manifest = tmp_path / "demo.yaml"
    manifest.write_text(
        "dataset: demo\n"
        "cases:\n"
        "  - id: a\n"
        "    start: [0, 0, 0]\n"
        "    goal: [1, 2, 3]\n"
        "    weight: 2\n"
        "    tags: [stairs]\n"
    )
    suite = load_suite(manifest)
    assert suite.dataset == "demo"
    assert suite.cases[0].goal == (1.0, 2.0, 3.0)
    assert suite.cases[0].weight == 2.0

    manifest.write_text(
        "dataset: demo\ncases:\n"
        "  - {id: a, start: [0, 0, 0], goal: [1, 2, 3]}\n"
        "  - {id: a, start: [0, 0, 0], goal: [4, 5, 6]}\n"
    )
    with pytest.raises(ValueError, match="duplicate"):
        load_suite(manifest)


def _tripwire_report(spec: dict[str, dict[str, tuple[bool, bool]]]) -> dict[str, object]:
    """Report JSON from a {dataset: {case_id: (inc, fin)}} pass/fail spec."""
    datasets = []
    for dataset, cases in spec.items():
        results = []
        for case_id, (inc, fin) in cases.items():
            case = Case(id=case_id, start=(0.0, 0.0, 0.0), goal=(1.0, 0.0, 0.0))
            results.append(
                CaseResult(
                    id=case_id,
                    dataset=dataset,
                    start=case.start,
                    goal=case.goal,
                    weight=1.0,
                    tags=[],
                    l_ref=1.0,
                    l_ref_snapped=False,
                    plan_ts=0.0,
                    online_voxels=0,
                    map_update_ms=0.0,
                    goal_seen=True,
                    expect_fail=False,
                    online=replace(_no_plan(case, 0.0), success=inc),
                    final=replace(_no_plan(case, 0.0), success=fin),
                    soft_progress=0.0,
                )
            )
        datasets.append(
            DatasetResult(
                dataset=dataset,
                cases=results,
                final_voxels=0,
                map_build_ms=0.0,
                add_frame_ms={},
                frames=0,
            )
        )
    report = Report(
        score=0.0,
        score_soft=0.0,
        final_score=0.0,
        n_cases=0,
        n_success=0,
        n_success_final=0,
        outcome_counts={},
        by_tag={},
        plan_ms={},
        map_update_ms={},
        datasets=datasets,
    )
    return json.loads(json.dumps(report.to_dict()))


def test_tripwire_outcomes() -> None:
    report = _tripwire_report({"office": {"a": (True, False), "b": (False, True)}})
    assert tripwire.outcomes(report) == {
        "office": {"a": {"inc": True, "fin": False}, "b": {"inc": False, "fin": True}}
    }
    d = tripwire.diff(report, report)
    assert d.fixed == [] and d.broke == [] and d.added == [] and d.removed == []


def test_tripwire_perf_violations() -> None:
    report = _tripwire_report({"office": {"a": (True, True)}})
    report["config"] = {"plan_p95_budget_ms": 60.0, "map_update_p95_budget_ms": 3000.0}
    report["plan_ms"] = {"p95": 30.0}
    report["map_update_ms"] = {"p95": 1500.0}
    assert tripwire.perf_violations(report) == []
    report["plan_ms"] = {"p95": 61.0}
    violations = tripwire.perf_violations(report)
    assert len(violations) == 1 and "plan_ms" in violations[0]
    # Reports predating the budgets pass.
    assert tripwire.perf_violations({"datasets": []}) == []


def test_tripwire_exact_differences() -> None:
    report = _tripwire_report({"office": {"a": (True, False)}})
    assert tripwire.exact_differences(report, report) == []
    changed = json.loads(json.dumps(report))
    changed["datasets"][0]["cases"][0]["online"]["length"] = 12.34
    changed["datasets"][0]["cases"][0]["online"]["plan_ms"] = 99.0
    diffs = tripwire.exact_differences(report, changed)
    assert len(diffs) == 1
    assert "length" in diffs[0] and "12.34" in diffs[0]


def test_tripwire_diff_names_every_flip() -> None:
    old = _tripwire_report(
        {"office": {"a": (False, True), "b": (True, True), "gone": (True, True)}}
    )
    new = _tripwire_report(
        {"office": {"a": (True, True), "b": (False, False), "fresh": (True, True)}}
    )
    d = tripwire.diff(old, new)
    assert [(f.key, f.test) for f in d.fixed] == [("office/a", "inc")]
    assert [(f.key, f.test) for f in d.broke] == [("office/b", "inc"), ("office/b", "fin")]
    assert d.added == ["office/fresh"]
    assert d.removed == ["office/gone"]
