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
from typing import TYPE_CHECKING, cast

import numpy as np
import pytest

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.cases import Case
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.final_map import MapCheckpoints, encode_deltas, replay_frames
from dimos.navigation.nav_3d.evaluator.recording import Frame, Trajectory
from dimos.navigation.nav_3d.evaluator.runner import _run_plan, score_negative
from dimos.navigation.nav_3d.evaluator.voxel_keys import key_centers, keys_contain, voxel_keys

if TYPE_CHECKING:
    from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner

VOXEL = 0.1


def _cfg(**overrides: float) -> EvalConfig:
    return replace(EvalConfig(voxel_size=VOXEL), **overrides)


def _wall(x: float) -> np.ndarray:
    ys, zs = np.meshgrid(np.arange(-1, 1, VOXEL), np.arange(0.05, 1.5, VOXEL))
    return np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1, dtype=np.float32)


def test_voxel_key_roundtrip() -> None:
    pts = np.array([[0.05, 0.05, 0.05], [-3.21, 4.7, -0.09], [80.0, -80.0, 12.3]], dtype=np.float32)
    centers = key_centers(voxel_keys(pts, VOXEL), VOXEL)
    assert np.all(np.abs(centers - pts) <= VOXEL / 2 + 1e-5)


def test_keys_contain() -> None:
    keys = np.sort(voxel_keys(np.array([[0, 0, 0], [1, 1, 1]], dtype=np.float32), VOXEL))
    query = voxel_keys(np.array([[0, 0, 0], [5, 5, 5]], dtype=np.float32), VOXEL)
    assert keys_contain(keys, query).tolist() == [True, False]
    assert keys_contain(np.array([], dtype=np.int64), query).tolist() == [False, False]


def _gate(waypoints: np.ndarray, obstacles: np.ndarray) -> metrics.GateResult:
    keys = np.unique(voxel_keys(obstacles, VOXEL))
    return metrics.check_path(waypoints, keys, _cfg())


def test_gate_blocks_wall_crossing() -> None:
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    result = _gate(path, _wall(0.0))
    assert not result.valid
    assert len(result.collision_points) > 0
    # Collisions fall within the box half-length (0.35) of the wall.
    assert np.all(np.abs(result.collision_points[:, 0]) < 0.45)


def test_gate_box_uses_travel_orientation() -> None:
    """The body box is long along travel (0.7) and narrow across it (0.31)."""
    path = np.array([[-0.5, 0, 0], [0.5, 0, 0]], dtype=np.float32)
    # 0.25 m ahead along travel is inside the 0.35 m half-length.
    ahead = np.array([[0.25, 0.0, 0.35]], dtype=np.float32)
    assert not _gate(path, ahead).valid
    # The same 0.25 m offset to the side is outside the 0.155 m half-width.
    beside = np.array([[0.0, 0.25, 0.35]], dtype=np.float32)
    assert _gate(path, beside).valid


def test_gate_passes_clear_path() -> None:
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    assert _gate(path, _wall(5.0)).valid


def test_gate_ignores_ground() -> None:
    xs, ys = np.meshgrid(np.arange(-2, 2, VOXEL), np.arange(-2, 2, VOXEL))
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.05)], axis=1, dtype=np.float32)
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    assert _gate(path, floor).valid


def test_gate_pitch_clears_rising_step() -> None:
    """A voxel ahead-and-up is inside a flat box but beyond the pitched one."""
    step = np.array([[0.3, 0.0, 0.4]], dtype=np.float32)
    # Level travel: the step sits in the horizontal body band and collides.
    assert not _gate(np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32), step).valid
    # Climbing at 45 degrees: the box pitches up, so the same voxel falls beyond
    # the tilted body and clears.
    assert _gate(np.array([[0, 0, 0], [1, 0, 1]], dtype=np.float32), step).valid


def test_gate_tolerates_stair_slope() -> None:
    """Terrain rising at stair slope inside the body box must not trigger the gate."""
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
    # Travel is +y here, so the wall 0.35 m away in x sits off the box's
    # 0.155 m half-width.
    assert result.min_clearance_m == pytest.approx(0.35 - 0.155, abs=0.02)
    crossing = _gate(np.array([[9, 0, 0], [11, 0, 0]], dtype=np.float32), wall)
    assert not crossing.valid
    assert crossing.min_clearance_m < 0
    far = _gate(np.array([[2, 0, 0], [3, 0, 0]], dtype=np.float32), wall)
    assert far.min_clearance_m == metrics.MARGIN_CAP_M


def test_reference_length_snaps_to_trajectory() -> None:
    positions = np.stack(
        [np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1
    ).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 10, 101), positions=positions)
    # Walking toward a never-yet-visited goal is not causal.
    ref = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), _cfg())
    assert ref.snapped
    assert ref.length == pytest.approx(10.0, abs=0.01)
    assert not ref.causal
    assert ref.start_ts == float("inf")
    # Returning to the walk's origin is causal.
    ref = metrics.reference_length(traj, (10, 0, 0), (0, 0, 0), _cfg())
    assert ref.causal
    assert 9.0 <= ref.start_ts <= 10.0
    ref = metrics.reference_length(traj, (0, 5, 0), (10, 0, 0), _cfg())
    assert not ref.snapped
    assert ref.start_ts == float("inf")


def test_reference_length_uses_shortest_revisit() -> None:
    """An out-and-back trajectory must not inflate the reference with the loop."""
    out = np.stack([np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1)
    detour = np.stack([np.full(101, 10.0), np.linspace(0, 30, 101), np.full(101, 0.3)], axis=1)
    back = detour[::-1]
    positions = np.concatenate([out, detour, back]).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 30, len(positions)), positions=positions)
    ref = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), _cfg())
    assert ref.snapped
    assert ref.length == pytest.approx(10.0, abs=0.2)


def test_checkpoint_deltas_roundtrip() -> None:
    snapshots = [
        np.array([1, 2, 3], dtype=np.int64),
        np.array([2, 3, 4, 5], dtype=np.int64),
        np.array([4, 5], dtype=np.int64),
    ]
    added, removed = encode_deltas(snapshots)
    ckpt = MapCheckpoints(times=np.arange(3, dtype=np.float64), added=added, removed=removed)
    for original, keys in zip(snapshots, ckpt.iter_snapshots(), strict=True):
        assert np.array_equal(original, keys)


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
    final, snapshots = replay_frames(frames, mapper, VOXEL, times)
    assert final.frames == 6
    sizes = [len(s) for s in snapshots]
    assert 0 < sizes[0] < sizes[1] < sizes[2]
    assert np.array_equal(snapshots[2], final.occupied_keys)
    for earlier, later in itertools.pairwise(snapshots):
        assert keys_contain(later, earlier).all()
    # Each checkpoint holds the walls mapped by its time: wall 1 by the first,
    # wall 3 only at the end.
    wall1 = np.unique(voxel_keys(_wall(5.0), VOXEL))
    wall3 = np.unique(voxel_keys(_wall(11.0), VOXEL))
    assert keys_contain(snapshots[0], wall1).all()
    assert not keys_contain(snapshots[0], wall3).any()
    assert keys_contain(snapshots[2], wall3).all()


def test_check_kinematics_rejects_cliff_jumps() -> None:
    stairs = np.array([[0, 0, 0], [0.4, 0, 0.16], [0.8, 0, 0.32]], dtype=np.float32)
    assert metrics.check_kinematics(stairs, _cfg(max_slope=1.0)).valid
    riser = np.array([[0, 0, 0], [0.08, 0, 0.16]], dtype=np.float32)
    assert metrics.check_kinematics(riser, _cfg(max_slope=1.0)).valid
    # A double riser between adjacent cells is quantization, not a cliff.
    quantized = np.array(
        [[0, 0, 0], [0.4, 0, 0.08], [0.56, 0, 0.4], [0.96, 0, 0.48]], dtype=np.float32
    )
    assert metrics.check_kinematics(quantized, _cfg(max_slope=1.0)).valid
    cliff = np.array([[0, 0, 0], [0.2, 0, 0.9], [1, 0, 0.9]], dtype=np.float32)
    result = metrics.check_kinematics(cliff, _cfg(max_slope=1.0))
    assert not result.valid
    assert len(result.violation_points) >= 1


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
    return np.array([[2, 0, 0], [2, 4, 0], [18, 4, 0], [18, 0, 0]], dtype=np.float32)


def test_meta_straight_line_cheat_scores_zero() -> None:
    """A planner that ignores the map and beelines must not score."""
    keys, cfg, case = _meta_scene()
    line = np.array([case.start, case.goal], dtype=np.float32)
    out, _ = _run_plan(_stub(line), case, 24.0, keys, keys, cfg)
    assert out.planned and out.reached and out.supported
    assert not out.valid
    assert out.spl == 0.0
    assert out.min_clearance is not None and out.min_clearance < 0


def test_meta_no_path_scores_zero() -> None:
    keys, cfg, case = _meta_scene()
    out, _ = _run_plan(_stub(None), case, 24.0, keys, keys, cfg)
    assert not out.planned
    assert out.spl == 0.0
    assert out.min_clearance is None


def test_meta_demonstrated_route_scores_full() -> None:
    """The route the robot actually walked must earn full SPL."""
    keys, cfg, case = _meta_scene()
    route = _u_route()
    l_ref = metrics.path_length(route)
    out, _ = _run_plan(_stub(route), case, l_ref, keys, keys, cfg)
    assert out.success
    assert out.spl == pytest.approx(1.0)
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
