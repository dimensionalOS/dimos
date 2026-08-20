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
from pathlib import Path
from typing import TYPE_CHECKING, cast

import numpy as np
from numpy.typing import NDArray
import pytest
import typer

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.evaluator import metrics, runner
from dimos.navigation.nav_3d.evaluator.cases import Case, Suite, load_suite, save_suite
from dimos.navigation.nav_3d.evaluator.cli import _apply_overrides
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.curation import CaseStore, CurationError, _curated_tags
from dimos.navigation.nav_3d.evaluator.generate import Candidate, _select_diverse, generate_cases
from dimos.navigation.nav_3d.evaluator.picker import pick_along_ray
from dimos.navigation.nav_3d.evaluator.pipeline import PIPELINES
from dimos.navigation.nav_3d.evaluator.recording import (
    Trajectory,
    iter_world_frames,
    load_trajectory,
)
from dimos.navigation.nav_3d.evaluator.runner import (
    Occupancy,
    _final_only,
    _run_plan,
    score_negative,
)
from dimos.navigation.nav_3d.evaluator.voxel_keys import (
    cylinder_offsets,
    keys_contain,
    offset_deltas,
    voxel_keys,
)

if TYPE_CHECKING:
    from dimos.navigation.nav_3d.evaluator.pipeline import NavPipeline

VOXEL = 0.1
Point = tuple[float, float, float]


def _cfg(**overrides: object) -> EvalConfig:
    return replace(EvalConfig(voxel_size=VOXEL), **overrides)


def _wall(x: float) -> NDArray[np.float32]:
    ys, zs = np.meshgrid(np.arange(-1, 1, VOXEL), np.arange(0.05, 1.5, VOXEL))
    return np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1, dtype=np.float32)


def _floor(x_lo: float = 0.0, x_hi: float = 20.0) -> NDArray[np.float32]:
    xs, ys = np.meshgrid(np.arange(x_lo, x_hi, VOXEL), np.arange(-2, 6, VOXEL))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.05)], axis=1, dtype=np.float32)


def _occupancy(points: NDArray[np.float32], whole_recording: bool = True) -> Occupancy:
    return Occupancy(np.unique(voxel_keys(points, VOXEL)), whole_recording)


def _gate(waypoints: NDArray[np.float32], obstacles: NDArray[np.float32]) -> metrics.GateResult:
    return metrics.check_path(waypoints, _occupancy(obstacles).keys, _cfg())


def test_gate_geometry() -> None:
    """Wall crossing, ground, box orientation/height band, pitch, stair slope, clearance."""
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    result = _gate(path, _wall(0.0))
    assert not result.valid and result.collision_boxes
    centers = np.array([box.center for box in result.collision_boxes])
    assert np.all(np.abs(centers[:, 0]) < 0.45)
    assert np.allclose(centers[:, 2], 0.35, atol=0.01)  # posed on the body, not the foot path
    assert _gate(path, _floor(-2.0, 2.0)).valid  # ground never collides

    # Long (0.7) along travel, narrow (0.31) across it.
    short = np.array([[-0.5, 0, 0], [0.5, 0, 0]], dtype=np.float32)
    assert not _gate(short, np.array([[0.25, 0.0, 0.35]], dtype=np.float32)).valid
    assert _gate(short, np.array([[0.0, 0.25, 0.35]], dtype=np.float32)).valid

    vox = 0.02
    cfg = _cfg(voxel_size=vox)
    tall_path = np.array([[0, 0, 0], [2, 0, 0]], dtype=np.float32)

    def hits(height: float) -> bool:
        point = np.array([[1.0, 0.0, height]], dtype=np.float32)
        return not metrics.check_path(tall_path, np.unique(voxel_keys(point, vox)), cfg).valid

    assert not any(hits(h) for h in (0.0, 0.1, 0.2))  # legs clear
    assert all(hits(h) for h in (0.3, 0.35, 0.4))  # trunk collides
    assert not any(hits(h) for h in (0.5, 0.6))  # overhead clears
    assert hits(cfg.ground_margin) and hits(cfg.body_clearance)  # band edges are the robot

    step = np.array([[0.3, 0.0, 0.4]], dtype=np.float32)
    assert not _gate(np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32), step).valid
    assert _gate(np.array([[0, 0, 0], [1, 0, 1]], dtype=np.float32), step).valid  # pitch clears it

    xs, ys = np.meshgrid(np.arange(-1, 2, VOXEL), np.arange(-1, 1, VOXEL))
    slope = np.stack([xs.ravel(), ys.ravel(), xs.ravel() * 0.7 - 0.05], axis=1, dtype=np.float32)
    stairs = np.stack(
        [np.arange(-0.5, 1.5, 0.1), np.zeros(20), np.arange(-0.5, 1.5, 0.1) * 0.7], axis=1
    ).astype(np.float32)
    assert _gate(stairs, slope).valid  # terrain at stair slope inside the box is not a collision

    t = np.arange(-3, 3.01, 0.1)
    c = 1 / np.sqrt(2)
    tilted = np.stack([t * c, np.zeros_like(t), t * c], axis=1).astype(np.float32)
    samples = metrics.densify(tilted, vox / 2)
    i = len(samples) // 2
    _, _, up = metrics.body_frames(samples, cfg.robot_length)
    mid_h = (cfg.ground_margin + cfg.body_clearance) / 2.0

    def tilted_hits(point: np.ndarray) -> bool:
        keys = np.unique(voxel_keys(np.asarray([point], dtype=np.float32), vox))
        return not metrics.check_path(tilted, keys, cfg).valid

    assert tilted_hits(samples[i] + mid_h * up[i])  # the band tilts with the body...
    assert not tilted_hits(samples[i] + 0.15 * up[i])  # ...so the leg zone stays clear

    wall = _wall(10.0)
    graze = np.array([[9.7, -0.5, 0], [9.7, 0.5, 0]], dtype=np.float32)
    clearance = _gate(graze, wall)
    assert clearance.valid
    assert clearance.min_clearance_m == pytest.approx(0.35 - 0.155, abs=0.02)
    crossing = _gate(np.array([[9, 0, 0], [11, 0, 0]], dtype=np.float32), wall)
    assert not crossing.valid and crossing.min_clearance_m < 0
    far = _gate(np.array([[2, 0, 0], [3, 0, 0]], dtype=np.float32), wall)
    assert far.min_clearance_m == metrics.MARGIN_CAP_M


def test_body_frames_chord_direction_and_voxel_keys() -> None:
    # A degenerate (zero-length) path still frames, right-handed.
    point = np.array([[1.0, 1.0, 0.0], [1.0, 1.0, 0.0]], dtype=np.float32)
    fwd, lateral, up = metrics.body_frames(point, 0.7)
    assert np.allclose(np.linalg.norm(fwd, axis=1), 1.0)
    assert np.linalg.det(np.stack((fwd, lateral, up), axis=-1)[0]) > 0

    # Heading follows the length chord, not local slope, so stairs do not flip it.
    xs = np.arange(0, 3.0, 0.1)
    zs = np.floor(xs / 0.2) * 0.1  # stairs: 0.1 m rise every 0.2 m, mean slope 0.5
    path = np.stack([xs, np.zeros_like(xs), zs], axis=1).astype(np.float32)
    chord_fwd = metrics.chord_directions(path, span=0.7)
    local = np.diff(path.astype(np.float64), axis=0)
    local /= np.linalg.norm(local, axis=1, keepdims=True)
    assert chord_fwd[5:-5, 2].std() < 0.5 * local[:, 2].std()
    assert chord_fwd[5:-5, 2].mean() > 0.2

    # A packed offset delta must equal packing the summed index.
    pts = np.array([[0.05, -3.2, 1.4], [12.0, 0.0, -2.0]], dtype=np.float32)
    offs = cylinder_offsets(0.4, -0.2, 0.3, VOXEL)
    idx = np.floor(pts.astype(np.float64) / VOXEL).astype(np.int64)[:, None, :] + offs[None, :, :]
    packed = voxel_keys((idx.reshape(-1, 3) * VOXEL + VOXEL / 2).astype(np.float32), VOXEL)
    shifted = voxel_keys(pts, VOXEL)[:, None] + offset_deltas(offs)[None, :]
    assert np.array_equal(shifted.ravel(), packed)

    keys = np.unique(voxel_keys(_wall(1.0), VOXEL))
    assert keys_contain(keys, keys).all()
    with pytest.raises(AssertionError, match="sorted"):
        keys_contain(keys[::-1].copy(), keys)  # unsorted keys would silently pass every gate


def test_config_and_kinematics_validation() -> None:
    """Config rejects bad values up front; kinematics rejects only real cliffs."""
    with pytest.raises(ValueError, match="body_clearance"):
        EvalConfig(ground_margin=0.5, body_clearance=0.45)  # inverted band admits nothing
    for name in ("voxel_size", "max_range", "robot_height", "goal_tolerance", "visit_radius_m"):
        with pytest.raises(ValueError, match=name):
            EvalConfig(**{name: 0.0})
    for name in ("goal_tolerance", "max_slope", "kinematic_window_m"):
        with pytest.raises(ValueError, match=name):
            EvalConfig(**{name: float("nan")})  # nan passes every bound check

    stairs = np.array([[0, 0, 0], [0.4, 0, 0.16], [0.8, 0, 0.32]], dtype=np.float32)
    assert metrics.check_kinematics(stairs, _cfg(max_slope=1.0)).valid
    riser = np.array([[0, 0, 0], [0.08, 0, 0.16]], dtype=np.float32)
    assert metrics.check_kinematics(riser, _cfg(max_slope=1.0)).valid
    quantized = np.array(
        [[0, 0, 0], [0.4, 0, 0.08], [0.56, 0, 0.4], [0.96, 0, 0.48]], dtype=np.float32
    )
    assert metrics.check_kinematics(
        quantized, _cfg(max_slope=1.0)
    ).valid  # double riser, not a cliff
    cliff = np.array([[0, 0, 0], [0.2, 0, 0.9], [1, 0, 0.9]], dtype=np.float32)
    result = metrics.check_kinematics(cliff, _cfg(max_slope=1.0))
    assert not result.valid and len(result.violation_points) >= 1


def test_reference_length_and_trajectory_memoization() -> None:
    positions = np.stack(
        [np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1
    ).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 10, 101), positions=positions)
    ref = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), _cfg())
    assert ref.snapped and ref.length == pytest.approx(10.0, abs=0.01)
    assert not ref.causal and ref.start_ts == float("inf")  # never-yet-visited goal is not causal
    ref = metrics.reference_length(traj, (10, 0, 0), (0, 0, 0), _cfg())
    assert ref.causal and 9.0 <= ref.start_ts <= 10.0  # returning to the walk's origin is causal
    ref = metrics.reference_length(traj, (0, 5, 0), (10, 0, 0), _cfg())
    assert not ref.snapped and ref.start_ts == float("inf")

    # An out-and-back trajectory must not inflate the reference with the loop.
    out = np.stack([np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1)
    detour = np.stack([np.full(101, 10.0), np.linspace(0, 30, 101), np.full(101, 0.3)], axis=1)
    loop_pos = np.concatenate([out, detour, detour[::-1]]).astype(np.float32)
    loop_traj = Trajectory(ts=np.linspace(0, 30, len(loop_pos)), positions=loop_pos)
    ref = metrics.reference_length(loop_traj, (0, 0, 0), (10, 0, 0), _cfg())
    assert ref.snapped and ref.length == pytest.approx(10.0, abs=0.2)

    # Derived views are memoized per trajectory, since every case asks for the same two.
    assert traj.arc_lengths() is traj.arc_lengths()
    assert traj.foot(0.3) is traj.foot(0.3)
    assert not np.array_equal(traj.foot(0.5), traj.foot(0.3))  # a new height replaces the memo


class _StubPipeline:
    """Configurable stub: a fixed route, or per-(start, goal) routes; tracks frames-at-plan."""

    def __init__(
        self,
        waypoints: NDArray[np.float32] | None = None,
        routes: dict[tuple[Point, Point], NDArray[np.float32]] | None = None,
    ) -> None:
        self._waypoints = waypoints
        self._routes = routes
        self.frames = 0
        self.frames_at_plan: list[int] = []

    def add_frame(self, points: NDArray[np.float32], origin: Point, ts: float) -> None:
        self.frames += 1

    def sync_map(self) -> None:
        pass

    def plan(self, start: Point, goal: Point) -> NDArray[np.float32] | None:
        self.frames_at_plan.append(self.frames)
        return self._routes.get((start, goal)) if self._routes is not None else self._waypoints


def _stub(waypoints: NDArray[np.float32] | None) -> NavPipeline:
    return cast("NavPipeline", _StubPipeline(waypoints))


def _meta_case() -> Case:
    return Case(id="meta", start=(2.0, 0.0, 0.0), goal=(18.0, 0.0, 0.0))


def _u_route() -> NDArray[np.float32]:
    return np.array([[2, 0, 0], [2, 4, 0], [18, 4, 0], [18, 0, 0]], dtype=np.float32)


def _meta_map() -> Occupancy:
    """A floored corridor with a wall at x=10, as the pipeline mapped it."""
    return _occupancy(np.concatenate([_floor(), _wall(10.0)]))


def test_run_plan_success_and_cheat_detection() -> None:
    out = _run_plan(_stub(None), _meta_case(), 24.0, _cfg(), _meta_map())
    assert not out.planned and out.spl == 0.0 and out.min_clearance is None

    route = _u_route()
    out = _run_plan(_stub(route), _meta_case(), metrics.path_length(route), _cfg(), _meta_map())
    assert out.success and out.spl == pytest.approx(1.0)
    assert out.min_clearance == metrics.MARGIN_CAP_M

    # A planner that beelines through occupancy its own mapper saw must not score.
    case = _meta_case()
    line = np.array([case.start, case.goal], dtype=np.float32)
    out = _run_plan(_stub(line), case, 24.0, _cfg(), _meta_map())
    assert out.planned and out.reached and out.supported
    assert not out.valid and out.spl == 0.0
    assert out.min_clearance is not None and out.min_clearance < 0 and out.collisions

    # No map exposed: the gates are skipped, scored on reachability and climb alone.
    out = _run_plan(_stub(line), case, 24.0, _cfg())
    assert out.success and out.valid and out.supported and out.min_clearance is None

    cliff = np.array([[2, 0, 0], [9.9, 0, 0], [10, 0, 3], [18, 0, 0]], dtype=np.float32)
    out = _run_plan(_stub(cliff), case, 24.0, _cfg())
    assert out.planned and out.reached and not out.success and out.spl == 0.0 and out.steep


def test_run_plan_support_gate_and_negative_scoring() -> None:
    case = _meta_case()
    line = np.array([case.start, case.goal], dtype=np.float32)
    gapped = np.concatenate([_floor(0.0, 6.0), _floor(14.0, 20.0)])
    out = _run_plan(_stub(line), case, 24.0, _cfg(), _occupancy(gapped))
    assert out.planned and out.reached and out.valid
    assert not out.supported and out.spl == 0.0
    gap_x = np.asarray(out.unsupported, dtype=np.float32)[:, 0]
    assert gap_x.min() > 5.5 and gap_x.max() < 14.5

    seen = _floor(0.0, 6.0)
    online = _run_plan(_stub(line), case, 24.0, _cfg(), _occupancy(seen, whole_recording=False))
    assert online.supported  # unmapped floor is not a void; support waits for the whole recording
    final = _run_plan(_stub(line), case, 24.0, _cfg(), _occupancy(seen))
    assert not final.supported

    # A certified-infeasible case scores 1.0 for refusal, 0.0 for any claim.
    refused = _run_plan(_stub(None), case, 16.0, _cfg())
    assert score_negative(refused).spl == 1.0
    claimed = _run_plan(_stub(_u_route()), case, 16.0, _cfg())
    out = score_negative(claimed)
    assert not out.success and out.spl == 0.0
    wander = np.array([case.start, [4.0, 2.0, 0.0]], dtype=np.float32)
    partial = _run_plan(_stub(wander), case, 16.0, _cfg())
    assert score_negative(partial).success  # never reaching the goal is still a refusal

    assert metrics.spl(True, 10.0, 20.0) == pytest.approx(0.5)
    assert metrics.spl(True, 10.0, 12.5) == pytest.approx(0.8)
    assert metrics.spl(True, 10.0, 4.0) == pytest.approx(1.0)  # never more than 1.0
    assert metrics.spl(False, 10.0, 10.0) == 0.0


def test_case_generation() -> None:
    """A U-shaped walk yields cases spanning the wall; the stairs quota backfills to the floor."""
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
    cfg = _cfg()
    cases = generate_cases(traj, cfg, max_cases=10)
    assert cases
    assert [c for c in cases if (c.start[0] - 10) * (c.goal[0] - 10) < 0]
    foot = traj.foot(cfg.robot_height)
    for c in cases:
        assert "flat" in c.tags
        for point in (c.start, c.goal):
            gap = np.linalg.norm(foot - np.asarray(point, dtype=np.float32), axis=1).min()
            assert gap < 1e-5

    candidates = [
        Candidate(
            start=(float(x), 0.0, 0.0),
            goal=(float(x), 20.0, 2.0),
            walked_m=30.0,
            detour_ratio=1.5,
            dz=2.0,
        )
        for x in np.arange(0.0, 24.0, 4.0)
    ]
    strict = _select_diverse(candidates, max_cases=6, min_cases=0)
    backfilled = _select_diverse(candidates, max_cases=6, min_cases=6)
    assert len(strict) < 6
    assert len(backfilled) == 6
    assert len({(c.start, c.goal) for c in backfilled}) == len(backfilled)
    assert len(_select_diverse(candidates, max_cases=3, min_cases=6)) == 3


def test_pick_along_ray() -> None:
    wall = _wall(10.0)
    origin = np.array([0.0, 0.0, 0.5])
    target = np.array([10.0, 0.35, 0.75])
    direction = target - origin
    direction /= np.linalg.norm(direction)
    picked = pick_along_ray(wall, origin, direction, VOXEL)
    assert picked is not None
    assert np.linalg.norm(picked - target) < 0.15
    two_walls = np.concatenate([_wall(10.0), _wall(15.0)])
    picked = pick_along_ray(two_walls, origin, direction, VOXEL)
    assert picked is not None
    assert abs(picked[0] - 10.0) < 0.2  # the nearest surface along the ray wins over one behind it
    up = np.array([0.0, 0.0, 1.0])
    assert pick_along_ray(wall, origin, up, VOXEL) is None  # a ray into empty space picks nothing


def _store(tmp_path: Path, cases: list[Case]) -> CaseStore:
    manifest = save_suite(Suite(dataset="demo", cases=cases), tmp_path / "demo.yaml")
    return CaseStore(load_suite(manifest))


def test_suite_yaml_roundtrip_and_curation(tmp_path: Path) -> None:
    suite = Suite(
        dataset="demo",
        cases=[
            Case(id="a", start=(0.0, 0.0, 0.0), goal=(1.0, 2.0, 3.0), tags=["x"]),
            Case(id="neg", start=(0.0, 0.0, 0.0), goal=(5.0, 5.0, 5.0), expect_fail=True),
            Case(id="dyn", start=(0.0, 0.0, 0.0), goal=(3.0, 0.0, 0.0), expect_final_fail=True),
        ],
        lidar_stream="other_lidar",
    )
    loaded = load_suite(save_suite(suite, tmp_path / "demo.yaml"))
    assert loaded.dataset == "demo" and loaded.lidar_stream == "other_lidar"
    assert loaded.cases[0].goal == (1.0, 2.0, 3.0) and loaded.cases[0].tags == ["x"]
    assert loaded.cases[1].expect_fail and loaded.cases[2].expect_final_fail

    case = Case(
        id="auto_00_flat", start=(1.0, 0.0, 0.0), goal=(5.0, 0.0, 0.0), tags=["auto", "flat"]
    )
    store = _store(tmp_path, [case])
    store.update("auto_00_flat", "auto_00_flat", ["flat", "stairs"], expect_fail=False)
    edited = store.get("auto_00_flat")
    assert "manual" not in edited.tags and not _final_only(edited)  # relabelling keeps provenance

    added = store.add((1.0, 0.0, 0.0), (4.0, 0.0, 0.0), ["flat"])
    assert added.tags[0] == "manual" and _final_only(added)  # hand-added is curated, final-only

    with pytest.raises(CurationError, match="already exists"):
        store.add((1.0, 0.0, 0.0), (5.0, 0.0, 0.0), [], case_id=added.id)
    with pytest.raises(CurationError, match="not found"):
        store.get("nope")
    far = store.add((1.0, 0.0, 0.0), (400.0, 0.0, 0.0), [])
    assert far.goal == (400.0, 0.0, 0.0)  # endpoints are not checked against any map
    assert _curated_tags(["flat", "negative"], True) == ["manual", "negative", "flat"]

    # expect_final_fail inverts the final outcome whatever the provenance.
    for tags in (["auto"], ["manual"]):
        marked = replace(_meta_case(), tags=tags, expect_final_fail=True)
        refused = _run_plan(_stub(None), marked, 16.0, _cfg())
        assert score_negative(refused).spl == 1.0
        assert _final_only(marked) == ("auto" not in tags)


def _write_recording(
    path: Path,
    clouds: list[tuple[float, np.ndarray, str]],
    poses: list[tuple[float, tuple[float, float, float]]],
) -> None:
    """A minimal mem2 recording: sensor-frame clouds plus odometry."""
    with SqliteStore(path=str(path)) as store:
        lidar = store.stream("lidar", PointCloud2)
        for ts, pts, frame_id in clouds:
            lidar.append(PointCloud2.from_numpy(pts, frame_id=frame_id, timestamp=ts), ts=ts)
        odom = store.stream("odom", Odometry)
        for ts, (x, y, z) in poses:
            odom.append(Odometry(ts=ts, pose=Pose(x, y, z)), ts=ts)


def test_recording_reads_frames_trajectory_and_rotation(tmp_path: Path) -> None:
    """Clouds are placed and rotated by aligned odometry; pre-registered world clouds refuse."""
    local = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]], dtype=np.float32)
    db = tmp_path / "rec.db"
    _write_recording(
        db,
        [(1.0, local, "lidar"), (2.0, local, "lidar"), (3.0, local, "lidar")],
        [(1.0, (10.0, 0.0, 0.5)), (2.0, (20.0, 0.0, 0.5)), (3.0, (30.0, 0.0, 0.5))],
    )
    frames = list(iter_world_frames(db, "lidar", "odom"))
    assert [f.ts for f in frames] == [1.0, 2.0, 3.0]
    assert np.allclose(frames[0].points, local + np.array([10.0, 0.0, 0.5]))
    assert frames[1].origin == (20.0, 0.0, 0.5)
    traj = load_trajectory(db, "odom")
    assert len(traj.positions) == 3
    assert np.allclose(traj.foot(0.5)[:, 2], 0.0)

    legacy = tmp_path / "legacy.db"
    world_pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    _write_recording(legacy, [(1.0, world_pts, "world")], [(1.0, (0.0, 0.0, 0.0))])
    with pytest.raises(ValueError, match="world-frame"):
        list(iter_world_frames(legacy, "lidar", "odom"))
    with pytest.raises(ValueError, match="no odometry"):
        load_trajectory(legacy, "missing_stream")

    # A yawed pose must rotate a sensor-frame cloud, not just shift it.
    yaw_db = tmp_path / "yaw.db"
    ahead = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    quarter = np.sqrt(0.5)  # 90 degrees about +z: sensor +x points along world +y
    with SqliteStore(path=str(yaw_db)) as store:
        store.stream("lidar", PointCloud2).append(
            PointCloud2.from_numpy(ahead, frame_id="lidar", timestamp=1.0), ts=1.0
        )
        store.stream("odom", Odometry).append(
            Odometry(ts=1.0, pose=Pose(5.0, 0.0, 0.0, 0.0, 0.0, quarter, quarter)), ts=1.0
        )
    yawed = list(iter_world_frames(yaw_db, "lidar", "odom"))
    assert np.allclose(yawed[0].points, [[5.0, 1.0, 0.0]], atol=1e-5)

    # The progress denominator is a query, so it has to agree with the replay.
    pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    count_db = tmp_path / "count.db"
    _write_recording(
        count_db,
        [(float(t), pts, "lidar") for t in range(1, 6)],
        [(float(t), (float(t), 0.0, 0.0)) for t in range(1, 6)],
    )
    suite = Suite(dataset=str(count_db), cases=[], lidar_stream="lidar", odom_stream="odom")
    assert suite.frame_count() == 5
    assert len(list(suite.world_frames(_cfg().align_tol))) == 5


def test_apply_overrides_is_the_sweep_interface() -> None:
    """--set is how a sweep varies the harness and the pipeline."""
    cfg = _apply_overrides(
        EvalConfig(), ["goal_tolerance=0.4", "planner.wall_clearance_m=0.0", "pipeline=mls"]
    )
    assert cfg.goal_tolerance == pytest.approx(0.4)
    assert isinstance(cfg.goal_tolerance, float)
    assert cfg.planner == {"wall_clearance_m": 0.0}
    assert cfg.pipeline == "mls"
    for bad in (["no_equals_sign"], ["not_a_field=1"], ["planner=0.5"]):
        with pytest.raises(typer.BadParameter):
            _apply_overrides(EvalConfig(), bad)
    # An override is validated after it is applied, since __post_init__ already ran.
    with pytest.raises(typer.BadParameter, match="voxel_size"):
        _apply_overrides(EvalConfig(), ["voxel_size=0"])


def _stub_harness(mp: pytest.MonkeyPatch, pipeline: object) -> None:
    """Register a stub pipeline, so run_suite never touches a native module."""
    mp.setitem(PIPELINES, "stub", lambda _cfg: pipeline)


WALK_HEIGHT = 0.5


def _corridor_recording(path: Path) -> Suite:
    """Ten frames of floor along +x, walked start to finish."""
    slabs = [
        (
            float(t),
            _floor(float(t) - 1.0, float(t) + 1.0)
            - np.array([t, 0, WALK_HEIGHT], dtype=np.float32),
        )
        for t in range(1, 11)
    ]
    _write_recording(
        path,
        [(ts, pts, "lidar") for ts, pts in slabs],
        [(float(t), (float(t), 0.0, WALK_HEIGHT)) for t in range(1, 11)],
    )
    return Suite(dataset=str(path), cases=[], lidar_stream="lidar", odom_stream="odom")


def test_run_suite_plans_each_case_on_the_map_as_of_its_start_time(tmp_path: Path) -> None:
    """A case is scored against exactly the frames the robot had by its start time."""
    suite = _corridor_recording(tmp_path / "corridor.db")
    # Both walk backwards, so the goal was visited before the start and the reference is
    # causal. The second starts later in the recording.
    suite.cases = [
        Case(id="auto_early", start=(4.0, 0.0, 0.0), goal=(2.0, 0.0, 0.0), tags=["auto"]),
        Case(id="auto_late", start=(9.0, 0.0, 0.0), goal=(7.0, 0.0, 0.0), tags=["auto"]),
    ]
    pipeline = _StubPipeline(np.array([[4, 0, 0], [2, 0, 0]], dtype=np.float32))
    with pytest.MonkeyPatch.context() as mp:
        _stub_harness(mp, pipeline)
        result = runner.run_suite(suite, _cfg(pipeline="stub"))

    assert result.frames == 10
    # Two online plans, then one final plan per case on the full recording.
    online_early, online_late = pipeline.frames_at_plan[:2]
    assert 0 < online_early < online_late < 10
    assert pipeline.frames_at_plan[2:] == [10, 10]


def test_evaluate_scores_only_online_cases_in_the_headline(tmp_path: Path) -> None:
    """A failing final-only case drops final_score and leaves score untouched."""
    suite = _corridor_recording(tmp_path / "corridor.db")
    walked: Point = (4.0, 0.0, 0.0)
    goal: Point = (2.0, 0.0, 0.0)
    suite.cases = [
        Case(id="auto_00", start=walked, goal=goal, tags=["auto"]),
        Case(id="manual_00", start=walked, goal=goal, tags=["manual"]),  # never replays online
        Case(id="manual_01", start=walked, goal=(80.0, 0.0, 0.0), tags=["manual"]),  # unplannable
    ]
    route = np.array([walked, (3.0, 0.0, 0.0), goal], dtype=np.float32)
    with pytest.MonkeyPatch.context() as mp:
        _stub_harness(mp, _StubPipeline(routes={(walked, goal): route}))
        report = runner.evaluate([suite], _cfg(pipeline="stub"))

    assert report.n_cases == 3
    assert report.n_online == 1
    assert [c.final_only for d in report.datasets for c in d.cases] == [False, True, True]
    assert report.score == pytest.approx(1.0)  # the online case succeeds on the demonstrated route
    assert report.n_success == 1
    # Two of three cases plan against the final map, so the refused one drags it down.
    assert report.n_success_final == 2
    assert report.final_score == pytest.approx(2 / 3)
    assert report.by_tag["manual"].n_online == 0


def test_a_pipeline_without_graph_layers_still_records(tmp_path: Path) -> None:
    """Recording a run must not require a pipeline to expose its internals."""
    suite = _corridor_recording(tmp_path / "corridor.db")
    suite.cases = [Case(id="auto_00", start=(4.0, 0.0, 0.0), goal=(2.0, 0.0, 0.0), tags=["auto"])]
    route = np.array([[4, 0, 0], [3, 0, 0], [2, 0, 0]], dtype=np.float32)
    with pytest.MonkeyPatch.context() as mp:
        _stub_harness(mp, _StubPipeline(route))
        result = runner.run_suite(suite, _cfg(pipeline="stub"), keep_artifacts=True)

    assert result.final_artifacts is None
    assert result.cases[0].online_artifacts is None
    assert result.cases[0].online.success
