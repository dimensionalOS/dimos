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

"""Crucial-invariant tests for the relocalize.py candidate-generation/judge split
and priors.py's pluggable priors.

All scenes are SIMULATED point clouds (numpy-seeded, deterministic) -- no hardware,
no replay. Only test_relocalize_parity_with_pre_refactor_baseline runs real RANSAC
(RANSAC_ITERS monkeypatched down purely for speed, the same knob/value used to
capture the baseline); every other test injects candidates or monkeypatches the
solve, per the "keep this suite fast" rule.
"""

from __future__ import annotations

from typing import cast

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import pytest
from reactivex import Subject
from scipy.spatial.transform import Rotation

import dimos.mapping.relocalization.module as module_mod
from dimos.mapping.relocalization.module import Config, RelocalizationModule
from dimos.mapping.relocalization.priors import (
    FiducialPrior,
    FiducialPriorConfig,
    RansacPrior,
    RansacPriorConfig,
    RelocPrior,
)
import dimos.mapping.relocalization.relocalize as relocalize_mod
from dimos.mapping.relocalization.relocalize import (
    InsufficientWallEvidenceError,
    refine_candidates,
    relocalize,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


def _pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(points)
    return p


def _rot_z(deg: float) -> np.ndarray:
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rigid(yaw_deg: float, t: tuple[float, float, float]) -> np.ndarray:
    """A gravity-upright (yaw-only) 4x4 rigid transform."""
    T = np.eye(4)
    T[:3, :3] = _rot_z(yaw_deg)
    T[:3, 3] = t
    return T


def _apply(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    return np.asarray((T[:3, :3] @ points.T).T + T[:3, 3])


def _sample_wall(
    rng: np.random.Generator,
    fixed_axis: str,
    fixed_val: float,
    span_lo: float,
    span_hi: float,
    height: float,
    n: int,
) -> np.ndarray:
    span = rng.uniform(span_lo, span_hi, n)
    z = rng.uniform(0.0, height, n)
    if fixed_axis == "x":
        return np.stack([np.full(n, fixed_val), span, z], axis=1)
    return np.stack([span, np.full(n, fixed_val), z], axis=1)


def _sample_plane(
    rng: np.random.Generator, x_extent: float, y_extent: float, z: float, n: int
) -> np.ndarray:
    xy = rng.uniform([0.0, 0.0], [x_extent, y_extent], size=(n, 2))
    return np.concatenate([xy, np.full((n, 1), z)], axis=1)


def _rect_room_scene(
    seed: int, yaw_deg: float, t: tuple[float, float, float]
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, np.ndarray]:
    """global_map (full walled room, `map` frame) + local_map (a partial sub-region,
    `world` frame, related by the returned T_true -- relocalize()'s convention:
    local_map_points -> T_true -> global_map frame). Small and cheap: only the
    tests that drive the real refine_candidates wall-rerank + ICP use it."""
    rng = np.random.default_rng(seed)
    x_extent, y_extent, height = 6.0, 4.0, 2.5
    walls = [
        _sample_wall(rng, "y", 0.0, 0.0, x_extent, height, 700),
        _sample_wall(rng, "y", y_extent, 0.0, x_extent, height, 700),
        _sample_wall(rng, "x", 0.0, 0.0, y_extent, height, 700),
        _sample_wall(rng, "x", x_extent, 0.0, y_extent, height, 700),
    ]
    room_pts = np.concatenate(
        [
            *walls,
            _sample_plane(rng, x_extent, y_extent, 0.0, 2000),
            _sample_plane(rng, x_extent, y_extent, height, 2000),
        ],
        axis=0,
    )
    mask = (room_pts[:, 0] <= 3.2) & (room_pts[:, 1] <= 2.2)
    T_true = _rigid(yaw_deg, t)
    local_pts = _apply(np.linalg.inv(T_true), room_pts[mask].copy())
    return _pcd(room_pts), _pcd(local_pts), T_true


# --- Real-RANSAC parity baseline (invariant 7) -----------------------------
SEED = 42
RANSAC_ITERS_FOR_TEST = 20_000  # matches how the baseline was captured (see below)

# SIMULATED baseline captured 2026-07-16 against the PRE-refactor relocalize() (the
# parent of the split-judge commit), via a throwaway script that built this exact
# deterministic L-shaped-room scene (numpy default_rng(42), o3d.utility.random.
# seed(42)), monkeypatched RANSAC_ITERS 500_000 -> 20_000 purely so the capture
# finished in ~0.3s, and called relocalize(). Three runs (2 pre-refactor, 1 post)
# produced this exact T (bit-identical) and fitness -- confirming the per-frame
# seeding determinism, and that the refactor preserves the judge's output.
_BASELINE_FITNESS = 0.9980842911877394
_BASELINE_T = np.array(
    [
        [9.06300310e-01, -4.22634295e-01, -3.08267634e-05, 1.49997708e00],
        [4.22634294e-01, 9.06300310e-01, -2.48298213e-05, -8.00273450e-01],
        [3.84322392e-05, 9.47482734e-06, 9.99999999e-01, 4.98791466e-02],
        [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
    ]
)


def _l_room_mask(xy: np.ndarray) -> np.ndarray:
    """L-shaped footprint: 8x6m rect minus a 3x3m corner at x in [5,8], y in [3,6]."""
    x, y = xy[:, 0], xy[:, 1]
    return (x >= 0) & (x <= 8) & (y >= 0) & (y <= 6) & ~((x >= 5) & (x <= 8) & (y >= 3) & (y <= 6))


def _l_room_plane(rng: np.random.Generator, z: float, n_target: int) -> np.ndarray:
    pts, got = [], 0
    while got < n_target:
        batch = rng.uniform([0, 0], [8, 6], size=(n_target * 2, 2))
        kept = batch[_l_room_mask(batch)]
        pts.append(kept)
        got += len(kept)
    xy = np.concatenate(pts, axis=0)[:n_target]
    return np.concatenate([xy, np.full((n_target, 1), z)], axis=1)


def _l_room_box(
    rng: np.random.Generator, center: np.ndarray, half_extent: np.ndarray, n: int
) -> np.ndarray:
    n_per_face = n // 6
    faces = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            local = np.zeros((n_per_face, 3))
            other = [a for a in range(3) if a != axis]
            local[:, other[0]] = rng.uniform(-1.0, 1.0, n_per_face)
            local[:, other[1]] = rng.uniform(-1.0, 1.0, n_per_face)
            local[:, axis] = sign
            faces.append(center + local * half_extent)
    return np.concatenate(faces, axis=0)


def _build_l_room(rng: np.random.Generator) -> np.ndarray:
    walls = [
        _sample_wall(rng, "y", 0.0, 0.0, 8.0, 2.5, 2500),
        _sample_wall(rng, "x", 8.0, 0.0, 3.0, 2.5, 2500),
        _sample_wall(rng, "y", 3.0, 5.0, 8.0, 2.5, 2500),
        _sample_wall(rng, "x", 5.0, 3.0, 6.0, 2.5, 2500),
        _sample_wall(rng, "y", 6.0, 0.0, 5.0, 2.5, 2500),
        _sample_wall(rng, "x", 0.0, 0.0, 6.0, 2.5, 2500),
    ]
    # RNG consumption order is load-bearing: the baseline was captured with floor and
    # ceiling drawn before the boxes -- reordering changes the scene and the solve.
    floor = _l_room_plane(rng, 0.0, 8000)
    ceiling = _l_room_plane(rng, 2.5, 8000)
    boxes = [
        _l_room_box(rng, np.array([2.0, 1.5, 0.4]), np.array([0.3, 0.3, 0.4]), 2000),
        _l_room_box(rng, np.array([6.0, 1.5, 0.6]), np.array([0.4, 0.2, 0.6]), 2000),
        _l_room_box(rng, np.array([2.5, 4.5, 0.5]), np.array([0.3, 0.4, 0.5]), 2000),
    ]
    return np.concatenate([*walls, floor, ceiling, *boxes], axis=0)


def test_relocalize_parity_with_pre_refactor_baseline(monkeypatch: pytest.MonkeyPatch) -> None:
    """Invariant: the refactored public relocalize() (generate_ransac_candidates ->
    refine_candidates) reproduces the pre-refactor baseline EXACTLY on the same
    deterministic scene + seed. The one real-RANSAC test -- it guards the judge."""
    monkeypatch.setattr(relocalize_mod, "RANSAC_ITERS", RANSAC_ITERS_FOR_TEST)
    rng = np.random.default_rng(SEED)
    o3d.utility.random.seed(SEED)

    room_pts = _build_l_room(rng)
    mask = (room_pts[:, 0] <= 5.3) & (room_pts[:, 1] <= 3.3)
    local_src = room_pts[mask].copy() + rng.normal(0.0, 0.003, room_pts[mask].shape)
    T_true = _rigid(25.0, (1.5, -0.8, 0.05))
    local_pts = _apply(np.linalg.inv(T_true), local_src)

    T, fitness = relocalize(_pcd(room_pts), _pcd(local_pts))

    assert abs(fitness - _BASELINE_FITNESS) < 1e-6
    np.testing.assert_allclose(T, _BASELINE_T, atol=1e-6)


# --- FiducialPrior: composition (invariant 6) + consume-on-use (invariant 2) ---


def test_fiducial_composes_map_T_world_then_consumes_it_once() -> None:
    """Invariant 6: ONE aggregated world_T_marker composes to map_T_world =
    map_T_marker @ inv(world_T_marker), exactly (non-identity both sides, so a
    transposed/swapped compose cannot pass). Invariant 2 (consume-on-use): that fix
    is proposed exactly once, then the pending pool is empty -- re-offering the same
    measurement against a drifted world could only score worse."""
    empty = o3d.geometry.PointCloud()
    map_T_marker = _rigid(30.0, (5.0, -2.0, 0.5))
    world_T_marker = _rigid(-40.0, (1.0, 0.5, 0.8))
    truth = map_T_marker @ np.linalg.inv(world_T_marker)

    prior = FiducialPrior({7: map_T_marker})
    assert prior.observe(7, world_T_marker) is None

    candidates = prior.propose(empty, empty)
    assert len(candidates) == 1 and candidates[0].source == "fiducial"
    np.testing.assert_allclose(candidates[0].T, truth, atol=1e-12)
    assert prior.propose(empty, empty) == []  # consumed: gone on the next fire


# --- The shared judge (refine_candidates): gravity gate + wall evidence --------


def _tilt_x(base: np.ndarray, deg: float) -> np.ndarray:
    """`base` (gravity-upright) tilted `deg` about body-x, so its z-axis sits exactly
    `deg` off world-up: _gravity_tilt_deg(_tilt_x(_rigid(...), d)) == d."""
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("x", deg, degrees=True).as_matrix()
    return base @ t


def test_gravity_gate_rejects_a_tilted_candidate() -> None:
    """Invariant 8: the gravity gate is live. A 15-deg-tilted near-truth candidate is
    filtered at the 10-deg default (an upright wrong-room decoy wins by walkover), but
    survives -- and wins on wall fitness -- once the gate opens to 20 deg. The winner
    flips on the tilt threshold alone, so the gate is what rejected it."""
    gm, lm, T_true = _rect_room_scene(seed=42, yaw_deg=-25.0, t=(0.8, 1.2, 0.0))
    candidates = [_tilt_x(T_true, 15.0), _rigid(120.0, (6.0, -5.0, 0.0))]

    assert relocalize_mod._gravity_tilt_deg(candidates[0]) > relocalize_mod.GRAVITY_TILT_MAX_DEG
    _, _, idx_default = refine_candidates(gm, lm, candidates)
    assert idx_default == 1  # tilted near-truth filtered out

    T, fitness, idx_open = refine_candidates(gm, lm, candidates, gravity_tilt_max_deg=20.0)
    assert idx_open == 0 and fitness > 0.3
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=1.0)


def test_min_wall_evidence_raises_below_the_floor(monkeypatch: pytest.MonkeyPatch) -> None:
    """Invariant 9: too few wall points refuses LOUDLY rather than scoring on floors
    (rotation-blind). At the production floor a walled room solves; raise the floor
    above the scene's wall count and refine_candidates raises instead of a silent
    full-cloud fallback."""
    gm, lm, T_true = _rect_room_scene(seed=51, yaw_deg=15.0, t=(1.0, 0.5, 0.0))
    _, fitness, idx = refine_candidates(gm, lm, [T_true.copy()])
    assert idx == 0 and fitness > 0.5

    monkeypatch.setattr(relocalize_mod, "MIN_WALL_POINTS", 10_000_000)
    with pytest.raises(InsufficientWallEvidenceError, match="insufficient wall evidence"):
        refine_candidates(gm, lm, [T_true.copy()])


# --- RelocalizationModule: bare shell (no coordinator, no start() wiring) -------


def _bare_module(config: Config) -> RelocalizationModule:
    """A RelocalizationModule with only the attributes the pure helpers read -- no
    coordinator, no start() wiring. _premap/_fiducial_prior default unset; a test
    overrides whichever it needs. Mirrors __init__'s derived state."""
    m = object.__new__(RelocalizationModule)
    m.config = config
    m._last_skip_log = 0.0
    ransac_entry = next(
        (p for p in config.priors if isinstance(p, RansacPriorConfig)), RansacPriorConfig()
    )
    m._ransac_prior = RansacPrior(interval_s=ransac_entry.interval_s)
    m._accept_threshold = {p.type: p.fitness_threshold for p in config.priors}
    m._ransac_min_local_points = ransac_entry.min_local_points
    m._premap = None
    m._fiducial_prior = None
    m._now_fn = lambda: 100.0  # driven clock; a trigger test swaps it
    m._world_to_map = Subject()
    m._last_fix_map_T_world = None  # no previous fix -> jump guard in acquisition
    m._last_fix_ts_s = 0.0
    m._last_local_map = None  # no cloud yet -> a burst cannot fire until one arrives
    return m


class _StubCloud:
    """Minimal PointCloud2 stand-in: __len__ (the point count the RANSAC floor and
    the n_pts log field read) plus a `.pointcloud` sentinel passed to the solve."""

    def __init__(self, n: int) -> None:
        self._n = n
        self.pointcloud = object()

    def __len__(self) -> int:
        return self._n


class _ModuleLogRecorder:
    """Captures module.py logger .info/.warning as (event, kwargs). The real logger
    sets propagate=False, so caplog can't see these; monkeypatching module_mod.logger
    is the deterministic way. An unexpected .exception (crash path) fails loudly."""

    def __init__(self) -> None:
        self.infos: list[tuple[str, dict[str, object]]] = []
        self.warnings: list[tuple[str, dict[str, object]]] = []

    def info(self, event: str, *args: object, **kwargs: object) -> None:
        self.infos.append((event, kwargs))

    def warning(self, event: str, *args: object, **kwargs: object) -> None:
        self.warnings.append((event, kwargs))

    def exception(self, msg: str, *args: object, **kwargs: object) -> None:
        raise AssertionError(f"unexpected logger.exception: {msg}")


@pytest.mark.parametrize(
    "winning_source, fitness, accepted",
    [
        pytest.param("fiducial", 0.40, True, id="fiducial_clears_its_own_0.35_bar"),
        pytest.param("ransac", 0.50, False, id="ransac_fails_its_own_0.55_bar"),
    ],
)
def test_accept_gate_is_per_prior(
    monkeypatch: pytest.MonkeyPatch, winning_source: str, fitness: float, accepted: bool
) -> None:
    """Invariant 3: the accept gate is PER-SOURCE. A fiducial fix at 0.40 clears a
    fiducial bar of 0.35, while a ransac fix at 0.50 fails a ransac bar of 0.55 -- the
    SAME two fitnesses would flip under one shared module gate. The reject line names
    the ransac bar that refused it."""
    m = _bare_module(
        Config(
            priors=[
                RansacPriorConfig(fitness_threshold=0.55),
                FiducialPriorConfig(fitness_threshold=0.35),
            ]
        )
    )
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._fiducial_prior = FiducialPrior({7: np.eye(4)})  # enables the fiducial entry
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(
        module_mod,
        "_relocalize_with_priors",
        lambda *a, **k: (_rigid(12.0, (0.4, -0.3, 0.02)), fitness, winning_source),
    )

    tf = m._try_relocalize(cast("PointCloud2", _StubCloud(1234)), m._enabled_prior_objects())

    if accepted:
        assert tf is not None
        assert rec.infos[0] == ("relocalize accepted", rec.infos[0][1])
        assert rec.infos[0][1]["source"] == winning_source and not rec.warnings
    else:
        assert tf is None
        assert rec.warnings[0] == (
            "relocalize rejected",
            {"source": "ransac", "fitness": 0.5, "threshold": 0.55},
        )


def _tracking_module(monkeypatch: pytest.MonkeyPatch, now: list[float]) -> RelocalizationModule:
    """A module that has already ACCEPTED one fix at the identity, so the next call is
    a tracking fix with a previous one to compare against. The clock is driven, never
    slept, so the per-second jump budgets are exact."""
    m = _bare_module(Config(priors=[RansacPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: now[0]
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())
    monkeypatch.setattr(
        module_mod, "_relocalize", lambda *a, **k: (_rigid(0.0, (0.0, 0.0, 0.0)), 0.9)
    )
    assert m._try_relocalize(cast("PointCloud2", _StubCloud(1234)), m._enabled_prior_objects())
    return m


@pytest.mark.parametrize(
    "seed_fix, dt_s, T, accepted",
    [
        pytest.param(True, 0.2, _rigid(180.0, (18.0, 0.0, 0.0)), False, id="gross_flip_rejected"),
        pytest.param(True, 2.0, _rigid(3.0, (0.4, 0.0, 0.0)), True, id="normal_drift_passes"),
        pytest.param(False, 0.0, _rigid(170.0, (40.0, -12.0, 0.0)), True, id="first_fix_exempt"),
    ],
)
def test_jump_guard(
    monkeypatch: pytest.MonkeyPatch, seed_fix: bool, dt_s: float, T: np.ndarray, accepted: bool
) -> None:
    """Invariant 1: the TRACKING jump guard rejects a gross mis-localization (18 m /
    180 deg in 0.2 s, a fitness that CLEARS the accept gate), passes a normal
    sub-metre drift correction, and never blocks the FIRST fix (acquisition is
    unguarded -- a robot booting far from the origin must still localize)."""
    now = [100.0]
    if seed_fix:
        m = _tracking_module(monkeypatch, now)
        now[0] = 100.0 + dt_s
    else:
        m = _bare_module(Config(priors=[RansacPriorConfig()]))
        m._premap = _StubCloud(10)  # type: ignore[assignment]
        m._now_fn = lambda: now[0]
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(module_mod, "_relocalize", lambda *a, **k: (T, 0.9))

    tf = m._try_relocalize(cast("PointCloud2", _StubCloud(1234)), m._enabled_prior_objects())

    if accepted:
        assert tf is not None and rec.infos[0][0] == "relocalize accepted" and not rec.warnings
    else:
        assert tf is None and rec.warnings[0][0] == "relocalize jump rejected"


# --- Per-prior triggers: the module fires one INDEPENDENT reloc per due prior ---


def _identity_burst(marker_id: int) -> Detection3DArray:
    """Wire form of ONE completed burst: a single-entry aggregated_detections array
    whose bbox.center is an identity world_T_marker."""
    det = Detection3D()
    det.id = str(marker_id)
    det.bbox.center.orientation.w = 1.0  # identity quat; position defaults to 0
    arr = Detection3DArray()
    arr.detections = [det]
    arr.detections_length = 1
    return arr


def test_completed_burst_fires_against_the_cached_cloud_no_new_frame(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Invariant 5: a tag burst relocalizes from the detections callback itself,
    judged against the LAST global_map received -- no new cloud arrives. Waiting for
    one would be dead time on acquisition; global_map accumulates in the world frame,
    so the cached cloud scores wall fitness as well as the next would."""
    m = _bare_module(Config(priors=[FiducialPriorConfig()]))
    m._fiducial_prior = FiducialPrior({7: np.eye(4)})
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())
    judged: list[object] = []

    def _fake(
        global_map: object, local_map: object, priors: list[RelocPrior], **k: object
    ) -> tuple[np.ndarray, float, str]:
        for p in priors:  # drain the pending fix as the real relocalize_with_priors does
            p.propose(o3d.geometry.PointCloud(), o3d.geometry.PointCloud())
        judged.append(local_map)
        return np.eye(4), 0.9, "fiducial"

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)

    cached = _StubCloud(60_000)
    m._on_local_map(cast("PointCloud2", cached))
    assert judged == []  # a cloud with no tag pending fires nothing

    m._on_aggregated_detections(_identity_burst(7))
    assert judged == [cached.pointcloud]  # fired on the cached cloud, no new frame
    assert m._fiducial_prior.is_due(100.0) is False  # that fire consumed the estimate


def test_ransac_starved_below_floor_keeps_trigger_then_fires_when_dense(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Invariant 4: min_local_points is the FPFH+RANSAC geometry floor. A sparse
    submap starves the cold (due) RANSAC search -- it logs the skip and is NOT acked,
    so the trigger stands -- and fires at the first frame that clears the floor."""
    m = _bare_module(Config(priors=[RansacPriorConfig(interval_s=2.0, min_local_points=50_000)]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    fires: list[str] = []

    def _fake(*a: object, **k: object) -> tuple[np.ndarray, float]:
        fires.append("ransac")
        return np.eye(4), 0.9

    monkeypatch.setattr(module_mod, "_relocalize", _fake)

    m._on_local_map(cast("PointCloud2", _StubCloud(10_000)))  # starved: skip, don't ack
    assert fires == []
    assert rec.warnings[0][0] == "ransac reloc skipped" and rec.warnings[0][1]["n_pts"] == 10_000

    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))  # dense: the un-acked trigger fires
    assert fires == ["ransac"]
