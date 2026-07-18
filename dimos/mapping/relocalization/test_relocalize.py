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

"""Tests for relocalize.py's refactored candidate-generation/judge split and
priors.py's pluggable-prior system built on top of it.

All scenes are SIMULATED point clouds (numpy-seeded, deterministic) -- no
hardware, no replay recording. Only test_relocalize_parity_with_pre_refactor_
baseline runs real RANSAC (with RANSAC_ITERS monkeypatched down purely for
test speed, same knob and value used to capture the baseline fixture itself
-- see the module docstring below for how it was captured); every other test
injects candidates directly and never calls generate_ransac_candidates, per
the "keep the suite fast" rule for this file.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.mapping.relocalization.priors import (
    Candidate,
    LastPosePrior,
    RelocPrior,
    relocalize_with_priors,
)
import dimos.mapping.relocalization.relocalize as relocalize_mod
from dimos.mapping.relocalization.relocalize import (
    InsufficientWallEvidence,
    refine_candidates,
    relocalize,
)


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
    return (T[:3, :3] @ points.T).T + T[:3, 3]


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


def _build_rect_room(
    rng: np.random.Generator, x_extent: float = 6.0, y_extent: float = 4.0, height: float = 2.5
) -> np.ndarray:
    """A simple rectangular room (walls + floor + ceiling), no L-shape/obstacles
    -- small and cheap, for the tests that inject candidates directly and only
    exercise refine_candidates' wall-rerank + ICP stages, never RANSAC."""
    walls = [
        _sample_wall(rng, "y", 0.0, 0.0, x_extent, height, 700),
        _sample_wall(rng, "y", y_extent, 0.0, x_extent, height, 700),
        _sample_wall(rng, "x", 0.0, 0.0, y_extent, height, 700),
        _sample_wall(rng, "x", x_extent, 0.0, y_extent, height, 700),
    ]
    floor = _sample_plane(rng, x_extent, y_extent, 0.0, 2000)
    ceiling = _sample_plane(rng, x_extent, y_extent, height, 2000)
    return np.concatenate([*walls, floor, ceiling], axis=0)


def _rect_room_scene(
    seed: int, yaw_deg: float, t: tuple[float, float, float]
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, np.ndarray]:
    """global_map (full room, `map` frame) + local_map (a partial sub-region,
    `world` frame, related to the map frame by the returned T_true -- same
    convention as relocalize()'s return: local_map_points -> T_true ->
    global_map frame)."""
    rng = np.random.default_rng(seed)
    room_pts = _build_rect_room(rng)
    mask = (room_pts[:, 0] <= 3.2) & (room_pts[:, 1] <= 2.2)
    local_src = room_pts[mask].copy()
    T_true = _rigid(yaw_deg, t)
    local_pts = _apply(np.linalg.inv(T_true), local_src)
    return _pcd(room_pts), _pcd(local_pts), T_true


def test_judge_integrity_wall_fitness_picks_near_truth_over_decoys() -> None:
    """Injected near-truth candidate + several wrong-room/wrong-yaw decoys ->
    the near-truth one wins the wall-only fine-fitness rerank and the final
    polished T lands close to ground truth."""
    global_map, local_map, T_true = _rect_room_scene(seed=1, yaw_deg=20.0, t=(1.0, 0.5, 0.02))

    near_truth = T_true.copy()
    near_truth[:3, 3] += np.array([0.03, -0.02, 0.0])  # 3cm off, still near
    decoys = [
        _rigid(110.0, (4.0, 3.5, 0.0)),
        _rigid(250.0, (-3.0, 2.0, 0.0)),
        _rigid(70.0, (5.5, -2.5, 0.0)),
    ]
    candidates = [near_truth, *decoys]

    T, fitness, winning_index = refine_candidates(global_map, local_map, candidates)

    assert winning_index == 0
    assert fitness > 0.5
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=0.15)
    np.testing.assert_allclose(T[:3, :3], T_true[:3, :3], atol=0.05)


def test_floors_only_scene_raises_insufficient_wall_evidence() -> None:
    """A floors-only scene (floor + ceiling, zero walls -> zero
    horizontal-normal points) must refuse loudly. Regression for the old
    silent fallback: _wall_subset used to hand back the FULL cloud below 100
    wall points, so the "wall-only" rerank scored on floors -- rotation-blind,
    a 180-deg flip fits as well as the truth -- and returned a confident
    fitness anyway. Now refine_candidates raises InsufficientWallEvidence
    (module.py catches -> logs + skips the frame) and never returns."""
    rng = np.random.default_rng(21)
    floor = _sample_plane(rng, 6.0, 4.0, 0.0, 2000)
    ceiling = _sample_plane(rng, 6.0, 4.0, 2.5, 2000)
    room_pts = np.concatenate([floor, ceiling], axis=0)
    mask = (room_pts[:, 0] <= 3.2) & (room_pts[:, 1] <= 2.2)
    T_true = _rigid(30.0, (1.0, 0.5, 0.0))
    local_pts = _apply(np.linalg.inv(T_true), room_pts[mask].copy())

    try:
        _T, fitness, _idx = refine_candidates(_pcd(room_pts), _pcd(local_pts), [T_true.copy()])
    except InsufficientWallEvidence as e:
        msg = str(e)
        assert "insufficient wall evidence" in msg
        assert "submap walls=" in msg and "map walls=" in msg
    else:
        raise AssertionError(
            f"expected InsufficientWallEvidence, got fitness={fitness:.3f} -- "
            "the silent full-cloud fallback is back"
        )


def test_priors_plumbing_stub_candidate_wins_with_right_source() -> None:
    """A stub prior proposing a known-good candidate (RANSAC not in the priors
    list at all -- "disabled" is simply omitting it) -> relocalize_with_priors
    returns it, tagged with the stub's own source name."""
    global_map, local_map, T_true = _rect_room_scene(seed=2, yaw_deg=-15.0, t=(0.6, 1.1, -0.01))

    class _StubPrior:
        name = "stub_good"

        def propose(
            self, global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
        ) -> list[Candidate]:
            return [Candidate(T=T_true.copy(), source=self.name, confidence=0.9)]

    priors: list[RelocPrior] = [_StubPrior()]
    T, fitness, winning_source = relocalize_with_priors(global_map, local_map, priors)

    assert winning_source == "stub_good"
    assert fitness > 0.5
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=0.15)


def test_last_pose_prior_unset_proposes_nothing() -> None:
    prior = LastPosePrior()
    global_map, local_map, _ = _rect_room_scene(seed=3, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    assert prior.propose(global_map, local_map) == []


def test_last_pose_prior_set_enters_pool() -> None:
    prior = LastPosePrior()
    global_map, local_map, T_true = _rect_room_scene(seed=4, yaw_deg=5.0, t=(0.2, 0.3, 0.0))
    prior.update(T_true)

    candidates = prior.propose(global_map, local_map)

    assert len(candidates) == 1
    assert candidates[0].source == "last_pose"
    assert candidates[0].confidence == 0.3
    np.testing.assert_array_equal(candidates[0].T, T_true)


def test_no_bypass_wrong_high_confidence_candidate_loses_to_correct_low_confidence() -> None:
    """A prior reporting high confidence for a candidate 5m off, alongside a
    prior reporting LOW confidence for the actually-correct candidate -> the
    correct one wins. The judge ranks by wall fitness only; confidence is
    never consulted."""
    global_map, local_map, T_true = _rect_room_scene(seed=5, yaw_deg=35.0, t=(0.9, 0.4, 0.0))
    wrong = T_true.copy()
    wrong[:3, 3] += np.array([5.0, 0.0, 0.0])

    class _OverconfidentWrongPrior:
        name = "overconfident_wrong"

        def propose(
            self, global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
        ) -> list[Candidate]:
            return [Candidate(T=wrong, source=self.name, confidence=0.99)]

    class _UnderconfidentCorrectPrior:
        name = "underconfident_correct"

        def propose(
            self, global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
        ) -> list[Candidate]:
            return [Candidate(T=T_true.copy(), source=self.name, confidence=0.01)]

    priors: list[RelocPrior] = [_OverconfidentWrongPrior(), _UnderconfidentCorrectPrior()]
    T, fitness, winning_source = relocalize_with_priors(global_map, local_map, priors)

    assert winning_source == "underconfident_correct"
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=0.15)


SEED = 42
RANSAC_ITERS_FOR_TEST = 20_000  # see docstring below: matches how the baseline was captured

# SIMULATED baseline captured 2026-07-16 against the pre-refactor relocalize()
# on this branch (commit bc846bb69, before this refactor), via a throwaway
# script (not part of this repo) that: built this exact deterministic
# L-shaped-room synthetic scene (numpy `default_rng(42)`, `o3d.utility.random.
# seed(42)`), monkeypatched RANSAC_ITERS from the production 500_000 down to
# 20_000 purely so the capture run finished in ~0.3s (the parity check below
# only needs identical conditions before/after the refactor, not production
# iteration counts), and called relocalize(global_map, local_map). Reran
# twice pre-refactor and once post-refactor: all three runs produced this
# exact T (bit-identical) and fitness -- confirms the determinism claim
# (#2137 program.md: "seeded per-frame, RANSAC variance zero").
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
    inside_bbox = (x >= 0) & (x <= 8) & (y >= 0) & (y <= 6)
    in_notch = (x >= 5) & (x <= 8) & (y >= 3) & (y <= 6)
    return inside_bbox & ~in_notch


def _l_room_plane(rng: np.random.Generator, z: float, n_target: int) -> np.ndarray:
    pts = []
    got = 0
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
            u = rng.uniform(-1.0, 1.0, n_per_face)
            v = rng.uniform(-1.0, 1.0, n_per_face)
            local = np.zeros((n_per_face, 3))
            other = [a for a in range(3) if a != axis]
            local[:, other[0]] = u
            local[:, other[1]] = v
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
    floor = _l_room_plane(rng, 0.0, 8000)
    ceiling = _l_room_plane(rng, 2.5, 8000)
    boxes = [
        _l_room_box(rng, np.array([2.0, 1.5, 0.4]), np.array([0.3, 0.3, 0.4]), 2000),
        _l_room_box(rng, np.array([6.0, 1.5, 0.6]), np.array([0.4, 0.2, 0.6]), 2000),
        _l_room_box(rng, np.array([2.5, 4.5, 0.5]), np.array([0.3, 0.4, 0.5]), 2000),
    ]
    return np.concatenate([*walls, floor, ceiling, *boxes], axis=0)


def test_relocalize_parity_with_pre_refactor_baseline(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The refactored public relocalize() (generate_ransac_candidates ->
    refine_candidates) reproduces the pre-refactor baseline exactly on the
    same deterministic synthetic scene + seed. The only test in this file
    that runs real RANSAC."""
    monkeypatch.setattr(relocalize_mod, "RANSAC_ITERS", RANSAC_ITERS_FOR_TEST)

    rng = np.random.default_rng(SEED)
    o3d.utility.random.seed(SEED)

    room_pts = _build_l_room(rng)
    mask = (room_pts[:, 0] <= 5.3) & (room_pts[:, 1] <= 3.3)
    local_src = room_pts[mask].copy()
    local_src += rng.normal(0.0, 0.003, local_src.shape)

    T_true = _rigid(25.0, (1.5, -0.8, 0.05))
    local_pts = _apply(np.linalg.inv(T_true), local_src)

    global_map = _pcd(room_pts)
    local_map = _pcd(local_pts)

    T, fitness = relocalize(global_map, local_map)

    assert abs(fitness - _BASELINE_FITNESS) < 1e-6
    np.testing.assert_allclose(T, _BASELINE_T, atol=1e-6)


# ---------------------------------------------------------------------------
# FiducialPrior (Phase 2): age gating/decay + judge integrity
# ---------------------------------------------------------------------------


def test_fiducial_prior_unset_proposes_nothing() -> None:
    from dimos.mapping.relocalization.priors import FiducialPrior

    gm, lm, _ = _rect_room_scene(seed=11, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    assert FiducialPrior().propose(gm, lm) == []


def test_fiducial_prior_fresh_fix_decays_with_age() -> None:
    from dimos.mapping.relocalization.priors import FiducialPrior

    gm, lm, T_true = _rect_room_scene(seed=12, yaw_deg=30.0, t=(1.0, -2.0, 0.0))
    clock = {"now": 100.0}
    p = FiducialPrior(age_tau_s=30.0, age_max_s=120.0, conf_max=0.9, now_fn=lambda: clock["now"])
    p.update(T_true, ts=100.0)

    fresh = p.propose(gm, lm)
    assert len(fresh) == 1 and fresh[0].source == "fiducial"
    assert np.isclose(fresh[0].confidence, 0.9)
    assert fresh[0].T is T_true

    clock["now"] = 130.0  # age 30 s = one tau
    aged = p.propose(gm, lm)
    assert np.isclose(aged[0].confidence, 0.9 * np.exp(-1.0))

    clock["now"] = 221.0  # age 121 s > cutoff
    assert p.propose(gm, lm) == []


def test_fiducial_prior_never_bypasses_judge() -> None:
    """A wrong marker fix (stale map, moved tag, bad PnP) at maximum
    confidence must lose to a correct low-confidence candidate — the fiducial
    tier gets no special treatment from the judge."""
    from dimos.mapping.relocalization.priors import FiducialPrior, relocalize_with_priors

    gm, lm, T_true = _rect_room_scene(seed=13, yaw_deg=45.0, t=(2.0, 1.0, 0.0))
    T_wrong = T_true.copy()
    T_wrong[:3, 3] += np.array([5.0, -4.0, 0.0])  # confidently 6.4 m off

    fid = FiducialPrior(conf_max=0.99, now_fn=lambda: 0.0)
    fid.update(T_wrong, ts=0.0)

    class NearTruthStub:
        name = "stub_near_truth"

        def propose(self, g: object, l: object) -> list:
            from dimos.mapping.relocalization.priors import Candidate

            T_near = T_true.copy()
            T_near[:3, 3] += np.array([0.03, -0.02, 0.0])
            return [Candidate(T=T_near, source=self.name, confidence=0.01)]

    T, fitness, source = relocalize_with_priors(gm, lm, [fid, NearTruthStub()])
    assert source == "stub_near_truth"
    assert np.linalg.norm(T[:3, 3] - T_true[:3, 3]) < 0.15


def test_gravity_gate_is_per_source_no_walkover() -> None:
    """The gravity-gate walkover (benchmark-found, repro hk_village1 frame
    831): with a pool-global gate, one upright stale seed un-empties
    `upright` and silently discards another source's ALL-TILTED candidates —
    the seed wins unopposed. Per-source gating (sources=) must keep each
    source's own fallback: the tilted-but-near-truth RANSAC candidate beats
    the upright-but-6m-stale seed on wall fitness, as it always did before
    the seed existed."""
    from scipy.spatial.transform import Rotation as _R

    gm, lm, T_true = _rect_room_scene(seed=31, yaw_deg=20.0, t=(1.0, -1.5, 0.0))

    T_tilted = T_true.copy()  # near-truth but 15 deg off-gravity (> 10 deg gate)
    tilt = np.eye(4)
    tilt[:3, :3] = _R.from_euler("x", 15.0, degrees=True).as_matrix()
    T_tilted = T_tilted @ tilt

    T_stale = T_true.copy()  # perfectly upright, 7.8 m from the truth
    T_stale[:3, 3] += np.array([6.0, -5.0, 0.0])

    pool = [T_tilted, T_stale]
    sources = ["ransac", "last_pose"]

    # Old global gate: the upright stale seed orphans the tilted candidate
    # and wins by walkover — the bug, reproduced.
    _, _, walkover_idx = refine_candidates(gm, lm, pool)
    assert walkover_idx == 1, "expected the walkover under the pool-global gate"

    # Per-source gate: the tilted near-truth candidate stays in and wins.
    T, fitness, idx = refine_candidates(gm, lm, pool, sources=sources)
    assert idx == 0, "per-source gating must keep the all-tilted source's pool"
    assert np.linalg.norm(T[:3, 3] - T_true[:3, 3]) < 1.0

    # And the full priors entry point attributes the win to ransac.
    class TiltedRansacStub:
        name = "ransac"

        def propose(self, g: object, l: object) -> list:
            from dimos.mapping.relocalization.priors import Candidate

            return [Candidate(T=T_tilted, source=self.name, confidence=0.5)]

    lp = LastPosePrior()
    lp.update(T_stale)
    from dimos.mapping.relocalization.priors import relocalize_with_priors as rwp

    _, _, source = rwp(gm, lm, [TiltedRansacStub(), lp])
    assert source == "ransac"
