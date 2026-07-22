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

import json
from pathlib import Path
from typing import cast

import cv2
from dimos_lcm.vision_msgs.ObjectHypothesisWithPose import ObjectHypothesisWithPose
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from pydantic import ValidationError
import pytest
from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprints import config_key
from dimos.core.coordination.worker_manager_python import _merge_config_kwargs
import dimos.mapping.relocalization.module as module_mod
from dimos.mapping.relocalization.module import Config, RelocalizationModule
import dimos.mapping.relocalization.priors as priors_mod
from dimos.mapping.relocalization.priors import (
    Candidate,
    FiducialPrior,
    FiducialPriorConfig,
    LastPosePrior,
    LastPosePriorConfig,
    RansacPrior,
    RansacPriorConfig,
    RelocPrior,
    relocalize_with_priors,
)
import dimos.mapping.relocalization.relocalize as relocalize_mod
from dimos.mapping.relocalization.relocalize import (
    InsufficientWallEvidenceError,
    refine_candidates,
    relocalize,
)
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.apriltag_aggregation import AggregationConfig, matrix_from_pose7
from dimos.robot.cli.dimos import load_config_args


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
    fitness anyway. Now refine_candidates raises InsufficientWallEvidenceError
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
    except InsufficientWallEvidenceError as e:
        msg = str(e)
        assert "insufficient wall evidence" in msg
        assert "submap walls=" in msg and "map walls=" in msg
    else:
        raise AssertionError(
            f"expected InsufficientWallEvidenceError, got fitness={fitness:.3f} -- "
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
            return [Candidate(T=T_true.copy(), source=self.name)]

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
            return [Candidate(T=wrong, source=self.name)]

    class _UnderconfidentCorrectPrior:
        name = "underconfident_correct"

        def propose(
            self, global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
        ) -> list[Candidate]:
            return [Candidate(T=T_true.copy(), source=self.name)]

    priors: list[RelocPrior] = [_OverconfidentWrongPrior(), _UnderconfidentCorrectPrior()]
    T, fitness, winning_source = relocalize_with_priors(global_map, local_map, priors)

    assert winning_source == "underconfident_correct"
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=0.15)


SEED = 42
RANSAC_ITERS_FOR_TEST = 20_000  # see docstring below: matches how the baseline was captured

# SIMULATED baseline captured 2026-07-16 against the pre-refactor relocalize()
# on this branch (the parent of the split-judge commit), via a throwaway
# script (not part of this repo) that: built this exact deterministic
# L-shaped-room synthetic scene (numpy `default_rng(42)`, `o3d.utility.random.
# seed(42)`), monkeypatched RANSAC_ITERS from the production 500_000 down to
# 20_000 purely so the capture run finished in ~0.3s (the parity check below
# only needs identical conditions before/after the refactor, not production
# iteration counts), and called relocalize(global_map, local_map). Reran
# twice pre-refactor and once post-refactor: all three runs produced this
# exact T (bit-identical) and fitness -- confirming the determinism #2137's
# program.md describes (per-frame seeding, zero RANSAC variance).
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
# FiducialPrior: multi-sighting Huber fusion + ambiguity gate + judge integrity
# ---------------------------------------------------------------------------


def _rigid_noise(rng: np.random.Generator, deg_std: float, m_std: float) -> np.ndarray:
    """A small random rigid perturbation: rotation ~deg_std, translation ~m_std."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rng.normal(0.0, np.radians(deg_std), 3)).as_matrix()
    T[:3, 3] = rng.normal(0.0, m_std, 3)
    return T


def _frontal_far_corners(k: np.ndarray, marker_length_m: float) -> np.ndarray:
    """Corners of a small tag viewed head-on at 2 m (weak perspective) with a bit
    of pixel noise -- the mirror-ambiguous regime the gate must reject."""
    h = marker_length_m / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
    pts, _ = cv2.projectPoints(obj, np.zeros(3), np.array([0.0, 0.0, 2.0]), k, np.zeros((0, 1)))
    return pts.reshape(4, 2) + np.random.default_rng(0).normal(0.0, 0.3, (4, 2))


def test_fiducial_prior_unset_proposes_nothing() -> None:
    gm, lm, _ = _rect_room_scene(seed=11, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    assert FiducialPrior({}).propose(gm, lm) == []


def test_fiducial_prior_fuses_sightings_into_one_robust_candidate() -> None:
    """Eight noisy sightings of one mapped tag PLUS a gross mirror-flip outlier
    fuse into ONE candidate whose map_T_world lands near the truth. The lever arm
    (map_T_marker 5.4 m from origin) turns the 150 deg outlier into a ~14 m
    single-frame fix, so a <0.25 m fused error is the Huber-medoid rejecting it."""
    gm, lm, _ = _rect_room_scene(seed=14, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    rng = np.random.default_rng(14)
    map_T_marker = _rigid(30.0, (5.0, -2.0, 0.5))
    world_T_marker_true = _rigid(-40.0, (1.0, 0.5, 0.8))
    truth = map_T_marker @ np.linalg.inv(world_T_marker_true)

    prior = FiducialPrior({7: map_T_marker}, now_fn=lambda: 0.0)
    for i in range(8):
        prior.observe(7, world_T_marker_true @ _rigid_noise(rng, 2.0, 0.01), ts=float(i) * 0.1)
    prior.observe(7, world_T_marker_true @ _rigid(150.0, (0.0, 0.0, 0.0)), ts=0.85)  # outlier

    candidates = prior.propose(gm, lm)
    assert len(candidates) == 1 and candidates[0].source == "fiducial"
    assert np.linalg.norm(candidates[0].T[:3, 3] - truth[:3, 3]) < 0.25


def test_fiducial_prior_drops_mirror_ambiguous_sighting() -> None:
    """A head-on small tag whose flipped IPPE pose reprojects nearly as well is
    dropped by the ambiguity gate -- no fix is stored."""
    gm, lm, _ = _rect_room_scene(seed=15, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    k = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
    info = CameraInfo.from_intrinsics(600.0, 600.0, 320.0, 240.0, 640, 480)
    prior = FiducialPrior(
        {7: np.eye(4)}, camera_info=info, marker_length_m=0.1, ambiguity_ratio_min=2.0
    )
    reason = prior.observe(7, np.eye(4), ts=0.0, corners_px=_frontal_far_corners(k, 0.1))
    assert reason == "mirror_ambiguous"
    assert prior.propose(gm, lm) == []


def test_fiducial_prior_age_gate_hard_cutoff() -> None:
    """A fused fix proposes right up to ``age_max_s`` and drops past it. Hard
    cutoff only -- no decay curve, no confidence."""
    gm, lm, _ = _rect_room_scene(seed=12, yaw_deg=30.0, t=(1.0, -2.0, 0.0))
    clock = {"now": 100.0}
    prior = FiducialPrior({7: np.eye(4)}, age_max_s=120.0, now_fn=lambda: clock["now"])
    for i in range(3):  # >= min_observations -> a fused fix is stamped at now=100
        prior.observe(7, np.eye(4), ts=float(i) * 0.1)

    fresh = prior.propose(gm, lm)
    assert len(fresh) == 1 and fresh[0].source == "fiducial"

    clock["now"] = 219.0  # age 119 s < cutoff -> still proposes
    assert len(prior.propose(gm, lm)) == 1

    clock["now"] = 221.0  # age 121 s > cutoff -> dropped
    assert prior.propose(gm, lm) == []


def test_fiducial_prior_never_bypasses_judge() -> None:
    """A consistent-but-wrong fused fix (stale map, moved tag) 6.4 m off must
    lose to a correct candidate — the fiducial source gets no special treatment;
    the judge ranks on wall fitness only."""
    gm, lm, T_true = _rect_room_scene(seed=13, yaw_deg=45.0, t=(2.0, 1.0, 0.0))
    T_wrong = T_true.copy()
    T_wrong[:3, 3] += np.array([5.0, -4.0, 0.0])  # 6.4 m off
    # world_T_marker chosen so the fused map_T_world (map_T_marker == I) is T_wrong.
    world_T_marker = np.linalg.inv(T_wrong)

    fid = FiducialPrior({7: np.eye(4)}, now_fn=lambda: 0.0)
    for i in range(4):
        fid.observe(7, world_T_marker, ts=float(i) * 0.1)

    class NearTruthStub:
        name = "stub_near_truth"

        def propose(
            self, global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
        ) -> list[Candidate]:
            T_near = T_true.copy()
            T_near[:3, 3] += np.array([0.03, -0.02, 0.0])
            return [Candidate(T=T_near, source=self.name)]

    T, fitness, source = relocalize_with_priors(gm, lm, [fid, NearTruthStub()])
    assert source == "stub_near_truth"
    assert np.linalg.norm(T[:3, 3] - T_true[:3, 3]) < 0.15


def test_gravity_gate_is_per_source_no_walkover() -> None:
    """The gravity-gate walkover (found in offline replay of hk_village1,
    repro frame 831): with a pool-global gate, one upright stale seed un-empties
    `upright` and silently discards another source's ALL-TILTED candidates —
    the seed wins unopposed. Per-source gating (sources=) must keep each
    source's own fallback: the tilted-but-near-truth RANSAC candidate beats
    the upright-but-6m-stale seed on wall fitness, as it always did before
    the seed existed."""
    gm, lm, T_true = _rect_room_scene(seed=31, yaw_deg=20.0, t=(1.0, -1.5, 0.0))

    T_tilted = T_true.copy()  # near-truth but 15 deg off-gravity (> 10 deg gate)
    tilt = np.eye(4)
    tilt[:3, :3] = Rotation.from_euler("x", 15.0, degrees=True).as_matrix()
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

        def propose(
            self, global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
        ) -> list[Candidate]:
            return [Candidate(T=T_tilted, source=self.name)]

    lp = LastPosePrior()
    lp.update(T_stale)
    _, _, source = relocalize_with_priors(gm, lm, [TiltedRansacStub(), lp])
    assert source == "ransac"


# ---------------------------------------------------------------------------
# RansacPrior: thin wrapper over generate_ransac_candidates
# ---------------------------------------------------------------------------


def test_ransac_prior_wraps_generate_and_tags_source(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """RansacPrior.propose is a pure adapter: it returns exactly the transforms
    generate_ransac_candidates hands back -- same objects, same order -- each
    boxed in a Candidate tagged source='ransac', and nothing else. Stubbed so
    the fast suite never runs real RANSAC (per this file's speed rule)."""
    gm, lm, _ = _rect_room_scene(seed=41, yaw_deg=10.0, t=(0.5, 0.5, 0.0))
    fake = [_rigid(10.0, (0.1, 0.2, 0.0)), _rigid(200.0, (3.0, -1.0, 0.0))]

    seen: dict[str, object] = {}

    def _fake_generate(
        global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
    ) -> list[np.ndarray]:
        seen["global_map"] = global_map
        seen["local_map"] = local_map
        return fake

    monkeypatch.setattr(priors_mod, "generate_ransac_candidates", _fake_generate)

    candidates = RansacPrior().propose(gm, lm)

    # The maps are threaded through untouched, and every transform is boxed 1:1
    # in original order with the ransac tag -- no filtering, no reordering.
    assert seen["global_map"] is gm and seen["local_map"] is lm
    assert [c.source for c in candidates] == ["ransac", "ransac"]
    assert len(candidates) == len(fake)
    for cand, T in zip(candidates, fake, strict=True):
        assert cand.T is T


def test_ransac_prior_empty_pool_stays_empty(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """RANSAC finding nothing (empty pool) yields zero candidates, not an error
    -- an expected, valid response per the RelocPrior contract."""
    gm, lm, _ = _rect_room_scene(seed=42, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    monkeypatch.setattr(priors_mod, "generate_ransac_candidates", lambda g, l: [])
    assert RansacPrior().propose(gm, lm) == []


# ---------------------------------------------------------------------------
# FiducialPrior: unmapped-id rejection + min_observations fusion threshold +
# per-tag propose fan-out
# ---------------------------------------------------------------------------


def test_fiducial_prior_rejects_unmapped_id() -> None:
    """A sighting of a tag absent from the surveyed marker map is refused with
    'unmapped_id' and never touches the aggregator -- there is no map_T_marker
    to compose a fix from."""
    gm, lm, _ = _rect_room_scene(seed=16, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    prior = FiducialPrior({7: np.eye(4)}, now_fn=lambda: 0.0)

    reason = prior.observe(99, np.eye(4), ts=0.0)  # id 99 is not surveyed

    assert reason == "unmapped_id"
    assert prior.propose(gm, lm) == []


def test_fiducial_prior_fix_appears_exactly_at_min_observations() -> None:
    """No fused fix exists until a visit reaches min_observations gated glimpses;
    the fix appears on precisely the min_observations-th sighting. Default
    min_observations is 3, so 2 glimpses propose nothing and the 3rd stamps one."""
    gm, lm, _ = _rect_room_scene(seed=17, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    assert AggregationConfig().min_observations == 3
    prior = FiducialPrior({7: np.eye(4)}, now_fn=lambda: 0.0)

    prior.observe(7, np.eye(4), ts=0.0)
    prior.observe(7, np.eye(4), ts=0.1)
    assert prior.propose(gm, lm) == []  # 2 < min_observations -> no fix yet

    prior.observe(7, np.eye(4), ts=0.2)  # the 3rd glimpse stamps the fix
    fixed = prior.propose(gm, lm)
    assert len(fixed) == 1 and fixed[0].source == "fiducial"


def test_fiducial_prior_proposes_one_candidate_per_mapped_tag() -> None:
    """Two independently-fused tags yield two candidates -- one per tag, each the
    tag's own map_T_marker @ inv(world_T_marker) (identity world poses here, so
    each fix equals that tag's surveyed map_T_marker)."""
    gm, lm, _ = _rect_room_scene(seed=18, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    map_T_marker_7 = _rigid(30.0, (5.0, -2.0, 0.5))
    map_T_marker_9 = _rigid(-70.0, (-1.0, 3.0, 0.2))
    prior = FiducialPrior({7: map_T_marker_7, 9: map_T_marker_9}, now_fn=lambda: 0.0)

    for tag in (7, 9):
        for i in range(3):
            prior.observe(tag, np.eye(4), ts=float(i) * 0.1)

    candidates = prior.propose(gm, lm)

    assert len(candidates) == 2
    assert {c.source for c in candidates} == {"fiducial"}
    got = sorted((round(c.T[0, 3], 6), round(c.T[1, 3], 6)) for c in candidates)
    want = sorted((round(T[0, 3], 6), round(T[1, 3], 6)) for T in (map_T_marker_7, map_T_marker_9))
    assert got == want


def test_fiducial_prior_age_gate_is_per_fix_not_all_or_nothing() -> None:
    """propose() ages each fix independently: with one stale and one fresh tag,
    only the stale one drops -- the fresh tag still proposes its candidate."""
    gm, lm, _ = _rect_room_scene(seed=19, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    clock = {"now": 0.0}
    map_T_marker_9 = _rigid(-70.0, (-1.0, 3.0, 0.2))
    prior = FiducialPrior(
        {7: np.eye(4), 9: map_T_marker_9}, age_max_s=120.0, now_fn=lambda: clock["now"]
    )

    for i in range(3):  # tag 7 fix stamped at now=0
        prior.observe(7, np.eye(4), ts=float(i) * 0.1)
    clock["now"] = 200.0
    for i in range(3):  # tag 9 fix stamped at now=200
        prior.observe(9, np.eye(4), ts=200.0 + float(i) * 0.1)

    clock["now"] = 210.0  # tag 7 age 210 s > cutoff (drop); tag 9 age 10 s (keep)
    survivors = prior.propose(gm, lm)
    assert len(survivors) == 1
    np.testing.assert_array_equal(survivors[0].T, map_T_marker_9)


# ---------------------------------------------------------------------------
# FiducialPrior LIVE corners_px gate chain: a CLEAN corners glimpse survives the
# ambiguity gate + the corners-derived distance/view gates and fuses to a fix,
# and the corners path agrees with the world_T_optical path on one clean geometry
# ---------------------------------------------------------------------------

# SIMULATED pinhole camera shared by the corners-path tests (640x480, zero
# distortion): corners are cv2.projectPoints of a KNOWN camera_optical<-marker
# pose, the same construction the marker_pose ambiguity tests use.
_CORNERS_K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
_CORNERS_INFO = CameraInfo.from_intrinsics(600.0, 600.0, 320.0, 240.0, 640, 480)
_CORNERS_MARKER_LENGTH_M = 0.1


def _oblique_close_optical_T_marker() -> np.ndarray:
    """A marker 0.5 m ahead tilted 35 deg about the camera x-axis: close and
    oblique enough that the IPPE mirror gate keeps it (strong perspective, the
    two solutions reproject far apart), yet inside the aggregator's 1.0 m
    distance and 45 deg view-angle gates. Returns camera_optical<-marker (4x4).
    Measured on this geometry: distance 0.50 m, view-angle 32.9 deg, tag 110 px."""
    optical_T_marker = np.eye(4)
    optical_T_marker[:3, :3] = Rotation.from_euler("x", 35.0, degrees=True).as_matrix()
    optical_T_marker[:3, 3] = (0.03, -0.02, 0.5)
    return optical_T_marker


def _project_marker_corners(optical_T_marker: np.ndarray, marker_length_m: float) -> np.ndarray:
    """Noise-free pixel corners of a marker at camera_optical<-marker under
    _CORNERS_K (OpenCV ArUco corner order, zero distortion) -- the exact input
    the live detector would hand FiducialPrior.observe as corners_px."""
    h = marker_length_m / 2.0
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)
    rvec = cv2.Rodrigues(optical_T_marker[:3, :3])[0]
    tvec = optical_T_marker[:3, 3]
    pts, _ = cv2.projectPoints(obj, rvec, tvec, _CORNERS_K, np.zeros((5, 1)))
    return pts.reshape(4, 2).astype(np.float64)


def test_fiducial_prior_clean_corners_glimpse_kept_and_fused() -> None:
    """A CLEAN corners_px glimpse drives observe()'s live gate chain end to end and
    is KEPT: ambiguity_gated_pose recovers a single trustworthy IPPE pose, the
    corners-derived distance (0.50 m) and view-angle (32.9 deg) clear their gates,
    and once min_observations such glimpses arrive the visit fuses to ONE candidate.
    The stored fix is map_T_marker @ inv(world_T_marker) -- the corners only feed
    the gates; the fused pose is the detector's drift-free world_T_marker -- so on
    this noise-free scene the candidate equals that composition to numerical
    precision. Complements the mirror-ambiguous reject test: this is the KEEP arm
    of the same corners branch. corners_px are cv2.projectPoints of a known pose
    (SIMULATED); no hardware, no replay."""
    gm, lm, _ = _rect_room_scene(seed=21, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    optical_T_marker = _oblique_close_optical_T_marker()
    world_T_optical = _rigid(25.0, (1.2, 0.4, 0.7))
    world_T_marker = world_T_optical @ optical_T_marker
    map_T_marker = _rigid(30.0, (5.0, -2.0, 0.5))
    truth = map_T_marker @ np.linalg.inv(world_T_marker)
    corners_px = _project_marker_corners(optical_T_marker, _CORNERS_MARKER_LENGTH_M)

    prior = FiducialPrior(
        {7: map_T_marker},
        camera_info=_CORNERS_INFO,
        marker_length_m=_CORNERS_MARKER_LENGTH_M,
        ambiguity_ratio_min=2.0,
        now_fn=lambda: 0.0,
    )
    assert AggregationConfig().min_observations == 3
    for i in range(2):  # kept (reason None) but below min_observations -> no fix yet
        assert prior.observe(7, world_T_marker, ts=float(i) * 0.1, corners_px=corners_px) is None
    assert prior.propose(gm, lm) == []

    assert prior.observe(7, world_T_marker, ts=0.2, corners_px=corners_px) is None
    candidates = prior.propose(gm, lm)
    assert len(candidates) == 1 and candidates[0].source == "fiducial"
    np.testing.assert_allclose(candidates[0].T, truth, atol=1e-9)


def test_fiducial_prior_corners_path_matches_world_T_optical_path() -> None:
    """On ONE clean geometry the two observe() input modes agree: feeding
    corners_px (which runs solvePnP + the mirror gate, then derives distance/view
    from the recovered optical_T_marker) yields the SAME stored fix as feeding
    world_T_optical directly (which composes optical_T_marker = inv(world_T_optical)
    @ world_T_marker and runs only the distance/view gates). Both KEEP their
    glimpses and both fuse the detector's world_T_marker, so the two priors'
    proposed candidates are identical to numerical precision -- proving the
    corners branch's gate derivation does not perturb the fix the simpler
    world_T_optical branch would store. SIMULATED corners (cv2.projectPoints)."""
    gm, lm, _ = _rect_room_scene(seed=22, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    optical_T_marker = _oblique_close_optical_T_marker()
    world_T_optical = _rigid(-15.0, (0.6, -0.3, 1.1))
    world_T_marker = world_T_optical @ optical_T_marker
    map_T_marker = _rigid(-40.0, (3.0, 4.0, 0.8))
    corners_px = _project_marker_corners(optical_T_marker, _CORNERS_MARKER_LENGTH_M)

    def _fresh_prior() -> FiducialPrior:
        return FiducialPrior(
            {7: map_T_marker},
            camera_info=_CORNERS_INFO,
            marker_length_m=_CORNERS_MARKER_LENGTH_M,
            ambiguity_ratio_min=2.0,
            now_fn=lambda: 0.0,
        )

    corners_prior = _fresh_prior()
    optical_prior = _fresh_prior()
    for i in range(3):  # min_observations glimpses down each input mode
        ts = float(i) * 0.1
        assert corners_prior.observe(7, world_T_marker, ts=ts, corners_px=corners_px) is None
        assert (
            optical_prior.observe(7, world_T_marker, ts=ts, world_T_optical=world_T_optical) is None
        )

    corners_candidates = corners_prior.propose(gm, lm)
    optical_candidates = optical_prior.propose(gm, lm)
    assert len(corners_candidates) == 1 and len(optical_candidates) == 1
    np.testing.assert_allclose(corners_candidates[0].T, optical_candidates[0].T, atol=1e-9)


# ---------------------------------------------------------------------------
# relocalize_with_priors: empty-pool guard + proposal-census log line
# ---------------------------------------------------------------------------


def test_relocalize_with_priors_raises_when_pool_empty() -> None:
    """Every prior proposing zero candidates leaves the judge nothing to rank;
    relocalize_with_priors raises ValueError rather than returning a bogus pose."""
    gm, lm, _ = _rect_room_scene(seed=20, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    empty_priors: list[RelocPrior] = [LastPosePrior(), FiducialPrior({})]  # both propose []

    try:
        relocalize_with_priors(gm, lm, empty_priors)
    except ValueError as e:
        assert "no prior proposed any candidate" in str(e)
    else:
        raise AssertionError("expected ValueError on an all-empty prior pool")


def test_relocalize_with_priors_census_reports_per_source_counts(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The 'relocalize candidates' census log's per-source counts (a dict, sorted
    by source name) reflect each source's proposed count, independent of which
    candidate later wins the judge -- so a fiducial that proposed then LOST still
    shows counts['fiducial'] == 1."""
    gm, lm, T_true = _rect_room_scene(seed=21, yaw_deg=12.0, t=(0.7, 0.4, 0.0))
    near = T_true.copy()
    near[:3, 3] += np.array([0.03, -0.02, 0.0])
    near2 = T_true.copy()
    near2[:3, 3] += np.array([-0.02, 0.03, 0.0])
    off = T_true.copy()
    off[:3, 3] += np.array([5.0, -4.0, 0.0])  # a losing fiducial fix, still counted

    class _MultiRansac:
        name = "ransac"

        def propose(
            self, global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
        ) -> list[Candidate]:
            return [Candidate(T=near, source="ransac"), Candidate(T=near2, source="ransac")]

    class _OneFiducial:
        name = "fiducial"

        def propose(
            self, global_map: o3d.geometry.PointCloud, local_map: o3d.geometry.PointCloud
        ) -> list[Candidate]:
            return [Candidate(T=off, source="fiducial")]

    logged: list[tuple[str, dict[str, object]]] = []

    class _Recorder:
        def info(self, event: str, *args: object, **kwargs: object) -> None:
            logged.append((event, kwargs))

    monkeypatch.setattr(priors_mod, "logger", _Recorder())

    _, _, source = relocalize_with_priors(gm, lm, [_MultiRansac(), _OneFiducial()])

    census = [kw for ev, kw in logged if ev == "relocalize candidates"]
    assert census == [{"counts": {"fiducial": 1, "ransac": 2}}]
    assert source == "ransac"  # the census counts a loser; ransac still wins the judge


# ---------------------------------------------------------------------------
# Gravity gate: fallback branch, threshold override, index attribution, log
# ---------------------------------------------------------------------------


def _tilt_x(base: np.ndarray, deg: float) -> np.ndarray:
    """`base` right-multiplied by a rotation of `deg` about body-x, tilting its
    z-axis off world-up by exactly `deg` when `base` is gravity-upright (yaw-only,
    as _rigid produces): _gravity_tilt_deg(_tilt_x(_rigid(...), d)) == d."""
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("x", deg, degrees=True).as_matrix()
    return base @ t


class _LogRecorder:
    """Stand-in for relocalize_mod.logger capturing .info() as (event, kwargs)
    pairs. The real logger sets propagate=False (logging_config.py), so caplog
    can't see the 'judge finalists' line; monkeypatching the module logger is
    the deterministic way to assert on it."""

    def __init__(self) -> None:
        self.infos: list[tuple[str, dict[str, object]]] = []

    def info(self, event: str, *args: object, **kwargs: object) -> None:
        self.infos.append((event, kwargs))


def test_all_tilted_pool_falls_back_and_still_solves() -> None:
    """When EVERY candidate is tilted past the gravity gate (degenerate clouds),
    the single-source path must not return an empty pool: it falls back to the
    full indexed set (relocalize.py:229 `pool = upright if upright else indexed`)
    and still ranks by wall fitness. The tilted near-truth candidate beats the
    tilted far decoy -- proving the fallback ran rather than raising/orphaning."""
    gm, lm, T_true = _rect_room_scene(seed=41, yaw_deg=20.0, t=(1.0, -1.5, 0.0))

    near_tilted = _tilt_x(T_true, 15.0)  # near truth, 15 deg off-gravity (> 10 gate)
    far_tilted = _tilt_x(_rigid(200.0, (6.0, -5.0, 0.0)), 15.0)  # wrong room, also tilted
    candidates = [near_tilted, far_tilted]

    # Both tilted -> `upright` is empty -> fallback to full pool; without the
    # fallback the pool would be empty and max() would raise ValueError.
    assert relocalize_mod._gravity_tilt_deg(near_tilted) > relocalize_mod.GRAVITY_TILT_MAX_DEG
    assert relocalize_mod._gravity_tilt_deg(far_tilted) > relocalize_mod.GRAVITY_TILT_MAX_DEG

    T, fitness, winning_index = refine_candidates(gm, lm, candidates)

    assert winning_index == 0
    assert fitness > 0.3
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=1.0)


def test_gravity_tilt_max_deg_override_changes_survivor() -> None:
    """The gravity_tilt_max_deg parameter is live: the SAME pool yields a
    different winner at different thresholds. A 15-deg-tilted near-truth
    candidate is filtered at the 10-deg default (an upright far decoy wins by
    walkover) but survives -- and wins on wall fitness -- once the gate opens
    to 20 deg."""
    gm, lm, T_true = _rect_room_scene(seed=42, yaw_deg=-25.0, t=(0.8, 1.2, 0.0))

    near_tilted = _tilt_x(T_true, 15.0)  # index 0: near truth but 15 deg tilted
    upright_far = _rigid(120.0, (6.0, -5.0, 0.0))  # index 1: upright, wrong room
    candidates = [near_tilted, upright_far]

    # Default 10-deg gate: tilted near-truth filtered, only the far decoy remains.
    _, _, idx_default = refine_candidates(gm, lm, candidates)
    assert idx_default == 1

    # Opened to 20 deg: the tilted near-truth survives the gate and wins.
    T, fitness, idx_open = refine_candidates(gm, lm, candidates, gravity_tilt_max_deg=20.0)
    assert idx_open == 0
    assert fitness > 0.3
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=1.0)


def test_winning_index_attributes_to_correct_nonzero_position_and_source() -> None:
    """winning_index is a real lookup, not a constant: the sole near-truth
    candidate placed at index 2 (flanked by wrong-room decoys) is the one
    returned, and sources[winning_index] names its owning prior. Guards against
    an off-by-one / always-0 regression in the index the judge reports back."""
    gm, lm, T_true = _rect_room_scene(seed=43, yaw_deg=15.0, t=(1.1, -0.6, 0.02))

    good = T_true.copy()
    good[:3, 3] += np.array([0.03, -0.02, 0.0])  # 3.6 cm off, clearly the truth
    candidates = [
        _rigid(100.0, (5.0, -4.0, 0.0)),
        _rigid(220.0, (-3.0, 3.0, 0.0)),
        good,  # index 2
        _rigid(300.0, (6.0, 2.0, 0.0)),
    ]
    sources = ["ransac", "last_pose", "fiducial", "ransac"]

    T, fitness, winning_index = refine_candidates(gm, lm, candidates, sources=sources)

    assert winning_index == 2
    assert sources[winning_index] == "fiducial"
    assert fitness > 0.5
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=0.15)


def test_judge_log_fires_only_for_multi_source_topk(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The per-candidate 'judge:' finalist log is a multi-source-only diagnostic
    (relocalize.py:293): it must stay silent when there is nothing to
    adjudicate -- sources=None, or a top-k drawn from a single source -- and
    fire, naming the finalists with a '*' on the winner, only when >1 distinct
    source reaches the top-k."""
    gm, lm, T_true = _rect_room_scene(seed=44, yaw_deg=10.0, t=(0.7, 0.9, 0.0))
    good = T_true.copy()
    good[:3, 3] += np.array([0.03, -0.02, 0.0])
    other = _rigid(140.0, (5.0, -4.0, 0.0))

    # (a) sources=None -> the log is guarded off entirely.
    rec = _LogRecorder()
    monkeypatch.setattr(relocalize_mod, "logger", rec)
    refine_candidates(gm, lm, [good, other])
    assert not any(ev == "judge finalists" for ev, _ in rec.infos)

    # (b) sources present but a SINGLE distinct source in the top-k -> silent.
    rec = _LogRecorder()
    monkeypatch.setattr(relocalize_mod, "logger", rec)
    refine_candidates(gm, lm, [good, other], sources=["ransac", "ransac"])
    assert not any(ev == "judge finalists" for ev, _ in rec.infos)

    # (c) two distinct sources reach the top-k -> exactly one 'judge:' line,
    # naming both sources and marking the winner.
    rec = _LogRecorder()
    monkeypatch.setattr(relocalize_mod, "logger", rec)
    _, _, idx = refine_candidates(gm, lm, [good, other], sources=["fiducial", "ransac"])
    judge_lines = [kw for ev, kw in rec.infos if ev == "judge finalists"]
    assert len(judge_lines) == 1
    finalists = cast("list[tuple[str, float]]", judge_lines[0]["finalists"])
    assert {src for src, _fit in finalists} == {"fiducial", "ransac"}  # both named
    assert judge_lines[0]["winner"] == "fiducial"  # winner named via kwarg, not a '*'
    assert idx == 0  # the near-truth 'fiducial' candidate won


# ---------------------------------------------------------------------------
# RelocalizationModule: Config contract + pure helpers (no coordinator).
#
# These exercise module.py directly, never through a live/replay Module: the
# Config defaults, _has_enough_points' min_local_points gate, the accept/reject
# log-line shape, marker-id parsing, and _on_detections routing each sighting's
# world_T_marker into the fiducial prior. Instances are built with object.__new__
# and only the attributes the helper under test reads -- the full Module
# lifecycle (start()'s reactive wiring) needs a coordinator these helpers do not.
# ---------------------------------------------------------------------------


def _bare_module(config: Config) -> RelocalizationModule:
    """A RelocalizationModule shell with just the attributes the pure helpers read
    -- no coordinator, no start() wiring. _premap/_fiducial_prior default to the
    unset state; a test overrides whichever it needs."""
    m = object.__new__(RelocalizationModule)
    m.config = config
    m._premap = None
    m._last_skip_log = 0.0
    m._last_pose_prior = LastPosePrior()
    m._fiducial_prior = None
    return m


class _StubCloud:
    """Minimal PointCloud2 stand-in: __len__ (the point count _has_enough_points
    and _try_relocalize read for gating / the n_pts log field) and a `.pointcloud`
    sentinel handed straight through to the monkeypatched relocalize()."""

    def __init__(self, n: int) -> None:
        self._n = n
        self.pointcloud = object()

    def __len__(self) -> int:
        return self._n


def test_config_requires_priors_no_module_default() -> None:
    """priors is REQUIRED (no default_factory): constructing Config without a pool
    raises, so there is no silent module-level reloc behavior -- a blueprint or -o
    must state the pool. The pydantic error names the missing field."""
    with pytest.raises(ValidationError, match="priors"):
        Config()  # type: ignore[call-arg]


def test_config_scalar_defaults_match_documented_contract() -> None:
    """Config's scalar defaults ARE the reloc gating contract; any silent drift
    changes production behavior, so pin every field the module documents exactly.
    priors is supplied (required); the two map-file knobs default to unset."""
    c = Config(priors=[RansacPriorConfig()])
    assert c.min_local_points == 50_000
    assert c.reloc_interval_s == 2.0
    assert c.gravity_tilt_max_deg == 10.0
    assert c.fitness_threshold == 0.45
    assert c.map_file is None
    assert c.marker_map_file is None  # top-level fiducial-survey -o override, unset


def test_fiducial_prior_config_defaults() -> None:
    """The fiducial entry owns the marker parameter surface; pin its defaults so a
    preset written today keeps meaning the same thing."""
    fid = FiducialPriorConfig()
    assert fid.marker_length_m == 0.10
    assert fid.ambiguity_ratio_min == 2.0
    assert fid.marker_map_file is None
    assert fid.aruco_dictionary == "DICT_APRILTAG_36h11"
    assert fid.age_max_s == 120.0


def test_has_enough_points_gates_on_config_min_local_points() -> None:
    """_has_enough_points reads config.min_local_points, not a module constant:
    exactly at the threshold passes, one point below fails, and changing the knob
    moves the boundary with it."""
    m = _bare_module(Config(priors=[RansacPriorConfig()], min_local_points=50_000))
    assert m._has_enough_points(cast("PointCloud2", _StubCloud(50_000))) is True
    assert m._has_enough_points(cast("PointCloud2", _StubCloud(49_999))) is False

    m2 = _bare_module(Config(priors=[RansacPriorConfig()], min_local_points=100))
    assert m2._has_enough_points(cast("PointCloud2", _StubCloud(100))) is True
    assert m2._has_enough_points(cast("PointCloud2", _StubCloud(99))) is False


def _detection(det_id: str = "", class_id: str | None = None) -> Detection3D:
    """A wire Detection3D with an optional `id` field and, if given, one result
    hypothesis carrying `class_id` (the 'DICT:id' marker-label channel)."""
    det = Detection3D()
    det.id = det_id
    if class_id is not None:
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        det.results = [hyp]
        det.results_length = 1
    return det


def test_marker_id_from_detection_prefers_numeric_id_field() -> None:
    """A numeric `id` field parses straight to an int -- the detector's primary
    marker-id channel, read ahead of the class label."""
    assert RelocalizationModule._marker_id_from_detection(_detection(det_id="7")) == 7


def test_marker_id_from_detection_falls_back_to_dict_class_label() -> None:
    """With no numeric id, the numeric tail of a 'DICT:id' class label is used --
    the detector's marker_label encoding ('DICT_APRILTAG_36h11:12' -> 12)."""
    det = _detection(det_id="", class_id="DICT_APRILTAG_36h11:12")
    assert RelocalizationModule._marker_id_from_detection(det) == 12


def test_marker_id_from_detection_returns_none_when_unparseable() -> None:
    """Neither a numeric id nor a ':'-delimited numeric tail -> None, so
    _on_detections skips the sighting rather than inventing a tag id."""
    assert RelocalizationModule._marker_id_from_detection(_detection(det_id="abc")) is None
    assert (
        RelocalizationModule._marker_id_from_detection(_detection(det_id="", class_id="chair"))
        is None
    )
    assert RelocalizationModule._marker_id_from_detection(_detection()) is None


def _pose_detection(
    det_id: int, pos: tuple[float, float, float], quat: tuple[float, float, float, float]
) -> Detection3D:
    """A Detection3D whose bbox.center is world_T_marker (pos + xyzw quat)."""
    det = Detection3D()
    det.id = str(det_id)
    center = det.bbox.center
    center.position.x, center.position.y, center.position.z = pos
    (
        center.orientation.x,
        center.orientation.y,
        center.orientation.z,
        center.orientation.w,
    ) = quat
    return det


def _detection_array(dets: list[Detection3D], ts: float) -> Detection3DArray:
    """A Detection3DArray carrying `dets` with header.stamp encoding `ts` (so
    msg.ts == ts, the timebase _on_detections stamps each observe with)."""
    arr = Detection3DArray()
    sec = int(ts)
    arr.header.stamp.sec = sec
    arr.header.stamp.nsec = round((ts - sec) * 1e9)
    arr.detections = dets
    arr.detections_length = len(dets)
    return arr


class _RecordingPrior:
    """FiducialPrior stand-in recording (marker_id, world_T_marker, ts) per
    observe -- lets _on_detections' routing be asserted without the aggregator."""

    def __init__(self) -> None:
        self.calls: list[tuple[int, np.ndarray, float]] = []

    def observe(self, marker_id: int, world_T_marker: np.ndarray, ts: float) -> None:
        self.calls.append((marker_id, world_T_marker, ts))


def test_on_detections_routes_each_world_T_marker_and_ts_into_prior() -> None:
    """Each parseable sighting's bbox.center (world_T_marker) is composed via
    matrix_from_pose7 and handed to the fiducial prior with the array's ts;
    unparseable detections are skipped rather than routed."""
    prior = _RecordingPrior()
    m = _bare_module(Config(priors=[FiducialPriorConfig()]))
    m._fiducial_prior = prior  # type: ignore[assignment]

    det_a = _pose_detection(5, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    quat_b = tuple(Rotation.from_euler("z", 30.0, degrees=True).as_quat())
    det_b = _pose_detection(8, (-0.5, 0.25, 4.0), quat_b)  # type: ignore[arg-type]
    det_skip = _detection(det_id="not-a-tag")  # unparseable -> dropped
    arr = _detection_array([det_a, det_skip, det_b], ts=3.5)

    m._on_detections(arr)

    assert [mid for mid, _, _ in prior.calls] == [5, 8]  # det_skip never routed
    for (_mid, world_T_marker, ts), det in (
        (prior.calls[0], det_a),
        (prior.calls[1], det_b),
    ):
        assert ts == 3.5
        c = det.bbox.center
        expected = matrix_from_pose7(
            (
                c.position.x,
                c.position.y,
                c.position.z,
                c.orientation.x,
                c.orientation.y,
                c.orientation.z,
                c.orientation.w,
            )
        )
        np.testing.assert_array_equal(world_T_marker, expected)


def test_on_detections_noop_without_fiducial_prior() -> None:
    """With the fiducial prior unbuilt (prior disabled / no marker_map_file),
    _on_detections is a silent no-op on wire traffic -- returns None, no raise."""
    m = _bare_module(Config(priors=[RansacPriorConfig()]))
    m._fiducial_prior = None
    m._on_detections(
        _detection_array([_pose_detection(1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))], ts=1.0)
    )
    assert m._fiducial_prior is None  # still a no-op: nothing built, nothing routed


class _ModuleLogRecorder:
    """Captures module.py logger .info/.warning calls as (event, kwargs) pairs.
    The real logger sets propagate=False (logging_config.py), so caplog can't see
    these lines; monkeypatching module_mod.logger is the deterministic way to
    assert them. An unexpected .exception (relocalize() crash path) fails loudly."""

    def __init__(self) -> None:
        self.infos: list[tuple[str, dict[str, object]]] = []
        self.warnings: list[tuple[str, dict[str, object]]] = []

    def info(self, event: str, *args: object, **kwargs: object) -> None:
        self.infos.append((event, kwargs))

    def warning(self, event: str, *args: object, **kwargs: object) -> None:
        self.warnings.append((event, kwargs))

    def exception(self, msg: str, *args: object, **kwargs: object) -> None:
        raise AssertionError(f"unexpected logger.exception in _try_relocalize: {msg}")


def test_try_relocalize_accept_log_puts_source_first_when_prior_wins(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """On an accepted fix from a prior pool, the info line leads with
    `source=<winner>` ahead of `fitness=`, so the winning prior is the first field
    read; a valid world->map TF is returned and no reject warning fires."""
    T = _rigid(25.0, (1.5, -0.8, 0.05))
    m = _bare_module(
        Config(priors=[RansacPriorConfig(), LastPosePriorConfig()], fitness_threshold=0.45)
    )
    m._premap = _StubCloud(10)  # type: ignore[assignment]

    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(
        module_mod, "_relocalize_with_priors", lambda *a, **k: (T, 0.90, "last_pose")
    )

    tf = m._try_relocalize(_StubCloud(1234))  # type: ignore[arg-type]

    assert tf is not None
    assert tf.frame_id == "world" and tf.child_frame_id == "map"
    assert len(rec.infos) == 1
    event, kwargs = rec.infos[0]
    assert event == "relocalize accepted"
    assert kwargs["source"] == "last_pose"
    keys = list(kwargs)
    assert keys.index("source") < keys.index("fitness")  # winner read first, ahead of fitness
    assert not rec.warnings


def test_try_relocalize_reject_log_warns_below_fitness_threshold(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A fix under fitness_threshold is rejected: no TF (returns None) and a
    'relocalize rejected' warning naming fitness < threshold -- never an accept
    info line."""
    T = _rigid(0.0, (0.0, 0.0, 0.0))
    # single RANSAC entry -> _try_relocalize takes the plain relocalize() path
    m = _bare_module(Config(priors=[RansacPriorConfig()], fitness_threshold=0.45))
    m._premap = _StubCloud(10)  # type: ignore[assignment]

    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(module_mod, "_relocalize", lambda *a, **k: (T, 0.30))

    assert m._try_relocalize(_StubCloud(1234)) is None  # type: ignore[arg-type]
    assert len(rec.warnings) == 1
    event, kwargs = rec.warnings[0]
    assert event == "relocalize rejected"
    assert kwargs["fitness"] == 0.3  # round(0.30, 3)
    assert kwargs["threshold"] == 0.45
    assert not rec.infos


def test_try_relocalize_accept_log_omits_source_when_no_prior(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The single-source path (no priors) accepts via plain relocalize() and its
    info line carries NO `source=` prefix -- source= is present only when a prior
    wins, so its absence is the single-source signature."""
    T = _rigid(10.0, (0.5, 0.5, 0.0))
    m = _bare_module(Config(priors=[RansacPriorConfig()], fitness_threshold=0.45))
    m._premap = _StubCloud(10)  # type: ignore[assignment]

    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(module_mod, "_relocalize", lambda *a, **k: (T, 0.95))

    tf = m._try_relocalize(_StubCloud(1234))  # type: ignore[arg-type]

    assert tf is not None
    assert len(rec.infos) == 1
    event, kwargs = rec.infos[0]
    assert event == "relocalize accepted"
    assert "source" not in kwargs  # single-source path omits source entirely


def test_marker_id_from_detection_colon_label_with_nonnumeric_tail_returns_none() -> None:
    """A class label carrying a ':' but a non-numeric tail ('DICT:abc') is NOT a
    marker id: the digit check on the tail fails, the scan falls through, and the
    sighting is dropped rather than routed to the prior under a garbage id."""
    det = _detection(det_id="", class_id="DICT_APRILTAG_36h11:abc")
    assert RelocalizationModule._marker_id_from_detection(det) is None


def test_try_relocalize_appends_fiducial_prior_to_judge_pool(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """With the fiducial prior enabled and built, _try_relocalize adds THAT
    prior instance to the judge pool (alongside RANSAC) and reports the winning
    source it returns -- the fiducial candidate is judged, never bypassed."""
    T = _rigid(15.0, (0.4, -0.3, 0.02))
    m = _bare_module(
        Config(priors=[RansacPriorConfig(), FiducialPriorConfig()], fitness_threshold=0.45)
    )
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    sentinel = FiducialPrior({7: np.eye(4)}, now_fn=lambda: 0.0)
    m._fiducial_prior = sentinel

    seen_pools: list[list[RelocPrior]] = []

    def _fake(global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object):  # type: ignore[no-untyped-def]
        seen_pools.append(priors)
        return T, 0.90, "fiducial"

    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())
    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)

    tf = m._try_relocalize(_StubCloud(1234))  # type: ignore[arg-type]

    assert tf is not None
    assert len(seen_pools) == 1
    assert sentinel in seen_pools[0]  # the built fiducial prior reached the judge
    assert any(isinstance(p, RansacPrior) for p in seen_pools[0])


def test_try_relocalize_returns_none_and_logs_on_relocalize_crash(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A crash inside the solve is caught at the module boundary: _try_relocalize
    logs it via logger.exception and returns None (no TF published) rather than
    letting the exception escape the reactive subscription."""
    m = _bare_module(Config(priors=[RansacPriorConfig()], fitness_threshold=0.45))
    m._premap = _StubCloud(10)  # type: ignore[assignment]

    logged: list[str] = []

    class _CrashRecorder:
        def info(self, msg: str, *args: object, **kwargs: object) -> None:
            raise AssertionError("no accept log expected on a crash")

        def warning(self, msg: str, *args: object, **kwargs: object) -> None:
            raise AssertionError("no reject log expected on a crash")

        def exception(self, msg: str, *args: object, **kwargs: object) -> None:
            logged.append(msg)

    def _boom(*args: object, **kwargs: object) -> tuple[np.ndarray, float]:
        raise RuntimeError("open3d blew up")

    monkeypatch.setattr(module_mod, "logger", _CrashRecorder())
    monkeypatch.setattr(module_mod, "_relocalize", _boom)

    assert m._try_relocalize(_StubCloud(1234)) is None  # type: ignore[arg-type]
    assert logged == ["relocalize() failed"]


def test_try_relocalize_returns_none_when_all_priors_disabled(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """Every prior disabled -> no proposer, so _try_relocalize short-circuits to
    None and never calls a solve, rather than handing the judge an empty pool."""
    m = _bare_module(Config(priors=[RansacPriorConfig(enabled=False)]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]

    def _boom(*args: object, **kwargs: object) -> tuple[np.ndarray, float]:
        raise AssertionError("no solve should run with zero enabled priors")

    monkeypatch.setattr(module_mod, "_relocalize", _boom)
    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _boom)
    assert m._try_relocalize(_StubCloud(1234)) is None  # type: ignore[arg-type]


def test_maybe_log_skip_warns_below_min_then_throttles(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A frame below min_local_points logs one throttled 'relocalize skipped'
    warning naming the count and the knob; a second skip inside the 5 s throttle
    window stays silent, and a frame at/above the floor never warns at all."""
    clock = {"now": 100.0}
    monkeypatch.setattr("dimos.mapping.relocalization.module.time.monotonic", lambda: clock["now"])
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)

    m = _bare_module(Config(priors=[RansacPriorConfig()], min_local_points=50_000))

    m._maybe_log_skip(cast("PointCloud2", _StubCloud(10_000)))
    assert len(rec.warnings) == 1
    event, kwargs = rec.warnings[0]
    assert event == "relocalize skipped"
    assert kwargs["n_pts"] == 10000
    assert kwargs["min_local_points"] == 50000

    clock["now"] = 102.0  # 2 s later, still inside the 5 s throttle window
    m._maybe_log_skip(cast("PointCloud2", _StubCloud(9_000)))
    assert len(rec.warnings) == 1  # throttled: no second warning

    m._maybe_log_skip(cast("PointCloud2", _StubCloud(50_000)))  # at the floor -> not a skip
    assert len(rec.warnings) == 1


def test_start_fiducial_prior_noops_without_marker_map_file(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """Fiducial prior enabled but no marker_map_file surveyed: _start_fiducial_prior
    warns and leaves the prior unbuilt (None) -- it never fabricates a map, mirroring
    start()'s no-map_file disable, so _on_detections stays a no-op afterwards."""
    m = _bare_module(Config(priors=[FiducialPriorConfig(marker_map_file=None)]))
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)

    m._start_fiducial_prior()

    assert m._fiducial_prior is None
    assert len(rec.warnings) == 1
    assert "no marker_map_file" in rec.warnings[0][0]  # unchanged plain-string warning event


def test_init_leaves_priors_in_unset_state(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A freshly constructed module starts with no premap and no fiducial prior
    (both built later in start()), a zeroed skip-log clock, and a real LastPosePrior
    ready to carry a seed forward -- the documented cold-start contract."""
    monkeypatch.setattr(
        "dimos.mapping.relocalization.module.Module.__init__", lambda self, **kwargs: None
    )
    m = RelocalizationModule()
    assert m._premap is None
    assert m._fiducial_prior is None
    assert m._last_skip_log == 0.0
    assert isinstance(m._last_pose_prior, LastPosePrior)


class _DetectionsStub:
    """Stands in for the `detections` In-stream: observable().subscribe(cb) returns
    a disposable sentinel, letting _start_fiducial_prior's wiring run without a live
    coordinator. Only the framework seam is stubbed; FiducialPrior is the real one."""

    def observable(self) -> _DetectionsStub:
        return self

    def subscribe(self, callback: object) -> str:
        return "detections_disposable"


class _FakeSurveyedTransform:
    """A load_marker_map entry: only .to_matrix() is read by _start_fiducial_prior."""

    def to_matrix(self) -> np.ndarray:
        return np.eye(4)


def test_start_fiducial_prior_loads_map_and_wires_detections(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """With a marker_map_file present, _start_fiducial_prior resolves + loads the
    survey into map_T_marker matrices, builds the real FiducialPrior, subscribes the
    detections stream into it (registering the disposable), and logs the marker
    count -- the fiducial source is now live and age-gated in the judge."""
    m = _bare_module(
        Config(priors=[FiducialPriorConfig(marker_map_file="survey", marker_length_m=0.1)])
    )
    m.detections = _DetectionsStub()  # type: ignore[assignment]
    registered: list[object] = []
    m.register_disposable = registered.append  # type: ignore[assignment,method-assign]

    monkeypatch.setattr(module_mod, "resolve_named_path", lambda name, ext: f"{name}{ext}")
    monkeypatch.setattr(
        module_mod,
        "load_marker_map",
        lambda path: {7: _FakeSurveyedTransform(), 9: _FakeSurveyedTransform()},
    )
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)

    m._start_fiducial_prior()

    assert isinstance(m._fiducial_prior, FiducialPrior)
    assert registered == ["detections_disposable"]  # detections wired into the prior
    assert len(rec.infos) == 1
    event, kwargs = rec.infos[0]
    assert event == "fiducial prior enabled"
    assert kwargs["n_markers"] == 2


def test_o_override_marker_map_file_reaches_fiducial_prior(tmp_path: Path) -> None:
    """End-to-end: `-o relocalizationmodule.marker_map_file=<survey>` reaches the
    live FiducialPrior through the REAL pipeline, not just a parse.

    Runs the actual CLI overlay (load_config_args), the actual deploy-time merge
    (_merge_config_kwargs) over the blueprint's fiducial preset, real Config
    validation, then real _start_fiducial_prior loading the survey. Proves three
    things the top-level override exists for: (1) the CLI pre-validation TOLERATES
    the partial override even though priors is now required (the preset fills it at
    deploy); (2) the merge leaves the blueprint's fiducial entry intact -- only the
    top-level field carries the operator's map; (3) the module actually loads THAT
    survey (marker id 42) into the prior."""
    survey = tmp_path / "custom_markers.json"
    survey.write_text(
        json.dumps({"markers": {"42": {"translation": [1.0, 2.0, 3.0], "rotation": [0, 0, 0, 1]}}})
    )

    # The as-shipped lidar+fiducial preset: RANSAC + a fiducial entry with no map.
    bp = RelocalizationModule.blueprint(
        priors=[RansacPriorConfig(), FiducialPriorConfig(marker_length_m=0.1)],
        map_file=None,
    )
    key = config_key(bp.blueprints[0].name)  # "relocalizationmodule"

    # 1. Operator overlay: the exact -o an operator types. load_config_args parses
    #    AND pre-validates against the real BlueprintConfig; priors is required but
    #    absent here, and the CLI tolerates that missing field (would raise otherwise).
    overlay = load_config_args(
        bp.config(), [f"{key}.marker_map_file={survey}"], tmp_path / "no_such_config"
    )
    assert overlay == {key: {"marker_map_file": str(survey)}}

    # 2. Deploy-time merge over the preset (the real coordinator merge), then real
    #    Config validation -- priors satisfied by the preset, not the overlay.
    merged = _merge_config_kwargs(dict(bp.blueprints[0].kwargs), overlay[key])
    config = Config(**merged)
    assert config.marker_map_file == str(survey)  # top-level override landed
    fid = next(p for p in config.priors if isinstance(p, FiducialPriorConfig))
    assert fid.marker_map_file is None  # blueprint entry untouched -> override wins

    # 3. The module loads THAT survey into the live FiducialPrior (real load path:
    #    absolute path -> resolve_named_path returns it -> real load_marker_map).
    m = _bare_module(config)
    m.detections = _DetectionsStub()  # type: ignore[assignment]
    registered: list[object] = []
    m.register_disposable = registered.append  # type: ignore[assignment,method-assign]
    m._start_fiducial_prior()

    assert isinstance(m._fiducial_prior, FiducialPrior)
    assert set(m._fiducial_prior._map_T_marker) == {42}  # the overridden survey's tag


def test_o_overlay_still_rejects_a_misspelled_reloc_key(tmp_path: Path) -> None:
    """Tolerating a missing required field in the -o overlay must NOT make the
    pre-validation permissive: a misspelled reloc key (extra_forbidden, not
    'missing') still raises, so typos are caught before deploy rather than silently
    ignored."""
    bp = RelocalizationModule.blueprint(priors=[RansacPriorConfig()], map_file=None)
    key = config_key(bp.blueprints[0].name)
    with pytest.raises(ValidationError):
        load_config_args(bp.config(), [f"{key}.marker_map_fil=/x.json"], tmp_path / "no_config")


def test_start_disabled_without_map_file_logs_and_returns(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """start() with no map_file configured is a clean disable: it logs the disabled
    line and returns without decoding a premap -- the module then publishes nothing
    rather than crashing on a missing map path."""
    m = object.__new__(RelocalizationModule)
    m.config = Config(priors=[RansacPriorConfig()], map_file=None)
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr("dimos.mapping.relocalization.module.Module.start", lambda self: None)

    m.start()

    assert rec.infos == [("Relocalization module disabled (no map_file configured)", {})]
    assert not hasattr(m, "_premap")  # no premap decoded on the disabled path


def test_fiducial_prior_aggregator_rejection_returns_reason() -> None:
    """A gated glimpse (here a tag 5 m out, past the 1 m distance gate) is refused
    by the aggregator: observe returns the gate name 'far' and stores no fix, so a
    single out-of-gate sighting can never seed a fused world->map candidate."""
    prior = FiducialPrior({7: np.eye(4)}, now_fn=lambda: 0.0)
    world_T_marker = np.eye(4)
    world_T_marker[:3, 3] = [5.0, 0.0, 0.0]  # 5 m from the optical origin
    reason = prior.observe(7, world_T_marker, ts=0.0, world_T_optical=np.eye(4))
    assert reason == "far"
    assert prior.propose(o3d.geometry.PointCloud(), o3d.geometry.PointCloud()) == []


def test_relocalize_with_priors_empty_pool_raises_named_valueerror() -> None:
    """Every prior proposing zero candidates raises ValueError whose message names
    the failure ('no prior proposed any candidate'), so an operator reading the log
    sees WHY the judge had nothing to rank, not a bare crash."""
    gm, lm, _ = _rect_room_scene(seed=41, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    empty_priors: list[RelocPrior] = [LastPosePrior(), FiducialPrior({})]
    with pytest.raises(ValueError, match="no prior proposed any candidate"):
        relocalize_with_priors(gm, lm, empty_priors)


def test_refine_candidates_raises_on_sources_length_mismatch() -> None:
    """A sources list whose length disagrees with the candidate pool is a caller
    bug that would silently mis-attribute the per-source gravity gate; refine_candidates
    refuses loudly with a ValueError naming both lengths."""
    gm, lm, T_true = _rect_room_scene(seed=42, yaw_deg=10.0, t=(0.3, 0.2, 0.0))
    with pytest.raises(ValueError, match=r"sources \(2\) and candidates \(1\) must align"):
        refine_candidates(gm, lm, [T_true.copy()], sources=["ransac", "last_pose"])


def test_floors_only_scene_raises_insufficient_wall_evidence_message() -> None:
    """A floors-only scene (no walls -> no horizontal-normal geometry) raises
    InsufficientWallEvidenceError whose message reports the wall counts, so the
    'wall-only' rerank refuses rather than scoring a rotation-blind fitness on floors."""
    rng = np.random.default_rng(43)
    floor = _sample_plane(rng, 6.0, 4.0, 0.0, 2000)
    ceiling = _sample_plane(rng, 6.0, 4.0, 2.5, 2000)
    room_pts = np.concatenate([floor, ceiling], axis=0)
    mask = (room_pts[:, 0] <= 3.2) & (room_pts[:, 1] <= 2.2)
    T_true = _rigid(30.0, (1.0, 0.5, 0.0))
    local_pts = _apply(np.linalg.inv(T_true), room_pts[mask].copy())
    with pytest.raises(InsufficientWallEvidenceError, match="insufficient wall evidence"):
        refine_candidates(_pcd(room_pts), _pcd(local_pts), [T_true.copy()])


# ---------------------------------------------------------------------------
# MIN_WALL_POINTS: the exact refuse/solve boundary (relocalize.py:259 guard
# `n_walls < MIN_WALL_POINTS`). The floors-only tests above cover 0 walls; this
# pins the comparison itself at the threshold. The scene's real wall count is
# read through the SAME production path refine_candidates uses (fine voxel
# downsample + normals + _wall_subset), so the bracketed boundary is the live
# one, not a re-derived guess -- then MIN_WALL_POINTS is monkeypatched to sit
# exactly on and one above it.
# ---------------------------------------------------------------------------


def test_min_wall_points_boundary_solves_at_threshold_raises_one_above(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The sparse-wall guard is a strict `<`: a cloud with exactly
    MIN_WALL_POINTS wall points still solves, and one more required point (the
    binding cloud now one short) refuses loudly. Reads the scene's true
    post-downsample wall count via the production _wall_subset so the boundary is
    the real one; the message names the raised threshold."""
    gm, lm, T_true = _rect_room_scene(seed=51, yaw_deg=15.0, t=(1.0, 0.5, 0.0))

    # Wall counts through the exact steps refine_candidates runs internally --
    # calling the real functions (not re-deriving) so `w` is the production
    # boundary the guard will compare against.
    src_fine = lm.voxel_down_sample(relocalize_mod.FINE_VOXEL)
    src_fine.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=relocalize_mod.FINE_VOXEL * 2, max_nn=30)
    )
    tgt_fine = relocalize_mod._global_fine(gm, relocalize_mod.FINE_VOXEL)
    n_src = len(relocalize_mod._wall_subset(src_fine).points)
    n_tgt = len(relocalize_mod._wall_subset(tgt_fine).points)
    w = min(n_src, n_tgt)  # the binding (smaller) cloud sets the boundary
    assert w >= 100  # sanity: a walled room clears the production MIN_WALL_POINTS

    # n == MIN_WALL_POINTS: `w < w` is False on both clouds -> the solve runs.
    monkeypatch.setattr(relocalize_mod, "MIN_WALL_POINTS", w)
    T, fitness, idx = refine_candidates(gm, lm, [T_true.copy()])
    assert idx == 0
    assert fitness > 0.5
    np.testing.assert_allclose(T[:3, 3], T_true[:3, 3], atol=0.15)

    # n == MIN_WALL_POINTS - 1: the binding cloud is one short -> loud refusal,
    # message naming the raised threshold.
    monkeypatch.setattr(relocalize_mod, "MIN_WALL_POINTS", w + 1)
    with pytest.raises(InsufficientWallEvidenceError, match=rf"< {w + 1} "):
        refine_candidates(gm, lm, [T_true.copy()])


# ---------------------------------------------------------------------------
# FiducialPrior negative-age clamp: the single-timebase guard in propose()
# (priors.py:283-289). fix_ts is stamped by now_fn() and MUST be compared only
# against now_fn(); a mixed/non-monotonic clock makes now < fix_ts, driving the
# age negative. Left raw, `age_s <= age_max_s` is then ALWAYS true and a stale
# fix proposes forever -- the age gate silently disabled. The fix clamps to 0
# (treats as fresh) and warns so the misconfig surfaces.
# ---------------------------------------------------------------------------


def test_fiducial_prior_negative_age_clamps_to_fresh_and_warns(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A fix whose computed age goes NEGATIVE (propose reads a clock behind the
    one that stamped fix_ts) is clamped to fresh -- it still proposes its ONE
    candidate rather than the age gate silently passing it forever -- and emits
    exactly one warning naming the negative age, so the single-timebase misconfig
    is loud, not silent."""
    gm, lm, _ = _rect_room_scene(seed=23, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    clock = {"now": 100.0}
    prior = FiducialPrior({7: np.eye(4)}, age_max_s=120.0, now_fn=lambda: clock["now"])
    for i in range(3):  # >= min_observations -> a fused fix is stamped at now=100
        prior.observe(7, np.eye(4), ts=float(i) * 0.1)

    warned: list[dict[str, object]] = []

    class _WarnRecorder:
        def warning(self, msg: str, *args: object, **kwargs: object) -> None:
            warned.append({"msg": msg, **kwargs})

    monkeypatch.setattr(priors_mod, "logger", _WarnRecorder())

    clock["now"] = 50.0  # propose reads a clock 50 s BEHIND fix_ts -> age = -50 s
    candidates = prior.propose(gm, lm)

    assert len(candidates) == 1 and candidates[0].source == "fiducial"  # clamped, still proposed
    assert len(warned) == 1  # exactly one warning, not per-frame spam
    assert warned[0]["msg"] == "fiducial fix age negative; clamping"
    assert warned[0]["age_s"] == -50.0  # the pre-clamp negative age, pre-rounded
