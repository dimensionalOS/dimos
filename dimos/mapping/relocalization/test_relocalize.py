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
import threading
import time
from typing import cast

from dimos_lcm.vision_msgs.ObjectHypothesisWithPose import ObjectHypothesisWithPose
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from pydantic import ValidationError
import pytest
from reactivex import Subject
from scipy.spatial.transform import Rotation
import yaml

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
    JudgeReport,
    refine_candidates,
    relocalize,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.apriltag_aggregation import matrix_from_pose7
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

        def is_due(self, now_s: float) -> bool:
            return True

        def on_fired(self, now_s: float) -> None:
            """No trigger state to ack; this stub only exercises the judge path."""

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
# FiducialPrior: composition + age + judge integrity. Gating and aggregation moved
# UPSTREAM into the detector's AggregateTagBursts (that is the only place corners_px,
# the reprojection error and the camera transform still exist), so every
# world_T_marker arriving here is one already-gated, already-aggregated pose and each
# observe() call IS one completed burst. The gate/aggregation tests live beside their
# code now: perception/fiducial/test_marker_transformer.py.
# ---------------------------------------------------------------------------


def test_fiducial_prior_unset_proposes_nothing() -> None:
    gm, lm, _ = _rect_room_scene(seed=11, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    assert FiducialPrior({}).propose(gm, lm) == []


def test_fiducial_prior_composes_map_T_world_from_one_aggregated_pose() -> None:
    """ONE aggregated world_T_marker composes straight to this tag's candidate,
    map_T_world = map_T_marker @ inv(world_T_marker_aggregated), exactly -- no second
    glimpse to wait for and no aggregation left to do, because the detector already did
    it. Non-identity on both sides so a transposed or swapped compose cannot pass."""
    gm, lm, _ = _rect_room_scene(seed=14, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    map_T_marker = _rigid(30.0, (5.0, -2.0, 0.5))
    world_T_marker_aggregated = _rigid(-40.0, (1.0, 0.5, 0.8))
    truth = map_T_marker @ np.linalg.inv(world_T_marker_aggregated)

    prior = FiducialPrior({7: map_T_marker})
    assert prior.observe(7, world_T_marker_aggregated) is None

    candidates = prior.propose(gm, lm)
    assert len(candidates) == 1 and candidates[0].source == "fiducial"
    np.testing.assert_allclose(candidates[0].T, truth, atol=1e-12)


def test_fiducial_prior_proposes_each_fix_exactly_once() -> None:
    """propose() CONSUMES: an aggregated fix is offered to the judge on one fire and gone
    on the next. Re-offering it would be the same measurement against a world that
    drifted further since, so it could only score worse than it just did."""
    gm, lm, _ = _rect_room_scene(seed=12, yaw_deg=30.0, t=(1.0, -2.0, 0.0))
    prior = FiducialPrior({7: np.eye(4)})
    prior.observe(7, np.eye(4))

    first = prior.propose(gm, lm)
    assert len(first) == 1 and first[0].source == "fiducial"

    assert prior.propose(gm, lm) == []  # consumed: nothing left to re-offer
    assert prior.is_due(0.0) is False  # and nothing left to ask a fire for


def test_fiducial_prior_propose_survives_concurrent_observe() -> None:
    """propose() must drain self._pending under the lock, not iterate the live
    dict. In the module, observe() runs on the detections transport thread while
    _try_relocalize -> propose() runs on the global_map transport thread; a NEW tag
    id landing mid-iteration otherwise raises "dictionary changed size during
    iteration", which the boundary except swallows -> a silently dropped reloc
    cycle. Writer sights fresh ids while the reader hammers propose(): the load
    case, since a scheduler that never preempts inside observe() never trips it.
    The exclusion itself is pinned deterministically in the test below."""
    gm, lm, _ = _rect_room_scene(seed=17, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    # Surveyed so the writer's ids compose instead of returning "unmapped_id": the
    # writer goes through the real observe(), the seam the detections thread uses.
    prior = FiducialPrior({tag_id: np.eye(4) for tag_id in range(1200)})
    for tag_id in range(50):  # a real iteration window for the writer to race into
        prior.observe(tag_id, np.eye(4))

    stop = threading.Event()
    errors: list[BaseException] = []

    def _writer() -> None:
        # Bounded on purpose: an unbounded writer grows _pending without limit, so
        # every propose() drain gets bigger and the test never finishes. A
        # fixed budget still overlaps the reader and reproduces the race.
        for tag_id in range(1000, 1200):
            if stop.is_set():
                return
            prior.observe(tag_id, np.eye(4))  # new key -> dict size changes

    writer = threading.Thread(target=_writer, name="detections_writer")
    writer.start()
    try:
        for _ in range(200):
            try:
                prior.propose(gm, lm)
            except RuntimeError as exc:  # "dictionary changed size during iteration"
                errors.append(exc)
                break
    finally:
        stop.set()
        writer.join(timeout=5.0)

    assert not writer.is_alive()
    assert errors == [], f"propose() raced with concurrent observe(): {errors!r}"


def test_fiducial_prior_observe_waits_for_the_drain() -> None:
    """Invariant: a sighting cannot enter the dict a fire is draining. The stress
    test above only catches the race when the scheduler happens to preempt inside
    observe()'s read-then-write; this pins the exclusion itself by holding the
    drain's lock, so a sighting arriving in that window lands on the NEXT fire
    instead of resizing the dict propose() is iterating."""
    prior = FiducialPrior({7: np.eye(4), 9: np.eye(4)})
    prior.observe(7, np.eye(4))
    landed = threading.Event()

    def _sighting() -> None:
        prior.observe(9, np.eye(4))
        landed.set()

    sighting = threading.Thread(target=_sighting, name="detections_sighting")
    try:
        with prior._pending_lock:  # the window propose() holds to swap the dict
            sighting.start()
            assert not landed.wait(timeout=0.2)  # blocked, not writing
            assert set(prior._pending) == {7}  # the dict being drained is untouched
        assert landed.wait(timeout=5.0)  # released -> the sighting proceeds
        assert set(prior._pending) == {7, 9}  # and is offered on the next fire
    finally:
        sighting.join(timeout=5.0)


def test_fiducial_prior_never_bypasses_judge() -> None:
    """A consistent-but-wrong aggregated fix (stale map, moved tag) 6.4 m off must
    lose to a correct candidate — the fiducial source gets no special treatment;
    the judge ranks on wall fitness only."""
    gm, lm, T_true = _rect_room_scene(seed=13, yaw_deg=45.0, t=(2.0, 1.0, 0.0))
    T_wrong = T_true.copy()
    T_wrong[:3, 3] += np.array([5.0, -4.0, 0.0])  # 6.4 m off
    # world_T_marker chosen so the aggregated map_T_world (map_T_marker == I) is T_wrong.
    world_T_marker = np.linalg.inv(T_wrong)

    fid = FiducialPrior({7: np.eye(4)})
    fid.observe(7, world_T_marker)

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
# FiducialPrior: unmapped-id rejection + per-tag propose fan-out
# ---------------------------------------------------------------------------


def test_fiducial_prior_rejects_unmapped_id_without_arming_a_fire() -> None:
    """An aggregated pose for a tag absent from the surveyed marker map is refused with
    'unmapped_id': there is no map_T_marker to compose against. It must also leave
    the burst counter alone -- an unsurveyed tag that armed the trigger would fire a
    relocalization with no candidate to judge, and relocalize_with_priors raises on
    an empty pool, once per burst, forever."""
    gm, lm, _ = _rect_room_scene(seed=16, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    prior = FiducialPrior({7: np.eye(4)})

    reason = prior.observe(99, np.eye(4))  # id 99 is not surveyed

    assert reason == "unmapped_id"
    assert prior.propose(gm, lm) == []
    assert prior.is_due(0.0) is False


def test_fiducial_prior_proposes_one_candidate_per_mapped_tag() -> None:
    """Two tags' aggregated poses yield two candidates -- one per tag, each the tag's own
    map_T_marker @ inv(world_T_marker) (identity world poses here, so each fix
    equals that tag's surveyed map_T_marker)."""
    gm, lm, _ = _rect_room_scene(seed=18, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    map_T_marker_7 = _rigid(30.0, (5.0, -2.0, 0.5))
    map_T_marker_9 = _rigid(-70.0, (-1.0, 3.0, 0.2))
    prior = FiducialPrior({7: map_T_marker_7, 9: map_T_marker_9})

    for tag in (7, 9):
        prior.observe(tag, np.eye(4))

    candidates = prior.propose(gm, lm)

    assert len(candidates) == 2
    assert {c.source for c in candidates} == {"fiducial"}
    got = sorted((round(c.T[0, 3], 6), round(c.T[1, 3], 6)) for c in candidates)
    want = sorted((round(T[0, 3], 6), round(T[1, 3], 6)) for T in (map_T_marker_7, map_T_marker_9))
    assert got == want


def test_fiducial_prior_drains_every_pending_tag_in_one_fire() -> None:
    """Two tags arriving before a fire BOTH propose on it, and the fire empties the
    pending set: a multi-tag pool is normal (the judge picks on wall fitness), and
    leaving either one behind would re-offer a drifted fix on the next fire."""
    gm, lm, _ = _rect_room_scene(seed=19, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    map_T_marker_9 = _rigid(-70.0, (-1.0, 3.0, 0.2))
    prior = FiducialPrior({7: np.eye(4), 9: map_T_marker_9})

    prior.observe(7, np.eye(4))
    prior.observe(9, np.eye(4))

    both = prior.propose(gm, lm)
    assert len(both) == 2
    assert prior.propose(gm, lm) == []  # one fire drained both


# ---------------------------------------------------------------------------
# relocalize_with_priors: empty-pool guard + proposal-census log line
# ---------------------------------------------------------------------------


def test_relocalize_with_priors_census_reports_per_source_counts(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The 'relocalize candidates' census log's per-source counts (a dict, sorted
    by source name) reflect each source's proposed count, independent of which
    candidate later wins the judge -- so a fiducial that proposed then LOST still
    shows counts['fiducial'] == 1. The census is an eval-only diagnostic: the same
    call logs NOTHING with verbose_eval_logging off (the operating default)."""
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

    _, _, source = relocalize_with_priors(
        gm, lm, [_MultiRansac(), _OneFiducial()], verbose_eval_logging=True
    )

    census = [kw for ev, kw in logged if ev == "relocalize candidates"]
    assert census == [{"counts": {"fiducial": 1, "ransac": 2}}]
    assert source == "ransac"  # the census counts a loser; ransac still wins the judge

    logged.clear()
    relocalize_with_priors(gm, lm, [_MultiRansac(), _OneFiducial()])
    assert [kw for ev, kw in logged if ev == "relocalize candidates"] == []


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
    full indexed set (relocalize.py:253 `pool = upright if upright else indexed`)
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
    """The per-candidate 'judge finalists' log is a multi-source-only EVAL
    diagnostic: under verbose_eval_logging it must stay silent when there is nothing
    to adjudicate -- sources=None, or a top-k drawn from a single source -- and fire,
    naming the finalists and the winner, only when >1 distinct source reaches the
    top-k. With verbose off (the operating default) it never fires at all."""
    gm, lm, T_true = _rect_room_scene(seed=44, yaw_deg=10.0, t=(0.7, 0.9, 0.0))
    good = T_true.copy()
    good[:3, 3] += np.array([0.03, -0.02, 0.0])
    other = _rigid(140.0, (5.0, -4.0, 0.0))

    # (a) sources=None -> the log is guarded off entirely.
    rec = _LogRecorder()
    monkeypatch.setattr(relocalize_mod, "logger", rec)
    refine_candidates(gm, lm, [good, other], verbose_eval_logging=True)
    assert not any(ev == "judge finalists" for ev, _ in rec.infos)

    # (b) sources present but a SINGLE distinct source in the top-k -> silent.
    rec = _LogRecorder()
    monkeypatch.setattr(relocalize_mod, "logger", rec)
    refine_candidates(
        gm, lm, [good, other], sources=["ransac", "ransac"], verbose_eval_logging=True
    )
    assert not any(ev == "judge finalists" for ev, _ in rec.infos)

    # (c) two distinct sources reach the top-k -> exactly one 'judge:' line,
    # naming both sources and marking the winner.
    rec = _LogRecorder()
    monkeypatch.setattr(relocalize_mod, "logger", rec)
    _, _, idx = refine_candidates(
        gm, lm, [good, other], sources=["fiducial", "ransac"], verbose_eval_logging=True
    )
    judge_lines = [kw for ev, kw in rec.infos if ev == "judge finalists"]
    assert len(judge_lines) == 1
    finalists = cast("list[tuple[str, float]]", judge_lines[0]["finalists"])
    assert {src for src, _fit in finalists} == {"fiducial", "ransac"}  # both named
    assert judge_lines[0]["winner"] == "fiducial"  # winner named via kwarg, not a '*'
    assert idx == 0  # the near-truth 'fiducial' candidate won

    # (d) same two-source pool with verbose off -> the operating default is silent.
    rec = _LogRecorder()
    monkeypatch.setattr(relocalize_mod, "logger", rec)
    refine_candidates(gm, lm, [good, other], sources=["fiducial", "ransac"])
    assert not any(ev == "judge finalists" for ev, _ in rec.infos)


def test_judge_report_margin_is_winner_minus_best_other_source() -> None:
    """The margin the quiet accept line carries: the winner's post-ICP wall fitness
    minus the BEST OTHER source's, computed off the judge's own finalists. A pool
    with one source has no rival to subtract, so margin is None -- never 0.0, which
    would read as a dead heat."""
    gm, lm, T_true = _rect_room_scene(seed=45, yaw_deg=10.0, t=(0.7, 0.9, 0.0))
    good = T_true.copy()
    good[:3, 3] += np.array([0.03, -0.02, 0.0])
    other = _rigid(140.0, (5.0, -4.0, 0.0))  # wrong room: near-zero wall fitness

    report = JudgeReport()
    _, fitness, idx = refine_candidates(
        gm, lm, [good, other], sources=["fiducial", "ransac"], report=report
    )
    assert idx == 0 and report.winner == "fiducial"
    best_ransac = max(fit for src, fit in report.finalists if src == "ransac")
    margin = report.margin
    assert margin is not None
    assert margin == pytest.approx(report.finalists[0][1] - best_ransac)
    assert margin > 0.0  # the near-truth candidate beat the wrong-room one

    # Single-source pool: nothing to compare against.
    solo = JudgeReport()
    refine_candidates(gm, lm, [good, other], sources=["ransac", "ransac"], report=solo)
    assert solo.winner == "ransac" and solo.margin is None
    assert fitness > 0.5


# ---------------------------------------------------------------------------
# RelocalizationModule: Config contract + pure helpers (no coordinator).
#
# These exercise module.py directly, never through a live/replay Module: the
# Config defaults, the per-prior accept bar + RANSAC point floor, the accept/reject
# log-line shape, marker-id parsing, and _on_aggregated_detections routing each entry's
# aggregated world_T_marker into the fiducial prior. Instances are built with object.__new__
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
    ransac_entry = next(
        (p for p in config.priors if isinstance(p, RansacPriorConfig)), RansacPriorConfig()
    )
    m._ransac_prior = RansacPrior(interval_s=ransac_entry.interval_s)
    m._fiducial_prior = None
    m._now_fn = time.monotonic  # a trigger test swaps this for a driven clock
    m._world_to_map = Subject()  # _on_local_map publishes accepted fixes onto it
    # Derived state __init__ builds (object.__new__ skips it): the per-source accept
    # bar and the RANSAC point floor.
    m._accept_threshold = {p.type: p.fitness_threshold for p in config.priors}
    m._ransac_min_local_points = ransac_entry.min_local_points
    m._last_fix_map_T_world = None  # no previous fix -> the jump guard is in acquisition
    m._last_fix_ts_s = 0.0
    m._last_local_map = None  # no cloud yet -> a burst cannot fire until one arrives
    return m


class _StubCloud:
    """Minimal PointCloud2 stand-in: __len__ (the point count _on_local_map reads
    for the RANSAC floor / the n_pts log field) and a `.pointcloud` sentinel handed
    straight through to the monkeypatched relocalize()."""

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


def test_accept_threshold_and_point_floor_resolve_per_prior() -> None:
    """__init__'s derived state: _accept_threshold maps each Candidate.source to its
    prior's fitness_threshold (per-prior override, base default otherwise), and
    _ransac_min_local_points reads the ransac entry's floor -- the two knobs that
    left Config for the priors."""
    m = _bare_module(
        Config(
            priors=[
                RansacPriorConfig(fitness_threshold=0.55, min_local_points=30_000),
                FiducialPriorConfig(fitness_threshold=0.35),
                LastPosePriorConfig(),  # base default 0.45
            ]
        )
    )
    assert m._accept_threshold == {"ransac": 0.55, "fiducial": 0.35, "last_pose": 0.45}
    assert m._ransac_min_local_points == 30_000


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
    _on_aggregated_detections skips the entry rather than inventing a tag id."""
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
    """A Detection3DArray carrying `dets` with header.stamp encoding `ts`."""
    arr = Detection3DArray()
    sec = int(ts)
    arr.header.stamp.sec = sec
    arr.header.stamp.nsec = round((ts - sec) * 1e9)
    arr.detections = dets
    arr.detections_length = len(dets)
    return arr


class _RecordingPrior:
    """FiducialPrior stand-in recording (marker_id, world_T_marker_aggregated) per
    observe -- lets _on_aggregated_detections' routing be asserted on its own."""

    def __init__(self) -> None:
        self.calls: list[tuple[int, np.ndarray]] = []

    def observe(self, marker_id: int, world_T_marker_aggregated: np.ndarray) -> None:
        self.calls.append((marker_id, world_T_marker_aggregated))


def test_on_aggregated_detections_routes_each_world_T_marker_into_prior() -> None:
    """Each parseable entry's bbox.center (the detector's aggregated world_T_marker) is
    composed via matrix_from_pose7 and handed to the fiducial prior; unparseable
    entries are skipped rather than routed. No timestamp travels with it -- the
    glimpse window that needed one lives upstream now."""
    prior = _RecordingPrior()
    m = _bare_module(Config(priors=[FiducialPriorConfig()]))
    m._fiducial_prior = prior  # type: ignore[assignment]

    det_a = _pose_detection(5, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    quat_b = tuple(Rotation.from_euler("z", 30.0, degrees=True).as_quat())
    det_b = _pose_detection(8, (-0.5, 0.25, 4.0), quat_b)  # type: ignore[arg-type]
    det_skip = _detection(det_id="not-a-tag")  # unparseable -> dropped
    arr = _detection_array([det_a, det_skip, det_b], ts=3.5)

    m._on_aggregated_detections(arr)

    assert [mid for mid, _ in prior.calls] == [5, 8]  # det_skip never routed
    for (_mid, world_T_marker_aggregated), det in (
        (prior.calls[0], det_a),
        (prior.calls[1], det_b),
    ):
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
        np.testing.assert_array_equal(world_T_marker_aggregated, expected)


def test_on_aggregated_detections_noop_without_fiducial_prior() -> None:
    """With the fiducial prior unbuilt (prior disabled / no marker_map_file),
    _on_aggregated_detections is a silent no-op on wire traffic -- returns None, no raise."""
    m = _bare_module(Config(priors=[RansacPriorConfig()]))
    m._fiducial_prior = None
    m._on_aggregated_detections(
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


def _accept_with_report(
    T: np.ndarray, fitness: float, source: str, finalists: list[tuple[str, float]]
):  # type: ignore[no-untyped-def]
    """Stand-in for _relocalize_with_priors that also fills the judge's report, the
    way the real judge does -- the margin path is invisible without it."""

    def fake(*args: object, report: JudgeReport | None = None, **kwargs: object):  # type: ignore[no-untyped-def]
        if report is not None:
            report.finalists = finalists
            report.winner = source
        return T, fitness, source

    return fake


def test_verbose_eval_logging_keeps_the_full_accept_and_reject_kwargs(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """--eval restores the full debugging trace: the accept line carries the whole
    kwarg set (published_t_m, which the eval parsers key on, is back), and the reject
    line's time_cost_s/n_pts ON TOP of the source/fitness/threshold the quiet reject
    already carries. margin belongs to the quiet line only."""
    T = _rigid(25.0, (1.5, -0.8, 0.05))
    config = Config(
        priors=[RansacPriorConfig(), LastPosePriorConfig(), FiducialPriorConfig()],
        verbose_eval_logging=True,
    )
    m = _bare_module(config)
    m._premap = _StubCloud(10)  # type: ignore[assignment]

    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(
        module_mod,
        "_relocalize_with_priors",
        _accept_with_report(T, 0.90, "fiducial", [("fiducial", 0.71), ("ransac", 0.53)]),
    )
    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]
    assert set(rec.infos[0][1]) == {
        "source",
        "fitness",
        "time_cost_s",
        "n_pts",
        "reloc_t_m",
        "tf_from",
        "tf_to",
        "published_t_m",
    }  # the full set is back under --eval; the console renderer sorts, so order is not asserted

    # 0.20 is under the fiducial entry's own 0.6 bar -- the reject the verbose branch exists for.
    monkeypatch.setattr(
        module_mod,
        "_relocalize_with_priors",
        _accept_with_report(T, 0.20, "fiducial", [("fiducial", 0.20)]),
    )
    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is None  # type: ignore[arg-type]
    event, kwargs = rec.warnings[0]
    assert event == "relocalize rejected"
    assert set(kwargs) == {"source", "fitness", "threshold", "time_cost_s", "n_pts"}
    assert kwargs["source"] == "fiducial"  # verbose names the refused prior too
    assert kwargs["fitness"] == 0.2 and kwargs["threshold"] == 0.6


def test_try_relocalize_gates_the_winner_on_its_own_source_threshold(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The module accept gate is PER-SOURCE: a fiducial fix at 0.40 clears a
    fiducial bar of 0.35 (accepted), while a ransac fix at 0.50 fails a ransac bar
    of 0.55 (rejected, the reject line naming the ransac bar) -- the SAME two
    fitnesses would flip under a single module gate, so this pins the per-source
    resolution end to end through _try_relocalize."""
    T = _rigid(12.0, (0.4, -0.3, 0.02))
    config = Config(
        priors=[
            RansacPriorConfig(fitness_threshold=0.55),
            FiducialPriorConfig(fitness_threshold=0.35),
        ],
    )
    m = _bare_module(config)
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._fiducial_prior = FiducialPrior({7: np.eye(4)})  # enables the entry

    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(
        module_mod, "_relocalize_with_priors", lambda *a, **k: (T, 0.40, "fiducial")
    )
    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]
    assert rec.infos and rec.infos[0][0] == "relocalize accepted"
    assert rec.infos[0][1]["source"] == "fiducial"
    assert not rec.warnings

    rec2 = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec2)
    monkeypatch.setattr(module_mod, "_relocalize_with_priors", lambda *a, **k: (T, 0.50, "ransac"))
    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is None  # type: ignore[arg-type]
    assert rec2.warnings and rec2.warnings[0][0] == "relocalize rejected"
    assert rec2.warnings[0][1]["source"] == "ransac"
    assert rec2.warnings[0][1]["fitness"] == 0.5 and rec2.warnings[0][1]["threshold"] == 0.55
    assert not rec2.infos


def test_try_relocalize_solo_path_gates_on_the_ransac_entry_bar(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The plain (ransac-only) path returns no winning_source, so the gate resolves
    to the RANSAC entry's OWN bar -- not a hardcoded default. A ransac entry set to
    0.55 rejects a 0.5 fix, and the reject line names both 0.55 and the prior that
    set it: a reject prints a per-prior threshold, so it always names the owner even
    where the ACCEPT line omits source= (single-source path)."""
    T = _rigid(0.0, (0.0, 0.0, 0.0))
    m = _bare_module(Config(priors=[RansacPriorConfig(fitness_threshold=0.55)]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]

    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(module_mod, "_relocalize", lambda *a, **k: (T, 0.5))

    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is None  # type: ignore[arg-type]
    event, kwargs = rec.warnings[0]
    assert event == "relocalize rejected"
    assert list(kwargs) == ["source", "fitness", "threshold"]
    assert kwargs["source"] == "ransac"  # the entry whose bar refused it
    assert kwargs["fitness"] == 0.5 and kwargs["threshold"] == 0.55


def _tracking_module(monkeypatch, clock: dict[str, float]) -> RelocalizationModule:  # type: ignore[no-untyped-def]
    """A module that has already ACCEPTED one fix at the identity, so the next call
    is a tracking fix with a previous one to be compared against. The clock is driven,
    never slept on, so the jump budgets (per second) are exact."""
    m = _bare_module(Config(priors=[RansacPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: clock["now"]
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())
    monkeypatch.setattr(
        module_mod, "_relocalize", lambda *a, **k: (_rigid(0.0, (0.0, 0.0, 0.0)), 0.9)
    )
    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]
    return m


def test_jump_guard_rejects_a_180_deg_flip_metres_away_from_the_last_fix(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The failure this guard exists for: a fix 18 m and 180 deg from the previous
    one, 0.2 s later, at a fitness that CLEARS the accept gate. The robot cannot walk
    18 m in 0.2 s, so the fix is a mis-localization: no TF is published and the
    warning carries the two numbers that refused it."""
    clock = {"now": 100.0}
    m = _tracking_module(monkeypatch, clock)

    clock["now"] = 100.2  # a 0.2 s gap still gets one second's worth: 5 m and 45 deg
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(
        module_mod, "_relocalize", lambda *a, **k: (_rigid(180.0, (18.0, 0.0, 0.0)), 0.9)
    )

    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is None  # type: ignore[arg-type]
    assert not rec.infos  # never logged as an accept
    event, kwargs = rec.warnings[0]
    assert event == "relocalize jump rejected"
    assert kwargs["jump_m"] == 18.0 and kwargs["yaw_deg"] == 180.0
    assert kwargs["dt_s"] == 0.2 and kwargs["source"] == "ransac"


def test_jump_guard_passes_a_normal_sub_metre_drift_correction(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The guard fires on gross violations ONLY: a 0.4 m, 3 deg correction one RANSAC
    cadence (2 s) after the last fix is exactly what relocalization is for, and it
    publishes -- well inside that gap's 10 m / 90 deg budget."""
    clock = {"now": 100.0}
    m = _tracking_module(monkeypatch, clock)

    clock["now"] = 102.0
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(
        module_mod, "_relocalize", lambda *a, **k: (_rigid(3.0, (0.4, 0.0, 0.0)), 0.9)
    )

    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]
    assert not rec.warnings
    assert rec.infos[0][0] == "relocalize accepted"


def test_jump_guard_never_blocks_the_first_fix(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """Acquisition is unguarded: with no previous fix there is nothing to be
    implausible against, so even a 40 m first fix publishes. A guard that blocked the
    first fix would leave a robot that boots far from the map origin unable to
    localize at all."""
    clock = {"now": 100.0}
    m = _bare_module(Config(priors=[RansacPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: clock["now"]

    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)
    monkeypatch.setattr(
        module_mod, "_relocalize", lambda *a, **k: (_rigid(170.0, (40.0, -12.0, 0.0)), 0.9)
    )

    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]
    assert not rec.warnings
    assert rec.infos[0][0] == "relocalize accepted"


def test_marker_id_from_detection_colon_label_with_nonnumeric_tail_returns_none() -> None:
    """A class label carrying a ':' but a non-numeric tail ('DICT:abc') is NOT a
    marker id: the digit check on the tail fails, the scan falls through, and the
    sighting is dropped rather than routed to the prior under a garbage id."""
    det = _detection(det_id="", class_id="DICT_APRILTAG_36h11:abc")
    assert RelocalizationModule._marker_id_from_detection(det) is None


def test_try_relocalize_appends_fiducial_prior_to_judge_pool(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """_try_relocalize judges exactly the pool it is handed: given the built
    fiducial prior instance alongside RANSAC, both reach the judge and the winning
    source it returns is reported -- the fiducial candidate is judged, never
    bypassed."""
    T = _rigid(15.0, (0.4, -0.3, 0.02))
    m = _bare_module(Config(priors=[RansacPriorConfig(), FiducialPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    sentinel = FiducialPrior({7: np.eye(4)})
    m._fiducial_prior = sentinel

    seen_pools: list[list[RelocPrior]] = []

    def _fake(global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object):  # type: ignore[no-untyped-def]
        seen_pools.append(priors)
        return T, 0.90, "fiducial"

    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())
    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)

    tf = m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects())  # type: ignore[arg-type]

    assert tf is not None
    assert len(seen_pools) == 1
    assert sentinel in seen_pools[0]  # the built fiducial prior reached the judge
    assert any(isinstance(p, RansacPrior) for p in seen_pools[0])


def test_try_relocalize_returns_none_and_logs_on_relocalize_crash(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A crash inside the solve is caught at the module boundary: _try_relocalize
    logs it via logger.exception and returns None (no TF published) rather than
    letting the exception escape the reactive subscription."""
    m = _bare_module(Config(priors=[RansacPriorConfig()]))
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

    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is None  # type: ignore[arg-type]
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
    assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is None  # type: ignore[arg-type]


def test_maybe_log_skip_warns_for_starved_ransac_then_throttles(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A starved-RANSAC frame logs one throttled 'ransac reloc skipped' naming the
    count and the ransac floor; a second inside the 5 s window stays silent, and it
    fires again once the window passes. The point CHECK lives in _on_local_map now;
    _maybe_log_skip is just the throttled logger it calls when it drops RANSAC."""
    clock = {"now": 100.0}
    monkeypatch.setattr("dimos.mapping.relocalization.module.time.monotonic", lambda: clock["now"])
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)

    m = _bare_module(Config(priors=[RansacPriorConfig(min_local_points=50_000)]))

    m._maybe_log_skip(cast("PointCloud2", _StubCloud(10_000)))
    assert len(rec.warnings) == 1
    event, kwargs = rec.warnings[0]
    assert event == "ransac reloc skipped"
    assert kwargs["n_pts"] == 10000
    assert kwargs["min_local_points"] == 50000

    clock["now"] = 102.0  # 2 s later, still inside the 5 s throttle window
    m._maybe_log_skip(cast("PointCloud2", _StubCloud(9_000)))
    assert len(rec.warnings) == 1  # throttled: no second warning

    clock["now"] = 106.0  # >5 s after the first -> the throttle window has passed
    m._maybe_log_skip(cast("PointCloud2", _StubCloud(9_000)))
    assert len(rec.warnings) == 2


def test_start_fiducial_prior_noops_without_marker_map_file(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """Fiducial prior enabled but no marker_map_file surveyed: _start_fiducial_prior
    warns and leaves the prior unbuilt (None) -- it never fabricates a map, mirroring
    start()'s no-map_file disable, so _on_aggregated_detections stays a no-op afterwards."""
    m = _bare_module(Config(priors=[FiducialPriorConfig(marker_map_file=None)]))
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)

    m._start_fiducial_prior()

    assert m._fiducial_prior is None
    assert len(rec.warnings) == 1
    assert "no marker_map_file" in rec.warnings[0][0]  # unchanged plain-string warning event


def test_init_leaves_priors_in_unset_state(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A freshly constructed module starts with no premap and no fiducial prior
    (both built later in start()), a zeroed skip-log clock, a real LastPosePrior
    ready to carry a seed forward, and ONE RansacPrior carrying the configured
    interval -- the documented cold-start contract. The ransac prior is built here,
    not per frame, because its interval timer is the trigger: a per-frame instance
    would restart the timer every frame and fire forever."""
    monkeypatch.setattr(
        "dimos.mapping.relocalization.module.Module.__init__",
        lambda self, **kwargs: setattr(self, "config", Config(**kwargs)),
    )
    m = RelocalizationModule(priors=[RansacPriorConfig(interval_s=7.5)])
    assert m._premap is None
    assert m._fiducial_prior is None
    assert m._last_skip_log == 0.0
    assert isinstance(m._last_pose_prior, LastPosePrior)
    assert isinstance(m._ransac_prior, RansacPrior)
    assert m._ransac_prior._interval_s == 7.5
    assert m._enabled_prior_objects() == [m._ransac_prior]  # the SAME instance every frame


class _AggregatedDetectionsStub:
    """Stands in for the `aggregated_detections` In-stream: observable().subscribe(cb)
    returns a disposable sentinel, letting _start_fiducial_prior's wiring run without
    a live coordinator. Only the framework seam is stubbed; FiducialPrior is real."""

    def observable(self) -> _AggregatedDetectionsStub:
        return self

    def subscribe(self, callback: object) -> str:
        return "aggregated_detections_disposable"


class _FakeSurveyedTransform:
    """A load_marker_map entry: only .to_matrix() is read by _start_fiducial_prior."""

    def to_matrix(self) -> np.ndarray:
        return np.eye(4)


def test_start_fiducial_prior_loads_map_and_wires_aggregated_detections(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """With a marker_map_file present, _start_fiducial_prior resolves + loads the
    survey into map_T_marker matrices, builds the real FiducialPrior, subscribes the
    detector's aggregated_detections stream into it (registering the disposable), and logs
    the marker count -- the fiducial source is now live and age-gated in the judge."""
    m = _bare_module(
        Config(priors=[FiducialPriorConfig(marker_map_file="survey", marker_length_m=0.1)])
    )
    m.aggregated_detections = _AggregatedDetectionsStub()  # type: ignore[assignment]
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
    assert registered == ["aggregated_detections_disposable"]  # aggregated stream wired in
    assert len(rec.infos) == 1
    event, kwargs = rec.infos[0]
    assert event == "fiducial prior enabled"
    assert kwargs["n_markers"] == 2


def test_start_fiducial_prior_loads_a_yaml_survey(tmp_path: Path) -> None:
    """A `.yaml` marker survey loads through the real path (real resolve_named_path,
    real load_marker_map) into map_T_marker matrices.

    The marker-map derivation pipeline writes yaml (test_unitree_go2_fiducial_
    relocalization_replay.py's marker_map_file fixture, `<rec>.markers.yaml`), so a
    deployment pointing marker_map_file at that survey must start, not fail parsing."""
    survey = tmp_path / "site_markers.yaml"
    survey.write_text(
        yaml.safe_dump(
            {"markers": {13: {"translation": [1.0, 2.0, 3.0], "rotation": [0.0, 0.0, 0.0, 1.0]}}}
        )
    )
    m = _bare_module(
        Config(priors=[FiducialPriorConfig(marker_map_file=str(survey), marker_length_m=0.1)])
    )
    m.aggregated_detections = _AggregatedDetectionsStub()  # type: ignore[assignment]
    m.register_disposable = lambda d: None  # type: ignore[assignment,method-assign]

    m._start_fiducial_prior()

    assert isinstance(m._fiducial_prior, FiducialPrior)
    assert set(m._fiducial_prior._map_T_marker) == {13}
    assert m._fiducial_prior._map_T_marker[13][:3, 3].tolist() == [1.0, 2.0, 3.0]


@pytest.mark.parametrize(
    ("marker_map_file", "expected_suffix"),
    [
        ("site_markers", ".json"),  # bare name -> the default survey format
        ("site_markers.json", ""),  # already stated -> unchanged
        ("site_markers.yaml", ""),  # never "site_markers.yaml.json"
        ("site_markers.yml", ""),
    ],
)
def test_marker_map_resolution_keeps_a_stated_survey_suffix(
    monkeypatch, marker_map_file: str, expected_suffix: str
) -> None:  # type: ignore[no-untyped-def]
    """Invariant: only a bare marker_map_file gets the .json default appended.
    resolve_named_path appends its suffix when the file isn't local (a data-dir
    name), so passing .json unconditionally would look up '<survey>.yaml.json' --
    a name nothing writes -- and a yaml survey would fail at start()."""
    m = _bare_module(Config(priors=[FiducialPriorConfig(marker_map_file=marker_map_file)]))
    m.aggregated_detections = _AggregatedDetectionsStub()  # type: ignore[assignment]
    m.register_disposable = lambda d: None  # type: ignore[assignment,method-assign]
    seen: list[tuple[str, str]] = []

    def _record(name: str, suffix: str) -> str:
        seen.append((name, suffix))
        return name

    monkeypatch.setattr(module_mod, "resolve_named_path", _record)
    monkeypatch.setattr(module_mod, "load_marker_map", lambda path: {})

    m._start_fiducial_prior()

    assert seen == [(marker_map_file, expected_suffix)]


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
    m.aggregated_detections = _AggregatedDetectionsStub()  # type: ignore[assignment]
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
    with pytest.raises(
        InsufficientWallEvidenceError,
        match=r"insufficient wall evidence: submap walls=\d+, map walls=\d+",
    ):
        refine_candidates(_pcd(room_pts), _pcd(local_pts), [T_true.copy()])


# ---------------------------------------------------------------------------
# MIN_WALL_POINTS: the exact refuse/solve boundary (relocalize.py:280 guard
# `n_src_walls < MIN_WALL_POINTS or n_tgt_walls < MIN_WALL_POINTS`). The
# floors-only tests above cover 0 walls; this
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
# Per-prior TRIGGERS. The relocalization cycle is no longer a module-level
# throttle: each prior says when it wants a fire (is_due) and acks the fire
# that answered it (on_fired), and the module runs one INDEPENDENT relocalization
# per prior that asked. RANSAC paces itself because a global search waits on
# nothing; the fiducial prior asks the moment an aggregated tag pose lands, so a tag fix
# publishes at the tag's latency, never behind a seconds-long RANSAC solve.
# Clocks are passed in (is_due(now_s), the module's _now_fn) so every case below
# is driven, not slept through.
# ---------------------------------------------------------------------------


def test_ransac_prior_fires_on_its_interval_and_not_before() -> None:
    """RansacPrior wants a fire immediately (nothing has run yet), then not again
    until interval_s has fully elapsed since the fire it acked: at 1.999 s it is
    silent, at exactly 2.0 s it asks again. This IS the 2 s cadence the module used
    to throttle at, now owned by the prior."""
    prior = RansacPrior(interval_s=2.0)

    assert prior.is_due(100.0) is True  # cold start: relocalize on frame one
    prior.on_fired(100.0)

    assert prior.is_due(100.0) is False
    assert prior.is_due(101.999) is False
    assert prior.is_due(102.0) is True  # exactly interval_s -> due

    prior.on_fired(102.0)
    assert prior.is_due(103.999) is False
    assert prior.is_due(104.0) is True


def test_ransac_prior_interval_comes_from_its_config_entry() -> None:
    """The cadence knob lives on RansacPriorConfig (it left the module Config), and
    a non-default interval_s is what the prior actually times against."""
    assert RansacPriorConfig().interval_s == 2.0  # unchanged default cadence, s
    prior = RansacPrior(interval_s=RansacPriorConfig(interval_s=10.0).interval_s)
    prior.on_fired(0.0)
    assert prior.is_due(9.999) is False
    assert prior.is_due(10.0) is True


def test_last_pose_prior_never_asks_for_a_fire() -> None:
    """LastPosePrior is a seed, not a source: re-judging the pose it just published
    announces nothing new, so it never triggers a fire on its own no matter how
    much the clock advances or whether it holds a seed."""
    prior = LastPosePrior()
    assert prior.is_due(0.0) is False
    prior.update(np.eye(4))
    assert prior.is_due(1e9) is False
    prior.on_fired(1e9)  # acking a fire it did not ask for is a harmless no-op
    assert prior.is_due(2e9) is False


def _burst(prior: FiducialPrior, marker_id: int, fix_T: np.ndarray | None = None) -> None:
    """One aggregated pose for one mapped marker. The detector publishes exactly one per
    (marker, visit), so a single observe() IS one completed burst here. Marker maps
    are identity in these tests, so observing inv(fix_T) composes map_T_world ==
    fix_T; the default identity pose leaves it identity."""
    prior.observe(marker_id, np.eye(4) if fix_T is None else np.linalg.inv(fix_T))


def test_fiducial_prior_asks_for_one_fire_per_aggregated_pose() -> None:
    """Every arriving aggregated pose IS a completed burst, so it asks for exactly one
    fire and the PROPOSE consumes it. Trigger and payload are one fact -- pending is
    what is_due reports -- so the prior cannot ask for a fire it has no candidate
    for, nor stay silent while holding one."""
    prior = FiducialPrior({7: np.eye(4)})
    gm, lm, _ = _rect_room_scene(seed=20, yaw_deg=0.0, t=(0.0, 0.0, 0.0))

    assert prior.is_due(100.0) is False  # nothing seen yet

    _burst(prior, 7)
    assert prior.is_due(100.0) is True
    prior.on_fired(100.0)
    assert prior.is_due(100.0) is True  # the ack alone must NOT drop the estimate
    prior.propose(gm, lm)
    assert prior.is_due(100.0) is False  # the propose did

    _burst(prior, 7)  # the tag's next visit: its own fire
    assert prior.is_due(100.0) is True


def test_fiducial_declined_fire_keeps_its_estimate() -> None:
    """A due prior the module declines to fire (no cloud cached yet) runs neither
    on_fired nor propose, so its estimate survives and the next real fire proposes
    it. Losing it here would drop the very first tag fix of a run -- exactly the
    acquisition the fiducial preset exists for."""
    gm, lm, _ = _rect_room_scene(seed=21, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    map_T_marker = _rigid(30.0, (5.0, -2.0, 0.5))
    prior = FiducialPrior({7: map_T_marker})

    _burst(prior, 7)
    for _declined in range(3):  # the module keeps declining; nothing acks, nothing drains
        assert prior.is_due(100.0) is True

    candidates = prior.propose(gm, lm)
    assert len(candidates) == 1
    np.testing.assert_allclose(candidates[0].T, map_T_marker, atol=1e-12)


def test_fiducial_prior_holds_each_markers_burst_separately() -> None:
    """Two markers' aggregated poses are two independent pending fixes: they ride the
    same fire (each is its own candidate for the judge) and the fire clears both."""
    gm, lm, _ = _rect_room_scene(seed=22, yaw_deg=0.0, t=(0.0, 0.0, 0.0))
    prior = FiducialPrior({7: np.eye(4), 9: np.eye(4)})

    _burst(prior, 7)
    _burst(prior, 9)
    assert prior.is_due(100.0) is True
    assert set(prior._pending) == {7, 9}

    assert len(prior.propose(gm, lm)) == 2
    assert prior.is_due(100.0) is False  # both served by that one fire


# ---------------------------------------------------------------------------
# Module dispatch: _on_local_map fires ONE independent relocalization per prior
# that asked, judging that prior's candidates alone. The point of the split is
# cost -- a RANSAC solve runs 4.4-23 s on the trial's go2 recordings, so a tag
# fix must never queue behind one. A pool of one is the expected shape, and
# refine_candidates is the validator that keeps it honest.
# ---------------------------------------------------------------------------


def _scene_cloud(pcd: o3d.geometry.PointCloud, n_pts: int) -> _StubCloud:
    """A _StubCloud whose `.pointcloud` is a REAL o3d cloud, so a test can drive
    _try_relocalize through the real judge instead of a monkeypatched solve."""
    cloud = _StubCloud(n_pts)
    cloud.pointcloud = pcd
    return cloud


def _fiducial_module(config: Config, fix_T: np.ndarray | None = None) -> RelocalizationModule:
    """A module with the real FiducialPrior wired (marker map identity). ``fix_T``
    seeds ONE pending fix through the real observe(), which arms the trigger exactly
    as a live burst does -- pending IS the trigger now, so a module holding a fix is
    a module due to fire. Without it the prior is idle until a test bursts it."""
    m = _bare_module(config)
    m._fiducial_prior = FiducialPrior({7: np.eye(4)})
    if fix_T is not None:
        _burst(m._fiducial_prior, 7, fix_T)
    return m


def _fired_pool(priors: list[RelocPrior]) -> list[str]:
    """Name the pool a fire judged, and consume what the real judge consumes:
    relocalize_with_priors asks every prior for candidates, and that ask is what
    clears the fiducial's pending fix. A stub skipping it leaves the trigger armed
    and re-fires the same fix forever. Only the fiducial prior is asked -- RansacPrior
    .propose would run a real FPFH search against these tests' stub clouds."""
    for prior in priors:
        if isinstance(prior, FiducialPrior):
            prior.propose(o3d.geometry.PointCloud(), o3d.geometry.PointCloud())
    return [prior.name for prior in priors]


def _aggregated_burst(marker_id: int) -> Detection3DArray:
    """The wire form of ONE completed burst: a single-entry aggregated_detections array
    whose bbox.center is an identity world_T_marker."""
    det = _pose_detection(marker_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    return _detection_array([det], ts=1.0)


def _judged_clouds(monkeypatch, m: RelocalizationModule) -> list[object]:  # type: ignore[no-untyped-def]
    """Record the local_map each fire hands the judge, draining the pool as the real
    relocalize_with_priors does. Returns the list the test asserts on."""
    judged: list[object] = []

    def _fake(global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object):  # type: ignore[no-untyped-def]
        judged.append(local_map)
        return np.eye(4), 0.90, _fired_pool(priors)[0]

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: 100.0
    return judged


def test_completed_burst_fires_against_the_cached_cloud_with_no_new_frame(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A tag burst relocalizes from the detections callback itself, judged against
    the last global_map received -- no new cloud arrives here at all. Waiting for one
    would be dead time on acquisition: the tag candidate is composed from the marker
    alone, and global_map ACCUMULATES in the world frame, so the cached cloud scores
    wall fitness as well as the next one would."""
    m = _fiducial_module(Config(priors=[FiducialPriorConfig()]))
    judged = _judged_clouds(monkeypatch, m)

    cached = _StubCloud(60_000)
    m._on_local_map(cast("PointCloud2", cached))
    assert judged == []  # a cloud with no tag pending fires nothing

    m._on_aggregated_detections(_aggregated_burst(7))

    assert judged == [cached.pointcloud]  # fired on the cached cloud, no new frame
    assert m._fiducial_prior is not None
    assert m._fiducial_prior.is_due(100.0) is False  # that fire consumed the estimate


def test_burst_before_the_first_cloud_stays_pending_and_fires_on_it(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The floor of the burst path: with no global_map yet the judge has nothing to
    score against, so the burst does not fire. It is left UNACKED and undrained --
    the same decline path a starved RANSAC cycle takes -- so the estimate survives
    and the first cloud to arrive publishes it."""
    m = _fiducial_module(Config(priors=[FiducialPriorConfig()]))
    judged = _judged_clouds(monkeypatch, m)

    m._on_aggregated_detections(_aggregated_burst(7))
    assert judged == []  # no cloud cached -> nothing to judge against
    assert m._fiducial_prior is not None
    assert m._fiducial_prior.is_due(100.0) is True  # declined, so the estimate survived

    first = _StubCloud(60_000)
    m._on_local_map(cast("PointCloud2", first))

    assert judged == [first.pointcloud]
    assert m._fiducial_prior.is_due(100.0) is False


def test_burst_path_never_touches_the_ransac_search_or_its_timer(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """RANSAC stays cloud-driven and untouched by the new path: it GENERATES its
    candidates from the cloud in hand, so it fires only from _on_local_map. A
    burst-triggered fire in between runs neither RANSAC entry point and does not ack
    RANSAC's interval, so the sweep still lands on the first cloud after it is due."""
    clock = {"now": 100.0}
    m = _fiducial_module(Config(priors=[RansacPriorConfig(interval_s=2.0), FiducialPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: clock["now"]
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())

    def _no_ransac(*args: object, **kwargs: object) -> object:
        raise AssertionError("a burst-triggered fire must not run a RANSAC search")

    monkeypatch.setattr(priors_mod, "generate_ransac_candidates", _no_ransac)
    monkeypatch.setattr(module_mod, "_relocalize", _no_ransac)

    fires: list[list[str]] = []

    def _fake(global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object):  # type: ignore[no-untyped-def]
        fires.append(_fired_pool(priors))
        return np.eye(4), 0.90, priors[0].name

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)

    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))  # cold start: RANSAC sweeps
    assert fires == [["ransac"]]

    clock["now"] = 101.0  # RANSAC has 1 s of its interval left
    m._on_aggregated_detections(_aggregated_burst(7))
    assert fires == [["ransac"], ["fiducial"]]  # tag alone, off the cached cloud

    clock["now"] = 102.0  # interval elapsed: the sweep the burst did not consume
    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert fires == [["ransac"], ["fiducial"], ["ransac"]]


def test_fiducial_fire_never_runs_ransac_candidate_generation(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """THE point of per-prior triggers: a burst-triggered fire judges the tag's
    candidates alone and never touches the RANSAC global search. Both RANSAC entry
    points are booby-trapped -- generate_ransac_candidates and the plain
    relocalize() path -- and neither may be reached while RANSAC's own interval has
    not elapsed."""
    clock = {"now": 100.0}
    m = _fiducial_module(
        Config(priors=[RansacPriorConfig(interval_s=2.0), FiducialPriorConfig()]),
        fix_T=_rigid(10.0, (0.2, 0.1, 0.0)),
    )
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: clock["now"]

    def _no_ransac(*args: object, **kwargs: object) -> object:
        raise AssertionError("a fiducial-triggered fire must not run a RANSAC search")

    monkeypatch.setattr(priors_mod, "generate_ransac_candidates", _no_ransac)
    monkeypatch.setattr(module_mod, "_relocalize", _no_ransac)
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())

    pools: list[list[str]] = []

    def _fake(global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object):  # type: ignore[no-untyped-def]
        pools.append(_fired_pool(priors))
        return np.eye(4), 0.90, "fiducial"

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)

    # RANSAC swept 1 s ago, so its 2 s interval has 1 s to run: the burst is the
    # only live trigger, and the fire it causes must stay clear of both traps.
    m._ransac_prior.on_fired(99.0)
    _burst(m._fiducial_prior, 7)
    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))

    assert pools == [["fiducial"]]  # one fire, tag candidates only, no RANSAC


def test_triggers_are_independent_timer_fires_ransac_burst_fires_fiducial(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """Neither prior can hold the other up. With no burst at all, advancing the
    clock past interval_s fires RANSAC alone; with the timer nowhere near due, a
    completed burst fires the fiducial alone. Once a tag prior is configured every
    fire goes through the judge (the plain relocalize() path is trapped), so both
    kinds of fire stay attributed in the log and the eval census."""
    clock = {"now": 100.0}
    m = _fiducial_module(Config(priors=[RansacPriorConfig(interval_s=2.0), FiducialPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: clock["now"]
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())

    fires: list[list[str]] = []

    def _fake_priors(  # type: ignore[no-untyped-def]
        global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object
    ):
        fires.append(_fired_pool(priors))
        return np.eye(4), 0.90, priors[0].name

    def _fake_ransac(*args: object, **kwargs: object) -> tuple[np.ndarray, float]:
        raise AssertionError("a configured tag prior sends every fire through the judge")

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake_priors)
    monkeypatch.setattr(module_mod, "_relocalize", _fake_ransac)

    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))  # cold start fires RANSAC
    assert fires == [["ransac"]]

    fires.clear()
    clock["now"] = 100.5  # timer not due, no burst -> nothing at all
    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert fires == []

    fires.clear()
    _burst(m._fiducial_prior, 7)  # burst only, timer still 1.5 s away
    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert fires == [["fiducial"]]

    fires.clear()
    clock["now"] = 102.5  # timer due, burst already served -> RANSAC only
    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert fires == [["ransac"]]


def test_pool_of_one_still_clears_the_real_judge_and_the_fitness_gate(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """A lone fiducial candidate is VALIDATED, not waved through: it runs the real
    refine_candidates and then the module's fitness_threshold. The truth candidate
    scores above the gate and publishes a TF; a candidate 5 m into the wrong room
    scores below it and is rejected, with no TF -- passing the gates is the safety
    property, beating a rival never was."""
    global_map, local_map, T_true = _rect_room_scene(seed=31, yaw_deg=12.0, t=(0.7, 0.4, 0.01))
    config = Config(priors=[FiducialPriorConfig()])

    good = _fiducial_module(config, fix_T=T_true.copy())
    good._premap = _scene_cloud(global_map, 20_000)  # type: ignore[assignment]
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)

    tf = good._try_relocalize(  # type: ignore[arg-type]
        _scene_cloud(local_map, 20_000), good._enabled_prior_objects()
    )
    assert tf is not None
    assert rec.infos and rec.infos[0][0] == "relocalize accepted"
    assert rec.infos[0][1]["source"] == "fiducial"  # one source, still attributed
    assert rec.infos[0][1]["fitness"] >= FiducialPriorConfig().fitness_threshold

    wrong = T_true.copy()
    wrong[:3, 3] += np.array([5.0, 5.0, 0.0])  # wrong room: ~0 wall inliers at FINE_VOXEL
    bad = _fiducial_module(config, fix_T=wrong)
    bad._premap = _scene_cloud(global_map, 20_000)  # type: ignore[assignment]
    rec2 = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec2)

    assert (
        bad._try_relocalize(  # type: ignore[arg-type]
            _scene_cloud(local_map, 20_000), bad._enabled_prior_objects()
        )
        is None
    )
    assert rec2.warnings and rec2.warnings[0][0] == "relocalize rejected"
    assert rec2.warnings[0][1]["fitness"] < FiducialPriorConfig().fitness_threshold
    assert not rec2.infos


def test_fiducial_only_preset_fires_on_a_burst_with_no_periodic_timer(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """The fiducial-only preset carries NO ransac entry, so nothing periodic can
    trigger it. Frames stream by with no fire however far the clock runs; the
    completed burst is what fires it -- burst-only, and not deadlocked into never
    firing."""
    clock = {"now": 100.0}
    m = _fiducial_module(Config(priors=[FiducialPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: clock["now"]
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())

    fires: list[list[str]] = []

    def _fake(global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object):  # type: ignore[no-untyped-def]
        fires.append(_fired_pool(priors))
        return np.eye(4), 0.90, "fiducial"

    def _no_ransac(*args: object, **kwargs: object) -> object:
        raise AssertionError("the fiducial-only preset has no RANSAC path")

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)
    monkeypatch.setattr(module_mod, "_relocalize", _no_ransac)
    monkeypatch.setattr(priors_mod, "generate_ransac_candidates", _no_ransac)

    for tick in range(5):  # a minute of frames, no tag: no fire, no timer to save it
        clock["now"] = 100.0 + 15.0 * tick
        m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert fires == []

    _burst(m._fiducial_prior, 7)
    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert fires == [["fiducial"]]

    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))  # edge consumed
    assert fires == [["fiducial"]]


def test_stale_reloc_interval_s_raises_naming_its_new_home(tmp_path: Path) -> None:
    """reloc_interval_s left the module Config for the ransac prior entry. An old
    config, and the `-o relocalizationmodule.reloc_interval_s=...` an operator
    still has in their shell history, must fail loudly naming where the cadence
    went -- accepted-and-ignored would run the robot at a cadence nobody chose."""
    with pytest.raises(ValidationError, match="reloc_interval_s moved onto the ransac prior entry"):
        Config(priors=[RansacPriorConfig()], reloc_interval_s=5.0)  # type: ignore[call-arg]

    # The real CLI overlay path: load_config_args pre-validates -o and tolerates
    # only "missing" errors, so this stale key raises there rather than at deploy.
    bp = RelocalizationModule.blueprint(priors=[RansacPriorConfig()])
    key = config_key(bp.blueprints[0].name)
    with pytest.raises(ValidationError, match="RansacPriorConfig"):
        load_config_args(bp.config(), [f"{key}.reloc_interval_s=5"], tmp_path / "no_such_config")


def test_moved_prior_fields_raise_naming_their_new_home() -> None:
    """fitness_threshold and min_local_points left the module Config for the prior
    entries. BaseConfig is extra='forbid', so a stale
    `-o relocalizationmodule.fitness_threshold=` must fail loudly naming the new home
    -- an opaque 'extra inputs' error would not say the accept bar is now per prior."""
    with pytest.raises(ValidationError, match="fitness_threshold moved onto each prior entry"):
        Config(priors=[RansacPriorConfig()], fitness_threshold=0.5)  # type: ignore[call-arg]
    with pytest.raises(ValidationError, match="min_local_points moved onto the ransac prior entry"):
        Config(priors=[RansacPriorConfig()], min_local_points=100)  # type: ignore[call-arg]


def test_last_pose_seed_rides_along_with_the_prior_that_fired(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """LastPosePrior has no trigger, so a fire it cannot ask for still carries it:
    an enabled last_pose entry joins whichever prior fired, contributing its one
    carried-forward candidate. Dropping it instead would silently disable a
    configured prior; it never pulls a RANSAC search into a tag fire."""
    clock = {"now": 100.0}
    m = _fiducial_module(Config(priors=[LastPosePriorConfig(), FiducialPriorConfig()]))
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: clock["now"]
    m._last_pose_prior.update(np.eye(4))
    monkeypatch.setattr(module_mod, "logger", _ModuleLogRecorder())

    pools: list[list[str]] = []

    def _fake(global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object):  # type: ignore[no-untyped-def]
        pools.append(_fired_pool(priors))
        return np.eye(4), 0.90, "fiducial"

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)

    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert pools == []  # last_pose alone triggers nothing: no fire without a source

    _burst(m._fiducial_prior, 7)
    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert pools == [["fiducial", "last_pose"]]  # the seed rode along, ransac absent


def test_min_local_points_gates_ransac_only_fiducial_fires_on_a_sparse_submap(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    """min_local_points is the FPFH+RANSAC geometry floor, not a global gate: a
    completed tag burst fires and reaches the judge even with the live submap far
    below the floor (a tag fix comes from the tag, not lidar density), while a
    RANSAC cycle stays blocked until the submap clears the floor. A blocked RANSAC
    is not acked, so it re-fires on the next dense frame."""
    clock = {"now": 100.0}
    m = _fiducial_module(
        Config(
            priors=[
                RansacPriorConfig(interval_s=2.0, min_local_points=50_000),
                FiducialPriorConfig(),
            ]
        )
    )
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._now_fn = lambda: clock["now"]
    rec = _ModuleLogRecorder()
    monkeypatch.setattr(module_mod, "logger", rec)

    fires: list[list[str]] = []

    def _fake(global_map: object, local_map: object, priors: list[RelocPrior], **kwargs: object):  # type: ignore[no-untyped-def]
        fires.append(_fired_pool(priors))
        return np.eye(4), 0.90, priors[0].name

    def _no_solo(*args: object, **kwargs: object) -> object:
        raise AssertionError("a configured tag prior routes every fire through the judge")

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _fake)
    monkeypatch.setattr(module_mod, "_relocalize", _no_solo)

    # SPARSE submap (below 50k) with a completed burst: RANSAC (cold, so due) is
    # blocked and logs the skip; the fiducial burst fires anyway.
    _burst(m._fiducial_prior, 7)
    m._on_local_map(cast("PointCloud2", _StubCloud(10_000)))
    assert fires == [["fiducial"]]  # fiducial NOT gated by min_local_points
    assert rec.warnings and rec.warnings[0][0] == "ransac reloc skipped"  # RANSAC was blocked
    assert rec.warnings[0][1]["n_pts"] == 10_000

    # DENSE submap: RANSAC (still due -- its blocked cycle was never acked) now fires.
    fires.clear()
    m._on_local_map(cast("PointCloud2", _StubCloud(60_000)))
    assert fires == [["ransac"]]  # RANSAC still gates at 50k, fires once the floor is cleared
