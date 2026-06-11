# Copyright 2025-2026 Dimensional Inc.
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

from pathlib import Path

import numpy as np
import pytest

from dimos.manipulation.reachability.capability_map import (
    CapabilityMap,
    MapParams,
    canonical_values,
)

_G1_MJCF = Path(__file__).parents[3] / "data" / "mujoco_sim" / "g1_gear_wbc.xml"


def _random_poses(n: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    from scipy.spatial.transform import Rotation

    # Cylinder, not box: the canonical offset radius equals the TCP planar
    # radius, so positions must stay within the grid's r_xy.
    radius = 0.85 * np.sqrt(rng.uniform(0.0, 1.0, n))
    angle = rng.uniform(-np.pi, np.pi, n)
    positions = np.stack(
        [radius * np.cos(angle), radius * np.sin(angle), rng.uniform(0.1, 1.6, n)], axis=1
    )
    rotations = Rotation.random(n, random_state=rng).as_matrix()
    return positions, rotations


def _yaw_rotated(positions: np.ndarray, rotations: np.ndarray, alpha: float):
    c, s = np.cos(alpha), np.sin(alpha)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return positions @ rz.T, np.einsum("ij,njk->nik", rz, rotations)


def test_canonical_values_are_yaw_gauge_invariant() -> None:
    """The load-bearing property: rotating a pose about the pelvis vertical
    axis (the quotiented symmetry) must not change any indexed value."""
    rng = np.random.default_rng(3)
    positions, rotations = _random_poses(200, rng)
    base = canonical_values(positions, rotations)
    for alpha in (0.3, -1.2, 2.9):
        rotated = canonical_values(*_yaw_rotated(positions, rotations, alpha))
        for original, transformed, name in zip(
            base[:5], rotated[:5], ("p_z", "theta", "x*", "y*", "gamma"), strict=True
        ):
            assert np.allclose(original, transformed, atol=1e-9), f"{name} not invariant"


def test_canonical_values_finite_at_poles() -> None:
    positions = np.array([[0.3, 0.2, 1.0], [0.1, -0.4, 0.8]])
    rotations = np.stack([np.eye(3), np.diag([1.0, -1.0, -1.0])])  # approach = ±ẑ
    values = canonical_values(positions, rotations)
    for array in values:
        assert np.all(np.isfinite(array))


def test_record_query_roundtrip() -> None:
    rng = np.random.default_rng(4)
    cap = CapabilityMap(MapParams())
    positions, rotations = _random_poses(500, rng)
    n_recorded = cap.record_batch(positions, rotations)
    assert n_recorded == 500
    assert np.all(cap.scores(positions, rotations) >= 1)
    assert np.all(cap.scores_4d(positions, rotations) >= 1)
    # A yaw-rotated copy of every recorded pose is also reachable (gauge).
    rotated = _yaw_rotated(positions, rotations, 1.1)
    assert np.all(cap.scores(*rotated) >= 1)


def test_out_of_bounds_scores_zero() -> None:
    cap = CapabilityMap(MapParams())
    positions = np.array([[5.0, 0.0, 0.5]])  # far outside r_xy
    rotations = np.eye(3)[None]
    assert cap.scores(positions, rotations)[0] == 0
    assert not cap.reachable(np.block([[rotations[0], positions.T], [np.zeros((1, 3)), 1.0]]))


def test_counts_saturate_not_wrap() -> None:
    cap = CapabilityMap(MapParams())
    positions = np.tile([[0.3, 0.0, 0.9]], (300, 1))
    rotations = np.tile(np.eye(3), (300, 1, 1))
    cap.record_batch(positions, rotations)
    cap.record_batch(positions, rotations)
    assert cap.scores(positions[:1], rotations[:1])[0] == 255


def test_mirror_identity() -> None:
    """A pose recorded in the left map is reachable in the right map at the
    reflected pose (y → -y reflection of position and orientation)."""
    rng = np.random.default_rng(5)
    cap = CapabilityMap(MapParams(), side="left")
    positions, rotations = _random_poses(300, rng)
    cap.record_batch(positions, rotations)
    mirrored = cap.mirrored()
    assert mirrored.side == "right"

    flip = np.diag([1.0, -1.0, 1.0])
    positions_m = positions @ flip
    # Proper reflection of a frame: conjugate then fix handedness by
    # negating the x and z axes' y components... equivalently R' = F R F
    # with det(F R F) = det(R) = 1 only if we re-orthogonalize handedness:
    rotations_m = np.einsum("ij,njk,kl->nil", flip, rotations, flip)
    # F R F has det = +1 (two reflections) — still a rotation.
    scores = mirrored.scores(positions_m, rotations_m)
    assert np.all(scores >= 1)


def test_save_load_roundtrip(tmp_path: Path) -> None:
    rng = np.random.default_rng(6)
    cap = CapabilityMap(MapParams(), side="left", model_id="abc123")
    positions, rotations = _random_poses(100, rng)
    cap.record_batch(positions, rotations)
    path = cap.save(tmp_path / "map.npz")

    loaded = CapabilityMap.load(path)
    assert loaded.params == cap.params
    assert loaded.side == "left"
    assert loaded.model_id == "abc123"
    assert np.array_equal(loaded.counts, cap.counts)
    assert np.array_equal(loaded.heading_hint, cap.heading_hint)


@pytest.mark.skipif(not _G1_MJCF.exists(), reason="G1 MJCF assets not present")
def test_g1_construction_smoke() -> None:
    """Tiny construction run: sampled FK poses must query reachable, and an
    absurd pose must not."""
    pytest.importorskip("mujoco")
    from dimos.manipulation.reachability.construct import construct, g1_spec

    spec = g1_spec("left")
    cap = construct(spec, n_samples=3000, workers=1, seed=7)
    assert cap.n_marked > 100
    assert cap.model_id

    # Forward-anchor: FK pose of a mid-range arm config is reachable.
    from dimos.manipulation.reachability.construct import _ArmSampler

    sampler = _ArmSampler(spec)
    rng = np.random.default_rng(7)
    positions, rotations, _ = sampler.sample_chunk(50, rng)
    scores = cap.scores(positions, rotations)
    assert (scores > 0).mean() > 0.5  # most exact re-samples hit marked cells

    # Negative anchor: a pose 2 m away is not reachable.
    far = np.eye(4)
    far[:3, 3] = (0.9, 0.0, 0.9)
    assert not cap.reachable(far)


def test_viewer_cloud_functions() -> None:
    from dimos.manipulation.reachability.viewer import (
        canonical_cloud,
        position_cloud,
        score_colors,
    )

    rng = np.random.default_rng(9)
    cap = CapabilityMap(MapParams())
    positions, rotations = _random_poses(2000, rng)
    cap.record_batch(positions, rotations)

    points, scores = canonical_cloud(cap, 0.0, 180.0, gamma_bin=None, min_score=1)
    assert len(points) == len(scores) > 0
    assert np.all(np.hypot(points[:, 0], points[:, 1]) <= cap.params.r_xy * np.sqrt(2) + 1e-9)
    assert np.all(points[:, 2] >= cap.params.z_min)
    # Higher threshold shows fewer cells.
    fewer, _ = canonical_cloud(cap, 0.0, 180.0, gamma_bin=None, min_score=2)
    assert len(fewer) <= len(points)
    # θ filter shows a subset.
    narrow, _ = canonical_cloud(cap, 80.0, 100.0, gamma_bin=None, min_score=1)
    assert 0 < len(narrow) < len(points)

    ring_points, ring_scores = position_cloud(cap, min_score=1)
    assert len(ring_points) == len(ring_scores) > 0

    colors = score_colors(scores)
    assert colors.shape == (len(scores), 3)
    assert colors.dtype == np.uint8


def test_body_frame_volume() -> None:
    """The body-frame companions record the raw TCP positions (no heading
    quotient) — recorded positions index occupied cells with dexterity > 0."""
    rng = np.random.default_rng(10)
    cap = CapabilityMap(MapParams())
    positions, rotations = _random_poses(1000, rng)
    cap.record_batch(positions, rotations)

    iz, ix, iy, valid = cap.body_indices(positions)
    assert np.all(valid)
    assert np.all(cap.body_counts[iz, ix, iy] >= 1)
    dexterity = cap.body_dexterity()
    assert np.all(dexterity[iz, ix, iy] > 0.0)
    assert dexterity.max() <= 1.0

    # Mirror flips the body volume across y.
    mirrored = cap.mirrored()
    flipped = positions * np.array([1.0, -1.0, 1.0])
    mz, mx, my, mvalid = mirrored.body_indices(flipped)
    assert np.all(mvalid)
    assert np.all(mirrored.body_counts[mz, mx, my] >= 1)


def test_body_voxel_mesh_and_slices() -> None:
    pytest.importorskip("trimesh")
    pytest.importorskip("matplotlib")
    from dimos.manipulation.reachability.viewer import (
        body_voxel_mesh,
        slice_image_height,
        slice_image_yaw,
    )

    rng = np.random.default_rng(11)
    cap = CapabilityMap(MapParams())
    positions, rotations = _random_poses(3000, rng)
    cap.record_batch(positions, rotations)

    mesh, n_voxels = body_voxel_mesh(cap, min_dexterity=0.0)
    assert mesh is not None and n_voxels > 0
    assert len(mesh.faces) == n_voxels * 12  # 12 triangles per box
    # A dexterity threshold prunes voxels.
    _, n_core = body_voxel_mesh(cap, min_dexterity=0.05)
    assert n_core < n_voxels
    none_mesh, n_none = body_voxel_mesh(cap, min_dexterity=1.1)
    assert none_mesh is None and n_none == 0

    image, width, height = slice_image_yaw(cap, 30.0)
    assert image.ndim == 3 and image.shape[2] == 3 and image.dtype == np.uint8
    assert width > 0 and height > 0
    image_h, _, _ = slice_image_height(cap, 0.9)
    assert image_h.ndim == 3 and image_h.dtype == np.uint8


@pytest.mark.skipif(not _G1_MJCF.exists(), reason="G1 MJCF assets not present")
def test_arm_ik_reaches_fk_pose() -> None:
    pytest.importorskip("mink")
    pytest.importorskip("mujoco")
    from dimos.manipulation.reachability.construct import _ArmSampler, g1_spec
    from dimos.manipulation.reachability.viewer import ArmIK

    sampler = _ArmSampler(g1_spec("left"))
    rng = np.random.default_rng(12)
    positions, rotations, _ = sampler.sample_chunk(5, rng)

    import mujoco

    solver = ArmIK("left")
    wxyz = np.empty(4)
    mujoco.mju_mat2Quat(wxyz, np.ascontiguousarray(rotations[0]).reshape(9))
    joints, reached, error = solver.solve(positions[0], wxyz)
    assert reached, f"IK failed with error {error * 1000:.1f} mm"
    assert set(joints) == set(solver.joint_names)


def test_plots_smoke(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    from dimos.manipulation.reachability.plots import render_all

    rng = np.random.default_rng(8)
    cap = CapabilityMap(MapParams())
    positions, rotations = _random_poses(2000, rng)
    cap.record_batch(positions, rotations)
    paths = render_all(cap, tmp_path)
    assert len(paths) == 4
    assert all(p.exists() and p.stat().st_size > 0 for p in paths)
