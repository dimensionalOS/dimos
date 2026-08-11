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

import json
from pathlib import Path

import numpy as np
import pytest

from dimos.manipulation.reachability.capability_map import (
    CapabilityMap,
    MapParams,
    canonical_values,
    canonical_values_from_poses,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

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


def _pose_matrices(positions: np.ndarray, rotations: np.ndarray) -> np.ndarray:
    poses = np.tile(np.eye(4), (len(positions), 1, 1))
    poses[:, :3, :3] = rotations
    poses[:, :3, 3] = positions
    return poses


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


def test_pose_api_matches_vectorized_components() -> None:
    rng = np.random.default_rng(5)
    positions, rotations = _random_poses(100, rng)
    ee_poses = _pose_matrices(positions, rotations)

    component_values = canonical_values(positions, rotations)
    pose_values = canonical_values_from_poses(ee_poses)
    for from_components, from_poses in zip(component_values, pose_values, strict=True):
        assert np.allclose(from_components, from_poses)

    cap = CapabilityMap(MapParams())
    assert cap.record_poses(ee_poses) == cap.record_batch(positions, rotations)
    assert np.array_equal(cap.score_poses(ee_poses), cap.scores(positions, rotations))
    assert np.array_equal(cap.score_poses_4d(ee_poses), cap.scores_4d(positions, rotations))
    assert cap.score_pose(ee_poses[0]) >= 1
    assert cap.reachable_pose(ee_poses[0])


def test_pose_stamped_api() -> None:
    cap = CapabilityMap(MapParams())
    pose = PoseStamped(
        position=Vector3(0.3, 0.0, 0.9),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    assert cap.record_poses(pose) == 1
    assert cap.score_pose(pose) >= 1
    assert cap.reachable_pose(pose)


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


def test_save_load_roundtrip(tmp_path: Path) -> None:
    rng = np.random.default_rng(6)
    cap = CapabilityMap(MapParams(), robot="g1-left", model_id="abc123")
    positions, rotations = _random_poses(100, rng)
    cap.record_batch(positions, rotations)
    path = cap.save(tmp_path / "map.npz")

    loaded = CapabilityMap.load(path)
    assert loaded.params == cap.params
    assert loaded.robot == "g1-left"
    assert loaded.model_id == "abc123"
    assert np.array_equal(loaded.counts, cap.counts)
    assert np.array_equal(loaded.heading_hint, cap.heading_hint)


def test_load_legacy_pelvis_height_and_side_metadata(tmp_path: Path) -> None:
    params_dict = {
        "r_xy": 1.0,
        "z_min": 0.0,
        "z_max": 1.8,
        "cell": 0.05,
        "n_theta": 36,
        "n_inplane": 12,
        "n_heading": 8,
        "pelvis_height": 1.23,
    }
    params = MapParams.from_json_dict(params_dict)
    shape5 = (params.n_z, params.n_theta, params.n_xy, params.n_xy, params.n_inplane)
    path = tmp_path / "legacy_map.npz"
    np.savez_compressed(
        path,
        counts=np.zeros(shape5, dtype=np.uint8),
        heading_hint=np.zeros(shape5[:4], dtype=np.uint8),
        params=np.frombuffer(json.dumps(params_dict).encode(), dtype=np.uint8),
        meta=np.frombuffer(
            json.dumps({"side": "right", "model_id": "legacy"}).encode(), dtype=np.uint8
        ),
    )

    loaded = CapabilityMap.load(path)
    assert loaded.robot == "g1-right"
    assert loaded.model_id == "legacy"
    assert loaded.params.base_link_pose.position.z == pytest.approx(1.23)


@pytest.mark.skipif(not _G1_MJCF.exists(), reason="G1 MJCF assets not present")
def test_g1_construction_smoke() -> None:
    """Tiny construction run: sampled FK poses must query reachable, and an
    absurd pose must not."""
    pytest.importorskip("mujoco")
    from dimos.manipulation.reachability.construct import arm_spec, construct

    spec = arm_spec("g1-left")
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
    from dimos.manipulation.visualization.viser.reachability import body_point_cloud, score_colors

    rng = np.random.default_rng(9)
    cap = CapabilityMap(MapParams())
    positions, rotations = _random_poses(2000, rng)
    cap.record_batch(positions, rotations)

    points, dexterity = body_point_cloud(cap, min_dexterity=0.0)
    assert len(points) == len(dexterity) > 0
    assert np.all(np.abs(points[:, :2]) <= cap.params.r_xy + 1e-9)
    assert np.all(points[:, 2] >= cap.params.z_min)
    assert np.all((dexterity > 0.0) & (dexterity <= 1.0))
    # A dexterity threshold prunes cells.
    fewer, _ = body_point_cloud(cap, min_dexterity=0.05)
    assert len(fewer) < len(points)

    colors = score_colors(dexterity)
    assert colors.shape == (len(dexterity), 3)
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


def test_body_voxel_mesh_and_slices() -> None:
    pytest.importorskip("trimesh")
    pytest.importorskip("matplotlib")
    from dimos.manipulation.visualization.viser.reachability import (
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


def test_registry_is_consistent() -> None:
    """Pure registry check (no model assets): every entry is well-formed."""
    from dimos.manipulation.reachability.robots import arm_model, list_robots

    robots = list_robots()
    assert "g1-left" in robots and "xarm7" in robots
    for key in robots:
        am = arm_model(key)
        assert am.key == key
        assert am.joint_names, f"{key} has no joints"
        assert am.ee_body
        if am.is_urdf:
            assert am.package_roots, f"{key} is URDF but declares no package roots"


@pytest.mark.self_hosted  # compiles real robot models pulled from the self-hosted LFS
@pytest.mark.parametrize("robot", ["g1-left", "g1-right", "xarm7", "piper", "a750", "openarm"])
def test_registered_model_compiles(robot: str) -> None:
    """Each registered model compiles and exposes its declared joints + EE
    body. Skipped per-robot when the model assets aren't present."""
    pytest.importorskip("mujoco")
    import mujoco

    from dimos.manipulation.reachability.robots import arm_model, compile_model

    am = arm_model(robot)
    try:
        model = compile_model(
            am.model_path,
            is_urdf=am.is_urdf,
            model_meshdir=am.model_meshdir,
            package_roots=am.package_roots,
        )
    except FileNotFoundError as e:
        pytest.skip(f"{robot} assets not present: {e}")
    except ValueError as e:
        if "opening file" in str(e):
            pytest.skip(f"{robot} assets not present: {e}")
        raise

    for joint in am.joint_names:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint) >= 0, (
            f"{robot}: joint {joint!r} not in model"
        )
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, am.ee_body) >= 0, (
        f"{robot}: ee body {am.ee_body!r} not in model"
    )
