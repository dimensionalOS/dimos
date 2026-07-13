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

import math

import numpy as np

from dimos.mapping.reconstruction.scene_model import SceneModel, quat_to_matrix


def _shoelace_area(loop: np.ndarray) -> float:
    x, y = loop[:-1, 0], loop[:-1, 1]
    return abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))) / 2


def _seed_traj() -> np.ndarray:
    """A few robot poses in the middle of the room (xy only)."""
    return np.array([[2.0, 2.0], [3.0, 2.0], [4.0, 2.0]], dtype=np.float32)


def _synthetic_room() -> np.ndarray:
    """A 6 x 4 x 3 m room: floor, four walls, and a 1 x 1 x 0.8 m table blob."""
    rng = np.random.default_rng(7)
    pts = []

    def _plane(xs, ys, zs, n):  # type: ignore[no-untyped-def]
        pts.append(
            np.column_stack(
                [rng.uniform(*xs, n), rng.uniform(*ys, n), rng.uniform(*zs, n)]
            )
        )

    _plane((0, 6), (0, 4), (0.0, 0.02), 4000)  # floor
    _plane((0, 6), (0, 0.05), (0, 3), 3000)  # south wall
    _plane((0, 6), (3.95, 4), (0, 3), 3000)  # north wall
    _plane((0, 0.05), (0, 4), (0, 3), 2000)  # west wall
    _plane((5.95, 6), (0, 4), (0, 3), 2000)  # east wall
    _plane((2.5, 3.5), (1.5, 2.5), (0.75, 0.8), 800)  # table top
    return np.vstack(pts).astype(np.float32)


def _synthetic_room_with_gap(gap_x: tuple[float, float] = (2.5, 3.5)) -> np.ndarray:
    """The same room, but the south wall has an unscanned 1 m opening."""
    rng = np.random.default_rng(7)
    pts = []

    def _plane(xs, ys, zs, n):  # type: ignore[no-untyped-def]
        pts.append(
            np.column_stack(
                [rng.uniform(*xs, n), rng.uniform(*ys, n), rng.uniform(*zs, n)]
            )
        )

    _plane((0, 6), (0, 4), (0.0, 0.02), 4000)  # floor
    _plane((0, gap_x[0]), (0, 0.05), (0, 3), 1500)  # south wall, west of the gap
    _plane((gap_x[1], 6), (0, 0.05), (0, 3), 1500)  # south wall, east of the gap
    _plane((0, 6), (3.95, 4), (0, 3), 3000)  # north wall
    _plane((0, 0.05), (0, 4), (0, 3), 2000)  # west wall
    _plane((5.95, 6), (0, 4), (0, 3), 2000)  # east wall
    return np.vstack(pts).astype(np.float32)


def test_horizontal_section_shows_walls_not_table() -> None:
    model = SceneModel(_synthetic_room())
    section = model.horizontal_section(z=1.5, thickness=0.2, resolution=0.05)
    occ = section.occupancy()
    u_lo, v_lo, u_hi, v_hi = section.extent
    # the cut spans the room
    assert u_hi - u_lo > 5.5 and v_hi - v_lo > 3.5
    # walls present: cells on the west edge occupied
    assert occ[:, 0].mean() > 0.5
    # the table (z<=0.8) must NOT appear in a 1.5 m cut: the room interior is empty
    h, w = occ.shape
    assert occ[h // 4 : 3 * h // 4, w // 4 : 3 * w // 4].mean() < 0.02


def test_horizontal_section_at_table_height_shows_table() -> None:
    model = SceneModel(_synthetic_room())
    section = model.horizontal_section(z=0.78, thickness=0.1, resolution=0.05)
    occ = section.occupancy()
    h, w = occ.shape
    # table blob occupies the center of the room at this height
    assert occ[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3].mean() > 0.2


def test_vertical_section_profile() -> None:
    model = SceneModel(_synthetic_room())
    # cut across the room at x in [0..6], y = 2
    section = model.vertical_section((0, 2), (6, 2), thickness=0.4, resolution=0.05)
    occ = section.occupancy()
    h, w = occ.shape
    # floor: bottom row mostly occupied
    assert occ[0, :].mean() > 0.5
    # east and west walls: full-height columns at both ends (walls are ~2
    # cells thick after binning, so check a 3-column band per side)
    assert occ[:, :3].any(axis=1).mean() > 0.5
    assert occ[:, -3:].any(axis=1).mean() > 0.5
    # air above the table in the middle is empty
    assert occ[2 * h // 3 :, w // 3 : 2 * w // 3].mean() < 0.02


def test_empty_section_is_well_formed() -> None:
    model = SceneModel(_synthetic_room())
    section = model.horizontal_section(z=50.0)  # far above the room
    assert section.density.shape == (1, 1)
    assert not section.occupancy().any()


def test_section_save_png(tmp_path) -> None:  # type: ignore[no-untyped-def]
    model = SceneModel(_synthetic_room())
    out = tmp_path / "plan.png"
    model.horizontal_section(z=1.5).save_png(out, title="test cut")
    assert out.stat().st_size > 1000


def test_quat_identity_and_yaw() -> None:
    assert np.allclose(quat_to_matrix(0, 0, 0, 1), np.eye(3))
    # 90 deg about z
    r = quat_to_matrix(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert np.allclose(r @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-9)


def test_colors_default_uncolored() -> None:
    model = SceneModel(_synthetic_room())
    assert not model.colored.any()
    section = model.horizontal_section(z=1.5)
    assert section.color is None


def test_to_pointcloud_downsamples() -> None:
    model = SceneModel(_synthetic_room())
    pcd = model.to_pointcloud(voxel=0.2)
    assert 0 < len(pcd.points) < len(model.points)


def test_sections_with_shared_bounds_are_aligned() -> None:
    model = SceneModel(_synthetic_room())
    bounds = (-0.5, -0.5, 6.5, 4.5)
    low = model.horizontal_section(z=0.78, thickness=0.1, bounds=bounds)
    high = model.horizontal_section(z=1.5, thickness=0.2, bounds=bounds)
    # same window, same shape → cell-aligned masks that can be combined
    assert low.extent == high.extent == bounds
    assert low.density.shape == high.density.shape
    # table cells occupied in the low band are empty in the high band
    table = low.occupancy() & ~high.occupancy()
    assert table.any()


def test_save_mesh_formats(tmp_path) -> None:  # type: ignore[no-untyped-def]
    model = SceneModel(_synthetic_room())
    mesh = model.to_mesh(voxel=0.2, poisson_depth=7)  # coarse: keep the test fast
    for suffix in ("glb", "obj", "ply"):
        out = model.save_mesh(tmp_path / f"room.{suffix}", mesh=mesh)
        assert out.stat().st_size > 1000, suffix


# --------------------------------------------------------------------------- #
# Voxel occupancy + closed regions
# --------------------------------------------------------------------------- #


def test_voxel_occupancy_dims_and_origin() -> None:
    model = SceneModel(_synthetic_room())
    vox = model.voxel_occupancy(voxel=0.1)
    nz, ny, nx = vox.shape
    # 6 x 4 x 3 m room + 0.3 m margins at 0.1 m voxels
    assert 60 <= nx <= 70 and 40 <= ny <= 50 and 30 <= nz <= 40
    assert vox.origin[0] < 0 and vox.origin[1] < 0  # margin pushes below the min point
    # a wall column is occupied, mid-air room interior is not (slab over a z
    # band — single voxels are legitimately sparse at this point density)
    band = vox.slab(1.0, 2.0)
    r_mid = int((2.0 - vox.origin[1]) / vox.voxel)
    c_wall = int((0.02 - vox.origin[0]) / vox.voxel)
    c_mid = int((3.0 - vox.origin[0]) / vox.voxel)
    assert band[r_mid, c_wall]
    assert not band[r_mid, c_mid]


def test_voxel_slab_aligns_with_section() -> None:
    model = SceneModel(_synthetic_room())
    vox = model.voxel_occupancy(voxel=0.1)
    slab = vox.slab(1.4, 1.6, min_hits=1)
    x_lo, y_lo, x_hi, y_hi = vox.xy_extent()
    section = model.horizontal_section(
        z=1.5, thickness=0.2, resolution=0.1, bounds=(x_lo, y_lo, x_hi, y_hi)
    )
    occ = section.occupancy()
    h = min(slab.shape[0], occ.shape[0])
    w = min(slab.shape[1], occ.shape[1])
    both = slab[:h, :w] & occ[:h, :w]
    either = slab[:h, :w] | occ[:h, :w]
    assert both.sum() / max(either.sum(), 1) > 0.9


def test_closed_region_seals_gap() -> None:
    model = SceneModel(_synthetic_room_with_gap())
    vox = model.voxel_occupancy(voxel=0.1)
    region = vox.closed_region(0.2, 2.8, _seed_traj())
    assert region is not None
    # sealed within a modest kernel — the 1 m gap must not require the 2.5 m bridge
    assert region.closure_m <= 1.2
    # interior must not leak to the raster edge
    assert not region.interior[0, :].any() and not region.interior[-1, :].any()
    assert not region.interior[:, 0].any() and not region.interior[:, -1].any()
    # the inferred (bridged) cells sit in the gap span on the south wall
    br_r, br_c = np.nonzero(region.bridged)
    assert len(br_r) > 0
    bx = region.origin[0] + (br_c + 0.5) * region.resolution
    by = region.origin[1] + (br_r + 0.5) * region.resolution
    in_gap = (bx > 2.3) & (bx < 3.7) & (np.abs(by) < 0.5)
    assert in_gap.any()


def test_closed_region_interior_area() -> None:
    model = SceneModel(_synthetic_room_with_gap())
    vox = model.voxel_occupancy(voxel=0.1)
    region = vox.closed_region(0.2, 2.8, _seed_traj())
    assert region is not None
    area = region.interior.sum() * region.resolution**2
    assert abs(area - 24.0) / 24.0 < 0.15  # 6 x 4 m room


def test_room_loops_closed() -> None:
    model = SceneModel(_synthetic_room_with_gap())
    vox = model.voxel_occupancy(voxel=0.1)
    region = vox.closed_region(0.2, 2.8, _seed_traj())
    assert region is not None and region.loops
    for loop in region.loops:
        assert np.allclose(loop[0], loop[-1])  # explicitly closed
    outer = max(region.loops, key=_shoelace_area)
    assert abs(_shoelace_area(outer) - 24.0) / 24.0 < 0.2


def test_closed_region_intact_room_no_bridging() -> None:
    # regression guard against over-closing: a fully scanned room needs no bridges
    model = SceneModel(_synthetic_room())
    vox = model.voxel_occupancy(voxel=0.1)
    region = vox.closed_region(0.2, 2.8, _seed_traj())
    assert region is not None
    assert region.closure_m == 0.3  # sealed at the smallest kernel
    # closing invents (almost) nothing on an intact envelope — a few corner
    # cells rounded off by the ellipse kernel are fine, a bridged wall is not
    assert region.bridged.sum() <= 0.05 * region.shell.sum()


def test_save_rerun_sliceable(tmp_path) -> None:  # type: ignore[no-untyped-def]
    model = SceneModel(_synthetic_room_with_gap())
    vox = model.voxel_occupancy(voxel=0.2)
    region = vox.closed_region(0.2, 2.8, _seed_traj())
    out = tmp_path / "model.rrd"
    model.save_rerun_sliceable(out, vox, regions=[region] if region else [], z_step=0.5)
    assert out.stat().st_size > 1000
