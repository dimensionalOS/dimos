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

"""Unit tests for the pure-numpy lidar signal tools (wishlist item 3)."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.mapping.pointclouds import signals

EMPTY = np.zeros((0, 3), dtype=np.float32)


def _room(width: float = 6.0, depth: float = 4.0, height: float = 2.4) -> np.ndarray:
    """A hollow box: four walls (perimeter) + a floor plane at z=0."""
    rng = np.random.default_rng(3)
    n = 4000
    xs = rng.uniform(-width / 2, width / 2, n)
    ys = rng.uniform(-depth / 2, depth / 2, n)
    side = rng.integers(0, 4, n)
    wx = np.where(side < 2, np.where(side == 0, -width / 2, width / 2), xs)
    wy = np.where(side < 2, ys, np.where(side == 2, -depth / 2, depth / 2))
    wz = rng.uniform(0.0, height, n)
    walls = np.column_stack([wx, wy, wz])
    floor = np.column_stack(
        [rng.uniform(-width / 2, width / 2, 1500), rng.uniform(-depth / 2, depth / 2, 1500), np.zeros(1500)]
    )
    return np.vstack([walls, floor])


# --- validation & building blocks -----------------------------------------
def test_as_xyz_rejects_wrong_shape() -> None:
    with pytest.raises(ValueError):
        signals.extents(np.zeros((5, 2)))


def test_crop_keeps_only_inside_box() -> None:
    pts = np.array([[0, 0, 0], [5, 5, 5], [1, 1, 1], [-1, 0, 0]], dtype=float)
    out = signals.crop(pts, lower=(0, 0, 0), upper=(2, 2, 2))
    assert out.shape == (2, 3)
    assert {tuple(p) for p in out} == {(0, 0, 0), (1, 1, 1)}


def test_filter_height_band() -> None:
    pts = np.array([[0, 0, 0.0], [0, 0, 0.5], [0, 0, 1.9], [0, 0, 3.0]], dtype=float)
    out = signals.filter_height(pts, z_min=0.3, z_max=2.0)
    assert sorted(out[:, 2].tolist()) == [0.5, 1.9]


def test_voxel_downsample_collapses_duplicates() -> None:
    # 500 copies of two nearby points -> at most 2 voxels occupied.
    a = np.tile([0.01, 0.01, 0.0], (500, 1))
    b = np.tile([0.02, 0.0, 0.0], (500, 1))
    out = signals.voxel_downsample(np.vstack([a, b]), voxel_size=0.05)
    assert len(out) == 1  # both fall in the same 5 cm voxel
    out2 = signals.voxel_downsample(np.vstack([a, b]), voxel_size=0.005)
    assert len(out2) == 2  # finer voxels separate them
    assert signals.voxel_downsample(EMPTY, 0.05).shape == (0, 3)


# --- objective 1: measure --------------------------------------------------
def test_extents_recovers_room_dimensions() -> None:
    ext = signals.extents(_room(6.0, 4.0, 2.4))
    assert ext.span[0] == pytest.approx(6.0, abs=0.1)
    assert ext.span[1] == pytest.approx(4.0, abs=0.1)
    assert ext.span[2] == pytest.approx(2.4, abs=0.1)
    assert ext.center == pytest.approx([0.0, 0.0, 1.2], abs=0.1)


def test_extents_empty_raises() -> None:
    with pytest.raises(ValueError):
        signals.extents(EMPTY)


def test_ground_height_finds_floor() -> None:
    assert signals.ground_height(_room()) == pytest.approx(0.0, abs=0.05)


# --- objective 2: clearance / safety ---------------------------------------
def test_nearest_obstacle_distance() -> None:
    # Walls only (floor filtered out). Nearest wall to the origin is the
    # front/back wall at half-depth = 2 m (closer than the side walls at 3 m).
    walls = signals.filter_height(_room(6.0, 4.0), z_min=0.2)
    assert signals.nearest_obstacle(walls, origin=(0, 0, 0)) == pytest.approx(2.0, abs=0.15)


def test_nearest_obstacle_height_band_ignores_floor() -> None:
    # A point right under the robot at z=0 (floor) plus a far wall point.
    pts = np.array([[0.1, 0.0, 0.0], [3.0, 0.0, 1.0]], dtype=float)
    assert signals.nearest_obstacle(pts) == pytest.approx(0.1, abs=1e-6)
    # Ignoring the floor band, the nearest thing at body height is the wall.
    assert signals.nearest_obstacle(pts, height_band=(0.3, 1.8)) == pytest.approx(3.0, abs=1e-6)


def test_nearest_obstacle_empty_is_inf() -> None:
    assert signals.nearest_obstacle(EMPTY) == float("inf")


def test_clearance_cone_direction() -> None:
    # Obstacle 2 m to the +x (east); clearance looking east is ~2 m,
    # looking west (pi) sees nothing in the cone -> inf.
    pts = np.array([[2.0, 0.0, 1.0]], dtype=float)
    assert signals.clearance(pts, heading=0.0, fov_deg=60) == pytest.approx(2.0, abs=1e-6)
    assert signals.clearance(pts, heading=np.pi, fov_deg=60) == float("inf")


def test_region_is_clear() -> None:
    pts = np.array([[5.0, 5.0, 0.5]], dtype=float)
    assert signals.region_is_clear(pts, lower=(-1, -1, 0), upper=(1, 1, 2)) is True
    assert signals.region_is_clear(pts, lower=(4, 4, 0), upper=(6, 6, 2)) is False


# --- objective 3: map artifacts --------------------------------------------
def test_occupancy_grid_shape_and_perimeter() -> None:
    og = signals.occupancy_grid(_room(6.0, 4.0), resolution=0.1, height_band=(0.2, 2.4))
    # ~6 m / 0.1 wide, ~4 m / 0.1 tall (+1 cell).
    assert og.grid.shape == (41, 61)
    assert og.origin == pytest.approx([-3.0, -2.0], abs=0.05)
    # Perimeter occupied; the room interior (center) is free once the floor is dropped.
    assert og.grid.sum() > 100
    assert not og.grid[20, 30]  # center cell, no wall/floor there


def test_occupancy_grid_fixed_bounds_handles_empty() -> None:
    og = signals.occupancy_grid(EMPTY, resolution=0.5, bounds=(0, 0, 2, 2))
    assert og.grid.shape == (5, 5)
    assert og.grid.sum() == 0


def test_height_map_max_z_per_cell() -> None:
    # Two points in the same cell -> the taller wins; NaN elsewhere.
    pts = np.array([[0.05, 0.05, 1.0], [0.06, 0.04, 2.0], [1.5, 1.5, 0.5]], dtype=float)
    hm = signals.height_map(pts, resolution=1.0, bounds=(0, 0, 2, 2))
    assert hm.grid[0, 0] == pytest.approx(2.0)
    assert hm.grid[1, 1] == pytest.approx(0.5)
    assert np.isnan(hm.grid[0, 1])


# --- objective 6: coverage -------------------------------------------------
def test_coverage_area_matches_occupied_cells() -> None:
    # A dense 2x2 m patch at 0.5 m resolution should cover ~4 m^2.
    rng = np.random.default_rng(1)
    patch = np.column_stack([rng.uniform(0, 2, 5000), rng.uniform(0, 2, 5000), np.zeros(5000)])
    area = signals.coverage_area(patch, resolution=0.5)
    assert area == pytest.approx(4.0, abs=0.5)


def test_coverage_area_empty_is_zero() -> None:
    assert signals.coverage_area(EMPTY) == 0.0
