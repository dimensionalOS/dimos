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

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.experimental.agent_encode.pointcloud.handlers.closest import Closest
from dimos.experimental.agent_encode.pointcloud.handlers.depth_view import DepthView
from dimos.experimental.agent_encode.pointcloud.handlers.occupancy_map import OccupancyMap
from dimos.experimental.agent_encode.pointcloud.handlers.overlap import Overlap
from dimos.experimental.agent_encode.pointcloud.handlers.sweep import Sweep
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.runtime.context import Request, Result
from dimos.experimental.agent_encode.pointcloud.runtime.dispatch import encode, legend
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.experimental.agent_encode.pointcloud.shapes.sphere import Sphere
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def room() -> PointCloud2:
    """Floor at z=0 over 6 x 4 m, a wall at x=3, a wall at y=2, a 0.4 m box at (1, -1)."""
    rng = np.random.default_rng(0)
    floor = np.column_stack([rng.uniform(-3, 3, 4000), rng.uniform(-2, 2, 4000), np.zeros(4000)])
    east = np.column_stack(
        [np.full(3000, 3.0), rng.uniform(-2, 2, 3000), rng.uniform(0, 2.5, 3000)]
    )
    north = np.column_stack(
        [rng.uniform(-3, 3, 3000), np.full(3000, 2.0), rng.uniform(0, 2.5, 3000)]
    )
    box = np.column_stack(
        [rng.uniform(0.8, 1.2, 500), rng.uniform(-1.2, -0.8, 500), rng.uniform(0, 0.8, 500)]
    )
    pts = np.vstack([floor, east, north, box]).astype(np.float32)
    return PointCloud2.from_numpy(pts, frame_id="map", timestamp=7.0)


def test_shapes_contain_and_measure() -> None:
    pts = np.array([[0.0, 0.0, 0.5], [2.0, 0.0, 0.5], [0.0, 0.0, 5.0]], dtype=np.float32)
    box = Box(center=(0.0, 0.0, 0.5), size=(1.0, 1.0, 1.0))
    assert box.contains(pts).tolist() == [True, False, False]
    assert box.distance(pts)[1] == pytest.approx(1.5)
    turned = Box(center=(0.0, 0.0, 0.5), size=(6.0, 0.2, 1.0), yaw_deg=90.0)
    assert turned.contains(np.array([[0.0, 2.5, 0.5]], dtype=np.float32))[0], "rotated onto y"
    cyl = Cylinder(center=(0.0, 0.0), radius=0.5, z_range=(0.0, 1.0))
    assert cyl.contains(pts).tolist() == [True, False, False]
    assert cyl.distance(pts)[1] == pytest.approx(1.5)
    assert np.isinf(cyl.distance(pts)[2]), "outside the z band"
    sph = Sphere(center=(0.0, 0.0, 0.5), radius=0.5)
    assert sph.contains(pts).tolist() == [True, False, False]
    assert sph.distance(pts)[2] == pytest.approx(4.0)


def run(
    cloud: PointCloud2, request: Request[Result], out_dir: Path | None = None
) -> dict[str, Any]:
    result: dict[str, Any] = encode(cloud, {"r": request}, out_dir=out_dir)["results"]["r"]
    return result


def test_overlap_counts_returns_inside(tmp_path: Path) -> None:
    cloud = room()
    r = run(cloud, Overlap(Box(center=(1.0, -1.0, 0.4), size=(0.6, 0.6, 0.8))), tmp_path)
    miss = run(cloud, Overlap(Box(center=(-2.0, 0.0, 0.5), size=(0.6, 0.6, 0.8))), tmp_path)
    assert r["count"] > 400
    assert r["bounds_m"][0][0] >= 0.69 and r["bounds_m"][1][0] <= 1.31, "box plus floor under it"
    assert miss["count"] == 0 and miss["bounds_m"] is None


def test_closest_finds_the_wall(tmp_path: Path) -> None:
    cloud = room()
    r = run(cloud, Closest(Cylinder(center=(2.0, 0.0), radius=0.0, z_range=(0.15, 1.0))), tmp_path)
    assert r["distance_m"] == pytest.approx(1.0, abs=0.05), "east wall is 1 m away, floor excluded"
    assert r["point_m"][0] == pytest.approx(3.0, abs=0.02)
    empty = PointCloud2.from_numpy(
        np.zeros((0, 3), dtype=np.float32), frame_id="map", timestamp=1.0
    )
    assert run(empty, Closest(Sphere((0, 0, 0), 1.0)), tmp_path)["distance_m"] is None


def test_sweep_stops_at_first_contact(tmp_path: Path) -> None:
    cloud = room()
    body = Cylinder(center=(0.0, 0.0), radius=0.3, z_range=(0.15, 1.0))
    east = run(cloud, Sweep(body, direction_deg=0.0, max_distance=5.0), tmp_path)
    assert east["hit"] and east["distance_m"] == pytest.approx(2.7, abs=0.1), (
        "3 m wall minus 0.3 m radius"
    )
    north = run(cloud, Sweep(body, direction_deg=90.0, max_distance=5.0), tmp_path)
    assert north["hit"] and north["distance_m"] == pytest.approx(1.7, abs=0.1)
    west = run(cloud, Sweep(body, direction_deg=180.0, max_distance=2.0), tmp_path)
    assert not west["hit"] and west["distance_m"] is None and west["start_inside"] == 0
    inside_box = run(
        cloud,
        Sweep(Cylinder((1.0, -1.0), 0.3, (0.0, 0.8)), direction_deg=0.0, max_distance=1.0),
        tmp_path,
    )
    assert inside_box["start_inside"] > 0 and inside_box["distance_m"] == 0.0
    into_box = run(
        cloud,
        Sweep(Cylinder((1.0, 0.5), 0.1, (0.15, 1.0)), direction_deg=270.0, max_distance=3.0),
        tmp_path,
    )
    assert into_box["hit"] and into_box["distance_m"] == pytest.approx(1.2, abs=0.15)


def test_views_write_images(tmp_path: Path) -> None:
    cloud = room()
    out = encode(
        cloud,
        {
            "depth": DepthView(view=(0.0, 0.0, 1.0, 0.0, 0.0)),
            "occ": OccupancyMap(z_range=(0.1, 2.5), mark=(0.0, 0.0, 90.0)),
        },
        out_dir=tmp_path,
    )
    depth, occ = out["results"]["depth"], out["results"]["occ"]
    assert Path(depth["image"]).exists() and Path(occ["image"]).exists()
    stops = depth["colour"]["stops"]
    assert stops[0]["value"] < stops[-1]["value"], "stops run from near to far"
    assert depth["point_size_m"][0] <= depth["point_size_m"][1]
    assert occ["mark_cell"] is not None and occ["height_scale"] is None
    assert Path(depth["image"]).parent == tmp_path.resolve()
    assert depth["image"] != occ["image"]


def occupancy_at(
    cloud: PointCloud2, x: float, y: float, z_range: tuple[float, float], free_radius: float = 0.0
) -> tuple[CostValues, float]:
    """The cell value and the highest in-band return at a world position."""
    raster = render.occupancy_grid(
        cloud,
        np.asarray(cloud.points_f32()),
        z_range=z_range,
        spacing=0.25,
        max_cells=256,
        free_radius=free_radius,
        heights=True,
    )
    assert raster.heights is not None
    col, row = render.cell_of(raster.grid, x, y)
    return (
        CostValues(render.grid_north_up(raster.grid)[row, col]),
        np.flipud(raster.heights)[row, col],
    )


def test_occupancy_band_is_absolute_and_free_means_floor_returns(tmp_path: Path) -> None:
    cloud = room()
    band = run(cloud, OccupancyMap(z_range=(0.1, 2.5)), tmp_path)
    low = run(cloud, OccupancyMap(z_range=(-0.5, 2.5)), tmp_path)
    spread = run(cloud, OccupancyMap(z_range=(0.1, 2.5), free_radius=1.0), tmp_path)
    tall = run(cloud, OccupancyMap(z_range=(0.1, 2.5), colour="height"), tmp_path)
    assert low["free_cells"] == 0, "a band under the floor makes the floor occupied"
    assert spread["free_cells"] > band["free_cells"]
    heights = [stop["value"] for stop in tall["height_scale"]["stops"]]
    assert heights[0] == 0.1 and heights[-1] == 2.5
    band_m = (0.1, 2.5)
    assert occupancy_at(cloud, -2.0, 0.0, band_m)[0] == CostValues.FREE, (
        "floor returns below the band mark free"
    )
    assert occupancy_at(cloud, 1.0, -1.0, band_m)[0] == CostValues.OCCUPIED
    assert occupancy_at(cloud, 3.5, 0.0, band_m)[0] == CostValues.UNKNOWN, (
        "nothing was returned beyond the wall"
    )
    assert occupancy_at(cloud, 3.5, 0.0, band_m, free_radius=1.0)[0] == CostValues.FREE, (
        "free_radius spreads free into unseen cells"
    )
    assert occupancy_at(cloud, 1.0, -1.0, band_m)[1] <= 0.85, "the 0.8 m box sits low in the band"
    assert max(occupancy_at(cloud, x, 0.0, band_m)[1] for x in (2.9, 3.0, 3.1)) >= 2.2, (
        "the 2.5 m wall reaches the top of the band"
    )
    rainbow = run(cloud, OccupancyMap(z_range=(0.1, 2.5), colour="rainbow"), tmp_path)
    assert rainbow["status"] == "invalid" and "colour" in rainbow["error"]


def test_height_image_and_bounds(tmp_path: Path) -> None:
    cloud = room()
    out = encode(
        cloud,
        {"occ": OccupancyMap(z_range=(0.1, 2.5), colour="height", zoom=(1.0, -1.0, 1.0))},
        out_dir=tmp_path,
    )
    assert out["bounds_m"][0][0] == pytest.approx(-3.0, abs=0.01)
    assert out["bounds_m"][1] == pytest.approx([3.0, 2.0, 2.5], abs=0.01)
    occ = out["results"]["occ"]
    assert occ["cell_m"] >= occ["spacing_m"] > 0.01, "cells never finer than the returns' gap"
    assert occ["size"][0] <= 2 * 1.0 / occ["cell_m"] + 3, "a 1 m zoom is about 2 m of cells"
    assert Path(occ["image"]).exists() and "stops" in occ["height_scale"]
    raster = render.occupancy_grid(
        cloud,
        np.asarray(cloud.points_f32()),
        z_range=(0.1, 2.5),
        spacing=0.1,
        max_cells=256,
        free_radius=0.0,
        heights=True,
    )
    top = raster.heights
    assert top is not None and top.shape == raster.grid.grid.shape
    assert not np.isnan(top[raster.grid.grid == CostValues.OCCUPIED]).any(), (
        "every occupied cell has a height"
    )


def test_encode_names_results_and_legend_lists_everything(tmp_path: Path) -> None:
    cloud = room()
    out = cloud.agent_encode(
        {"near": Overlap(Sphere((0, 0, 0.5), 0.2)), "far": Closest(Sphere((0, 0, 0.5), 0.0))},
        out_dir=tmp_path,
    )
    assert {name: r["request"]["type"] for name, r in out["results"].items()} == {
        "near": "Overlap",
        "far": "Closest",
    }
    metadata = encode(cloud, {})
    assert metadata["results"] == {} and metadata["bounds_m"] is not None
    with pytest.raises(TypeError, match="mapping"):
        encode(cloud, Overlap(Sphere((0, 0, 0.5), 0.2)))  # type: ignore[arg-type]
    unrunnable = encode(cloud, {"box": Box((0, 0, 0), (1, 1, 1))})  # type: ignore[dict-item]
    assert unrunnable["results"]["box"]["status"] == "invalid"
    text = legend()
    for name in (
        "DepthView",
        "OccupancyMap",
        "Overlap",
        "Sweep",
        "Closest",
        "Box",
        "Cylinder",
        "Sphere",
    ):
        assert name in text
    assert PointCloud2.agent_encode_legend() == text
    assert "x east, y north, z up" in text
