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

import numpy as np
import pytest

from dimos.experimental.agent_encode.pointcloud import constants
from dimos.experimental.agent_encode.pointcloud.handlers.closest import Closest
from dimos.experimental.agent_encode.pointcloud.handlers.depth_view import DepthView
from dimos.experimental.agent_encode.pointcloud.handlers.occupancy_map import OccupancyMap
from dimos.experimental.agent_encode.pointcloud.handlers.overlap import Overlap
from dimos.experimental.agent_encode.pointcloud.handlers.sweep import Sweep
from dimos.experimental.agent_encode.pointcloud.render import raster as render
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


def test_overlap_counts_returns_inside(tmp_path: Path) -> None:
    cloud = room()
    hit = encode(
        cloud, Overlap(Box(center=(1.0, -1.0, 0.4), size=(0.6, 0.6, 0.8))), out_dir=tmp_path
    )
    miss = encode(
        cloud, Overlap(Box(center=(-2.0, 0.0, 0.5), size=(0.6, 0.6, 0.8))), out_dir=tmp_path
    )
    r = hit["results"][0]
    assert r["handler"] == "Overlap" and r["count"] > 400
    assert r["bounds_m"][0][0] >= 0.69 and r["bounds_m"][1][0] <= 1.31, "box plus floor under it"
    assert miss["results"][0]["count"] == 0 and miss["results"][0]["bounds_m"] is None


def test_closest_finds_the_wall(tmp_path: Path) -> None:
    cloud = room()
    out = encode(
        cloud,
        Closest(Cylinder(center=(2.0, 0.0), radius=0.0, z_range=(0.15, 1.0))),
        out_dir=tmp_path,
    )
    r = out["results"][0]
    assert r["distance_m"] == pytest.approx(1.0, abs=0.05), "east wall is 1 m away, floor excluded"
    assert r["point_m"][0] == pytest.approx(3.0, abs=0.02)
    empty = PointCloud2.from_numpy(
        np.zeros((0, 3), dtype=np.float32), frame_id="map", timestamp=1.0
    )
    assert encode(empty, Closest(Sphere((0, 0, 0), 1.0)))["results"][0]["distance_m"] is None


def test_sweep_stops_at_first_contact(tmp_path: Path) -> None:
    cloud = room()
    body = Cylinder(center=(0.0, 0.0), radius=0.3, z_range=(0.15, 1.0))
    east = encode(cloud, Sweep(body, direction_deg=0.0, max_distance=5.0), out_dir=tmp_path)[
        "results"
    ][0]
    assert east["hit"] and east["distance_m"] == pytest.approx(2.7, abs=0.1), (
        "3 m wall minus 0.3 m radius"
    )
    north = encode(cloud, Sweep(body, direction_deg=90.0, max_distance=5.0), out_dir=tmp_path)[
        "results"
    ][0]
    assert north["hit"] and north["distance_m"] == pytest.approx(1.7, abs=0.1)
    west = encode(cloud, Sweep(body, direction_deg=180.0, max_distance=2.0), out_dir=tmp_path)[
        "results"
    ][0]
    assert not west["hit"] and west["distance_m"] is None and west["start_inside"] == 0
    inside_box = encode(
        cloud,
        Sweep(Cylinder((1.0, -1.0), 0.3, (0.0, 0.8)), direction_deg=0.0, max_distance=1.0),
        out_dir=tmp_path,
    )["results"][0]
    assert inside_box["start_inside"] > 0 and inside_box["distance_m"] == 0.0
    into_box = encode(
        cloud,
        Sweep(Cylinder((1.0, 0.5), 0.1, (0.15, 1.0)), direction_deg=270.0, max_distance=3.0),
        out_dir=tmp_path,
    )["results"][0]
    assert into_box["hit"] and into_box["distance_m"] == pytest.approx(1.2, abs=0.15)


def test_views_follow_the_build_form(tmp_path: Path) -> None:
    cloud = room()
    out = encode(
        cloud,
        DepthView(view=(0.0, 0.0, 1.0, 0.0, 0.0)),
        OccupancyMap(z_range=(0.1, 2.5), mark=(0.0, 0.0, 90.0)),
        out_dir=tmp_path,
    )
    depth, occ = out["results"]
    assert (
        out["form"] == "image"
        and depth["handler"] == "DepthView"
        and occ["handler"] == "OccupancyMap"
    )
    assert Path(depth["image"]).exists() and Path(occ["image"]).exists()
    assert depth["colour"]["colour_none"] == [0, 0, 0]
    assert depth["point_size_m"][0] <= depth["point_size_m"][1]
    assert occ["mark_cell"] is not None and occ["colour"] == "flat"
    assert Path(depth["image"]).parent == tmp_path.resolve()
    assert depth["image"] != occ["image"]
    saved = constants.FORM
    try:
        constants.FORM = "text"
        text = encode(
            cloud,
            DepthView(view=(0.0, 0.0, 1.0, 0.0, 0.0)),
            OccupancyMap(z_range=(0.1, 2.5)),
            out_dir=tmp_path / "none",
        )
    finally:
        constants.FORM = saved
    assert (
        text["form"] == "text" and "ascii" in text["results"][0] and "ascii" in text["results"][1]
    )
    assert "mark_cell" not in text["results"][1]
    assert not (tmp_path / "none").exists()


def occupancy_at(result: dict, x: float, y: float) -> str:
    """'#', '.', '?' or a digit at a world position in a text-form map."""
    col = int((x - result["origin_xy"][0]) / result["cell_m"]) // result["ascii_cells_per_char"]
    rows = result["ascii"].split("\n")
    row = int((y - result["origin_xy"][1]) / result["cell_m"]) // result["ascii_cells_per_char"]
    return rows[len(rows) - 1 - row][col]


def test_occupancy_band_is_absolute_and_free_means_floor_returns(tmp_path: Path) -> None:
    cloud = room()
    saved = constants.FORM
    try:
        constants.FORM = "text"
        band = encode(cloud, OccupancyMap(z_range=(0.1, 2.5)))["results"][0]
        low = encode(cloud, OccupancyMap(z_range=(-0.5, 2.5)))["results"][0]
        spread = encode(cloud, OccupancyMap(z_range=(0.1, 2.5), free_radius=1.0))["results"][0]
        tall = encode(cloud, OccupancyMap(z_range=(0.1, 2.5), colour="height"))["results"][0]
    finally:
        constants.FORM = saved
    assert band["z_range_m"] == [0.1, 2.5] and band["free_radius_m"] == 0
    assert occupancy_at(band, -2.0, 0.0) == ".", "floor returns below the band mark free"
    assert occupancy_at(band, 1.0, -1.0) == "#", "the box is inside the band"
    assert occupancy_at(band, 3.5, 0.0) == "?", "nothing was returned beyond the wall"
    assert occupancy_at(low, -2.0, 0.0) == "#", "a band under the floor makes the floor occupied"
    assert low["free_cells"] == 0
    assert occupancy_at(spread, 3.5, 0.0) == ".", "free_radius spreads free into unseen cells"
    assert spread["free_cells"] > band["free_cells"]
    assert tall["height_scale"]["z_low_m"] == 0.1 and tall["height_scale"]["z_high_m"] == 2.5
    assert occupancy_at(tall, 1.0, -1.0) in "0123", "a 0.8 m box sits low in a 0.1..2.5 band"
    near_wall = {occupancy_at(tall, x, 0.0) for x in (2.9, 3.0, 3.1)}
    assert near_wall & set("789"), "the 2.5 m wall reaches the top of the band"
    with pytest.raises(ValueError, match="colour"):
        encode(cloud, OccupancyMap(z_range=(0.1, 2.5), colour="rainbow"))


def test_height_image_and_bounds(tmp_path: Path) -> None:
    cloud = room()
    out = encode(
        cloud,
        OccupancyMap(z_range=(0.1, 2.5), colour="height", zoom=(1.0, -1.0, 1.0)),
        out_dir=tmp_path,
    )
    assert out["bounds_m"][0][0] == pytest.approx(-3.0, abs=0.01)
    assert out["bounds_m"][1] == pytest.approx([3.0, 2.0, 2.5], abs=0.01)
    occ = out["results"][0]
    assert occ["zoom_m"] == [1.0, -1.0, 1.0]
    assert occ["cell_m"] >= occ["spacing_m"] > 0.01, "cells never finer than the returns' gap"
    assert occ["size"][0] <= 2 * 1.0 / occ["cell_m"] + 3, "a 1 m zoom is about 2 m of cells"
    assert Path(occ["image"]).exists() and "stops" in occ["height_scale"]
    grid, top = render.occupancy_grid(
        cloud,
        np.asarray(cloud.points_f32()),
        z_range=(0.1, 2.5),
        spacing=0.1,
        max_cells=256,
        free_radius=0.0,
        heights=True,
    )
    assert top is not None and top.shape == grid.grid.shape
    assert not np.isnan(top[grid.grid == CostValues.OCCUPIED]).any(), (
        "every occupied cell has a height"
    )


def test_encode_orders_results_and_legend_lists_everything(tmp_path: Path) -> None:
    cloud = room()
    out = encode(
        cloud,
        Overlap(Sphere((0, 0, 0.5), 0.2)),
        Closest(Sphere((0, 0, 0.5), 0.0)),
        out_dir=tmp_path,
    )
    assert [r["handler"] for r in out["results"]] == ["Overlap", "Closest"]
    metadata = encode(cloud, {})
    assert metadata["results"] == {} and metadata["bounds_m"] is not None
    with pytest.raises(TypeError, match="not a query"):
        encode(cloud, "DepthView")  # type: ignore[arg-type]
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
    assert (
        cloud.agent_encode(Overlap(Sphere((1.0, -1.0, 0.4), 0.3)), out_dir=tmp_path)["results"][0][
            "count"
        ]
        > 0
    )
    assert PointCloud2.AGENT_ENCODE_LEGEND == text
    assert PointCloud2.Overlap is Overlap and PointCloud2.Box is Box
    assert "x east, y north, z up" in text
