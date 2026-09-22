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

import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from dimos.experimental.agent_encode.pointcloud import api as pc
from dimos.experimental.agent_encode.pointcloud.fields import HeightField
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.render.overlays import (
    Segment,
    draw_overlays,
    grid_pixel,
)
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def cloud():
    return PointCloud2.from_numpy(
        np.array(
            [[0.25, 0.25, 0], [0.25, 0.25, 1], [1.25, 0.25, 0.5], [2, 0.25, 9]], dtype=np.float32
        ),
        frame_id="map",
        timestamp=7,
    )


@pytest.fixture
def grid():
    return pc.Grid(origin=(0, 0), shape=(2, 2), cell_m=1, frame="map")


def test_shared_field_samples_and_exact_half_open_cells(cloud, grid, tmp_path, mocker):
    height = pc.HeightField(grid)
    spy = mocker.spy(HeightField, "run")
    out = cloud.agent_encode(
        {
            "sample": pc.Sample(height, at=[(0.5, 0.5), (2, 0)]),
            "cells": pc.Window(height, cells=(0, 0, 2, 2)),
        },
        out_dir=tmp_path,
    )
    assert out["results"]["cells"]["values"] == {
        "count": [[2, 1], [0, 0]],
        "min": [[0, 0.5], [None, None]],
        "max": [[1, 0.5], [None, None]],
    }
    assert out["results"]["sample"]["samples"][0]["cell"] == [0, 0]
    outside = out["results"]["sample"]["samples"][1]
    assert outside["status"] == "outside_grid" and outside["values"] is None
    assert spy.call_count == 1
    assert out["frame_id"] == "map" and out["ts"] == 7
    assert out["bounds_m"] == [[0.25, 0.25, 0.0], [2.0, 0.25, 9.0]]


def test_bounds_round_to_millimetres_and_legend_discloses_precision(tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array([[0.12349, -0.98751, 1.23456]], dtype=np.float32), frame_id="sensor"
    )

    out = cloud.agent_encode({}, out_dir=tmp_path)

    assert out["bounds_m"] == [[0.123, -0.988, 1.235], [0.123, -0.988, 1.235]]
    assert "rounded to 0.001 m" in PointCloud2.agent_encode_legend()


def test_selection_is_shared_by_fields_and_existing_queries(cloud, grid, tmp_path):
    selection = pc.Select(include=pc.Band("z", 0.5, 2))
    out = cloud.agent_encode(
        {
            "height": pc.Window(pc.HeightField(grid, selection), (0, 0, 2, 1)),
            "nearest": pc.Closest(pc.Sphere((0.25, 0.25, 0), 0), source=selection),
        },
        out_dir=tmp_path,
    )
    assert out["results"]["height"]["values"]["count"] == [[1, 1]]
    assert out["results"]["nearest"]["distance_m"] == 1


def test_distance_includes_targets_outside_grid_and_empty_is_not_free(cloud, grid, tmp_path):
    far = pc.Select(include=pc.Band("z", 8, 10))
    absent = pc.Select(include=pc.Band("z", 20, 30))
    out = cloud.agent_encode(
        {
            "distance": pc.Sample(pc.DistanceField(grid, far), [(1.5, 0.5)], decimals=9),
            "default": pc.Sample(pc.DistanceField(grid, far), [(1.5, 0.5)]),
            "missing": pc.Sample(pc.DistanceField(grid, absent), [(0.5, 0.5)]),
        },
        out_dir=tmp_path,
    )
    assert out["results"]["distance"]["samples"][0]["values"]["distance_m"] == pytest.approx(
        np.hypot(0.5, 0.25)
    )
    # Default precision is a thousandth of the cell: 1 m cells -> 2 decimals.
    assert out["results"]["default"]["decimals"] == 2
    assert out["results"]["default"]["samples"][0]["values"]["distance_m"] == 0.56
    assert out["results"]["missing"]["samples"][0]["status"] == "no_targets"


def test_resample_looks_up_the_containing_cell_so_grids_combine(cloud, grid, tmp_path):
    fine = pc.Grid(origin=(-0.5, 0), shape=(5, 4), cell_m=0.5, frame="map")
    held = pc.Resample(pc.Threshold(pc.HeightField(grid).count, ">", 0), fine)
    low = pc.Threshold(pc.HeightField(fine).max, "<", 2)
    out = cloud.agent_encode(
        {
            "held": pc.Window(held, (0, 0, 5, 4)),
            "both": pc.Window(held & low, (0, 0, 5, 4)),
            "mixed": pc.Window(
                pc.Threshold(pc.HeightField(grid).count, ">", 0) & low, (0, 0, 5, 4)
            ),
        },
        out_dir=tmp_path,
    )
    # Coarse row 0 holds returns, row 1 none; the fine x<0 column lies outside the coarse grid.
    assert out["results"]["held"]["values"]["mask"] == [
        [None, 1, 1, 1, 1],
        [None, 1, 1, 1, 1],
        [None, 0, 0, 0, 0],
        [None, 0, 0, 0, 0],
    ]
    assert out["results"]["both"]["status"] == "ok", "a resampled mask stays a mask"
    assert out["results"]["mixed"]["status"] == "invalid"
    assert "Resample" in out["results"]["mixed"]["error"]


def test_disc_sample_reports_every_label_it_covers(cloud, grid, tmp_path):
    fine = pc.Grid(origin=(0, 0), shape=(4, 2), cell_m=0.5, frame="map")
    labels = pc.Components(pc.Threshold(pc.HeightField(fine).count, ">", 0))
    out = cloud.agent_encode(
        {"disc": pc.Sample(labels, at=[(1.0, 0.25), (9, 9)], radius=0.8)}, out_dir=tmp_path
    )
    near, far = out["results"]["disc"]["samples"]
    # Returns sit in fine cells (0,0), (2,0) and (3,0): the disc covers regions 1 and 2 and empty cells.
    assert near["values"]["label"]["distinct"] == [0, 1, 2]
    assert far["status"] == "outside_grid" and far["cells"] == 0


def test_unbounded_cylinder_and_point_sized_sweep(cloud, tmp_path):
    out = cloud.agent_encode(
        {
            "above": pc.Overlap(pc.Cylinder(center=(0.25, 0.25), radius=0.1, z_range=(0.5, None))),
            "thin": pc.Sweep(
                pc.Cylinder((0, 0.25), 0.001, (0, 2)), direction_deg=0, max_distance=2
            ),
        },
        out_dir=tmp_path,
    )
    assert out["results"]["above"]["count"] == 1
    assert out["results"]["thin"]["status"] == "invalid"
    assert "body-sized" in out["results"]["thin"]["error"]


def test_masks_preserve_unknown_and_components_count_geometry(cloud, grid, tmp_path):
    height = pc.HeightField(grid)
    supported = pc.Threshold(height.count, ">", 0)
    raised = pc.Threshold(height.min, ">", 0.1)
    out = cloud.agent_encode(
        {
            "masked": pc.Window(supported & raised, (0, 0, 2, 2)),
            "regions": pc.Components(raised),
            "unknown": pc.Window(raised, (0, 0, 2, 2)),
            "inverted": pc.Window(~raised, (0, 0, 2, 2)),
        },
        out_dir=tmp_path,
    )
    assert out["results"]["masked"]["values"]["value"] == [[0, 1], [0, 0]]
    assert out["results"]["unknown"]["values"]["mask"] == [[0, 1], [None, None]]
    assert out["results"]["inverted"]["values"]["value"] == [[1, 0], [None, None]]
    assert out["results"]["regions"]["regions"] == [
        {"id": 1, "cells": 1, "centroid": [1.5, 0.5], "bounds": [[1, 0], [2, 1]]}
    ]


def test_frame_and_alignment_errors_do_not_erase_other_outputs(cloud, grid, tmp_path):
    wrong = pc.Grid((0, 0), (2, 2), 1, frame="other")
    offset = pc.Grid((0.5, 0), (2, 2), 1, frame="map")
    out = cloud.agent_encode(
        {
            "frame": pc.HeightField(wrong),
            "alignment": pc.HeightField(grid).min - pc.HeightField(offset).min,
            "good": pc.Sample(pc.HeightField(grid), [(0.5, 0.5)]),
        },
        out_dir=tmp_path,
    )
    assert out["results"]["frame"]["status"] == "invalid"
    assert out["results"]["alignment"]["status"] == "invalid"
    assert out["results"]["good"]["samples"][0]["values"]["min"] == 0


def test_budget_does_not_change_numeric_resolution(cloud, tmp_path):
    grid = pc.Grid((0, 0), (32, 32), 0.125)
    height = pc.HeightField(grid)
    out = cloud.agent_encode(
        {"big": pc.Window(height, (0, 0, 32, 32)), "small": pc.Sample(height.min, [(0.25, 0.25)])},
        budget=pc.EncodeBudget(2048),
        out_dir=tmp_path,
    )
    oversized = out["results"]["big"]
    assert oversized["status"] == "too_large"
    assert oversized["result_bytes"] > 2048
    assert oversized["response_bytes"] > oversized["max_text_bytes"] == 2048
    assert "small Sample" in oversized["suggestion"]
    assert out["results"]["small"]["grid"]["cell_m"] == 0.125
    assert out["results"]["small"]["samples"][0]["values"]["min"] == 0
    assert len(json.dumps(out, ensure_ascii=True, allow_nan=False).encode()) <= 2048

    recovered = cloud.agent_encode(
        {"big": pc.Window(height, (2, 2, 1, 1), fields=("min",))},
        budget=pc.EncodeBudget(2048),
        out_dir=tmp_path,
    )
    assert recovered["results"]["big"]["status"] == "ok"
    assert recovered["results"]["big"]["values"] == {"min": [[0]]}


def test_irreducible_metadata_returns_bounded_terminal_envelope(tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array([[0, 0, 0]], dtype=np.float32), frame_id="f" * 2000, timestamp=7
    )

    out = cloud.agent_encode(
        {"sample": pc.Sample(pc.HeightField(pc.Grid((0, 0), (1, 1), 1)), [(0.5, 0.5)])},
        budget=pc.EncodeBudget(1024),
        out_dir=tmp_path,
    )

    assert out["status"] == "too_large"
    assert out["minimum_envelope_bytes"] > out["max_text_bytes"] == 1024
    assert out["output_names"] == ["sample"]
    assert len(json.dumps(out, allow_nan=False).encode()) <= 1024


def test_field_renders_use_distinct_files_and_registered_overlays(cloud, grid, tmp_path):
    height = pc.HeightField(grid)
    closest = pc.Closest(pc.Sphere((0.5, 0.5, 0), 0))
    out = cloud.agent_encode(
        {
            "min": pc.Map(height.min, max_side=32, overlays=(closest,)),
            "max": pc.Map(height.max, max_side=32),
        },
        out_dir=tmp_path,
    )
    low, high = out["results"]["min"], out["results"]["max"]
    assert low["image"] != high["image"]
    assert low["pixels_per_cell"] == 16
    assert "point" in low["overlays"][0]["geometry"]
    with Image.open(low["image"]) as picture:
        assert picture.size == (32, 32)
    assert low["grid"] == high["grid"]


def test_three_dimensional_sweep_checks_nonmultiple_endpoint(tmp_path):
    cloud = PointCloud2.from_numpy(np.array([[0, 0, 0.13]], dtype=np.float32), frame_id="map")
    out = cloud.agent_encode(
        {
            "sweep": pc.Sweep(
                pc.Sphere((0, 0, 0), 0.025), direction=(0, 0, 1), max_distance=0.13, step_m=0.05
            )
        },
        out_dir=tmp_path,
    )
    assert out["results"]["sweep"]["status"] == "ok"
    assert out["results"]["sweep"]["hit"] is True
    assert out["results"]["sweep"]["distance_m"] == 0.13


def test_non_finite_request_returns_json_safe_invalid_result(cloud, tmp_path):
    out = cloud.agent_encode(
        {"bad": pc.Sweep(pc.Sphere((0, 0, 0), 1), direction_deg=float("nan"))},
        out_dir=tmp_path,
    )

    assert out["results"]["bad"]["status"] == "invalid"
    assert out["results"]["bad"]["request"]["direction_deg"] == {"invalid_number": "nan"}
    json.dumps(out, allow_nan=False)


def test_other_projection_plane_and_negative_coordinates(tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array([[-0.75, 4, -0.75], [-0.75, 6, -0.75]], dtype=np.float32), frame_id="sensor"
    )
    grid = pc.Grid((-1, -1), (2, 2), 0.5, plane="xz", frame="sensor")
    out = cloud.agent_encode(
        {"sample": pc.Sample(pc.HeightField(grid), [(-0.75, -0.75)])}, out_dir=tmp_path
    )
    assert out["results"]["sample"]["samples"][0]["values"] == {"count": 2, "min": 4, "max": 6}


def test_overlap_selection_can_feed_a_field_and_depth_view(cloud, grid, tmp_path):
    overlap = pc.Overlap(pc.Box((0.25, 0.25, 0.5), (0.5, 0.5, 2)))
    out = cloud.agent_encode(
        {
            "selected": pc.Window(pc.HeightField(grid, overlap.selection), (0, 0, 2, 1)),
            "view": pc.DepthView(
                view=(-1, 0.25, 0.5, 0, 0), source=overlap.selection, overlays=(overlap,)
            ),
        },
        out_dir=tmp_path,
    )
    assert out["results"]["selected"]["values"]["count"] == [[2, 0]]
    assert out["results"]["view"]["covered_pixels"] > 0
    assert "shape" in out["results"]["view"]["overlays"][0]["geometry"]


def test_explicit_occupancy_grid_preserves_field_coordinates(cloud, grid, tmp_path):
    out = cloud.agent_encode(
        {
            "occupancy": pc.OccupancyMap(z_range=(0.4, 2), grid=grid),
            "heights": pc.Sample(pc.HeightField(grid), [(0.5, 0.5)]),
        },
        out_dir=tmp_path,
    )
    occupancy = out["results"]["occupancy"]
    assert occupancy["cell_m"] == 1
    assert occupancy["origin_xy"] == [0, 0]
    assert occupancy["size"] == [2, 2]
    assert occupancy["occupied_cells"] == 2
    assert occupancy["unseen_cells"] == 2


def test_mask_distances_use_cell_centres(cloud, grid, tmp_path):
    occupied = pc.Threshold(pc.HeightField(grid).count, ">", 0)
    out = cloud.agent_encode(
        {"distance": pc.Window(pc.DistanceField(grid, occupied), (0, 0, 2, 2))}, out_dir=tmp_path
    )
    assert out["results"]["distance"]["values"]["distance_m"] == [[0, 0], [1, 1]]


def test_default_artifacts_use_state_or_current_run_directory(cloud, grid, tmp_path, monkeypatch):
    state_dir = tmp_path / "state"
    monkeypatch.delenv("AGENT_ENCODE_DIR", raising=False)
    monkeypatch.delenv("DIMOS_RUN_LOG_DIR", raising=False)
    monkeypatch.setattr(render, "STATE_DIR", state_dir)

    out = cloud.agent_encode({"map": pc.Map(pc.HeightField(grid).min, max_side=8)})

    image = Path(out["results"]["map"]["image"])
    assert image.parent == state_dir / "agent_encode"
    assert image.is_file()

    run_dir = tmp_path / "run"
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(run_dir))
    out = cloud.agent_encode({"map": pc.Map(pc.HeightField(grid).min, max_side=8)})
    assert Path(out["results"]["map"]["image"]).parent == run_dir / "agent_encode"


@pytest.mark.parametrize("frame_id", ["../escaped", "sensor/child", "/absolute", "sensor a:$"])
def test_artifact_names_are_confined_and_deterministic(frame_id, tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array([[0.5, 0.5, 0], [0.5, 0.5, 1]], dtype=np.float32),
        frame_id=frame_id,
        timestamp=7,
    )
    grid = pc.Grid((0, 0), (1, 1), 1)
    request = {
        "minimum": pc.Map(pc.HeightField(grid).min, max_side=8),
        "maximum": pc.Map(pc.HeightField(grid).max, max_side=8),
    }
    out_dir = tmp_path / "artifacts"

    first = cloud.agent_encode(request, out_dir=out_dir)
    second = cloud.agent_encode(request, out_dir=out_dir)

    assert first == second
    paths = [Path(result["image"]) for result in first["results"].values()]
    assert len(set(paths)) == 2
    assert all(path.parent == out_dir.resolve() and path.is_file() for path in paths)
    assert all(".." not in path.name and "/" not in path.name for path in paths)


def test_relative_artifact_directories_are_resolved_with_explicit_precedence(
    cloud, grid, tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("AGENT_ENCODE_DIR", "environment")
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", "run")

    explicit = cloud.agent_encode(
        {"map": pc.Map(pc.HeightField(grid).min, max_side=8)}, out_dir="explicit"
    )
    assert Path(explicit["results"]["map"]["image"]).parent == tmp_path / "explicit"

    environment = cloud.agent_encode({"map": pc.Map(pc.HeightField(grid).min, max_side=8)})
    assert Path(environment["results"]["map"]["image"]).parent == tmp_path / "environment"

    monkeypatch.delenv("AGENT_ENCODE_DIR")
    run = cloud.agent_encode({"map": pc.Map(pc.HeightField(grid).min, max_side=8)})
    assert Path(run["results"]["map"]["image"]).parent == tmp_path / "run" / "agent_encode"


def test_named_render_failure_is_contained(cloud, grid, tmp_path):
    unwritable = tmp_path / "not-a-directory"
    unwritable.write_text("file blocks artifact directory")

    out = cloud.agent_encode(
        {
            "render": pc.Map(pc.HeightField(grid).min, max_side=8),
            "sample": pc.Sample(pc.HeightField(grid), [(0.5, 0.5)]),
        },
        out_dir=unwritable,
    )

    assert out["results"]["render"]["status"] == "invalid"
    assert out["results"]["render"]["error_type"] == "FileExistsError"
    assert out["results"]["sample"]["status"] == "ok"
    assert out["results"]["sample"]["samples"][0]["values"]["min"] == 0


def test_grid_overlay_projection_targets_pixel_centres():
    assert grid_pixel(np.array([0.5, 0.5, 4.0]), (0, 1), (0, 0), 1, 2, 10) == (
        4.5,
        14.5,
    )
    assert grid_pixel(np.array([-0.5, 4.0, -0.5]), (0, 2), (-1, -1), 1, 2, 10) == (
        4.5,
        14.5,
    )


def test_grid_overlay_is_drawn_at_projected_pixel_centre(cloud, tmp_path):
    path = tmp_path / "overlay.png"
    Image.new("RGB", (20, 20), "black").save(path)

    draw_overlays(
        path,
        (Segment(((0.5, 0.5, 0), (1.5, 0.5, 0))),),
        EncodeContext(cloud, cloud.points_f32(), tmp_path, "overlay"),
        lambda point: grid_pixel(point, (0, 1), (0, 0), 1, 2, 10),
    )

    with Image.open(path) as picture:
        assert picture.getpixel((5, 14)) == (255, 59, 204)
        assert picture.getpixel((5, 16)) == (0, 0, 0)
