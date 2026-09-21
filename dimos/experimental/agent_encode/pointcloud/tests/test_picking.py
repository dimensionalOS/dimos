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

"""Visual picks retain actual returns, raster coordinates, and portable source semantics."""

from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image
import pytest

from dimos.experimental.agent_encode.pointcloud import constants
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as P


@pytest.fixture
def cloud():
    return P.from_numpy(
        np.array([[2, 0, 0], [4, 0, -1]], dtype=np.float32), frame_id="sensor", timestamp=7
    )


def result(cloud, node, tmp_path):
    return cloud.agent_encode({"value": node}, out_dir=tmp_path)["results"]["value"]


@pytest.fixture
def view():
    return P.DepthView((0, 0, 1, 0, 0), size=(100, 100), point_size_m=0.4)


@pytest.mark.parametrize(
    "uv,provenance",
    [((50, 75), "projected_return"), ((50, 80), "splat"), ((50, 81), "filled_pixel")],
)
def test_depth_returns_original_point_through_splat_and_fill(cloud, view, uv, provenance, tmp_path):
    ref = result(cloud, view, tmp_path)["view_ref"]

    picked = result(cloud, P.Pick(ref, uv=uv), tmp_path)

    assert picked["measurement_status"] == "hit"
    assert picked["point_m"] == [2, 0, 0]
    assert picked["forward_depth_m"] == 2
    assert picked["projected_uv"] == [50, 75]
    assert picked["pixel_provenance"] == provenance


@pytest.mark.parametrize("yaw,pitch", [(0, 30), (90, -30), (180, 0)])
def test_projection_uses_camera_pitch_heading_and_cloud_frame(yaw, pitch, tmp_path):
    pose = render.View(1, -3, 2, yaw, pitch)
    forward, right, up = pose.axes()
    point = np.array([1, -3, 2]) + 3 * forward + 0.2 * right + 0.1 * up
    cloud = P.from_numpy(np.array([point], dtype=np.float32), frame_id="optical", timestamp=9)
    view = P.DepthView((1, -3, 2, yaw, pitch), size=(100, 100), point_size_m=0)
    ref = result(cloud, view, tmp_path)["view_ref"]

    picked = result(cloud, P.Pick(ref, uv=(53, 48)), tmp_path)

    assert picked["point_m"] == pytest.approx(point)
    assert picked["forward_depth_m"] == pytest.approx(3)
    assert picked["projected_uv"] == pytest.approx([53 + 1 / 3, 48 + 1 / 3])


def test_occlusion_ties_missing_and_clipped_neighborhood(cloud, view, tmp_path):
    ref = result(cloud, view, tmp_path)["view_ref"]
    picked = result(cloud, P.Pick(ref, uv=(50, 75), radius_px=5), tmp_path)
    assert picked["hit_count"] == 1
    assert picked["point_id"] == 0
    assert picked["depth_span_m"] == [2, 2]
    assert result(cloud, P.Pick(ref, uv=(0, 0), radius_px=2), tmp_path)["selected_pixels"] == 9
    assert result(cloud, P.Pick(ref, uv=(0, 0)), tmp_path)["measurement_status"] == "no_return"
    assert (
        result(cloud, P.Pick(ref, uv=(100, 0)), tmp_path)["measurement_status"] == "outside_image"
    )
    duplicate = P.from_numpy(np.array([[2, 0, 0], [2, 0, 0]], dtype=np.float32), frame_id="sensor")
    ref = result(duplicate, view, tmp_path)["view_ref"]
    assert result(duplicate, P.Pick(ref, uv=(50, 75)), tmp_path)["point_id"] == 0


def test_distinct_region_hits_report_span_and_keep_all_returns(tmp_path):
    cloud = P.from_numpy(np.array([[2, -0.5, 0], [4, 0.5, 0]], dtype=np.float32))
    view = P.DepthView((0, 0, 0, 0, 0), size=(100, 100), point_size_m=0)
    ref = result(cloud, view, tmp_path)["view_ref"]
    picked = result(cloud, P.Pick(ref, rect=(40, 45, 30, 10), max_items=1), tmp_path)
    assert picked["measurement_status"] == "ambiguous"
    assert picked["hit_count"] == 2 and picked["hits_omitted"] == 1
    assert picked["depth_span_m"] == [2, 4]
    selection = P.SelectionRef(picked["selection_ref"])
    assert (
        result(cloud, P.Overlap(P.Sphere((0, 0, 0), 10), source=selection), tmp_path)["count"] == 2
    )


@pytest.mark.parametrize("plane", ["xy", "xz", "yz"])
def test_height_cell_bounds_north_up_edges_and_selected_source(plane, tmp_path):
    axes = ["xyz".index(c) for c in plane]
    normal = next(i for i in range(3) if i not in axes)
    points = np.zeros((4, 3), dtype=np.float32)
    points[:, axes] = [[-0.75, -0.75], [-0.75, -0.75], [-0.5, -0.75], [-0.75, -0.75]]
    points[:, normal] = [2, 4, 6, 9]
    cloud = P.from_numpy(points, frame_id="custom")
    grid = P.Grid((-1, -1), (2, 2), 0.5, plane=plane, frame="custom")
    source = P.Select(include=P.Band("xyz"[normal], None, 8))
    field = P.HeightField(grid, source)
    ref = result(cloud, P.Map(field.min, max_side=20), tmp_path)["view_ref"]
    picked = result(cloud, P.Pick(ref, uv=(0, 10)), tmp_path)
    cell = picked["cells"][0]
    assert cell["cell"] == [0, 0]
    assert cell["bounds_m"] == [[-1, -1], [-0.5, -0.5]]
    assert cell["centre_m"] == [-0.75, -0.75]
    assert (cell["count"], cell["min_m"], cell["max_m"], cell["value"]) == (2, 2, 4, 2)
    edge = result(cloud, P.Pick(ref, uv=(10, 10)), tmp_path)["cells"][0]
    assert edge["cell"] == [1, 0] and edge["count"] == 1 and edge["value"] == 6
    missing = result(cloud, P.Pick(ref, uv=(0, 0)), tmp_path)
    assert missing["measurement_status"] == "no_return"
    assert missing["cells"][0]["count"] == 0
    assert missing["cells"][0]["min_m"] is None


def test_derived_field_values_do_not_claim_remote_targets_as_local_points(tmp_path):
    cloud = P.from_numpy(np.array([[5, 5, 1]], dtype=np.float32))
    grid = P.Grid((0, 0), (2, 2), 1)
    distance = P.DistanceField(grid)
    for field, expected in (
        (distance, np.hypot(4.5, 4.5)),
        (P.Threshold(distance, ">", 1), 1),
        (P.Components(P.Threshold(distance, ">", 1)), 1),
    ):
        ref = result(cloud, P.Map(field, max_side=20), tmp_path)["view_ref"]
        picked = result(cloud, P.Pick(ref, uv=(0, 10)), tmp_path)
        assert picked["measurement_status"] == "field_value"
        assert picked["cells"][0]["value"] == pytest.approx(expected)
        assert picked["cells"][0]["count"] is None
        assert picked["selected_return_count"] == 0


def test_occupancy_fixed_grid_band_height_and_dilated_empty_cells(tmp_path):
    cloud = P.from_numpy(
        np.array([[0.25, 0.25, 0], [1.25, 0.25, 2], [1.25, 0.25, 9]], dtype=np.float32)
    )
    grid = P.Grid((0, 0), (3, 2), 1)
    view = P.OccupancyMap((1, 3), grid=grid, free_radius=1, colour="height")
    ref = result(cloud, view, tmp_path)["view_ref"]
    scale = round(1024 / 3)
    occupied = result(cloud, P.Pick(ref, uv=(scale, scale)), tmp_path)["cells"][0]
    assert occupied["classification"] == "occupied"
    assert occupied["count"] == 1 and occupied["height_colour_m"] == 2
    empty = result(cloud, P.Pick(ref, uv=(0, 0)), tmp_path)
    assert empty["measurement_status"] == "no_return"
    assert empty["cells"][0]["classification"] == "free"
    assert empty["cells"][0]["count"] == 0 and empty["cells"][0]["min_m"] is None


def test_occupancy_crop_uses_actual_origin_and_builder_cell_ownership(tmp_path):
    points = np.array(
        [
            [x, y, z]
            for x, y, z in [(0.13, -0.17, 0), (0.43, 0.23, 1), (0.77, 0.58, 2), (1.11, 1.22, 4)]
        ],
        dtype=np.float32,
    )
    cloud = P.from_numpy(points, frame_id="map")
    view = P.OccupancyMap((0.5, 3), zoom=(0.6, 0.4, 0.4), max_cells=32, colour="height")
    output = result(cloud, view, tmp_path)
    picked = result(cloud, P.Pick(output["view_ref"], uv=(0, 0)), tmp_path)
    spec = picked["grid"]
    with Image.open(output["image"]) as picture:
        assert picked["image_size"] == list(picture.size)
    assert picked["cells"][0]["cell"] == [0, spec["shape"][1] - 1]
    assert picked["cells"][0]["bounds_m"][0] == [
        spec["origin"][0],
        spec["origin"][1] + (spec["shape"][1] - 1) * spec["cell_m"],
    ]
    assert spec["origin"] != output["origin_xy"]


def test_polygon_and_rectangle_deduplicate_and_limits_are_contained(cloud, view, tmp_path):
    ref = result(cloud, view, tmp_path)["view_ref"]
    rectangle = result(cloud, P.Pick(ref, rect=(45, 70, 11, 11)), tmp_path)
    polygon = result(cloud, P.Pick(ref, polygon=((45, 70), (56, 70), (56, 81), (45, 81))), tmp_path)
    assert rectangle["hit_count"] == polygon["hit_count"] == 1
    assert rectangle["selected_pixel_count"] == polygon["selected_pixel_count"] == 121
    for bad in (
        P.Pick(ref, uv=(1, 1), rect=(0, 0, 2, 2)),
        P.Pick(ref, uv=(1.5, 2)),
        P.Pick(ref, uv=(1, 1), radius_px=65),
        P.Pick(ref, rect=(-1, 0, 3, 3)),
        P.Pick(ref, polygon=((0, 0), (1, 1))),
        P.Pick(ref, uv=(1, 1), max_items=65),
    ):
        assert result(cloud, bad, tmp_path)["status"] == "invalid"
    large = result(cloud, replace(view, size=(300, 300)), tmp_path)["view_ref"]
    assert result(cloud, P.Pick(large, rect=(0, 0, 300, 300)), tmp_path)["status"] == "invalid"


def test_json_roundtrip_in_new_process_and_nested_selected_view(cloud, view, tmp_path):
    ref = result(cloud, view, tmp_path)["view_ref"]
    pick = P.Pick(ref, uv=(50, 75))
    selected = result(cloud, pick, tmp_path)["selection_ref"]
    code = """import json,sys,numpy as np
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as P
p=json.loads(sys.stdin.read())
c=P.from_numpy(np.array([[2,0,0],[4,0,-1]],dtype=np.float32),frame_id='sensor',timestamp=7)
s=P.SelectionRef(p['selection'])
v=P.DepthView((0,0,1,0,0),size=(100,100),point_size_m=.4,source=s)
r=c.agent_encode({'v':v},out_dir=p['out'])['results']['v']
print(json.dumps(c.agent_encode({'p':P.Pick(r['view_ref'],uv=(50,75))},out_dir=p['out'])))
"""
    process = subprocess.run(
        [sys.executable, "-c", code],
        input=json.dumps({"selection": selected, "out": str(tmp_path)}),
        capture_output=True,
        text=True,
        check=True,
        timeout=30,
        cwd=Path(__file__).parents[4],
    )
    picked = json.loads(process.stdout)["results"]["p"]
    assert picked["status"] == "ok" and picked["point_m"] == [2, 0, 0]


@pytest.mark.parametrize("change", ["point", "frame", "timestamp"])
def test_reference_rejects_stale_cloud_identity(cloud, view, change, tmp_path):
    ref = result(cloud, view, tmp_path)["view_ref"]
    points = np.array([[2, 0, 0], [4, 0, -1]], dtype=np.float32)
    if change == "point":
        points[1, 0] = 5
    other = P.from_numpy(
        points,
        frame_id="other" if change == "frame" else "sensor",
        timestamp=8 if change == "timestamp" else 7,
    )
    picked = result(other, P.Pick(ref, uv=(50, 75)), tmp_path)
    assert picked["status"] == "invalid" and "stale reference" in picked["error"]


def test_selection_composes_with_every_point_source_handler(cloud, view, tmp_path):
    ref = result(cloud, view, tmp_path)["view_ref"]
    pick = P.Pick(ref, uv=(50, 75))
    source = P.Select(source=pick.selection, include=P.Band("z", -0.1, 0.1))
    grid = P.Grid((1, -1), (2, 2), 1)
    sphere = P.Sphere((2, 0, 0), 0.1)
    requests = {
        "closest": P.Closest(sphere, source=source),
        "overlap": P.Overlap(sphere, source=source),
        "sweep": P.Sweep(sphere, direction_deg=0, max_distance=1, source=source),
        "depth": replace(view, source=source),
        "occupancy": P.OccupancyMap((-0.1, 0.1), source=source, grid=grid),
        "height": P.Sample(P.HeightField(grid, source), [(2, 0)]),
        "distance": P.Sample(P.DistanceField(grid, source), [(2, 0)]),
    }
    output = cloud.agent_encode(requests, out_dir=tmp_path, budget=P.EncodeBudget(65536))["results"]
    assert all(value["status"] == "ok" for value in output.values())
    assert output["overlap"]["count"] == 1
    assert output["height"]["samples"][0]["values"] == {"count": 1, "min": 0, "max": 0}
    assert output["closest"]["point_m"] == [2, 0, 0]


def test_overlay_confirmation_and_budget_preserve_measurement(cloud, view, tmp_path):
    plain = result(cloud, view, tmp_path)
    pick = P.Pick(plain["view_ref"], uv=(50, 80))
    overlay = result(cloud, replace(view, overlays=(pick,)), tmp_path)
    assert overlay["view_ref"] == plain["view_ref"]
    assert "picked_pixels" in overlay["overlays"][0]["geometry"]
    with Image.open(plain["image"]) as a, Image.open(overlay["image"]) as b:
        assert a.getpixel((50, 80)) != b.getpixel((50, 80))
    before = result(cloud, pick, tmp_path)
    small = cloud.agent_encode({"pick": pick}, out_dir=tmp_path, budget=P.EncodeBudget(1024))
    assert small["results"]["pick"]["status"] == "too_large"
    assert len(json.dumps(small).encode()) <= 1024
    after = result(cloud, pick, tmp_path)
    assert after == before
    grid = P.Grid((1, -1), (2, 2), 1)
    map_view = P.Map(P.HeightField(grid).min, max_side=20)
    map_ref = result(cloud, map_view, tmp_path)["view_ref"]
    map_pick = P.Pick(map_ref, uv=(10, 0))
    assert (
        "picked_pixels"
        in result(cloud, replace(map_view, overlays=(map_pick,)), tmp_path)["overlays"][0][
            "geometry"
        ]
    )


def test_reference_validation_refuses_tampering_and_nested_overflow(
    cloud, view, tmp_path, monkeypatch
):
    ref = result(cloud, view, tmp_path)["view_ref"]
    tampered = json.loads(json.dumps(ref))
    tampered["recipe"]["args"]["size"] = [10, 10]
    assert result(cloud, P.Pick(tampered, uv=(0, 0)), tmp_path)["status"] == "invalid"
    nested = {}
    nested["loop"] = nested
    assert result(cloud, P.Pick(nested, uv=(0, 0)), tmp_path)["status"] == "invalid"
    monkeypatch.setattr(constants, "FORM", "text")
    assert "image build" in result(cloud, P.Pick(ref, uv=(0, 0)), tmp_path)["error"]


def test_three_visual_pick_cycles_keep_original_cloud_binding(cloud, view, tmp_path):
    grid = P.Grid((1, -1), (2, 2), 1)
    ref = result(cloud, P.Map(P.HeightField(grid).max, max_side=20), tmp_path)["view_ref"]
    picked = result(cloud, P.Pick(ref, uv=(10, 0)), tmp_path)
    for _ in range(3):
        source = P.SelectionRef(json.loads(json.dumps(picked["selection_ref"])))
        filtered_view = result(cloud, replace(view, source=source), tmp_path)
        assert filtered_view["status"] == "ok"
        picked = result(cloud, P.Pick(filtered_view["view_ref"], uv=(50, 75)), tmp_path)
        assert picked["status"] == "ok" and picked["point_m"] == [2, 0, 0]
