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

import numpy as np
import pytest

from dimos.experimental.agent_encode.pointcloud.fields import (
    Band,
    Components,
    Grid,
    HeightField,
    Select,
    Threshold,
)
from dimos.experimental.agent_encode.pointcloud.handlers.field_outputs import Window
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def strip():
    """Four regions along x: two cells below zero, one cell, two cells, one cell."""
    points = [
        [0.25, 0.25, -1],
        [1.25, 0.25, -1],
        [3.25, 0.25, 3],
        [5.25, 0.25, 5],
        [6.25, 0.25, 6],
        [8.25, 0.25, 8],
    ]
    return PointCloud2.from_numpy(np.array(points, dtype=np.float32), frame_id="map")


def test_region_statistics_cover_only_the_regions_finite_cells(strip, tmp_path):
    grid = Grid((0, 0), (9, 1), 1)
    mask = Threshold(HeightField(grid).count, ">", 0)
    above_zero = HeightField(grid, Select(Band("z", 0))).min

    out = strip.agent_encode({"regions": Components(mask, values=above_zero)}, out_dir=tmp_path)

    regions = out["results"]["regions"]["regions"]
    assert [region["id"] for region in regions] == [1, 2, 3, 4]
    assert regions[0] == {
        "id": 1,
        "cells": 2,
        "centroid": [1, 0.5],
        "bounds": [[0, 0], [2, 1]],
        "valid_cells": 0,
        "min": None,
        "p10": None,
        "p50": None,
        "p90": None,
        "max": None,
    }
    assert regions[2] == {
        "id": 3,
        "cells": 2,
        "centroid": [6, 0.5],
        "bounds": [[5, 0], [7, 1]],
        "valid_cells": 2,
        "min": 5,
        "p10": pytest.approx(5.1),
        "p50": 5.5,
        "p90": pytest.approx(5.9),
        "max": 6,
    }
    json.dumps(out, allow_nan=False)


def test_region_limit_keeps_the_largest_and_counts_the_rest_while_labels_stay_complete(
    strip, tmp_path
):
    grid = Grid((0, 0), (9, 1), 1)
    regions = Components(Threshold(HeightField(grid).count, ">", 0), max_regions=2)

    out = strip.agent_encode(
        {"regions": regions, "labels": Window(regions, (0, 0, 9, 1))}, out_dir=tmp_path
    )["results"]

    assert [region["id"] for region in out["regions"]["regions"]] == [1, 3]
    assert out["regions"]["region_count"] == 4
    assert out["regions"]["omitted_regions"] == 2
    assert out["regions"]["omitted_cells"] == 2
    assert out["labels"]["values"] == {"label": [[1, 1, 0, 2, 0, 3, 3, 0, 4]]}


def test_table_without_options_is_unchanged(strip, tmp_path):
    grid = Grid((0, 0), (9, 1), 1)

    out = strip.agent_encode(
        {"regions": Components(Threshold(HeightField(grid).count, ">", 0))}, out_dir=tmp_path
    )["results"]["regions"]

    assert out["regions"][1] == {
        "id": 2,
        "cells": 1,
        "centroid": [3.5, 0.5],
        "bounds": [[3, 0], [4, 1]],
    }
    assert "omitted_regions" not in out and "omitted_cells" not in out


def test_empty_mask_has_no_regions_and_nothing_omitted(tmp_path):
    cloud = PointCloud2.from_numpy(np.empty((0, 3), dtype=np.float32), frame_id="map")
    height = HeightField(Grid((0, 0), (2, 2), 0.5))

    out = cloud.agent_encode(
        {"regions": Components(Threshold(height.count, ">", 0), values=height.min, max_regions=3)},
        out_dir=tmp_path,
    )["results"]["regions"]

    assert out["status"] == "ok"
    assert out["region_count"] == 0 and out["regions"] == []
    assert out["omitted_regions"] == 0 and out["omitted_cells"] == 0


def test_values_on_another_grid_are_rejected(strip, tmp_path):
    mask = Threshold(HeightField(Grid((0, 0), (9, 1), 1)).count, ">", 0)
    shifted = HeightField(Grid((0.5, 0), (9, 1), 1)).min

    out = strip.agent_encode({"regions": Components(mask, values=shifted)}, out_dir=tmp_path)

    assert out["results"]["regions"]["status"] == "invalid"
    assert "grids differ" in out["results"]["regions"]["error"]


@pytest.mark.parametrize("max_regions", [0, -1, 1.5, True])
def test_invalid_region_limit_is_rejected(max_regions, strip, tmp_path):
    mask = Threshold(HeightField(Grid((0, 0), (9, 1), 1)).count, ">", 0)

    out = strip.agent_encode(
        {"regions": Components(mask, max_regions=max_regions)}, out_dir=tmp_path
    )["results"]["regions"]

    assert out["status"] == "invalid"
    assert "max_regions" in out["error"]


def test_view_reference_roundtrips_components_with_values(strip, tmp_path):
    height = HeightField(Grid((0, 0), (9, 1), 1))
    regions = Components(Threshold(height.count, ">", 0), values=height.min, max_regions=2)
    view = strip.agent_encode({"view": PointCloud2.Map(regions, max_side=90)}, out_dir=tmp_path)[
        "results"
    ]["view"]
    reference = json.loads(json.dumps(view["view_ref"]))

    picked = strip.agent_encode(
        {"picked": PointCloud2.Pick(reference, uv=(55, 5))}, out_dir=tmp_path
    )["results"]["picked"]

    assert picked["status"] == "ok"
    assert picked["cells"][0]["value"] == 3
