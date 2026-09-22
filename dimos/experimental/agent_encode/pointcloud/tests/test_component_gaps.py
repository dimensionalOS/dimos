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

import numpy as np
import pytest

from dimos.experimental.agent_encode.pointcloud.fields import (
    Components,
    Grid,
    HeightField,
    Threshold,
)
from dimos.experimental.agent_encode.pointcloud.handlers.field_outputs import Window
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.mark.parametrize("channel, expected_gap", [("count", 0), ("min", None)])
def test_one_cell_gap_links_regions_without_filling_missing_or_false_cells(
    channel, expected_gap, tmp_path
):
    cloud = PointCloud2.from_numpy(
        np.array([[0.25, 0.25, 2], [2.25, 0.25, 4]], dtype=np.float32), frame_id="map"
    )
    height = HeightField(Grid((0, 0), (3, 1), 1))
    regions = Components(
        Threshold(getattr(height, channel), ">", 0), gap_cells=1, values=height.min
    )

    result = cloud.agent_encode(
        {"labels": Window(regions, (0, 0, 3, 1)), "regions": regions}, out_dir=tmp_path
    )["results"]

    assert result["labels"]["values"] == {"label": [[1, expected_gap, 1]]}
    assert result["regions"]["regions"] == [
        {
            "id": 1,
            "cells": 2,
            "centroid": [1.5, 0.5],
            "bounds": [[0, 0], [3, 1]],
            "valid_cells": 2,
            "min": 2,
            "p10": pytest.approx(2.2),
            "p50": 3,
            "p90": pytest.approx(3.8),
            "max": 4,
        }
    ]


def test_two_cell_gap_stays_separate_and_merged_ids_follow_raster_order(tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array(
            [[0.25, 0.25, 1], [2.25, 0.25, 1], [5.25, 0.25, 1], [7.25, 0.25, 1]],
            dtype=np.float32,
        ),
        frame_id="map",
    )
    height = HeightField(Grid((0, 0), (8, 1), 1))
    regions = Components(Threshold(height.count, ">", 0), gap_cells=1)

    result = cloud.agent_encode(
        {"labels": Window(regions, (0, 0, 8, 1)), "regions": regions}, out_dir=tmp_path
    )["results"]

    assert result["labels"]["values"] == {"label": [[1, 0, 1, 0, 0, 2, 0, 2]]}
    assert result["regions"]["region_count"] == 2
    assert result["regions"]["regions"] == [
        {"id": 1, "cells": 2, "centroid": [1.5, 0.5], "bounds": [[0, 0], [3, 1]]},
        {"id": 2, "cells": 2, "centroid": [6.5, 0.5], "bounds": [[5, 0], [8, 1]]},
    ]


def test_default_gap_preserves_original_components_and_metadata(tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array([[0.25, 0.25, 1], [2.25, 0.25, 1]], dtype=np.float32), frame_id="map"
    )
    height = HeightField(Grid((0, 0), (3, 1), 1))
    mask = Threshold(height.count, ">", 0)

    result = cloud.agent_encode(
        {
            "default": Window(Components(mask), (0, 0, 3, 1)),
            "zero": Window(Components(mask, gap_cells=0), (0, 0, 3, 1)),
            "table": Components(mask),
        },
        out_dir=tmp_path,
    )["results"]

    assert {**result["default"], "request": None} == {**result["zero"], "request": None}
    assert result["default"]["values"] == {"label": [[1, 0, 2]]}
    assert result["table"]["region_count"] == 2


@pytest.mark.parametrize(
    "connectivity, expected",
    [(4, [[1, 0, 0], [0, 0, 0], [0, 0, 2]]), (8, [[1, 0, 0], [0, 0, 0], [0, 0, 1]])],
)
def test_gap_linking_uses_manhattan_or_chebyshev_distance(connectivity, expected, tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array([[0.25, 0.25, 1], [2.25, 2.25, 1]], dtype=np.float32), frame_id="map"
    )
    height = HeightField(Grid((0, 0), (3, 3), 1))
    regions = Components(Threshold(height.count, ">", 0), connectivity=connectivity, gap_cells=1)

    result = cloud.agent_encode({"labels": Window(regions, (0, 0, 3, 3))}, out_dir=tmp_path)[
        "results"
    ]["labels"]

    assert result["values"] == {"label": expected}


def test_gap_connections_are_transitive_without_wrapping_grid_boundaries(tmp_path):
    cloud = PointCloud2.from_numpy(
        np.array(
            [[0.25, 0.25, 1], [2.25, 0.25, 1], [4.25, 0.25, 1], [0.25, 1.25, 1]],
            dtype=np.float32,
        ),
        frame_id="map",
    )
    height = HeightField(Grid((0, 0), (5, 2), 1))
    regions = Components(Threshold(height.count, ">", 0), gap_cells=1)

    out = cloud.agent_encode(
        {"labels": Window(regions, (0, 0, 5, 2)), "table": regions}, out_dir=tmp_path
    )["results"]

    assert out["labels"]["values"] == {"label": [[1, 0, 1, 0, 1], [1, 0, 0, 0, 0]]}
    assert out["table"]["region_count"] == 1


@pytest.mark.parametrize("gap_cells", [-1, 5, 1.5, True])
def test_invalid_gap_radius_is_rejected(gap_cells, tmp_path):
    cloud = PointCloud2.from_numpy(np.empty((0, 3), dtype=np.float32), frame_id="map")
    height = HeightField(Grid((0, 0), (1, 1), 1))
    regions = Components(Threshold(height.count, ">", 0), gap_cells=gap_cells)

    result = cloud.agent_encode({"regions": regions}, out_dir=tmp_path)["results"]["regions"]

    assert result["status"] == "invalid"
    assert "gap_cells" in result["error"]
