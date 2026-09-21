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

from dimos.experimental.agent_encode.pointcloud.fields import Band, Grid, HeightField, Select
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def context(tmp_path):
    points = np.array(
        [
            [0.25, 0.25, 10],
            [1.25, 0.25, 100],
            [0.25, 1.25, -5],
            [0.25, 0.25, 0],
            [1.25, 0.25, 0],
            [0.25, 1.25, -2],
            [0.25, 0.25, 4],
            [0.25, 1.25, -1],
            [0.25, 0.25, 2],
            [0.25, 1.25, 1],
            [0.25, 1.25, 9],
            [2, 0.25, -100],
        ],
        dtype=np.float32,
    )
    cloud = PointCloud2.from_numpy(points, frame_id="map")
    return EncodeContext(cloud, points, tmp_path, "percentile")


@pytest.mark.parametrize("q", [0, 10, 25, 50, 100])
def test_percentile_interpolates_each_cells_selected_returns(context, q):
    height = HeightField(Grid((0, 0), (2, 2), 1))

    result = context.evaluate(height.percentile(q, min_count=1))

    expected = [
        [np.percentile([10, 0, 4, 2], q), np.percentile([100, 0], q)],
        [np.percentile([-5, -2, -1, 1, 9], q), np.nan],
    ]
    np.testing.assert_allclose(result.scalar(), expected)


def test_percentile_masks_insufficient_support_and_preserves_extrema(context):
    height = HeightField(Grid((0, 0), (2, 2), 1))

    result = context.cloud.agent_encode(
        {
            "lower_surface": PointCloud2.Window(height.percentile(10), (0, 0, 2, 2)),
            "heights": PointCloud2.Window(height, (0, 0, 2, 2)),
        },
        out_dir=context.out_dir,
    )["results"]

    assert result["lower_surface"]["values"] == {"percentile": [[0.6, None], [-3.8, None]]}
    assert result["lower_surface"]["coordinate"] == "z"
    assert result["lower_surface"]["units"] == {"percentile": "m"}
    assert result["lower_surface"]["q"] == 10
    assert result["lower_surface"]["min_count"] == 4
    assert result["lower_surface"]["method"] == "linear"
    assert result["heights"]["values"] == {
        "count": [[4, 2], [5, 0]],
        "min": [[0, 0], [-5, None]],
        "max": [[10, 100], [9, None]],
    }
    assert result["heights"]["coordinate"] == "z"
    assert result["heights"]["units"] == {"count": "returns", "min": "m", "max": "m"}


def test_percentile_uses_selection_and_remaining_axis(context):
    grid = Grid((0, -10), (2, 1), 20, plane="xz")
    source = Select(include=Band("x", 0, 1))
    height = HeightField(grid, source)

    result = context.evaluate(height.percentile(50, min_count=1))

    # The z=10 point lies on the excluded upper boundary; eight selected points remain.
    np.testing.assert_allclose(result.scalar(), [[1.25, np.nan]])
    assert result.metadata["coordinate"] == "y"


def test_percentile_view_roundtrip_preserves_selected_cell_returns(context):
    source = Select(include=Band("z", 0, 5))
    height = HeightField(Grid((0, 0), (2, 2), 1), source)
    view = context.cloud.agent_encode(
        {"view": PointCloud2.Map(height.percentile(10, min_count=3), max_side=20)},
        out_dir=context.out_dir,
    )["results"]["view"]
    reference = json.loads(json.dumps(view["view_ref"]))

    picked = context.cloud.agent_encode(
        {"picked": PointCloud2.Pick(reference, uv=(0, 10))}, out_dir=context.out_dir
    )["results"]["picked"]

    assert picked["cells"][0]["value"] == pytest.approx(0.4)
    assert picked["cells"][0]["count"] == 3
    assert picked["selected_return_count"] == 3
    selected = PointCloud2.SelectionRef(json.loads(json.dumps(picked["selection_ref"])))
    measurement = context.cloud.agent_encode(
        {"cell": PointCloud2.Window(HeightField(height.grid, selected), (0, 0, 2, 2))},
        out_dir=context.out_dir,
    )["results"]["cell"]
    assert measurement["values"] == {
        "count": [[3, 0], [0, 0]],
        "min": [[0, None], [None, None]],
        "max": [[4, None], [None, None]],
    }


@pytest.mark.parametrize("empty_cloud", [False, True])
def test_percentile_missing_cells_stay_missing(context, empty_cloud):
    if empty_cloud:
        cloud = PointCloud2.from_numpy(np.empty((0, 3), dtype=np.float32), frame_id="map")
        context = EncodeContext(cloud, cloud.points_f32(), context.out_dir, "empty")
    height = HeightField(Grid((0, 0), (2, 2), 1), Select(include=Band("z", 1000, 2000)))

    result = context.evaluate(height.percentile(10))

    np.testing.assert_array_equal(result.scalar(), np.full((2, 2), np.nan))


@pytest.mark.parametrize("q", [-1, 101, np.nan, np.inf, -np.inf])
def test_percentile_rejects_invalid_quantile(q):
    height = HeightField(Grid((0, 0), (1, 1), 1))

    with pytest.raises(ValueError, match="q must be finite and between 0 and 100"):
        height.percentile(q)


@pytest.mark.parametrize("min_count", [0, -1, 1.5, True])
def test_percentile_rejects_invalid_support(min_count):
    height = HeightField(Grid((0, 0), (1, 1), 1))

    with pytest.raises(ValueError, match="min_count must be a positive integer"):
        height.percentile(10, min_count=min_count)
