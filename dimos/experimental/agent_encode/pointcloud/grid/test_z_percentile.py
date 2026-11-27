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

from dimos.experimental.agent_encode.pointcloud.grid.count import Count
from dimos.experimental.agent_encode.pointcloud.grid.z_max import ZMax
from dimos.experimental.agent_encode.pointcloud.grid.z_min import ZMin
from dimos.experimental.agent_encode.pointcloud.grid.z_percentile import ZPercentile
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

nan = np.nan
AREA = ((0.5, 0.5), (1.5, 1.5))


@pytest.fixture
def cloud():
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
    return PointCloud2.from_numpy(points, frame_id="map")


@pytest.mark.parametrize("q", [0, 10, 25, 50, 100])
def test_percentile_interpolates_each_cells_returns(cloud, q):
    result = ZPercentile(q, 1, min_count=1, area=AREA).run(cloud)

    expected = [
        [np.percentile([10, 0, 4, 2], q), np.percentile([100, 0], q)],
        [np.percentile([-5, -2, -1, 1, 9], q), nan],
    ]
    np.testing.assert_allclose(result.values, expected)


def test_percentile_masks_insufficient_support_and_preserves_extrema(cloud):
    np.testing.assert_allclose(
        ZPercentile(10, 1, area=AREA).run(cloud).values, [[0.6, nan], [-3.8, nan]]
    )
    np.testing.assert_array_equal(Count(1, AREA).run(cloud).values, [[4, 2], [5, 0]])
    np.testing.assert_array_equal(ZMin(1, AREA).run(cloud).values, [[0, 0], [-5, nan]])
    np.testing.assert_array_equal(ZMax(1, AREA).run(cloud).values, [[10, 100], [9, nan]])


def test_percentile_missing_cells_stay_missing():
    cloud = PointCloud2.from_numpy(np.empty((0, 3), dtype=np.float32), frame_id="map")

    result = ZPercentile(10, 1, area=AREA).run(cloud)

    np.testing.assert_array_equal(result.values, np.full((2, 2), nan))


@pytest.mark.parametrize("q", [-1, 101, np.nan, np.inf, -np.inf])
def test_percentile_rejects_invalid_quantile(q):
    with pytest.raises(ValueError, match="q must be finite and between 0 and 100"):
        ZPercentile(q, 1)


@pytest.mark.parametrize("min_count", [0, -1, 1.5, True])
def test_percentile_rejects_invalid_support(min_count):
    with pytest.raises(ValueError, match="min_count must be a positive integer"):
        ZPercentile(10, 1, min_count=min_count)
