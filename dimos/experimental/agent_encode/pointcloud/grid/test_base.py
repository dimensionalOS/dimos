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

from dimos.experimental.agent_encode.pointcloud.grid.base import Grid
from dimos.experimental.agent_encode.pointcloud.grid.count import Count
from dimos.experimental.agent_encode.pointcloud.grid.z_max import ZMax
from dimos.experimental.agent_encode.pointcloud.grid.z_min import ZMin
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

nan = np.nan


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
def area():
    """Two by two 1 m cells from the origin."""
    return ((0.5, 0.5), (1.5, 1.5))


def test_cells_are_half_open_and_sampled_by_containing_cell(cloud, area):
    count, low, high = (grid(1, area).run(cloud) for grid in (Count, ZMin, ZMax))

    assert count.origin == (0, 0) and count.shape == (2, 2)
    np.testing.assert_array_equal(count.values, [[2, 1], [0, 0]])
    np.testing.assert_array_equal(low.values, [[0, 0.5], [nan, nan]])
    np.testing.assert_array_equal(high.values, [[1, 0.5], [nan, nan]])
    assert count.cloud is cloud
    assert low.cell_of((0.5, 0.5)) == (0, 0)
    assert low.at((0.5, 0.5)) == 0
    assert low.at((0.5, 1.5)) is None
    assert low.at((2, 0)) is None
    assert count.window(((0, 0), (2, 2))) == [[0, 0], [2, 1]]


def test_masks_preserve_unknown_and_regions_count_geometry(cloud, area):
    supported = Count(1, area).run(cloud) > 0
    raised = ZMin(1, area).run(cloud) > 0.1

    np.testing.assert_array_equal((supported & raised).values, [[0, 1], [0, 0]])
    np.testing.assert_array_equal(raised.values, [[0, 1], [nan, nan]])
    np.testing.assert_array_equal((~raised).values, [[1, 0], [nan, nan]])
    np.testing.assert_array_equal((supported | raised).values, [[1, 1], [nan, nan]])
    assert raised.cloud is None
    (region,) = raised.regions()
    assert (region.id, region.cell_count, region.centroid, region.bounds) == (
        1,
        1,
        (1.5, 0.5),
        ((1, 0), (2, 1)),
    )


def test_logic_requires_masks(cloud, area):
    count = Count(1, area).run(cloud)

    with pytest.raises(ValueError, match="requires a mask"):
        _ = count & (count > 0)


def test_grids_combine_over_the_cells_both_cover(cloud, area):
    both = Count(1, area).run(cloud)
    right = Count(1, ((1.5, 0.5), (2.5, 0.5))).run(cloud)

    combined = (both > 0) & (right > 0)

    assert combined.origin == (1, 0) and combined.shape == (1, 1)
    np.testing.assert_array_equal(combined.values, [[1]])


def test_misaligned_or_unnested_cells_are_rejected(cloud, area):
    low = ZMin(1, area).run(cloud)
    offset = Grid((0.5, 0), 1, np.zeros((2, 2)))

    with pytest.raises(ValueError, match="not aligned"):
        _ = low - offset
    with pytest.raises(ValueError, match="do not nest"):
        _ = (Count(1, area).run(cloud) > 0) & (Count(0.4, area).run(cloud) > 0)


def test_nested_cells_combine_at_the_finer_cell(cloud, area):
    coarse, fine = Count(1, area).run(cloud) > 0, Count(0.5, area).run(cloud) > 0

    both = coarse & fine

    refined = np.repeat(np.repeat(coarse.values, 2, 0), 2, 1)[1:, 1:]
    assert (both.cell_m, both.origin, both.shape) == (0.5, fine.origin, fine.shape)
    np.testing.assert_array_equal(both.values, np.minimum(fine.values, refined))


def test_mask_distances_use_cell_centres(cloud, area):
    occupied = Count(1, area).run(cloud) > 0

    np.testing.assert_array_equal(occupied.distance().values, [[0, 0], [1, 1]])


def test_disc_reports_every_region_it_covers(cloud):
    fine = Count(0.5, ((0.25, 0.25), (1.75, 0.75))).run(cloud)
    regions = (fine > 0).regions()

    assert fine.shape == (4, 2)
    assert [region.id for region in regions.near((1.0, 0.25), radius=0.8)] == [1, 2]
    assert regions.near((9, 9), radius=0.8) == []
    assert fine.near((1.0, 0.25), radius=0.8) == (0, 2)
    assert fine.near((9, 9), radius=0.8) is None


def test_constructors_refuse_more_cells_than_the_limit(cloud):
    with pytest.raises(ValueError, match="larger cell_m"):
        Count(0.001, ((0, 0), (1000, 1000))).run(cloud)
