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
from dimos.experimental.agent_encode.pointcloud.grid.regions import Stats
from dimos.experimental.agent_encode.pointcloud.grid.z_min import ZMin
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def row_cloud(*points):
    return PointCloud2.from_numpy(np.array(points, dtype=np.float32), frame_id="map")


def row(cells):
    """1 m cells from the origin along x, one row."""
    return ((0.5, 0.5), (cells - 0.5, 0.5))


@pytest.fixture
def strip():
    """Four regions along x: two cells below zero, one cell, two cells, one cell."""
    return row_cloud(
        [0.25, 0.25, -1],
        [1.25, 0.25, -1],
        [3.25, 0.25, 3],
        [5.25, 0.25, 5],
        [6.25, 0.25, 6],
        [8.25, 0.25, 8],
    )


@pytest.mark.parametrize("grid, expected_gap", [(Count, 0), (ZMin, np.nan)])
def test_one_cell_gap_links_regions_without_filling_missing_or_false_cells(grid, expected_gap):
    cloud = row_cloud([0.25, 0.25, 2], [2.25, 0.25, 4])

    regions = (grid(1, row(3)).run(cloud) > 0).regions(gap=1, measure=ZMin(1, row(3)).run(cloud))

    np.testing.assert_array_equal(regions.labels.values, [[1, expected_gap, 1]])
    (region,) = regions
    assert (region.id, region.cell_count, region.centroid, region.bounds) == (
        1,
        2,
        (1.5, 0.5),
        ((0, 0), (3, 1)),
    )
    assert region.stats == Stats(2, 2, pytest.approx(2.2), 3, pytest.approx(3.8), 4)


def test_two_cell_gap_stays_separate_and_merged_ids_follow_raster_order():
    cloud = row_cloud([0.25, 0.25, 1], [2.25, 0.25, 1], [5.25, 0.25, 1], [7.25, 0.25, 1])

    regions = (Count(1, row(8)).run(cloud) > 0).regions(gap=1)

    np.testing.assert_array_equal(regions.labels.values, [[1, 0, 1, 0, 0, 2, 0, 2]])
    assert [(r.id, r.cell_count, r.centroid, r.bounds) for r in regions] == [
        (1, 2, (1.5, 0.5), ((0, 0), (3, 1))),
        (2, 2, (6.5, 0.5), ((5, 0), (8, 1))),
    ]


def test_default_gap_preserves_original_components():
    cloud = row_cloud([0.25, 0.25, 1], [2.25, 0.25, 1])
    mask = Count(1, row(3)).run(cloud) > 0

    default, zero = mask.regions(), mask.regions(gap=0)

    np.testing.assert_array_equal(default.labels.values, zero.labels.values)
    np.testing.assert_array_equal(default.labels.values, [[1, 0, 2]])
    assert len(default) == 2


@pytest.mark.parametrize(
    "connectivity, expected",
    [(4, [[1, 0, 0], [0, 0, 0], [0, 0, 2]]), (8, [[1, 0, 0], [0, 0, 0], [0, 0, 1]])],
)
def test_gap_linking_uses_manhattan_or_chebyshev_distance(connectivity, expected):
    cloud = row_cloud([0.25, 0.25, 1], [2.25, 2.25, 1])
    mask = Count(1, ((0.5, 0.5), (2.5, 2.5))).run(cloud) > 0

    regions = mask.regions(gap=1, connectivity=connectivity)

    np.testing.assert_array_equal(regions.labels.values, expected)


def test_gap_connections_are_transitive_without_wrapping_grid_boundaries():
    cloud = row_cloud([0.25, 0.25, 1], [2.25, 0.25, 1], [4.25, 0.25, 1], [0.25, 1.25, 1])
    mask = Count(1, ((0.5, 0.5), (4.5, 1.5))).run(cloud) > 0

    regions = mask.regions(gap=1)

    np.testing.assert_array_equal(regions.labels.values, [[1, 0, 1, 0, 1], [1, 0, 0, 0, 0]])
    assert len(regions) == 1


@pytest.mark.parametrize("gap", [-1, 5, 1.5, True])
def test_invalid_gap_radius_is_rejected(gap):
    cloud = PointCloud2.from_numpy(np.empty((0, 3), dtype=np.float32), frame_id="map")
    mask = Count(1, row(1)).run(cloud) > 0

    with pytest.raises(ValueError, match="gap"):
        mask.regions(gap=gap)


def test_region_statistics_cover_only_the_regions_finite_cells(strip):
    mask = Count(1, row(9)).run(strip) > 0
    points = strip.points_f32()
    above_zero = ZMin(1, row(9)).run(row_cloud(*points[points[:, 2] >= 0]))

    regions = mask.regions(measure=above_zero)

    assert [region.id for region in regions] == [1, 3, 2, 4]
    first, second = regions[:2]
    assert (first.cell_count, first.centroid, first.bounds, first.stats) == (
        2,
        (1, 0.5),
        ((0, 0), (2, 1)),
        Stats(0, None, None, None, None, None),
    )
    assert (second.id, second.cell_count, second.centroid, second.bounds) == (
        3,
        2,
        (6, 0.5),
        ((5, 0), (7, 1)),
    )
    assert second.stats == Stats(2, 5, pytest.approx(5.1), 5.5, pytest.approx(5.9), 6)


def test_regions_are_largest_first_while_labels_stay_complete(strip):
    regions = (Count(1, row(9)).run(strip) > 0).regions()

    assert [(region.id, region.cell_count) for region in regions] == [
        (1, 2),
        (3, 2),
        (2, 1),
        (4, 1),
    ]
    np.testing.assert_array_equal(regions.labels.values, [[1, 1, 0, 2, 0, 3, 3, 0, 4]])
    single = regions[2]
    assert (single.centroid, single.bounds, single.stats) == ((3.5, 0.5), ((3, 0), (4, 1)), None)


def test_empty_mask_has_no_regions():
    cloud = PointCloud2.from_numpy(np.empty((0, 3), dtype=np.float32), frame_id="map")
    area = ((0.25, 0.25), (0.75, 0.75))

    regions = (Count(0.5, area).run(cloud) > 0).regions(measure=ZMin(0.5, area).run(cloud))

    assert len(regions) == 0 and regions[:] == ()


def test_measure_on_other_cells_is_rejected(strip):
    mask = Count(1, row(9)).run(strip) > 0
    shifted = ZMin(1, ((1.5, 0.5), (8.5, 0.5))).run(strip)

    with pytest.raises(ValueError, match="same cells"):
        mask.regions(measure=shifted)
