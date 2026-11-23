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


import pytest

from dimos.mapping.wifi.coverage import CoverageMap
from dimos.msgs.sensor_msgs.WifiScan import WifiAccessPoint, WifiScan


def observation(ts, bssid="aa:bb:cc:dd:ee:01", signal=-60):
    return WifiAccessPoint(ts, bssid, "shared-ssid", 5500, signal)


def test_cached_observation_is_mapped_at_capture_pose_not_current_pose():
    grid = CoverageMap(cell_size=1)
    grid.add_pose(10, "map", (0, 0, 0))
    grid.add_pose(11, "map", (2, 0, 0))
    grid.add_pose(12, "map", (4, 0, 0))
    assert grid.add_scan(WifiScan(12, "wifi", [observation(10.5)])) == 1
    cell = grid.selected_cells()[0]
    assert (cell.position, cell.last_seen) == ((1.5, 0.5, 0.5), 10.5)


def test_same_ssid_radios_stay_separate_and_strongest_layer_selects_per_cell():
    grid = CoverageMap()
    grid.add_pose(10, "map", (0, 0, 0))
    grid.add_scan(
        WifiScan(
            10, "wifi", [observation(10, signal=-80), observation(10, "aa:bb:cc:dd:ee:02", -50)]
        )
    )
    assert len(grid.cells) == 2
    assert grid.selected_cells()[0].bssid == "aa:bb:cc:dd:ee:02"
    assert grid.selected_cells("aa:bb:cc:dd:ee:01")[0].mean_dbm == -80


def test_duplicate_cached_observation_does_not_repaint_or_reweight():
    grid = CoverageMap()
    grid.add_pose(10, "map", (0, 0, 0))
    grid.add_pose(10.05, "map", (1, 0, 0))
    grid.add_scan(WifiScan(10, "wifi", [observation(10)]))
    assert grid.add_scan(WifiScan(11, "wifi", [observation(10.02)])) == 0
    assert len(grid.cells) == 1
    assert grid.selected_cells()[0].count == 1


@pytest.mark.parametrize("sample_ts,collection_ts", [(1, 10), (11, 10), (float("nan"), 10)])
def test_stale_future_or_invalid_observations_are_not_mapped(sample_ts, collection_ts):
    grid = CoverageMap()
    grid.add_pose(10, "map", (0, 0, 0))
    assert grid.add_scan(WifiScan(collection_ts, "wifi", [observation(sample_ts)])) == 0
    assert not grid.cells


def test_missing_or_stale_pose_does_not_paint_origin():
    grid = CoverageMap()
    assert grid.add_scan(WifiScan(10, "wifi", [observation(10)])) == 0
    grid.add_pose(8, "map", (0, 0, 0))
    assert grid.add_scan(WifiScan(10, "wifi", [observation(10)])) == 0
    assert not grid.cells


def test_revisits_update_count_mean_and_range():
    grid = CoverageMap()
    for ts, signal in [(1, -80), (2, -60)]:
        grid.add_pose(ts, "map", (0, 0, 0))
        grid.add_scan(WifiScan(ts, "wifi", [observation(ts, signal=signal)]))
    cell = grid.selected_cells()[0]
    assert (cell.count, cell.mean_dbm, cell.min_dbm, cell.max_dbm) == (2, -70, -80, -60)


def test_lru_bounds_map_memory():
    grid = CoverageMap(max_cells=2)
    for ts, position, signal in [
        (1, (0, 0, 0), -80),
        (2, (0, 0, 0), -60),
        (3, (1, 0, 0), -55),
        (4, (2, 0, 0), -50),
    ]:
        grid.add_pose(ts, "map", position)
        grid.add_scan(WifiScan(ts, "wifi", [observation(ts, signal=signal)]))
    assert len(grid.cells) == 2
    assert [cell.mean_dbm for cell in grid.selected_cells()] == [-55, -50]


def test_frame_change_discards_incompatible_coverage_and_pose_history():
    grid = CoverageMap()
    grid.add_pose(10, "odom", (5, 0, 0))
    grid.add_scan(WifiScan(10, "wifi", [observation(10)]))
    grid.add_pose(11, "map", (100, 0, 0))
    assert not grid.cells
    assert grid.position_at(10) is None
    assert grid.frame_id == "map"


def test_out_of_order_poses_are_sorted_without_erasing_the_map():
    grid = CoverageMap()
    grid.add_pose(11, "map", (2, 0, 0))
    grid.add_pose(10, "map", (0, 0, 0))
    assert grid.position_at(10.5) == (1, 0, 0)
