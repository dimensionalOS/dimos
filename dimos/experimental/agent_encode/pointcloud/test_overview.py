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

import json

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def levels():
    """A 10 m floor sampled every 6.25 cm, with a 2 m platform and a 2 m pit."""
    x, y = np.meshgrid(np.linspace(0, 10, 161), np.linspace(0, 10, 161))
    z = np.zeros_like(x)
    z[(x >= 2) & (x <= 4) & (y >= 2) & (y <= 4)] = 0.5
    z[(x >= 6) & (x <= 8) & (y >= 6) & (y <= 8)] = -0.5
    return np.column_stack((x.ravel(), y.ravel(), z.ravel())).astype(np.float32)


def overview(points):
    cloud = PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape(-1, 3), frame_id="map", timestamp=7
    )
    return cloud.agent_encode()["overview"]


def test_default_reports_both_relief_signs(levels):
    cloud = PointCloud2.from_numpy(levels, frame_id="map", timestamp=7)

    out = cloud.agent_encode()

    result = out["overview"]
    assert out["bounds_m"] == [[0, 0, -0.5], [10, 10, 0.5]]
    assert result["lower_surface"]["status"] == "measured"
    assert result["lower_surface"]["reference_z_m"] == 0
    pit, platform = result["relief"]["regions"]
    assert platform == {
        "centroid": [3, 3],
        "area_m2": 4,
        "offset_quantiles_m": {"p10": 0.5, "p50": 0.5, "p90": 0.5},
    }
    # A low percentile follows the lower surface, so a drop also claims the edge
    # cells it shares with the floor; a rise does not.
    assert pit["area_m2"] == 5 and pit["offset_quantiles_m"]["p50"] == -0.5
    assert pit["centroid"] == pytest.approx([7.1125, 7.1125], abs=1e-3)
    assert result["structure"]["region_count"] == 1
    assert len(json.dumps(out).encode()) < 6000


def test_cells_follow_return_spacing_not_extent(levels):
    result = overview(levels)

    assert result["spacing_m"] == 0.0625
    assert result["coverage"]["grid"] == {
        "origin": [0, 0],
        "shape": [81, 81],
        "cell_m": 0.125,
        "limited": False,
    }
    assert result["lower_surface"]["grid"] == {
        "origin": [0, 0],
        "shape": [41, 41],
        "cell_m": 0.25,
        "limited": False,
    }


def test_one_far_return_leaves_nearby_measurements_unchanged(levels):
    near = overview(levels)

    far = overview(np.vstack((levels, [[60, 5, 0]])))

    assert far["span_m"][0] == 60
    assert far["coverage"]["grid"]["cell_m"] == near["coverage"]["grid"]["cell_m"]
    assert far["coverage"]["occupied_cells"] == near["coverage"]["occupied_cells"] + 1
    assert far["lower_surface"]["reference_z_m"] == near["lower_surface"]["reference_z_m"]
    assert far["relief"] == near["relief"]
    assert far["structure"] == near["structure"]


def test_reference_and_band_follow_z_translation(levels):
    first = overview(levels)

    second = overview(levels + np.array([0, 0, 5], dtype=np.float32))

    assert second["lower_surface"]["reference_z_m"] == first["lower_surface"]["reference_z_m"] + 5
    assert second["relief"] == first["relief"]
    assert second["structure"]["z_range_m"] == pytest.approx(
        np.array(first["structure"]["z_range_m"]) + 5
    )
    assert second["structure"]["regions"] == first["structure"]["regions"]


def test_empty_and_unsupported_clouds_do_not_claim_level_ground():
    empty = overview(np.empty((0, 3)))
    sparse = overview([[0, 0, 0], [10, 10, 1]])

    assert empty["coverage"] is None and empty["lower_surface"] is None
    assert sparse["lower_surface"]["status"] == "insufficient_support"
    assert sparse["lower_surface"]["reference_z_m"] is None
    assert sparse["lower_surface"]["z_quantiles_m"] == {"p10": None, "p50": None, "p90": None}
    assert sparse["structure"] is None and sparse["relief"] is None
    json.dumps(sparse, allow_nan=False)


def test_overview_limits_regions_and_accounts_for_omissions():
    x, y = np.meshgrid(np.linspace(0, 10, 201), np.linspace(0, 10, 201))
    floor = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    obstacles = np.array([[x, y, 0.5] for x in (1, 3, 5, 7, 9) for y in (1, 3, 5, 7, 9)])

    structure = overview(np.vstack((floor, obstacles)))["structure"]

    assert len(structure["regions"]) == 8
    assert structure["region_count"] == 25
    assert structure["omitted_regions"] == 17
    assert structure["observed_area_m2"] == pytest.approx(25 * 0.1**2)
    assert structure["omitted_area_m2"] == pytest.approx(17 * 0.1**2)


def test_only_the_oversized_grid_grows_its_cells():
    x, y = np.meshgrid(np.linspace(0, 1, 101), np.linspace(0, 1, 101))
    dense = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))

    result = overview(np.vstack((dense, [[16, 16, 0]])))

    assert result["coverage"]["grid"]["limited"] is True
    assert result["lower_surface"]["grid"]["limited"] is False
