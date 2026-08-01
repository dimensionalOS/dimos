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
from shapely.geometry import Point

from dimos.benchmark.dimsim.geometry import (
    GeometryGateError,
    feasible_stopping_region,
    outer_edge_distance,
    surface_distance,
)
from dimos.benchmark.dimsim.models import NavigationGeometry


def _box(x0: float, z0: float, x1: float, z1: float) -> tuple[tuple[float, float], ...]:
    return ((x0, z0), (x1, z0), (x1, z1), (x0, z1))


def test_surface_and_outer_edge_distance_use_polygon_boundaries() -> None:
    first = _box(0, 0, 2, 2)
    second = _box(3, 0, 4, 1)

    assert surface_distance(first, second) == 1.0
    assert outer_edge_distance((1, 1), first) == 1.0


def test_invalid_polygon_is_rejected() -> None:
    bow_tie = ((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0))

    with pytest.raises(GeometryGateError, match="valid non-empty polygon"):
        surface_distance(bow_tie, _box(3, 3, 4, 4))


def test_stopping_region_must_share_reachable_free_component() -> None:
    navigation = NavigationGeometry(
        navigable=(_box(0, 0, 2, 2), _box(8, 8, 10, 10)),
    )

    with pytest.raises(GeometryGateError, match="unreachable"):
        feasible_stopping_region(
            _box(8.5, 8.5, 9, 9),
            navigation,
            (1, 1),
            threshold_m=1,
            footprint_radius_m=0.1,
            clearance_m=0.05,
        )


def test_precleared_navigation_is_not_shrunk_twice() -> None:
    navigation = NavigationGeometry(
        navigable=(_box(0, 0, 0.3, 2),),
        clearance_radius_m=0.15,
        collision_source_count=10,
    )

    result = feasible_stopping_region(
        _box(0.2, 0.8, 0.3, 1.2),
        navigation,
        (0.1, 0.1),
        threshold_m=1,
        footprint_radius_m=0.1,
        clearance_m=0.05,
    )

    assert not result.is_empty


def test_stopping_region_measures_from_robot_footprint() -> None:
    navigation = NavigationGeometry(
        navigable=(_box(-2, -2, 4, 3),),
        clearance_radius_m=0.2,
        collision_source_count=10,
    )

    result = feasible_stopping_region(
        _box(0, 0, 1, 1),
        navigation,
        (-1, 0.5),
        threshold_m=1.0,
        footprint_radius_m=0.2,
        clearance_m=0.0,
    )

    assert result.covers(Point(2.15, 0.5))
