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

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.simulation.sensors.mid360.lidar import OcclusionMap, SimMid360
from dimos.simulation.sensors.mid360.pattern import Mid360Pattern
from dimos.utils.data import get_data

pytestmark = pytest.mark.self_hosted

ORIGIN = np.zeros(3)
UPRIGHT = np.eye(3)


@pytest.fixture(scope="module")
def pattern() -> Mid360Pattern:
    return Mid360Pattern.load(get_data("go2_sim") / "mid360_pattern.npz")


class Sphere:
    """A sphere of the given radius around the sensor, so every ray hits it head on."""

    def __init__(self, radius: float) -> None:
        self.radius = radius

    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return np.full(len(directions), self.radius), -directions


class Grazing:
    """Every ray hits at the same incidence angle, at 3 m."""

    def __init__(self, incidence_deg: float) -> None:
        self.cos_incidence = np.cos(np.radians(incidence_deg))

    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        sideways = np.cross(directions, [0.0, 0.0, 1.0])
        sideways /= np.linalg.norm(sideways, axis=1, keepdims=True)
        tilt = np.sqrt(1.0 - self.cos_incidence**2)
        return np.full(len(directions), 3.0), -directions * self.cos_incidence + sideways * tilt


class Void:
    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return np.full(len(directions), -1.0), -directions


def test_pattern_covers_the_measured_elevation_band(pattern: Mid360Pattern) -> None:
    d = pattern.directions(0, 200_000)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    el = np.degrees(np.arcsin(d[:, 2]))
    assert -9.0 < el.min() < -6.0
    assert 51.0 < el.max() < 54.0


def test_pattern_is_independent_of_batching(pattern: Mid360Pattern) -> None:
    whole = pattern.directions(1001, 800)
    parts = np.concatenate([pattern.directions(1001, 333), pattern.directions(1334, 467)])
    assert np.allclose(whole, parts)


def test_pattern_rejects_a_coefficient_count_that_does_not_match_its_orders() -> None:
    with pytest.raises(ValueError, match="coefficients"):
        Mid360Pattern(181.0, 9.9, 1, 1, np.zeros((4, 3)))


def test_returns_follow_range_with_millimeter_noise(pattern: Mid360Pattern) -> None:
    points = SimMid360(Sphere(3.0), pattern, seed=1).cast(ORIGIN, UPRIGHT, 20_000)
    r = np.linalg.norm(points, axis=1)
    assert len(r) == 20_000
    assert abs(float(np.median(r)) - 3.0) < 0.001
    assert 0.003 < float(np.std(r)) < 0.008


def test_misses_produce_no_points(pattern: Mid360Pattern) -> None:
    assert len(SimMid360(Void(), pattern, seed=1).cast(ORIGIN, UPRIGHT, 5_000)) == 0


def test_blind_zone_drops_near_returns(pattern: Mid360Pattern) -> None:
    assert len(SimMid360(Sphere(0.1), pattern, seed=1).cast(ORIGIN, UPRIGHT, 5_000)) == 0


def test_grazing_incidence_drops_returns_by_the_measured_table(pattern: Mid360Pattern) -> None:
    kept = len(SimMid360(Grazing(82.5), pattern, seed=1).cast(ORIGIN, UPRIGHT, 20_000))
    assert 0.45 < kept / 20_000 < 0.55
    assert len(SimMid360(Grazing(90.0), pattern, seed=1).cast(ORIGIN, UPRIGHT, 5_000)) == 0


def test_occlusion_map_blocks_the_directions_it_marks(pattern: Mid360Pattern) -> None:
    probability = np.zeros((64, 360), np.float32)
    probability[:, :180] = 1.0
    blocked_right = OcclusionMap(probability, -180.0, -8.0)
    points = SimMid360(Sphere(3.0), pattern, seed=1, occlusion=blocked_right).cast(
        ORIGIN, UPRIGHT, 20_000
    )
    assert 5_000 < len(points) < 15_000
    assert np.all(points[:, 1] > 0)


def test_same_seed_same_scan(pattern: Mid360Pattern) -> None:
    a = SimMid360(Sphere(3.0), pattern, seed=7).cast(ORIGIN, UPRIGHT, 4_000)
    b = SimMid360(Sphere(3.0), pattern, seed=7).cast(ORIGIN, UPRIGHT, 4_000)
    assert np.array_equal(a, b)
