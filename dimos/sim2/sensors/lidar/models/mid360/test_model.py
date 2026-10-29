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

"""Mid360 firing pattern, timing and return model."""

import numpy as np
import pytest

from dimos.sim2.sensors.lidar.models.mid360.model import Mid360
from dimos.sim2.sensors.lidar.models.mid360.pattern import Mid360Pattern


def test_return_noise_is_seeded_per_scan_and_rejects_grazing_misses():
    model = Mid360(seed=7)
    ranges = np.full(1000, 10.0)
    normal = np.ones(1000)
    noisy = model.measure(ranges, normal, 0.2)
    assert np.std(noisy - ranges) > 0.003
    np.testing.assert_array_equal(noisy, model.measure(ranges, normal, 0.2))
    assert not np.array_equal(noisy, model.measure(ranges, normal, 0.3))
    assert np.all(model.measure(ranges, np.zeros(1000), 0.2) == -1)
    assert model.measure(np.array([-1.0, 0.1, 41.0]), np.ones(3), 0).tolist() == [-1, -1, -1]


def test_noise_free_diagnostics_keep_the_same_valid_ranges():
    model = Mid360(noise=False, dropout=False)
    assert model.measure(np.array([-1.0, 0.2, 10]), np.ones(3), 0).tolist() == [-1, 0.2, 10]


def test_fourier_evaluation_matches_real_sine_cosine_series_across_channels():
    coefs = np.random.default_rng(7).normal(size=(4, 9, 3))
    pattern = Mid360Pattern(3.25, 1.1, 1, 1, coefs)
    indices = np.array([0, 1, 2, 3, 41, 802, 800_003], dtype=np.int64)
    expected = []
    for index in indices:
        fast, slow = 2 * np.pi * (index // 4 * 4 / 200_000) * np.array([3.25, 1.1])
        phases = np.array([slow, fast - slow, fast, fast + slow])
        basis = np.r_[1, np.column_stack((np.cos(phases), np.sin(phases))).ravel()]
        vector = basis @ coefs[index % 4]
        expected.append(vector / np.linalg.norm(vector))
    np.testing.assert_allclose(
        np.concatenate([pattern.directions(int(index), 1) for index in indices]),
        expected,
        atol=1e-12,
    )


@pytest.fixture
def pattern(mocker):
    coefs = np.zeros((4, 3, 3))
    coefs[:, 1] = [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]]
    coefs[:, 2] = [[0, 1, 0], [-1, 0, 0], [0, -1, 0], [1, 0, 0]]
    mocker.patch(
        "dimos.sim2.sensors.lidar.models.mid360.model._pattern",
        return_value=Mid360Pattern(2.3, 0, 1, 0, coefs),
    )
    return Mid360()


def test_scan_preserves_channels_timing_and_continuous_absolute_phase(pattern):
    first = pattern.scan(0, 0.1)
    second = pattern.scan(0.1, 0.1)
    assert len(first.offsets) == 20_000
    assert first.offsets[[0, 1, -1]] == pytest.approx([0, 0.000005, 0.099995])
    assert first.lines[:8].tolist() == [0, 1, 2, 3, 0, 1, 2, 3]
    np.testing.assert_allclose(
        first.directions[:4], [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]], atol=1e-12
    )
    np.testing.assert_allclose(
        second.directions[0], [np.cos(2 * np.pi * 0.23), np.sin(2 * np.pi * 0.23), 0], atol=1e-12
    )
    np.testing.assert_allclose(
        pattern.scan(0, 0.2).directions, np.concatenate([first.directions, second.directions])
    )
    assert not np.allclose(pattern.scan(4, 0.1).directions, first.directions)
    np.testing.assert_allclose(np.linalg.norm(first.directions, axis=1), 1)


def test_downsampling_keeps_four_lasers_and_original_time_offsets(pattern):
    rays = Mid360(downsample=8).scan(0, 0.1)
    assert len(rays.offsets) == 2500
    assert rays.lines[:8].tolist() == [0, 1, 2, 3, 0, 1, 2, 3]
    assert rays.offsets[4] == 32 / 200_000
    assert rays.offsets[-1] > 0.099
    np.testing.assert_allclose(
        rays.directions, pattern.scan(0, 0.1).directions.reshape(-1, 4, 3)[::8].reshape(-1, 3)
    )


@pytest.mark.parametrize("duration", [0, -0.1, 0.100001, 0.000005, float("nan")])
def test_scan_rejects_partial_channel_groups(pattern, duration):
    with pytest.raises(ValueError, match="four-laser"):
        pattern.scan(0, duration)


@pytest.mark.parametrize("start", [-1, float("inf"), float("nan")])
def test_scan_rejects_invalid_start(pattern, start):
    with pytest.raises(ValueError, match="start"):
        pattern.scan(start, 0.1)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"point_rate_hz": 100_000},
        {"downsample": 1.5},
        {"downsample": 0},
        {"motion_sample_rate_hz": 0},
        {"max_range": 0},
        {"min_range": float("nan")},
    ],
)
def test_invalid_sensor_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        Mid360(**kwargs)


@pytest.mark.self_hosted
def test_installed_calibration_is_continuous_and_covers_the_sensor_field():
    rays = Mid360(noise=False, dropout=False).scan(0, 0.1)
    elevation = np.degrees(np.arcsin(rays.directions[:, 2]))
    assert -10 < elevation.min() < -5
    assert 50 < elevation.max() < 56
    np.testing.assert_allclose(np.linalg.norm(rays.directions, axis=1), 1)
