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

from dimos.sim2.sensors.lidar.mid360 import Mid360Returns, Mid360Scan


@pytest.fixture
def calibration(tmp_path):
    path = tmp_path / "fourier.npz"
    np.savez(
        path,
        f1=0.0,
        f2=0.0,
        m1=0,
        m2=0,
        coefs=np.array([[[1.0, 0, 0]], [[0.0, 1, 0]], [[-1.0, 0, 0]], [[0.0, -1, 0]]]),
    )
    return path


def test_decimation_keeps_four_channels_and_absolute_firing_times(calibration):
    scan = Mid360Scan(calibration, downsample=10)
    first, offsets, lines = scan.rays(0, 100_000_000)
    second, next_offsets, next_lines = scan.rays(100_000_000, 100_000_000)
    assert len(first) == 2000
    np.testing.assert_array_equal(lines[:8], [0, 1, 2, 3, 0, 1, 2, 3])
    np.testing.assert_array_equal(
        offsets[:8], [0, 5000, 10000, 15000, 200000, 205000, 210000, 215000]
    )
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(offsets, next_offsets)
    np.testing.assert_array_equal(lines, next_lines)


def test_return_model_drops_misses_and_grazing_rays_without_fabricating_hits():
    response = Mid360Returns(noise_floor_m=0, noise_per_m=0)
    measured = response.measure(
        np.array([2.0, 2.0, -1.0]), np.array([1.0, 0.0, 1.0]), np.random.default_rng(0)
    )
    np.testing.assert_array_equal(measured, [2.0, -1.0, -1.0])
