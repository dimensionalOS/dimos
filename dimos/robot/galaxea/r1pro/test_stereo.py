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

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.spatial_sensors import R1SpatialSensors
from dimos.robot.galaxea.r1pro.stereo import disparity_depth


def test_metric_depth_and_invalid_disparity_holes():
    result = disparity_depth(np.array([[12, 6, 0, -1, float("nan"), 0.01]], dtype=np.float32), 12)
    np.testing.assert_array_equal(result, [[1, 2, 0, 0, 0, 0]])


@pytest.fixture
def sensors():
    module = R1SpatialSensors()
    yield module
    module.stop()


def image_stamp(ts):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(ts), nanosec=round((ts - int(ts)) * 1e9))
        )
    )


def test_stereo_pairs_close_timestamps_and_never_reuses_or_restamps_old_frames(sensors, mocker):
    mocker.patch("dimos.robot.galaxea.r1pro.spatial_sensors.time.time", return_value=100)
    left = image_stamp(99.9)
    right = image_stamp(99.907)
    sensors._receive("a", left)
    sensors._receive("b", image_stamp(99.84))
    assert sensors._pair("a", "b") is None
    sensors._receive("b", right)
    assert sensors._pair("a", "b") == (left, right)
    assert sensors._pair("a", "b") is None
    sensors._receive("c", image_stamp(99))
    sensors._receive("d", image_stamp(99.007))
    assert sensors._pair("c", "d") is None
