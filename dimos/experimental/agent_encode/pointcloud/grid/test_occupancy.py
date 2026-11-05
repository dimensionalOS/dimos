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

from dimos.experimental.agent_encode.pointcloud.grid.occupancy import Occupancy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

nan = np.nan


def test_occupancy_marks_band_returns_occupied_and_lower_returns_free():
    cloud = PointCloud2.from_numpy(
        np.array(
            [[0.25, 0.25, 0], [0.25, 0.25, 1], [1.25, 0.25, 0.5], [2, 0.25, 9], [0.25, 1.25, 0]],
            dtype=np.float32,
        ),
        frame_id="map",
    )

    grid = Occupancy((0.4, 2), 1, ((0.5, 0.5), (2.5, 1.5))).run(cloud)

    np.testing.assert_array_equal(grid.values, [[1, 1, nan], [0, nan, nan]])
