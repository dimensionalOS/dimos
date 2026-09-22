# Copyright 2025-2026 Dimensional Inc.
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

from dimos_generated.geometry_msgs.msg import Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.utils.data import get_data


@pytest.fixture
def occupancy() -> OccupancyGrid:
    cells = np.load(get_data("occupancy_simple.npy"))
    return OccupancyGrid(
        info=MapMetaData(
            width=cells.shape[1],
            height=cells.shape[0],
            resolution=0.05,
            origin=Pose(orientation=Quaternion(w=1)),
        ),
        data=cells.astype(np.int8).ravel(),
    )


@pytest.fixture
def occupancy_gradient(occupancy) -> OccupancyGrid:
    return gradient(occupancy, max_distance=1.5)
