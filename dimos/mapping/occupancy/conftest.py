# Copyright 2025 Dimensional Inc.
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

from dimos.mapping.occupancy.gradient import gradient
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.utils.data import get_data


@pytest.fixture
def occupancy() -> OccupancyGrid:
    return OccupancyGrid(np.load(get_data("occupancy_simple.npy")))


@pytest.fixture
def occupancy_gradient(occupancy) -> OccupancyGrid:
    return gradient(occupancy, max_distance=1.5)
