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

import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.message_utils import occupancy, occupancy as occupancy_utils, time as message_time
from dimos.messages.nav_msgs.msg import OccupancyGrid
from dimos.messages.std_msgs.msg import Header
from dimos.utils.data import get_data


@pytest.fixture
def occupancy() -> OccupancyGrid:
    return occupancy_utils.from_array(
        np.asarray(np.load(get_data("occupancy_simple.npy")), dtype=np.int8),
        header=Header(stamp=message_time.time_from_seconds(0.0), frame_id=""),
        resolution=0.05,
        origin=None,
    )


@pytest.fixture
def occupancy_gradient(occupancy) -> OccupancyGrid:
    return gradient(occupancy, max_distance=1.5)
