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

from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster


@pytest.fixture
def lidar(world):
    module = SimLidarModule(
        scene=world, sensor=LidarConfig("left/lidar"), raycaster=MujocoRaycaster
    )
    module.open_sensor(world)
    yield module
    module.close_sensor()
    module.dispose()


def test_scan_uses_timed_geometry_history_and_rejects_gap(poses, lidar):
    scans = []
    for tick in range(51):
        sample = lidar.measure(poses(tick * 2_000_000))
        if sample is not None:
            scans.append(sample)
    assert len(scans) == 1
    cloud = scans[0]
    assert cloud.sample_header.time_ns == 0
    assert cloud.sample_header.clock_id == "test"
    assert len(cloud) > 30
    assert np.ptp(cloud.offset_times_u32()) > 80_000_000
    assert lidar.measure(poses(300_000_000)) is None
    assert lidar.measure(poses(0, epoch=1)) is None
    assert lidar.statistics()["gaps"] >= 2
