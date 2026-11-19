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

from dimos.mapping.relocalization import module as lidar_module
from dimos.mapping.relocalization.module import PremapModule
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def module():
    module = PremapModule(live_cloud="local_map", min_local_points=3)
    yield module
    module.dispose()


def test_sparse_clouds_warn_only_after_grace_and_remain_throttled(module, mocker):
    logger = mocker.patch.object(lidar_module, "logger")
    clock = mocker.patch.object(lidar_module.time, "monotonic")
    cloud = PointCloud2.from_numpy(np.zeros((2, 3), dtype=np.float32))

    for now in (100.0, 101.0, 105.0, 109.999, 110.0, 111.0, 115.0):
        clock.return_value = now
        module._maybe_log_skip(cloud)

    message = "relocalize skipped: not enough local points"
    fields = {"n_pts": 2, "min_local_points": 3}
    assert logger.method_calls == [
        mocker.call.info(message, **fields),
        mocker.call.info(message, **fields),
        mocker.call.warning(message, **fields),
        mocker.call.warning(message, **fields),
    ]
    assert not module._has_enough_points(cloud)


def test_enough_points_clear_sparse_cloud_warning_state(module, mocker):
    logger = mocker.patch.object(lidar_module, "logger")
    clock = mocker.patch.object(lidar_module.time, "monotonic", return_value=100.0)
    sparse = PointCloud2.from_numpy(np.zeros((2, 3), dtype=np.float32))
    ready = PointCloud2.from_numpy(np.zeros((3, 3), dtype=np.float32))
    module._maybe_log_skip(sparse)
    clock.return_value = 105.0
    module._maybe_log_skip(ready)
    clock.return_value = 120.0
    module._maybe_log_skip(sparse)

    assert logger.method_calls == [
        mocker.call.info(
            "relocalize skipped: not enough local points", n_pts=2, min_local_points=3
        ),
        mocker.call.info(
            "relocalize skipped: not enough local points", n_pts=2, min_local_points=3
        ),
    ]
    assert module._has_enough_points(ready)
