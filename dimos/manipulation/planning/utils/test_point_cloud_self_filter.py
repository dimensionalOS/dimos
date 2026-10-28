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

"""Shared filtering works with a backend that has no robot model."""

from collections.abc import Iterator

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.message_utils import pointcloud, time as message_time
from dimos.messages.sensor_msgs.msg import PointCloud2, PointField
from dimos.messages.std_msgs.msg import Header


class _PositiveXFilter(PointCloudSelfFilter):
    def _compute_keep_mask(
        self, cloud: PointCloud2, points: NDArray[np.float32]
    ) -> NDArray[np.bool_]:
        return points[:, 0] > 0


@pytest.fixture
def point_filter() -> Iterator[_PositiveXFilter]:
    module = _PositiveXFilter()
    yield module
    module.dispose()


def test_backend_mask_preserves_fields_and_capture_metadata(point_filter: _PositiveXFilter) -> None:
    cloud = pointcloud.from_xyz(
        np.asarray([[-1, 0, 0], [1, 2, 3]], dtype=np.float32),
        header=Header(stamp=message_time.time_from_seconds(2.0), frame_id="camera"),
    )
    records = np.zeros(2, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    for index, name in enumerate(("x", "y", "z")):
        records[name] = pointcloud.xyz(cloud)[:, index]
    records["intensity"] = [10, 20]
    cloud.fields.append(PointField("intensity", 12, PointField.FLOAT32, 1))
    cloud.point_step = 16
    cloud.row_step = 32
    cloud.data = np.frombuffer(records.tobytes(), dtype=np.uint8).copy()

    result = point_filter.filter_cloud(cloud)

    assert result is not None
    assert (result.header.frame_id, message_time.to_seconds(result.header.stamp)) == ("camera", 2.0)
    np.testing.assert_array_equal(pointcloud.xyz(result).astype(np.float32), [[1, 2, 3]])
    np.testing.assert_array_equal(pointcloud.view(result)["intensity"].reshape(-1, 1), [[20]])
    assert (
        point_filter.filter_cloud(
            pointcloud.from_xyz(
                np.asarray([[1, 0, 0]], dtype=np.float32),
                header=Header(stamp=message_time.time_from_seconds(1.0), frame_id="world"),
            )
        )
        is None
    )
