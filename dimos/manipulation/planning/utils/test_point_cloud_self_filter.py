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
from open3d.core import Tensor
import pytest

from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


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
    cloud = PointCloud2.from_numpy(
        np.asarray([[-1, 0, 0], [1, 2, 3]], dtype=np.float32),
        frame_id="camera",
        timestamp=2.0,
    )
    cloud.seq = 7
    cloud.pointcloud_tensor.point["intensity"] = Tensor(np.asarray([[10], [20]], dtype=np.float32))

    result = point_filter.filter_cloud(cloud)

    assert result is not None
    assert (result.frame_id, result.ts, result.seq) == ("camera", 2.0, 7)
    np.testing.assert_array_equal(result.points_f32(), [[1, 2, 3]])
    np.testing.assert_array_equal(result.pointcloud_tensor.point["intensity"].numpy(), [[20]])
    assert (
        point_filter.filter_cloud(
            PointCloud2.from_numpy(np.asarray([[1, 0, 0]], dtype=np.float32), timestamp=1.0)
        )
        is None
    )
