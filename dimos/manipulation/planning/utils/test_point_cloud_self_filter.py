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

from dimos_generated.sensor_msgs.msg import JointState, PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_view, pointcloud_xyz
from dimos.msgs.time import time_from_seconds, to_seconds


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
    cloud = pointcloud_from_xyz(
        np.asarray([[-1, 0, 0], [1, 2, 3]], dtype=np.float32),
        header=Header(frame_id="camera", stamp=time_from_seconds(2.0)),
    )
    records = np.array(
        [(-1, 0, 0, 10), (1, 2, 3, 20)],
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")],
    )
    cloud.fields.append(
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)
    )
    cloud.point_step = 16
    cloud.row_step = 32
    cloud.data = records.view(np.uint8)

    result = point_filter.filter_cloud(cloud)

    assert result is not None
    assert (result.header.frame_id, to_seconds(result.header.stamp)) == ("camera", 2.0)
    np.testing.assert_array_equal(pointcloud_xyz(result), [[1, 2, 3]])
    np.testing.assert_array_equal(pointcloud_view(result)["intensity"].reshape(-1), [20])
    assert (
        point_filter.filter_cloud(
            pointcloud_from_xyz(
                np.asarray([[1, 0, 0]], dtype=np.float32),
                header=Header(frame_id="camera", stamp=time_from_seconds(1.0)),
            )
        )
        is None
    )


def test_joint_state_buffer_copies_values_and_replaces_duplicate_capture(
    point_filter: _PositiveXFilter,
) -> None:
    state = JointState(
        header=Header(frame_id="base", stamp=time_from_seconds(2.0)),
        name=["joint"],
        position=np.array([0.5]),
        velocity=np.array([]),
        effort=np.array([]),
    )
    point_filter.add_joint_state(state)
    state.position[0] = 99
    sample = point_filter._states.find_closest(2.0, 0.001)
    assert sample is not None
    np.testing.assert_array_equal(sample.value.position, [0.5])
    point_filter.add_joint_state(state)
    sample = point_filter._states.find_closest(2.0, 0.001)
    assert sample is not None
    np.testing.assert_array_equal(sample.value.position, [99])
