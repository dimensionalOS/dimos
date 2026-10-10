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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz


def test_cpu_accumulation_preserves_latest_exact_stamp_and_output_frame() -> None:
    first = pointcloud_from_xyz(
        np.array([[0.25, 0.25, 0.25]], dtype=np.float32),
        header=Header(frame_id="lidar", stamp=Time(sec=1, nanosec=987654321)),
    )
    second = pointcloud_from_xyz(
        np.array([[1.25, 1.25, 1.25]], dtype=np.float32),
        header=Header(frame_id="lidar", stamp=Time(sec=2, nanosec=123456789)),
    )
    grid = VoxelGrid(voxel_size=0.5, device="CPU:0", carve_columns=False, frame_id="map")
    try:
        grid.add_frame(first)
        grid.add_frame(second)
        result = grid.get_global_pointcloud2()
        decoded = cdr_decode(cdr_encode(result), type(result))
        assert decoded.header.frame_id == "map"
        assert decoded.header.stamp == second.header.stamp
        assert first.header.frame_id == second.header.frame_id == "lidar"
        np.testing.assert_allclose(
            pointcloud_xyz(decoded), [[0.25, 0.25, 0.25], [1.25, 1.25, 1.25]]
        )
    finally:
        grid.dispose()
    with pytest.raises(RuntimeError, match="disposed"):
        grid.get_global_pointcloud2()
