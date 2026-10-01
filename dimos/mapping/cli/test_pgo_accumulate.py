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

from dimos_generated.geometry_msgs.msg import Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.mapping.cli.map import _accumulate
from dimos.mapping.loop_closure.pgo import PoseGraph
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_nanoseconds


def test_pgo_accumulate_applies_generated_correction_before_mapping(mocker):
    graph = mocker.Mock(spec=PoseGraph)
    graph.correction_at.return_value = TransformStamped(
        header=Header(frame_id="world_corrected"),
        child_frame_id="world",
        transform=Transform(translation=Vector3(x=2.0)),
    )
    mapper = mocker.Mock(side_effect=lambda observations: observations)
    mocker.patch("dimos.mapping.voxels.module.VoxelMapTransformer", return_value=mapper)
    stamp = time_from_nanoseconds(42_123_456_789)
    cloud = pointcloud_from_xyz(
        np.array([[1.0, 2.0, 3.0]]), header=Header(stamp=stamp, frame_id="world")
    )
    with MemoryStore() as store:
        stream = store.stream("cloud", type(cloud))
        stream.append(cloud, ts=42.0, pose=(0.0, 0.0, 0.0))
        result = _accumulate(stream, voxel=0.1, block_count=10, device="CPU:0", graph=graph)
    assert result is not None
    np.testing.assert_array_equal(pointcloud_xyz(result), [[3.0, 2.0, 3.0]])
    assert result.header.frame_id == "world_corrected"
    assert result.header.stamp.sec == 42
    assert result.header.stamp.nanosec == 123_456_789
    graph.correction_at.assert_called_once_with(42.0)
