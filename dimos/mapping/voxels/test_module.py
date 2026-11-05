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

from dimos_lcm.common_msgs import SampleHeader
import numpy as np

from dimos.mapping.voxels.module import VoxelMapTransformer
from dimos.memory.type.observation import Observation
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_map_preserves_acquisition_and_clears_on_reset():
    def observation(x, epoch, time_ns):
        cloud = PointCloud2.from_numpy(
            np.array([[x, 0.0, 0.0]], dtype=np.float32), frame_id="world"
        )
        cloud.with_sample_header(
            SampleHeader(
                clock_id="sim", epoch=epoch, time_ns=time_ns, source_id="lidar", frame_id="world"
            )
        )
        return Observation(_data=cloud, data_type=PointCloud2)

    mapper = VoxelMapTransformer(device="CPU:0", carve_columns=False, voxel_size=0.1)
    result = list(
        mapper(
            iter(
                [
                    observation(1.0, 0, 100_000_000),
                    observation(2.0, 0, 200_000_000),
                    observation(5.0, 1, 0),
                    observation(9.0, 0, 300_000_000),
                ]
            )
        )
    )
    assert [len(o.data) for o in result] == [1, 2, 1]
    assert result[-1].data.sample_header.epoch == 1
    assert result[-1].data.sample_header.time_ns == 0
    assert result[-1].data.sample_header.source_id == "global_map"
    assert result[-1].data.points_f32()[0, 0] > 4.9
