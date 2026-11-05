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

from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_mapper_keeps_configured_frame_and_last_acquisition_timestamp():
    cloud = PointCloud2.from_numpy(
        np.array([[1.0, 0.0, 0.0]], dtype=np.float32), frame_id="map", timestamp=12.25
    )
    module = VoxelGridMapper(frame_id="map", device="CPU:0", carve_columns=False)
    try:
        with MemoryStore() as store:
            stream = store.stream("lidar", PointCloud2)
            stream.append(cloud)
            result = list(module.pipeline(stream))
            assert result[-1].data.frame_id == "map"
            assert result[-1].data.ts == 12.25
            assert len(result[-1].data) == 1
    finally:
        module.dispose()
