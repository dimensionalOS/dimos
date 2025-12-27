# Copyright 2025 Dimensional Inc.
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

import pickle
import time

import pytest

from dimos.mapping.pointclouds.occupancy import OCCUPANCY_ALGOS
from dimos.mapping.voxels import VoxelGridMapper
from dimos.utils.cli.plot import bar
from dimos.utils.data import _get_data_dir, get_data
from dimos.utils.testing import TimedSensorReplay


@pytest.mark.tool
def test_build_map():
    mapper = VoxelGridMapper(publish_interval=-1)

    for ts, frame in TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate_duration():
        print(ts, frame)
        mapper.add_frame(frame)

    pickle_file = _get_data_dir() / "unitree_go2_bigoffice_map.pickle"
    global_pcd = mapper.get_global_pointcloud2()

    with open(pickle_file, "wb") as f:
        pickle.dump(global_pcd, f)

    mapper.stop()


def test_costmap_calc():
    path = get_data("unitree_go2_bigoffice_map.pickle")
    pointcloud = pickle.loads(path.read_bytes())

    names = []
    times_ms = []
    for name, algo in OCCUPANCY_ALGOS.items():
        start = time.perf_counter()
        result = algo(pointcloud)
        elapsed = time.perf_counter() - start
        names.append(name)
        times_ms.append(elapsed * 1000)
        print(f"{name}: {elapsed * 1000:.1f}ms - {result}")

    bar(names, times_ms, title="Occupancy Algorithm Speed", ylabel="ms")
