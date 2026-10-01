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

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from dimos_generated.sensor_msgs.msg import PointCloud2
import pytest

from dimos.mapping.voxels.module import VoxelMapTransformer
from dimos.memory.store.sqlite import SqliteStore
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from collections.abc import Iterator


@pytest.fixture(scope="module")
def store() -> Iterator[SqliteStore]:
    db = SqliteStore(path=get_data("go2_bigoffice.db"))
    with db:
        yield db


def test_build_global_map(store: SqliteStore) -> None:
    t_total = time.perf_counter()

    lidar = store.stream("lidar", PointCloud2)
    n_frames = lidar.count()

    t0 = time.perf_counter()
    result = lidar.transform(VoxelMapTransformer(voxel_size=0.05)).last()
    t_transform = time.perf_counter() - t0

    t_total = time.perf_counter() - t_total

    global_map = result.data
    frame_count = result.tags["frame_count"]

    assert frame_count == n_frames
    assert global_map.width * global_map.height > 0

    print(
        lidar.summary(),
        f"\n{frame_count} frames -> {global_map.width * global_map.height} voxels"
        f"\n  transform: {t_transform:.2f}s ({t_transform / frame_count * 1000:.1f}ms/frame)"
        f"\n  total wall: {t_total:.2f}s",
    )
