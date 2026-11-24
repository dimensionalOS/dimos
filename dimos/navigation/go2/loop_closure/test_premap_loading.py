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

from contextlib import closing
from pathlib import Path
import sqlite3

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.core.module import Module
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.loop_closure.module import PGOVoxelMapper
from dimos.navigation.go2.loop_closure.premap import KEYFRAMES, Premap


@pytest.fixture(params=["cloud", "empty", "cloud_with_empty_keyframes"])
def invalid_premap(tmp_path: Path, request: pytest.FixtureRequest) -> Path:
    path = tmp_path / "invalid.db"
    with SqliteStore(path=path) as store:
        if request.param != "empty":
            cloud = PointCloud2.from_numpy(np.ones((1, 3)), frame_id="map/v1")
            store.stream("map_v1_cloud", PointCloud2).append(cloud, ts=1.0)
        if request.param != "cloud":
            store.stream(KEYFRAMES, PointCloud2)
    return path


def test_load_rejects_invalid_premap_without_modifying_file(invalid_premap: Path) -> None:
    before = invalid_premap.read_bytes()
    with pytest.raises(ValueError, match="keyframe premap"):
        Premap.load(invalid_premap)
    assert invalid_premap.read_bytes() == before


@pytest.mark.parametrize("save_map", [False, True])
def test_mapper_rejects_invalid_premap(
    invalid_premap: Path, save_map: bool, mocker: MockerFixture
) -> None:
    # Module.start initializes transports unrelated to premap validation.
    mocker.patch.object(Module, "start")
    before = invalid_premap.read_bytes()
    mapper = PGOVoxelMapper(map_file=str(invalid_premap), save_map=save_map)
    try:
        with pytest.raises(ValueError, match="keyframe premap"):
            mapper.start()
    finally:
        mapper.stop()
    assert invalid_premap.read_bytes() == before


def test_load_valid_premap_without_edges_does_not_modify_file(tmp_path: Path) -> None:
    path = tmp_path / "office.map.db"
    points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    with SqliteStore(path=path) as store:
        cloud = PointCloud2.from_numpy(points, frame_id="keyframe/0", timestamp=1.0)
        store.stream(KEYFRAMES, PointCloud2).append(cloud, ts=1.0, pose=Pose())
    # Loading must not change a database's journal mode either.
    with closing(sqlite3.connect(path)) as conn:
        conn.execute("PRAGMA journal_mode=DELETE")
    before = path.read_bytes()

    loaded = Premap.load(path)

    np.testing.assert_array_equal(loaded.stamps, [1.0])
    np.testing.assert_array_equal(loaded.scans[0], points)
    np.testing.assert_array_equal(loaded.poses, np.eye(4)[None])
    assert len(loaded.edges) == 0
    assert path.read_bytes() == before
