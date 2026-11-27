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

from collections.abc import Iterator
from pathlib import Path

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.loop_closure import module
from dimos.navigation.go2.loop_closure.module import (
    PGOVoxelMapper,
    PGOVoxelMapperConfig,
    _pgo_config,
)
from dimos.navigation.go2.loop_closure.pgo import PGOConfig
from dimos.navigation.go2.loop_closure.pgo_map import PGOMap
from dimos.navigation.go2.loop_closure.premap import Premap
from dimos.navigation.go2.loop_closure.relocalizer import PremapMatcher


@pytest.fixture
def mapper(tmp_path: Path) -> Iterator[PGOVoxelMapper]:
    premap = Premap(
        stamps=np.array([0.0]),
        poses=np.eye(4)[None],
        scans=(np.array([[0.02, 0.02, 0.02]], dtype=np.float32),),
        edges=np.empty((0, 2), dtype=np.int64),
        edge_poses=np.empty((0, 4, 4)),
        edge_covariances=np.empty((0, 6, 6)),
    )
    path = tmp_path / "office.map.db"
    premap.save(path)
    mapper = PGOVoxelMapper(save_map=True, map_file=str(path), backup_keep_last=0)
    mapper._map_path = path
    mapper._map = PGOMap(premap=premap, pgo=PGOConfig(loop_closure=False))
    cloud = PointCloud2.from_numpy(np.array([[1.02, 0.02, 0.02]]), timestamp=10.0)
    mapper._map.add(cloud, Pose(1.0, 0.0, 0.0))
    yield mapper
    if not mapper._stopping.is_set():
        mapper.stop()


def _settle(mapper: PGOVoxelMapper) -> None:
    assert mapper._map is not None
    for _ in range(3):
        mapper._map.pgo.relocalize(Transform(ts=10.0))
    assert mapper._map.pgo.settled


@pytest.fixture
def reattached_mapper(mapper: PGOVoxelMapper) -> PGOVoxelMapper:
    assert mapper._map is not None and mapper._map_path is not None
    _settle(mapper)
    assert mapper._save_map(background=False)
    mapper._map.pgo.relocalize(Transform(translation=Vector3(2.0, 0.0, 0.0), ts=10.0), 0.9, 0.2)
    assert not mapper._map.pgo.settled
    cloud = PointCloud2.from_numpy(np.array([[4.02, 0.02, 0.02]]), timestamp=20.0)
    mapper._map.add(cloud, Pose(4.0, 0.0, 0.0))
    return mapper


def test_stop_after_reattach_keeps_latest_keyframes(
    reattached_mapper: PGOVoxelMapper, mocker: MockerFixture
) -> None:
    warning = mocker.patch.object(module.logger, "warning")
    reattached_mapper.stop()

    assert 20.0 in Premap.load(reattached_mapper._map_path).stamps
    warning.assert_not_called()


def test_background_save_after_reattach_keeps_latest_keyframes(
    reattached_mapper: PGOVoxelMapper,
) -> None:
    assert reattached_mapper._save_map(background=True)
    assert reattached_mapper._writer is not None
    reattached_mapper._writer.join(timeout=5.0)

    assert not reattached_mapper._writer.is_alive()
    assert 20.0 in Premap.load(reattached_mapper._map_path).stamps


def test_reattach_before_first_settlement_preserves_premap(mapper: PGOVoxelMapper) -> None:
    assert mapper._map is not None and mapper._map_path is not None
    original = mapper._map_path.read_bytes()
    mapper._map.pgo.relocalize(Transform(ts=10.0))
    mapper._map.pgo.relocalize(Transform(translation=Vector3(2.0, 0.0, 0.0), ts=10.0), 0.9, 0.2)
    assert not mapper._save_map(background=False)
    mapper.stop()

    assert mapper._map_path.read_bytes() == original


def test_reattach_retries_without_the_settled_delay(
    mapper: PGOVoxelMapper, mocker: MockerFixture
) -> None:
    _settle(mapper)
    mapper._matcher = mocker.Mock(spec=PremapMatcher)
    mocker.patch.object(
        module,
        "attempt_fix",
        return_value=(Transform(translation=Vector3(2.0, 0.0, 0.0), ts=10.0), 0.9, 0.2),
    )
    wait = mocker.patch.object(mapper._stopping, "wait", side_effect=[False, True])

    mapper._relocalize_loop()

    assert wait.call_args_list == [mocker.call(mapper.config.reloc_recheck_s), mocker.call(2.0)]


def test_loop_closure_is_on_for_a_real_robot_and_off_in_simulation() -> None:
    real = PGOVoxelMapperConfig(g=GlobalConfig())
    sim = PGOVoxelMapperConfig(g=GlobalConfig(simulation="dimsim"))

    assert _pgo_config(real).loop_closure is True
    assert _pgo_config(sim).loop_closure is False
    # the rest of the PGO settings are kept
    assert _pgo_config(sim).key_pose_delta_deg == sim.pgo.key_pose_delta_deg


def test_loop_closure_flag_overrides_the_simulation_default() -> None:
    sim_on = PGOVoxelMapperConfig(
        g=GlobalConfig(simulation="dimsim"), pgo=PGOConfig(loop_closure=True)
    )
    real_off = PGOVoxelMapperConfig(g=GlobalConfig(), pgo=PGOConfig(loop_closure=False))

    assert _pgo_config(sim_on) is sim_on.pgo
    assert _pgo_config(real_off) is real_off.pgo


def test_stop_without_start() -> None:
    # the coordinator stops every deployed module when another one fails to start
    PGOVoxelMapper().stop()
