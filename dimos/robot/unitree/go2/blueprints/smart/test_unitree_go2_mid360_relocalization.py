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

"""Composition tests for unitree_go2_mid360_relocalization base selection."""

from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import (
    unitree_go2_mid360_relocalization_base,
)
from dimos.robot.unitree.go2.connection import (
    DEFAULT_REPLAY_ODOM_STREAMS,
    GO2Connection,
    MID360_REPLAY_ODOM_STREAMS,
)


def _go2_connection_kwargs(lidar_config: str) -> dict:  # type: ignore[type-arg]
    base = unitree_go2_mid360_relocalization_base(lidar_config)
    return next(atom.kwargs for atom in base.blueprints if atom.module is GO2Connection)


def test_mid360_lidar_config_uses_pointlio_odom_preference() -> None:
    kwargs = _go2_connection_kwargs("mid360")
    assert kwargs.get("replay_odom_streams", DEFAULT_REPLAY_ODOM_STREAMS) == MID360_REPLAY_ODOM_STREAMS


def test_default_lidar_config_uses_go2_odom_preference() -> None:
    kwargs = _go2_connection_kwargs("default")
    assert kwargs.get("replay_odom_streams", DEFAULT_REPLAY_ODOM_STREAMS) == DEFAULT_REPLAY_ODOM_STREAMS


def test_mid360_base_has_single_voxel_mapper() -> None:
    base = unitree_go2_mid360_relocalization_base("mid360")
    voxels = [atom for atom in base.blueprints if atom.module is VoxelGridMapper]
    assert len(voxels) == 1
