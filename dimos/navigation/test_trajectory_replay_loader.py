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

"""Tests for trajectory odom replay loader (921 P5-3)."""

import pytest

from dimos.navigation.trajectory_replay_loader import (
    open_trajectory_odom_replay,
    trajectory_odom_replay_mini_fixture_dir,
)
from dimos.robot.unitree.type.odometry import Odometry


def test_mini_fixture_dir_exists_and_has_pickles() -> None:
    d = trajectory_odom_replay_mini_fixture_dir()
    assert d.is_dir()
    pickles = sorted(d.glob("*.pickle"))
    assert len(pickles) == 3


def test_open_fixture_without_get_data() -> None:
    replay = open_trajectory_odom_replay("fixture", autocast=Odometry.from_msg)
    msgs = []
    for ts, odom in replay.iterate_ts():
        msgs.append((ts, odom))
        if len(msgs) >= 3:
            break
    assert len(msgs) == 3
    assert all(isinstance(o, Odometry) for _, o in msgs)
    assert msgs[0][0] < msgs[1][0] < msgs[2][0]


def test_open_absolute_path_same_as_fixture() -> None:
    path = trajectory_odom_replay_mini_fixture_dir()
    assert path.is_absolute()
    r1 = open_trajectory_odom_replay("fixture", autocast=Odometry.from_msg)
    r2 = open_trajectory_odom_replay(path, autocast=Odometry.from_msg)
    assert r1.first_timestamp() == r2.first_timestamp()


@pytest.mark.lfs_data
def test_open_unitree_office_walk_via_get_data() -> None:
    """Requires ``get_data`` plus Git LFS for ``unitree_office_walk``; excluded from default pytest."""
    replay = open_trajectory_odom_replay("unitree_office_walk/odom", autocast=Odometry.from_msg)
    ts, odom = next(replay.iterate_ts())
    assert isinstance(ts, float)
    assert isinstance(odom, Odometry)
