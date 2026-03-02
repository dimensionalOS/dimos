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

import functools
from typing import TypedDict

import pytest

from dimos.core import LCMTransport
from dimos.core.global_config import GlobalConfig
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.utils.data import get_data
from dimos.utils.testing.replay import TimedSensorReplay


class Moment(TypedDict):
    global_map: PointCloud2
    costmap: OccupancyGrid
    navigation_costmap: OccupancyGrid
    path: Path


@pytest.fixture(scope="session")
def get_moment():
    @functools.lru_cache(maxsize=1)
    def moment_provider(seek=10.0) -> Moment:
        data_dir = "unitree_go2_bigoffice"
        get_data(data_dir)

        lidar_data = TimedSensorReplay(f"{data_dir}/lidar")

        voxels = VoxelGridMapper()

        for frame in lidar_data:
            voxels.add_frame(frame)

        global_map = voxels.get_global_pointcloud2()
        voxels.stop()

        costmap = height_cost_occupancy(global_map)

        cfg = GlobalConfig()
        nav_map = NavigationMap(cfg)
        nav_map.update(costmap)
        navigation_costmap = nav_map.make_gradient_costmap()

        start = Vector3(-12.5, 12.5)
        goal = PoseStamped(position=Vector3(-3, -12))

        path = min_cost_astar(navigation_costmap, goal.position, start)
        assert path is not None, f"No path found from {start} to {goal}"
        path = smooth_resample_path(path, goal, spacing=0.1)

        return Moment(
            global_map=global_map,
            costmap=costmap,
            navigation_costmap=navigation_costmap,
            path=path,
        )

    return moment_provider


@pytest.fixture(scope="session")
def publish_moment():
    def publisher(moment: Moment) -> None:
        for key, value in moment.items():
            t = LCMTransport(f"/{key}", type(value))
            t.publish(value)
            t.lcm.stop()

    return publisher


def test_basic(get_moment, publish_moment):
    """Test that we can create a Moment and publish it."""
    moment = get_moment()
    print(moment)
    assert "global_map" in moment
    assert "costmap" in moment
    assert "navigation_costmap" in moment
    assert "path" in moment
    assert len(moment["path"].poses) > 0
    print(moment.get("path"))
    publish_moment(moment)
