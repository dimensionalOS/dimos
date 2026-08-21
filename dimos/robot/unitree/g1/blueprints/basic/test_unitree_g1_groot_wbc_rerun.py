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

from dimos.robot.unitree.g1.blueprints.basic import unitree_g1_groot_wbc
from dimos.visualization.rerun.urdf_robot import UrdfRobotTransformFilter


def test_g1_rerun_live_streams_are_bounded() -> None:
    config = unitree_g1_groot_wbc._rerun_config

    assert config["memory_limit"] == "2GB"
    assert config["max_hz"]["world/lidar"] == 2.0
    assert config["max_hz"]["world/global_map"] == 1.0
    assert config["max_hz"]["world/global_costmap"] == 2.0
    assert config["max_hz"]["world/navigation_costmap"] == 2.0
    assert config["max_hz"]["world/tf"] == 15.0
    assert config["max_hz"][unitree_g1_groot_wbc._G1_JOINTS_ENTITY] == 20.0
    assert "world/lidar" in config["latest_state"]
    assert "world/global_map" in config["latest_state"]
    assert "world/global_costmap" in config["latest_state"]
    assert "world/navigation_costmap" in config["latest_state"]
    assert isinstance(config["visual_override"]["world/tf"], UrdfRobotTransformFilter)
