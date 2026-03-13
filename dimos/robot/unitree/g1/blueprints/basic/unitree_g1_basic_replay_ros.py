#!/usr/bin/env python3
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

"""Basic G1 sim stack with ROS nav: sim connection and ROS navigation stack."""

from dimos.core.blueprints import autoconnect
from dimos.navigation.rosnav_docker import ros_nav
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_cam import (
    unitree_g1_primitive_no_cam,
)
from dimos.utils.data import LfsPath

unitree_g1_basic_replay_ros = autoconnect(
    unitree_g1_primitive_no_cam,
    ros_nav(use_rviz=True, bagfile_path=LfsPath("rosbags/rosnav_sim_short.db3")),
)

__all__ = ["unitree_g1_basic_replay_ros"]
