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

"""Basic G1 stack: base sensors plus real robot connection and ROS nav."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_nav import (
    unitree_g1_primitive_no_nav,
)
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.skill_container import UnitreeG1SkillContainer

# The same stock connection accepts Desktop velocity commands on hardware or
# either physics backend. Camera/lidar navigation is an additional composition.
unitree_g1_control = autoconnect(
    G1Connection.blueprint(),
    UnitreeG1SkillContainer.blueprint(),
).global_config(robot_model="unitree_g1", n_workers=2, viewer="none")

unitree_g1_basic = autoconnect(
    unitree_g1_primitive_no_nav,
    G1Connection.blueprint(),
)
