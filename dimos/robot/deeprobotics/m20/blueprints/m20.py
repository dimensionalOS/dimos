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

"""Robot-local M20 integration for the current DimOS 3D navigation stack.

The relocalizer fixes ``map -> map/<name>``, since the M20's live LIO frame is already ``map``.
Record with ``--record sqlite --record-topics 'tf,global_map,named_locations'``, build the map in
the Map Editor, then run with ``--map-file <recording>.db``.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.deeprobotics.m20.blueprints._nav import m20_nav_stack, m20_relocalization
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.model import M20_MODEL, M20StandingJointState
from dimos.robot.urdf_manager import UrdfManager

m20 = autoconnect(
    M20Connection.blueprint().remappings([(M20Connection, "odometry", "slam_odom")]),
    UrdfManager.blueprint(model=M20_MODEL),
    M20StandingJointState.blueprint(),
    m20_nav_stack().remappings([(MovementManager, "cmd_vel", "requested_cmd_vel")]),
    m20_relocalization(),
).global_config(
    n_workers=5,
    transport="zenoh",
)
