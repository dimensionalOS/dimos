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

It also runs the premap relocalizer, which keeps named map locations. The M20's live LIO
frame is already called ``map``, so the premap gets its own ``premap`` frame (``map -> premap``
is the fix). A new map, where the locations get named::

    dtk run m20 --record sqlite --record-topics 'tf,global_map,named_locations'

Back on that map, relocalized against that recording's last global_map, with its locations::

    dtk run m20 --map-file <that recording>.db --map-stream global_map
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import MID360_NAV
from dimos.robot.deeprobotics.m20.blueprints._nav import m20_nav_stack
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.constants import WORLD_FRAME
from dimos.robot.deeprobotics.m20.model import M20_MODEL, M20StandingJointState
from dimos.robot.deeprobotics.m20.rerun import m20_rerun_config
from dimos.robot.urdf_manager import UrdfManager
from dimos.visualization.vis_module import vis_module

m20 = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=m20_rerun_config()),
    M20Connection.blueprint().remappings([(M20Connection, "odometry", "slam_odom")]),
    UrdfManager.blueprint(model=M20_MODEL),
    M20StandingJointState.blueprint(),
    m20_nav_stack(),
    # loaded_map is a never-drop channel under zenoh, so no republish
    LocalMapRelocalization.blueprint(
        world_frame=WORLD_FRAME,
        map_frame="premap",
        relocalize=MID360_NAV,
        republish_loaded_map=0.0,
        new_map=True,
        tf_interval=1.0,
        # pre_trace_subsample=5 leaves a carved local map of ~12k points, under the 20k default
        min_local_points=8_000,
    ),
).global_config(
    n_workers=5,
    transport="zenoh",
)
