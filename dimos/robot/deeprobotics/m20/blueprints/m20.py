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
from dimos.core.transport import ZenohTransport
from dimos.mapping.relocalization.lidar_relocalizer import MID360_NAV
from dimos.mapping.relocalization.module import PremapModule
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic
from dimos.robot.deeprobotics.m20.blueprints._nav import VOXEL_SIZE_M, m20_nav_stack
from dimos.robot.deeprobotics.m20.camera import M20CameraRelay
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.constants import WORLD_FRAME
from dimos.robot.deeprobotics.m20.model import M20_MODEL, M20StandingJointState
from dimos.robot.urdf_manager import UrdfManager

m20 = (
    autoconnect(
        M20CameraRelay.blueprint(instance_name="M20CameraRelay"),
        M20Connection.blueprint().remappings([(M20Connection, "odometry", "slam_odom")]),
        UrdfManager.blueprint(model=M20_MODEL),
        M20StandingJointState.blueprint(),
        m20_nav_stack().remappings([(MovementManager, "cmd_vel", "requested_cmd_vel")]),
        PremapModule.blueprint(
            world_frame=WORLD_FRAME,
            map_frame="premap",
            # matches against the office premap top out near 0.7 fitness, under the nav preset's 0.8
            relocalize=MID360_NAV.model_copy(update={"fitness_threshold": 0.65}),
            # body scans placed in map (odometry), so a world -> map jump leaves the kept live map as it was
            live_cloud="lidar",
            relocalize_once=False,
            new_map=True,
            # live voxels the first match waits for, under the 20k default
            min_local_points=8_000,
            # one kept point per ray tracer voxel, so a standing robot's map is not thinned below min_local_points
            live_voxel_m=VOXEL_SIZE_M,
        ).remappings([(PremapModule, "lidar", "slam_body_points")]),
    )
    .global_config(
        n_workers=5,
        transport="zenoh",
    )
    .transports(
        {
            ("front_camera", CompressedVideo): ZenohTransport.spec(
                ZenohTopic("dimos/front_camera", CompressedVideo, qos=QOS_LATEST_WINS)
            ),
            ("rear_camera", CompressedVideo): ZenohTransport.spec(
                ZenohTopic("dimos/rear_camera", CompressedVideo, qos=QOS_LATEST_WINS)
            ),
        }
    )
)
