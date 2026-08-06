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

"""Independently runnable PX4 module diagnostics for the remote viewer."""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.drone.px4.camera import (
    camera_viewer,
    gazebo_camera_source_from_environment,
    px4_camera_from_environment,
)
from dimos.robot.drone.px4.config import mavsdk_config_from_environment
from dimos.robot.drone.px4.flight_control import FlightController
from dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera import Px4GstTeeCamera
from dimos.robot.drone.px4.mid360_mount_tf import Mid360MountStaticTf
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

px4_video_debug = (
    autoconnect(
        px4_camera_from_environment(),
        camera_viewer(),
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport.spec(
                "/color_image",
                default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE,
            ),
            ("video_h264", CompressedVideo): LCMTransport.spec("/video_h264", CompressedVideo),
        }
    )
    .global_config(n_workers=4, robot_model="px4_video_debug")
)

px4_flight_debug = (
    autoconnect(
        FlightController.blueprint(mavsdk_config=mavsdk_config_from_environment()),
        vis_module(global_config.viewer),
    )
    .remappings([(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")])
    .global_config(n_workers=4, robot_model="px4_flight_debug")
)

px4_gazebo_harmonic = (
    autoconnect(
        FlightController.blueprint(
            mavsdk_config=mavsdk_config_from_environment(
                default_connection_url="udpin://0.0.0.0:14540"
            )
        ),
        Px4GstTeeCamera.blueprint(source_config=gazebo_camera_source_from_environment()),
        camera_viewer(),
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport.spec(
                "/color_image",
                default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE,
            ),
            ("video_h264", CompressedVideo): LCMTransport.spec("/video_h264", CompressedVideo),
        }
    )
    .remappings([(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")])
    .global_config(n_workers=4, robot_model="px4_gazebo_harmonic")
)

px4_mapping_debug = autoconnect(
    PointLio.blueprint(sensor_frame_id="mid360_link"),
    Mid360MountStaticTf.blueprint(),
    RayTracingVoxelMap.blueprint(
        voxel_size=0.1,
        max_range=5.0,
        shadow_depth=0.1,
        min_health=0,
        max_health=5,
        emit_every=2,
        ray_subsample=1,
        global_emit_every=10,
    ),
    vis_module(
        global_config.viewer,
        rerun_config={
            "max_hz": {
                "world/global_map": 1.0,
                "world/local_map": 2.0,
            },
            "visual_override": {"world/region_bounds": None},
        },
    ),
).global_config(n_workers=6, robot_model="px4_mapping_debug")
