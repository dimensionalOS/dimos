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

"""PX4 basic flight blueprint: PointLIO, flight control, H.264 video, and ray-tracing."""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.drone.px4.camera import (
    RERUN_CAMERA_IMAGE_ENTITY,
    RERUN_CAMERA_IMAGE_MAX_HZ,
    camera_rerun_config,
    px4_camera_from_environment,
)
from dimos.robot.drone.px4.config import mavsdk_config_from_environment
from dimos.robot.drone.px4.flight_control import FlightController
from dimos.robot.drone.px4.mid360_mount_tf import Mid360MountStaticTf
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

_default_mavsdk_config = mavsdk_config_from_environment()

px4_basic = (
    autoconnect(
        PointLio.blueprint(sensor_frame_id="mid360_link"),
        Mid360MountStaticTf.blueprint(),
        FlightController.blueprint(mavsdk_config=_default_mavsdk_config),
        px4_camera_from_environment(),
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
            rerun_config=camera_rerun_config(
                max_hz={
                    RERUN_CAMERA_IMAGE_ENTITY: RERUN_CAMERA_IMAGE_MAX_HZ,
                    "world/global_map": 1.0,
                    "world/local_map": 2.0,
                },
                visual_override={"world/region_bounds": None},
            ),
        ),
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport.spec(
                "/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
            ("video_h264", CompressedVideo): LCMTransport.spec("/video_h264", CompressedVideo),
        }
    )
    .remappings([(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")])
    .global_config(n_workers=8, robot_model="px4_basic")
)
