#!/usr/bin/env python3

# Copyright 2025 Dimensional Inc.
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

from dimos_lcm.sensor_msgs import CameraInfo

from dimos.agents2.agent import llm_agent
from dimos.agents2.cli.human import human_input
from dimos.agents2.skills.navigation import navigation_skill
from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE, DEFAULT_CAPACITY_DEPTH_IMAGE
from dimos.core.blueprints import autoconnect
from dimos.core.transport import JpegLcmTransport, JpegShmTransport, LCMTransport, pSHMTransport
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.bt_navigator.navigator import (
    behavior_tree_navigator,
)
from dimos.navigation.frontier_exploration import (
    wavefront_frontier_explorer,
)
from dimos.navigation.global_planner import astar_planner
from dimos.navigation.local_planner.holonomic_local_planner import (
    holonomic_local_planner,
)
from dimos.perception.object_tracker import object_tracking
from dimos.perception.spatial_perception import spatial_memory
from dimos.robot.foxglove_bridge import foxglove_bridge
from dimos.robot.unitree_webrtc.depth_module import depth_module
from dimos.robot.unitree_webrtc.type.map import mapper
from dimos.robot.unitree_webrtc.unitree_go2 import connection
from dimos.utils.monitoring import utilization
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

basic = (
    autoconnect(
        connection(),
        mapper(voxel_size=0.5, global_publish_interval=2.5),
        astar_planner(),
        holonomic_local_planner(),
        behavior_tree_navigator(),
        wavefront_frontier_explorer(),
        websocket_vis(),
        foxglove_bridge(),
    )
    .global_config(n_dask_workers=4)
    .transports(
        # These are kept the same so that we don't have to change foxglove configs.
        # Although we probably should.
        {
            ("color_image", Image): LCMTransport("/go2/color_image", Image),
            ("camera_pose", PoseStamped): LCMTransport("/go2/camera_pose", PoseStamped),
            ("camera_info", CameraInfo): LCMTransport("/go2/camera_info", CameraInfo),
        }
    )
)

standard = autoconnect(
    basic,
    spatial_memory(),
    object_tracking(frame_id="camera_link"),
    utilization(),
).global_config(n_dask_workers=8)

standard_with_shm = autoconnect(
    standard.transports(
        {
            ("color_image", Image): pSHMTransport(
                "/go2/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
        }
    ),
    foxglove_bridge(
        shm_channels=[
            "/go2/color_image#sensor_msgs.Image",
        ]
    ),
)

standard_with_jpeglcm = standard.transports(
    {
        ("color_image", Image): JpegLcmTransport("/go2/color_image", Image),
    }
)

standard_with_jpegshm = autoconnect(
    standard.transports(
        {
            ("color_image", Image): JpegShmTransport("/go2/color_image", quality=75),
        }
    ),
    foxglove_bridge(
        jpeg_shm_channels=[
            "/go2/color_image#sensor_msgs.Image",
        ]
    ),
)

agentic = autoconnect(
    standard,
    llm_agent(),
    human_input(),
    navigation_skill(),
)
