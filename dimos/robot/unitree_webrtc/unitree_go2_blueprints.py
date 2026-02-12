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


import platform

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,  # type: ignore[import-untyped]
)
from dimos_lcm.foxglove_msgs.SceneUpdate import SceneUpdate  # type: ignore[import-untyped]

from dimos.agents.agent import llm_agent
from dimos.agents.cli.human import human_input
from dimos.agents.cli.web import web_input
from dimos.agents.ollama_agent import ollama_installed
from dimos.agents.skills.navigation import navigation_skill
from dimos.agents.skills.person_follow import person_follow_skill
from dimos.agents.skills.speak_skill import speak_skill
from dimos.agents.spec import Provider
from dimos.agents.vlm_agent import vlm_agent
from dimos.agents.vlm_stream_tester import vlm_stream_tester
from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import (
    JpegLcmTransport,
    LCMTransport,
    ROSTransport,
    pSHMTransport,
)
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.navigation.frontier_exploration import (
    wavefront_frontier_explorer,
)
from dimos.navigation.replanning_a_star.module import (
    replanning_a_star_planner,
)
from dimos.perception.detection.module3D import Detection3DModule, detection3d_module
from dimos.perception.experimental.temporal_memory import temporal_memory
from dimos.perception.spatial_perception import spatial_memory
from dimos.protocol.mcp.mcp import MCPModule
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.connection.go2 import GO2Connection, go2_connection
from dimos.robot.unitree_webrtc.unitree_skill_container import unitree_skills
from dimos.utils.monitoring import utilization
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

# Mac has some issue with high bandwidth UDP, so we use pSHMTransport for color_image
# actually we can use pSHMTransport for all platforms, and for all streams
# TODO need a global transport toggle on blueprints/global config
mac_transports: dict[tuple[str, type], pSHMTransport[Image]] = {
    ("color_image", Image): pSHMTransport(
        "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
}

base = autoconnect() if platform.system() == "Linux" else autoconnect().transports(mac_transports)


rerun_config = {
    # any pubsub that supports subscribe_all and topic that supports str(topic)
    # is acceptable here
    "pubsubs": [LCM(autoconf=True)],
    # Custom converters for specific rerun entity paths
    # Normally all these would be specified in their respectative modules
    # Until this is implemented we have central overrides here
    #
    # This is unsustainable once we move to multi robot etc
    "visual_override": {
        "world/camera_info": lambda camera_info: camera_info.to_rerun(
            image_topic="/world/color_image",
            optical_frame="camera_optical",
        ),
        "world/global_map": lambda grid: grid.to_rerun(voxel_size=0.1),
        "world/navigation_costmap": lambda grid: grid.to_rerun(
            colormap="Accent",
            z_offset=0.015,
            opacity=0.2,
            background="#484981",
        ),
    },
    # slapping a go2 shaped box on top of tf/base_link
    "static": {
        "world/tf/base_link": lambda rr: [
            rr.Boxes3D(
                half_sizes=[0.35, 0.155, 0.2],
                colors=[(0, 255, 127)],
                fill_mode="wireframe",
            ),
            rr.Transform3D(parent_frame="tf#/base_link"),
        ]
    },
}


match global_config.viewer_backend:
    case "foxglove":
        from dimos.robot.foxglove_bridge import foxglove_bridge

        with_vis = autoconnect(
            base,
            foxglove_bridge(shm_channels=["/color_image#sensor_msgs.Image"]),
        )
    case "rerun":
        from dimos.visualization.rerun.bridge import rerun_bridge

        with_vis = autoconnect(base, rerun_bridge(**rerun_config))
    case "rerun-web":
        from dimos.visualization.rerun.bridge import rerun_bridge

        with_vis = autoconnect(base, rerun_bridge(viewer_mode="web", **rerun_config))
    case _:
        with_vis = base


unitree_go2_basic = autoconnect(
    with_vis,
    go2_connection(),
    websocket_vis(),
).global_config(n_dask_workers=4, robot_model="unitree_go2")

unitree_go2 = autoconnect(
    unitree_go2_basic,
    voxel_mapper(voxel_size=0.1),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
).global_config(n_dask_workers=6, robot_model="unitree_go2")


unitree_go2_ros = unitree_go2.transports(
    {
        ("lidar", PointCloud2): ROSTransport("lidar", PointCloud2),
        ("global_map", PointCloud2): ROSTransport("global_map", PointCloud2),
        ("odom", PoseStamped): ROSTransport("odom", PoseStamped),
        ("color_image", Image): ROSTransport("color_image", Image),
    }
)

unitree_go2_detection = (
    autoconnect(
        unitree_go2,
        detection3d_module(
            camera_info=GO2Connection.camera_info_static,
        ),
    )
    .remappings(
        [
            (Detection3DModule, "pointcloud", "global_map"),
        ]
    )
    .transports(
        {
            # Detection 3D module outputs
            ("detections", Detection3DModule): LCMTransport(
                "/detector3d/detections", Detection2DArray
            ),
            ("annotations", Detection3DModule): LCMTransport(
                "/detector3d/annotations", ImageAnnotations
            ),
            ("scene_update", Detection3DModule): LCMTransport(
                "/detector3d/scene_update", SceneUpdate
            ),
            ("detected_pointcloud_0", Detection3DModule): LCMTransport(
                "/detector3d/pointcloud/0", PointCloud2
            ),
            ("detected_pointcloud_1", Detection3DModule): LCMTransport(
                "/detector3d/pointcloud/1", PointCloud2
            ),
            ("detected_pointcloud_2", Detection3DModule): LCMTransport(
                "/detector3d/pointcloud/2", PointCloud2
            ),
            ("detected_image_0", Detection3DModule): LCMTransport("/detector3d/image/0", Image),
            ("detected_image_1", Detection3DModule): LCMTransport("/detector3d/image/1", Image),
            ("detected_image_2", Detection3DModule): LCMTransport("/detector3d/image/2", Image),
        }
    )
)


unitree_go2_spatial = autoconnect(
    unitree_go2,
    spatial_memory(),
    utilization(),
).global_config(n_dask_workers=8)

_with_jpeglcm = unitree_go2.transports(
    {
        ("color_image", Image): JpegLcmTransport("/color_image", Image),
    }
)

_common_agentic = autoconnect(
    human_input(),
    navigation_skill(),
    person_follow_skill(camera_info=GO2Connection.camera_info_static),
    unitree_skills(),
    web_input(),
    speak_skill(),
)

unitree_go2_agentic = autoconnect(
    unitree_go2_spatial,
    llm_agent(),
    _common_agentic,
)

unitree_go2_agentic_mcp = autoconnect(
    unitree_go2_agentic,
    MCPModule.blueprint(),
)

unitree_go2_agentic_ollama = autoconnect(
    unitree_go2_spatial,
    llm_agent(
        model="qwen3:8b",
        provider=Provider.OLLAMA,  # type: ignore[attr-defined]
    ),
    _common_agentic,
).requirements(
    ollama_installed,
)

unitree_go2_agentic_huggingface = autoconnect(
    unitree_go2_spatial,
    llm_agent(
        model="Qwen/Qwen2.5-1.5B-Instruct",
        provider=Provider.HUGGINGFACE,  # type: ignore[attr-defined]
    ),
    _common_agentic,
)

unitree_go2_vlm_stream_test = autoconnect(
    unitree_go2_basic,
    vlm_agent(),
    vlm_stream_tester(),
)

unitree_go2_temporal_memory = autoconnect(
    unitree_go2_agentic,
    temporal_memory(),
)
