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

import platform

from dimos.agents.agent import llm_agent
from dimos.agents.cli.human import human_input
from dimos.agents.cli.web import web_input
from dimos.agents.ollama_agent import ollama_installed
from dimos.agents.skills.navigation import navigation_skill
from dimos.agents.skills.speak_skill import speak_skill
from dimos.agents.spec import Provider
from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.blueprints import autoconnect
from dimos.core.transport import JpegLcmTransport, JpegShmTransport, pSHMTransport
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.frontier_exploration import (
    wavefront_frontier_explorer,
)
from dimos.navigation.replanning_a_star.module import (
    replanning_a_star_planner,
)
from dimos.perception.spatial_perception import spatial_memory
from dimos.robot.foxglove_bridge import foxglove_bridge
from dimos.robot.unitree.connection.go2 import go2_connection
from dimos.robot.unitree_webrtc.unitree_skill_container import unitree_skills
from dimos.utils.monitoring import utilization
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

# Mac has some issue with high bandwidth UDP
#
# so we use pSHMTransport for color_image
# (Could we adress this on the system config layer? Is this fixable on mac?)
mac = autoconnect(
    foxglove_bridge(
        shm_channels=[
            "/color_image#sensor_msgs.Image",
        ]
    ),
).transports(
    {
        ("color_image", Image): pSHMTransport(
            "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
        ),
    }
)


linux = autoconnect(foxglove_bridge())

basic = autoconnect(
    go2_connection(),
    linux if platform.system() == "Linux" else mac,
    websocket_vis(),
).global_config(n_dask_workers=4, robot_model="unitree_go2")

nav = autoconnect(
    basic,
    voxel_mapper(voxel_size=0.05),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
).global_config(n_dask_workers=6, robot_model="unitree_go2")

spatial = autoconnect(
    nav,
    spatial_memory(),
    utilization(),
).global_config(n_dask_workers=8)

with_jpeglcm = nav.transports(
    {
        ("color_image", Image): JpegLcmTransport("/color_image", Image),
    }
)

with_jpegshm = autoconnect(
    nav.transports(
        {
            ("color_image", Image): JpegShmTransport("/color_image", quality=75),
        }
    ),
    foxglove_bridge(
        jpeg_shm_channels=[
            "/color_image#sensor_msgs.Image",
        ]
    ),
)

_common_agentic = autoconnect(
    human_input(),
    navigation_skill(),
    unitree_skills(),
    web_input(),
    speak_skill(),
)

agentic = autoconnect(
    spatial,
    llm_agent(),
    _common_agentic,
)

agentic_ollama = autoconnect(
    spatial,
    llm_agent(
        model="qwen3:8b",
        provider=Provider.OLLAMA,  # type: ignore[attr-defined]
    ),
    _common_agentic,
).requirements(
    ollama_installed,
)

agentic_huggingface = autoconnect(
    spatial,
    llm_agent(
        model="Qwen/Qwen2.5-1.5B-Instruct",
        provider=Provider.HUGGINGFACE,  # type: ignore[attr-defined]
    ),
    _common_agentic,
)
