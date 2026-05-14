#!/usr/bin/env python3
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

"""
DimSim Agent — runs the dimos nav + agent stack connected to DimSim via LCM.

DimSim acts as the robot (like simplerobot.py but richer):
  - Publishes: /odom, /color_image, /lidar, /depth_image
  - Subscribes: /cmd_vel

This script runs the dimos brain that processes those sensors and sends commands.

Usage (run with dimos venv):
    ../dimos/.venv/bin/python dimos-cli/agent.py
    ../dimos/.venv/bin/python dimos-cli/agent.py --nav-only    # no LLM agent, just exploration
"""

import argparse

from dimos.core.blueprints import autoconnect
from dimos.core.transport import JpegLcmTransport, LCMTransport
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.protocol.service.lcmservice import autoconf

# LCM transports — same channels DimSim publishes/subscribes on.
_transports = {
    ("color_image", Image): JpegLcmTransport("/color_image", Image),
    ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
    ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
    ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
}

# Navigation stack: LiDAR → voxels → costmap → frontier explorer → path planner
nav = (
    autoconnect(
        voxel_mapper(voxel_size=0.1),
        cost_mapper(algo="simple"),
        replanning_a_star_planner(),
        wavefront_frontier_explorer(),
    )
    .transports(_transports)
    .global_config(n_dask_workers=6, robot_model="dimsim")
)


def build_agentic():
    """Full agentic: nav + spatial memory + LLM agent + skills."""
    from dimos.agents.agent import llm_agent
    from dimos.agents.cli.human import human_input
    from dimos.agents.cli.web import web_input
    from dimos.agents.skills.navigation import navigation_skill
    from dimos.agents.skills.speak_skill import speak_skill
    from dimos.perception.spatial_perception import spatial_memory
    from dimos.utils.monitoring import utilization

    return autoconnect(
        nav,
        spatial_memory(),
        utilization(),
        llm_agent(),
        human_input(),
        navigation_skill(),
        web_input(),
        speak_skill(),
    ).global_config(n_dask_workers=8)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DimSim dimos agent")
    parser.add_argument(
        "--nav-only",
        action="store_true",
        help="Run nav stack only (no LLM agent)",
    )
    args = parser.parse_args()

    autoconf()

    blueprint = nav if args.nav_only else build_agentic()
    coordinator = blueprint.build()

    print("DimSim agent running.")
    print("  Subscribing: /odom, /color_image, /lidar")
    print("  Publishing:  /cmd_vel")
    print("  Ctrl+C to exit")

    coordinator.loop()
