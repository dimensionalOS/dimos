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

"""Go2 navigation stack with H.264 transport enabled for the color image stream."""

from typing import Any, cast

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import H264LcmTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.protocol.video.demo_h264_video_e2e import H264VideoProbe
from dimos.protocol.video.h264 import H264Config
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module

_go2_h264_config = H264Config(
    bitrate=2_000_000,
    target_fps=15,
    keyframe_interval=30,
)


def _convert_h264_color_image(image: Image) -> Any:
    """Convert decoded color frames before logging them in Rerun."""
    return image.to_rerun()


_h264_rerun_config = {
    **rerun_config,
    "visual_override": {
        **cast("dict[str, Any]", rerun_config["visual_override"]),
        "world/color_image": _convert_h264_color_image,
    },
}

unitree_go2_h264_video = (
    autoconnect(
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=_h264_rerun_config,
        ),
        GO2Connection.blueprint(),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        WavefrontFrontierExplorer.blueprint(),
        PatrollingModule.blueprint(),
        MovementManager.blueprint(),
        H264VideoProbe.blueprint(),
    )
    .transports(
        {
            ("color_image", Image): H264LcmTransport(
                "/color_image",
                Image,
                config=_go2_h264_config,
                decode_images=True,
            ),
        }
    )
    .global_config(n_workers=11, robot_model="unitree_go2")
)
