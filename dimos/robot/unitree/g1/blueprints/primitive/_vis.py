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

"""Visualization sub-blueprint: Rerun viewer with G1-specific visual overrides."""

from typing import Any

from dimos.core.global_config import global_config
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.vis_module import vis_module


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_global_map(grid: Any) -> Any:
    return grid.to_rerun(voxel_size=0.1, mode="boxes")


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _static_path_frame(rr: Any) -> list[Any]:
    return [rr.Transform3D(parent_frame="tf#/sensor")]


def _static_map_frame(rr: Any) -> list[Any]:
    return [rr.Transform3D(parent_frame="tf#/map")]


def _static_base_link(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.2, 0.15, 0.62],
            centers=[[0, 0, -0.62]],
            colors=[(0, 255, 127)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "pubsubs": [LCM()],
        "visual_override": {
            "world/camera_info": _convert_camera_info,
            "world/global_map": _convert_global_map,
            "world/navigation_costmap": _convert_navigation_costmap,
        },
        "static": {
            "world/tf/base_link": _static_base_link,
            "world/path": _static_path_frame,
            # Registered scan and global terrain map are in map-frame coordinates.
            # Anchor them to tf#/map so they render at the correct height in the
            # Rerun world frame (which is shifted from map by -vehicle_height).
            "world/lidar": _static_map_frame,
            "world/global_pointcloud": _static_map_frame,
        },
    },
)
