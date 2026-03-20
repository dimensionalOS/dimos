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

"""Shared visualization module factory for all robot blueprints."""

from typing import Any

from dimos.core.blueprints import Blueprint, autoconnect
from dimos.core.global_config import ViewerBackend
from dimos.protocol.pubsub.impl.lcmpubsub import LCM


def vis_module(
    viewer_backend: ViewerBackend,
    rerun_config: dict[str, Any] | None = None,
    foxglove_config: dict[str, Any] | None = None,
) -> Blueprint:
    """
    Example usage:
        from dimos.core.global_config import global_config
        viz = vis_module(
            global_config.viewer,
            rerun_config={
                "visual_override": {
                    "world/camera_info": lambda camera_info: camera_info.to_rerun(
                        image_topic="/world/color_image",
                        optical_frame="camera_optical",
                    ),
                    "world/global_map": lambda grid: grid.to_rerun(voxel_size=0.1, mode="boxes"),
                    "world/navigation_costmap": lambda grid: grid.to_rerun(
                        colormap="Accent",
                        z_offset=0.015,
                        opacity=0.2,
                        background="#484981",
                    ),
                },
                "static": {
                    "world/tf/base_link": lambda rr: [
                        rr.Boxes3D(
                            half_sizes=[0.2, 0.15, 0.75],
                            colors=[(0, 255, 127)],
                            fill_mode="MajorWireframe",
                        ),
                        rr.Transform3D(parent_frame="tf#/base_link"),
                    ]
                },
            }
        )
    """
    if foxglove_config is None:
        foxglove_config = {}
    if rerun_config is None:
        rerun_config = {}
    rerun_config = {**rerun_config}
    rerun_config.setdefault("pubsubs", [LCM()])

    match viewer_backend:
        case "foxglove":
            from dimos.robot.foxglove_bridge import foxglove_bridge

            result = autoconnect(foxglove_bridge(**foxglove_config))
        case "rerun" | "rerun-web" | "rerun-connect":
            from dimos.visualization.rerun.bridge import _BACKEND_TO_MODE, rerun_bridge

            viewer_mode = _BACKEND_TO_MODE.get(viewer_backend, "native")
            result = autoconnect(rerun_bridge(viewer_mode=viewer_mode, **rerun_config))
        case _:
            result = autoconnect()

    return result
