#!/usr/bin/env python3
# Copyright 2027 Dimensional Inc.
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

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import unitree_go2_agentic
from dimos.visualization.vis_module import vis_module


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _static_base_link(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.35, 0.155, 0.2],
            colors=[(0, 255, 127)],
            fill_mode="wireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _go2_rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.Spatial2DView(origin="world/depth_image", name="Depth"),
                rrb.Spatial2DView(origin="world/tracking_image", name="Track"),
                row_shares=[1, 1, 1],
            ),
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/tracking_image", name="Info"),
                rrb.Spatial3DView(origin="world", name="3D"),
                row_shares=[1, 2],
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


_rerun_config = {
    "blueprint": _go2_rerun_blueprint,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/navigation_costmap": _convert_navigation_costmap,
    },
    "static": {
        "world/tf/base_link": _static_base_link,
    },
}

unitree_go2_security = autoconnect(
    unitree_go2_agentic,
    vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
)

__all__ = ["unitree_go2_security"]
