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

"""Minimal X2 Ultra stack: vis module only. Base for larger blueprints."""

from typing import Any

from dimos.core.global_config import global_config
from dimos.visualization.vis_module import vis_module


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _static_base_link(rr: Any) -> list[Any]:
    # X2 Ultra is ~1.65 m tall, ~0.45 m wide, ~0.3 m deep
    return [
        rr.Boxes3D(
            half_sizes=[0.225, 0.15, 0.825],
            colors=[(0, 200, 255)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _x2_rerun_blueprint() -> Any:
    """Split layout: camera feed on left, 3D world view on right."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
    )


rerun_config = {
    "blueprint": _x2_rerun_blueprint,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
    },
    "static": {
        "world/tf/base_link": _static_base_link,
    },
}

agibot_x2_primitive = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=rerun_config,
)

__all__ = ["agibot_x2_primitive"]
