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

"""The Go2's Rerun view: the front camera beside the 3D world, with the navigation maps and plans drawn in it."""

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING, Any

from dimos.mapping.ray_tracing.viz import MAP_REGIONS_ENTITY, render_map_region
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.global_planner.mls_planner.viz import (
    SURFACE_MAP_ENTITY,
    render_surface_region,
)
from dimos.navigation.global_planner.viz import HEIGHT_RANGE, nav_static, nav_visual_override
from dimos.navigation.local_planner.viz import motion_visual_override
from dimos.robot.unitree.go2.config import (
    ROBOT_HEIGHT,
    ROBOT_LENGTH,
    ROBOT_WIDTH,
    planner_viz_hz,
    voxel_size,
    wall_clearance_m,
)

if TYPE_CHECKING:
    from rerun.blueprint import Blueprint

    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.visualization.rerun.bridge import RerunData

CAMERA_ENTITY = "world/color_image"


def _camera_info_to_pinhole(camera_info: CameraInfo) -> RerunData:
    # onto the image's own entity: a Pinhole only projects its entity and its children
    return camera_info.to_rerun(image_topic=CAMERA_ENTITY)


def _render_cloud(msg: Any) -> Any:
    return msg.to_rerun(voxel_size=0.01)


def _rerun_blueprint() -> Blueprint:
    """Camera feed beside the 3D world; the raw lidar and planner graph are hidden, tickable in the viewer."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin=CAMERA_ENTITY, name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                    "world/nodes": rrb.EntityBehavior(visible=False),
                    "world/node_edges": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def rerun_config() -> dict[str, Any]:
    return {
        "blueprint": _rerun_blueprint,
        "tf_axes": 0.5,
        # The robot box hangs off base_link on its own entity: a static transform under world/tf would override the live one.
        "static": nav_static(ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT, wall_clearance_m),
        "visual_override": {
            "world/camera_info": _camera_info_to_pinhole,
            "world/lidar": _render_cloud,
            MAP_REGIONS_ENTITY: partial(
                render_map_region, voxel_size=voxel_size, height_range=HEIGHT_RANGE
            ),
            SURFACE_MAP_ENTITY: partial(
                render_surface_region,
                voxel_size=voxel_size,
                wall_clearance_m=wall_clearance_m,
                clearance_clamp_m=1.0,
            ),
            **nav_visual_override(planner_viz_hz, voxel_size, wall_clearance_m),
            **motion_visual_override(embodiment=GO2),
        },
        "max_hz": {
            "world/color_image": 0,
            "world/lidar": 1,
        },
    }
