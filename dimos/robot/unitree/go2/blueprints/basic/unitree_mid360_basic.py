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

"""Basic Go2 + Mid-360 / Point-LIO visualization blueprint.

Same stack as ``unitree_go2_basic``, with Rerun tuned for Point-LIO worlds
where the floor often sits at Z << 0 (start pose is the origin), plus a
``VoxelGridMapper`` at Mid-360 resolution (3 cm).
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.voxels.lidar_defaults import MID360_VOXEL_SIZE
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_global_map(grid: Any) -> Any:
    # No bottom_cutoff: Point-LIO / Mid-360 worlds often have floor Z << 0
    # (start pose is the origin), so cutting at z=0 hides most of the map.
    return grid.to_rerun()


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
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _mid360_rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world view side by side."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            # Kept even when the recording has no color_image (blank panel).
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                # Hide the fixed XY grid: a plane at z≈0.5 sits *above* Point-LIO
                # floors (often z≪0), which makes the live map look buried.
                line_grid=rrb.LineGrid3D(visible=False),
                # Track the Go2 box so reloc / map alignment stays in view as
                # replay odometry moves base_link through the scene.
                eye_controls=rrb.EyeControls3D(
                    kind="Orbital",
                    tracking_entity="world/tf/base_link",
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_config: dict[str, Any] = {
    "blueprint": _mid360_rerun_blueprint,
    # Custom converters for specific rerun entity paths
    # Normally all these would be specified in their respectative modules
    # Until this is implemented we have central overrides here
    #
    # This is unsustainable once we move to multi robot etc
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/global_map": _convert_global_map,
        "world/merged_map": _convert_global_map,
        "world/loaded_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
    },
    "max_hz": {
        "world/global_map": 0,  # publishes at ~7.8 Hz
        "world/merged_map": 0,
        "world/loaded_map": 1,  # prior map; low rate is enough
        "world/color_image": 0,  # publishes at ~14 Hz
        "world/global_costmap": 0,  # publishes at ~7.6 Hz
        "world/lidar": 1,  # publishes at ~7.7 Hz; hidden by default in the blueprint
    },
    # slapping a go2 shaped box on top of tf/base_link
    "static": {
        "world/tf/base_link": _static_base_link,
    },
}

_with_vis = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=rerun_config,
    ),
)


unitree_mid360_basic = (
    autoconnect(
        _with_vis,
        GO2Connection.blueprint(),
        VoxelGridMapper.blueprint(emit_every=5, voxel_size=MID360_VOXEL_SIZE),
    ).global_config(n_workers=5, robot_model="unitree_go2", lidar_config="mid360")
    # we temporarily disabled sensor timestamps
    # and are derriving all timestmaps upon reception
    # this is because image webrtc stream doesn't have timestamps,
    # so it's difficult to corelate the streams otherwise
    #
    #    .configurators(ClockSyncConfigurator())
)
