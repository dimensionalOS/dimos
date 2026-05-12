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

"""Go2 + DimSim blueprint — browser-based sim with nav stack.

Drop-in replacement for unitree_go2 that uses DimSim instead of
hardware or MuJoCo. Works on macOS and ARM Linux.
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import JpegLcmTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.robot.sim.adapter import DimSimAdapter
from dimos.robot.sim.bridge import DimSimBridge
from dimos.robot.sim.jpeg_lcm import SimJpegLCM
from dimos.visualization.vis_module import vis_module


def _go2_sim_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="camera/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.2),
                ),
            ),
            column_shares=[1, 2],
        ),
    )


def _convert_color_image(image: Any) -> Any:
    rerun_data = image.to_rerun()
    return [
        ("camera/color_image", rerun_data),
        ("world/tf/camera_optical/image", rerun_data),
    ]


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _static_body(rr: Any) -> list[Any]:
    # Robot box, parented to TF frame "body" (FRAME_BODY in nav_stack).
    return [
        rr.Boxes3D(
            half_sizes=[0.35, 0.155, 0.2],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/body"),
    ]


# Use nav_stack's standard rerun defaults (paint costmap_cloud, terrain
# maps, paths, way_points, etc.) and overlay our go2/dimsim-specific
# entries. Without this base, SimplePlanner's ``costmap_cloud`` and
# ``goal_path`` channels have no visual_override and don't show up.
rerun_config = nav_stack_rerun_config(
    {
        "blueprint": _go2_sim_rerun_blueprint,
        "pubsubs": [SimJpegLCM()],
        "visual_override": {
            "world/camera_info": DimSimBridge.rerun_suppress_camera_info,
            "world/color_image": _convert_color_image,
            "world/navigation_costmap": _convert_navigation_costmap,
        },
        "static": {
            "world/tf/body": _static_body,
            "world/tf/camera_optical": DimSimBridge.rerun_static_pinhole,
        },
    },
)

# DimSim publishes JPEG-encoded image bytes on /color_image and
# /depth_image. JpegLcmTransport on the consumer side decodes them
# transparently — no Python intermediary needed in the bridge.
_image_transports = autoconnect().transports({
    ("color_image", Image): JpegLcmTransport("/color_image", Image),
    ("depth_image", Image): JpegLcmTransport("/depth_image", Image),
})

unitree_go2_dimsim = (
    autoconnect(
        _image_transports,
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=rerun_config,
        ),
        DimSimBridge.blueprint(
            scene="apt",
            vehicle_height=0.3,
        ),
        DimSimAdapter.blueprint(),
        # Full nav stack: TerrainAnalysis + LocalPlanner + PathFollower +
        # SimplePlanner + PGO + TerrainMapExt. The local planner produces
        # cmd_vel from the path it gets from SimplePlanner.
        create_nav_stack(
            planner="simple",
            vehicle_height=0.3,
            max_speed=1.0,
        ),
        # Relays planner cmd_vel through to bridge.cmd_vel.
        MovementManager.blueprint(),
    )
    .remappings(
        [
            # nav_stack expects the persistent terrain map as DimSim's input.
            (DimSimBridge, "terrain_map", "terrain_map_ext"),
            # SimplePlanner owns way_point — disconnect MovementManager's relay.
            (MovementManager, "way_point", "_mgr_way_point_unused"),
        ]
    )
    .global_config(n_workers=10, robot_model="unitree_go2", simulation=True)
)

__all__ = ["unitree_go2_dimsim"]
