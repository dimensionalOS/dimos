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

from typing import Any

from dimos.constants import (
    DEFAULT_CAPACITY_COLOR_IMAGE,
    DEFAULT_CAPACITY_OCCUPANCY_GRID,
    DEFAULT_CAPACITY_POINTCLOUD,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import pSHMTransport
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module

# Route large local replay and mapping streams through SHM on every platform.
# Small control/status streams continue to use the default LCM transport.
_local_high_bandwidth_transports: dict[tuple[str, type], pSHMTransport[Any]] = {
    ("color_image", Image): pSHMTransport(
        "/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
    ("lidar", PointCloud2): pSHMTransport("/lidar", default_capacity=DEFAULT_CAPACITY_POINTCLOUD),
    ("pointcloud", PointCloud2): pSHMTransport(
        "/pointcloud", default_capacity=DEFAULT_CAPACITY_POINTCLOUD
    ),
    ("global_map", PointCloud2): pSHMTransport(
        "/global_map", default_capacity=DEFAULT_CAPACITY_POINTCLOUD
    ),
    ("merged_map", PointCloud2): pSHMTransport(
        "/merged_map", default_capacity=DEFAULT_CAPACITY_POINTCLOUD
    ),
    ("global_costmap", OccupancyGrid): pSHMTransport(
        "/global_costmap", default_capacity=DEFAULT_CAPACITY_OCCUPANCY_GRID
    ),
    ("navigation_costmap", OccupancyGrid): pSHMTransport(
        "/navigation_costmap", default_capacity=DEFAULT_CAPACITY_OCCUPANCY_GRID
    ),
}

_transports_base = autoconnect().transports(_local_high_bandwidth_transports)


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_global_map(grid: Any) -> Any:
    return grid.to_rerun(bottom_cutoff=0)


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


def _go2_rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world view side by side."""
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
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_config = {
    "blueprint": _go2_rerun_blueprint,
    # Custom converters for specific rerun entity paths
    # Normally all these would be specified in their respectative modules
    # Until this is implemented we have central overrides here
    #
    # This is unsustainable once we move to multi robot etc
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/global_map": _convert_global_map,
        "world/merged_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
    },
    "max_hz": {
        "world/global_map": 0,  # publishes at ~7.8 Hz
        "world/color_image": 0,  # publishes at ~14 Hz
        "world/global_costmap": 0,  # publishes at ~7.6 Hz
    },
    # slapping a go2 shaped box on top of tf/base_link
    "static": {
        "world/tf/base_link": _static_base_link,
    },
}

_with_vis = autoconnect(
    _transports_base,
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=rerun_config,
    ),
)


unitree_go2_basic = (
    autoconnect(
        _with_vis,
        GO2Connection.blueprint(),
    ).global_config(n_workers=4, robot_model="unitree_go2")
    # we temporarily disabled sensor timestamps
    # and are derriving all timestmaps upon reception
    # this is because image webrtc stream doesn't have timestamps,
    # so it's difficult to corelate the streams otherwise
    #
    #    .configurators(ClockSyncConfigurator())
)

__all__ = [
    "unitree_go2_basic",
]
