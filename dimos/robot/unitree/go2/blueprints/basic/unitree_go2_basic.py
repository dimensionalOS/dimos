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

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.memory.module import Recorder, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module


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


def _static_robot_body(rr: Any) -> list[Any]:
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


rerun_config: dict[str, Any] = {
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
        "world/lidar": 1,  # publishes at ~7.7 Hz; hidden by default in the blueprint
    },
    "tf_axes": 0.5,
    # slapping a go2 shaped box on the base_link frame
    "static": {
        "world/robot_body": _static_robot_body,
    },
}


class Go2Memory(Recorder):
    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]

    _last_odom_pose: Pose | None = None

    @pose_setter_for("odom")
    async def _odom_pose(self, msg: PoseStamped) -> Pose | None:
        self._last_odom_pose = msg
        return self._last_odom_pose

    @pose_setter_for("lidar")
    async def _lidar_pose(self, msg: PointCloud2) -> Pose | None:
        # Yes, it doesn't make sense to register lidar at the odom pose because the
        # go2 lidar is in the world frame, but map.py (for now) needs this.
        # TODO: fix map.py to use a transform frame
        return getattr(self, "_last_odom_pose", None)


_with_vis = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=rerun_config,
    ),
)


unitree_go2_basic = (
    autoconnect(
        _with_vis,
        GO2Connection.blueprint(),
        Go2Memory.blueprint(),
    ).global_config(n_workers=5, robot_model="unitree_go2")
    # we temporarily disabled sensor timestamps
    # and are derriving all timestmaps upon reception
    # this is because image webrtc stream doesn't have timestamps,
    # so it's difficult to corelate the streams otherwise
    #
    #    .configurators(ClockSyncConfigurator())
)
