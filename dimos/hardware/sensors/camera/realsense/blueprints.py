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

"""Standalone RealSense viewer: color, depth, and the RGBD cloud in Rerun.

``dimos run real-sense-camera-vis``

The cloud comes from the camera's own ``enable_pointcloud`` path
(:meth:`PointCloud2.from_rgbd`), so it lands in the color optical frame and rides
whatever tf tree the camera is mounted under.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.visualization.vis_module import vis_module

# Dot radius in screen-space UI points.
_CLOUD_RADIUS_UI = 1.0


class RealSenseMountTf(StaticTfPublisher):
    """The standalone viewer's mount: camera_link sits at the world origin.

    The camera publishes only its own subtree; without this edge rerun has no
    path from world to it and draws nothing.
    """

    def transforms(self) -> list[Transform]:
        return [
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                child_frame_id="camera_link",
            )
        ]


def _rerun_blueprint() -> Any:
    """Color + depth stacked beside the 3D view."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Color"),
                rrb.Spatial2DView(origin="world/depth_image", name="Depth"),
            ),
            rrb.Spatial3DView(origin="world", name="3D"),
            column_shares=[1, 2],
        ),
    )


def _cloud(cloud: PointCloud2) -> Any:
    # The default "spheres" mode is sized for sparse lidar; a dense RGBD cloud
    # reads better as flat dots that keep their size as you zoom.
    return cloud.to_rerun(mode="points", ui_radius=_CLOUD_RADIUS_UI)


_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config={
        "blueprint": _rerun_blueprint,
        "visual_override": {"world/pointcloud": _cloud},
        "tf_axes": 0.2,
    },
)

real_sense_camera_vis = autoconnect(
    RealSenseCamera.blueprint(),
    RealSenseMountTf.blueprint(),
    _vis,
).global_config(n_workers=4)
