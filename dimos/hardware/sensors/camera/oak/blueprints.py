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

"""Standalone OAK viewer: colour, depth and the camera in Rerun.

``dimos run oak-camera-vis`` (add ``--enable-pointcloud true`` for the RGBD cloud).
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.oak.camera import OakCamera, OakCameraConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from dimos.visualization.vis_module import vis_module


class OakMountTfConfig(StaticTfPublisherConfig):
    # The camera's link, named as OakCamera names it: the same default, and the same
    # namespace and frame_id_prefix applied by Module.frame_id.
    frame_id: str | None = OakCameraConfig.model_fields["frame_id"].default


class OakMountTf(StaticTfPublisher):
    """The standalone viewer's mount: the camera's link sits at the world origin.

    The camera publishes only its own frames; without this edge Rerun has no path from
    world to them and draws nothing in 3D.
    """

    config: OakMountTfConfig

    def transforms(self) -> list[Transform]:
        return [
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                child_frame_id=self.frame_id,
            )
        ]


def _rerun_blueprint() -> Any:
    """Colour and depth stacked beside the 3D view."""
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


oak_camera_vis = autoconnect(
    OakCamera.blueprint(),
    OakMountTf.blueprint(),
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={"blueprint": _rerun_blueprint, "tf_axes": 0.1},
    ),
)
