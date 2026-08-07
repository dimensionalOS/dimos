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

"""cuVSLAM on a RealSense stereo camera and nothing else.

    dimos run demo-cuvslam --viewer rerun --rerun-host 0.0.0.0

The smallest thing that shows whether cuVSLAM is tracking: a camera, the tracker, and a
viewer. Wire it into a robot and when that misbehaves this narrows down whether the
problem is the tracker or everything around it.

``emitter_enabled`` is **off**: the projector's dot pattern is fixed to the camera, so it
moves exactly with it and feature trackers latch onto it and bias motion toward zero.

What to look for: ``odometry`` advancing pose after pose, and restarts staying rare.
Frames arriving in the viewer only proves the camera works -- cuVSLAM restarting its world
frame constantly still publishes odometry and still draws. ``world/path`` is the trail of
everywhere the camera has been, which is the quickest read on both: it should retrace your
own route, and a restart shows up as a straight jump across it.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.mapping.odometry_path import OdometryPath
from dimos.msgs.nav_msgs.Path import Path
from dimos.visualization.vis_module import vis_module

CAMERA_NAME = "d455"


def _path_at_true_height(path: Path) -> Any:
    """Draw the trail where it actually is.

    ``to_rerun`` lifts the line half a metre by default so it clears a costmap. There
    is no costmap here and the camera flies at whatever height you carry it, so the
    lift would just put the trail somewhere the camera never was.
    """
    return path.to_rerun(z_offset=0.0, radii=0.02)


def _rerun_blueprint() -> Any:
    """The cameras down one side, the 3D world taking the rest.

    One view, not one per imager: the rerun bridge names an entity after the topic, and
    every camera shares the one the tracker reads, so this alternates between them.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/image", name="cameras"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
            ),
            column_shares=[1, 3],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


demo_cuvslam = (
    autoconnect(
        RealSenseCamera.blueprint(
            camera_name=CAMERA_NAME,
            width=848,
            height=480,
            fps=30,
            # cuVSLAM tracks on the IR pair. The device delivers it already rectified,
            # which is what lets the module run a pinhole model with an identity
            # inter-camera rotation.
            enable_infrared=True,
            emitter_enabled=False,
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            enable_imu=False,
        ),
        CuvslamOdometry.blueprint(),
        # cuVSLAM publishes only where the camera is now. This keeps the history so
        # the viewer can draw where it has been.
        OdometryPath.blueprint(),
        vis_module(
            global_config.viewer,
            rerun_config={
                "blueprint": _rerun_blueprint,
                "visual_override": {"world/path": _path_at_true_height},
            },
        ),
    )
    .remappings(
        [
            # Both imagers onto the one stream: the tracker tells its cameras apart by
            # frame_id, so a second camera is two more lines here and nothing else.
            (RealSenseCamera, "infrared_left", "image"),
            (RealSenseCamera, "infrared_right", "image"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
            (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
        ]
    )
    .global_config(n_workers=4)
)
