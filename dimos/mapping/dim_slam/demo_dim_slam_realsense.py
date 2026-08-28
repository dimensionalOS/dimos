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

"""dimSLAM on a RealSense stereo camera.

    dimos run demo-dim-slam-realsense --viewer rerun --rerun-host 0.0.0.0

``world/odom_hist`` should retrace the route walked.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.dim_slam.dim_slam import DimSlam
from dimos.mapping.odometry_hist import OdometryHist, path_at_true_height
from dimos.visualization.vis_module import vis_module


def dim_slam_rerun_blueprint() -> Any:
    """Rerun names entities after the topic, which both cameras share."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/image", name="cameras"),
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


demo_dim_slam_realsense = (
    autoconnect(
        RealSenseCamera.blueprint(
            fps=30,
            enable_infrared=True,
            emitter_enabled=False,
            enable_color=False,
            enable_depth=False,
        ),
        DimSlam.blueprint(camera_mode="stereo"),
        OdometryHist.blueprint(),
        vis_module(
            global_config.viewer,
            rerun_config={
                "blueprint": dim_slam_rerun_blueprint,
                "visual_override": {"world/odom_hist": path_at_true_height},
            },
        ),
    )
    .remappings(
        [
            (RealSenseCamera, "infrared_left", "image"),
            (RealSenseCamera, "infrared_right", "image"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
            (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
        ]
    )
    .global_config(n_workers=4)
)
