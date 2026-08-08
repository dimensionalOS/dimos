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

"""cuVSLAM on a ZED stereo camera.

dimos run demo-zed-cuvslam --viewer rerun --rerun-host 0.0.0.0
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.zed.camera import ZEDCamera
from dimos.mapping.cuvslam.cuvslam import CuvslamOdometry
from dimos.mapping.cuvslam.demo_cuvslam import cuvslam_rerun_blueprint, path_at_true_height
from dimos.mapping.odometry_path import OdometryPath
from dimos.visualization.vis_module import vis_module

demo_zed_cuvslam = (
    autoconnect(
        ZEDCamera.blueprint(
            fps=30,
            enable_right_image=True,
            enable_depth=False,
            enable_pointcloud=False,
            # The ZED's own tracker would compete with cuVSLAM for the same job.
            enable_tracking=False,
        ),
        CuvslamOdometry.blueprint(),
        OdometryPath.blueprint(),
        vis_module(
            global_config.viewer,
            rerun_config={
                "blueprint": cuvslam_rerun_blueprint,
                "visual_override": {"world/path": path_at_true_height},
            },
        ),
    )
    .remappings(
        [
            # Both eyes onto the one stream; the tracker tells them apart by frame_id.
            (ZEDCamera, "color_image", "image"),
            (ZEDCamera, "right_image", "image"),
            (ZEDCamera, "right_camera_info", "camera_info"),
        ]
    )
    .global_config(n_workers=4)
)
