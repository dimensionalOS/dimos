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

"""The R1's head stereo depth, configured for this rig.

Galaxea ships the head as two RGB eyes and no depth topic, so depth is matched
here, on the Orin, by :class:`StereoCloud`. Everything that is a property of
this particular head lives in one place -- the rectification the rig was
fitted with, the range the matcher can be trusted to, the resolution the Orin
can afford at camera rate -- so a second blueprint that matches the head
cannot quietly disagree with the first.

The calibration (baseline and how the right eye is aimed relative to the left)
comes from :func:`dimos.robot.galaxea.r1pro.config.stereo_calibration`: a
``calibration.json`` written by ``calibrate_stereo`` when one exists, the
committed rig numbers otherwise.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.depth_cloud.module import StereoCloud
from dimos.robot.galaxea.r1pro.stereo_calibration import (
    R1StereoCalibration,
    stereo_cloud_kwargs,
)

# The tf frame the head's left eye sits on, and the one R1ProConnection's FK
# publishes. It is an OPTICAL frame in the vendor URDF (x right, y down, z
# forward), which is the convention the matcher's cloud comes out in, so the
# cloud can carry this frame_id as it is. Matches
# R1ProConnectionConfig.head_camera_frame_id.
HEAD_CAMERA_FRAME = "camera_head_left_link"

# Where that optical frame sits in base_link with the torso at its neutral
# pose, from the vendor URDF through the same forward kinematics the connection
# publishes on tf (pinocchio, `ArticulatedTf`, every joint at zero): 1.62 m up,
# looking forward and 20 degrees down. The stereo module has no tf of its own,
# so its height cutoff is expressed against this fixed pose; with the torso
# moved the cutoff is off by however far the head moved, which for the torso's
# range is centimetres. `test_stereo.py` checks these against the URDF.
HEAD_CAMERA_IN_BASE_XYZ_M = (-0.0145, 0.0589, 1.624)
HEAD_CAMERA_IN_BASE_RPY_RAD = (-1.9202, 0.0, -1.5708)

# 6 m is roughly where stereo range error exceeds a voxel: at the R1's 120 mm
# baseline and fx 1012.6, one pixel of disparity is already ~0.24 m of depth.
MAX_RANGE_M = 6.0


def r1pro_stereo_cloud(
    calibration: R1StereoCalibration | None = None,
    *,
    min_height_m: float | None = None,
    max_height_m: float | None = None,
    denoise: str | None = None,
) -> Blueprint:
    """The head matched at the rate and resolution the Orin can sustain.

    ``min_height_m``/``max_height_m`` gate the CLOUD (never the depth image) by
    height above ``base_link``. A navigation run that also has the chassis
    lidar wants only the band the lidar cannot see -- the floor and what stands
    on it below the lidar's plane -- and passes a low ``max_height_m``.

    ``denoise`` overrides the module's default chain; ``"none"`` turns it off.
    """
    if calibration is None:
        from dimos.robot.galaxea.r1pro.config import stereo_calibration

        calibration = stereo_calibration()

    options: dict[str, object] = {}
    if min_height_m is not None:
        options["min_height_m"] = min_height_m
    if max_height_m is not None:
        options["max_height_m"] = max_height_m
    if denoise is not None:
        options["denoise"] = denoise

    return StereoCloud.blueprint(
        **stereo_cloud_kwargs(calibration),
        max_range_m=MAX_RANGE_M,
        # 1920x1536 per eye at 30 Hz is far more than the Orin can match while
        # it runs the vendor stack too. At downscale 8 a rectified pixel is
        # 3.8 cm at 6 m, and measured on the robot's recordings the depth
        # quality is FLAT across downscales -- halving the resolution averages
        # disparity noise away -- so 8 is both the fastest and the best.
        downscale=8,
        # The floor past 1 m never exceeds 32 px of disparity at this scale,
        # and the sweep reads the same quality at 96, 48 and 32.
        disparity_range=32,
        # `decimation` is a 2D pixel stride: N keeps 1 pixel in N squared. At
        # downscale 8 a stride of 2 samples the world every 7.6 cm at 6 m --
        # one point per 8 cm voxel, which is what the map can actually use.
        decimation=2,
        frame_id=HEAD_CAMERA_FRAME,
        base_from_camera_xyz_m=HEAD_CAMERA_IN_BASE_XYZ_M,
        base_from_camera_rpy_rad=HEAD_CAMERA_IN_BASE_RPY_RAD,
        **options,
    ).remappings(
        [
            (StereoCloud, "left", "head_left_color"),
            (StereoCloud, "right", "head_right_color"),
            (StereoCloud, "left_info", "head_left_info"),
            (StereoCloud, "right_info", "head_right_info"),
            (StereoCloud, "cloud", "head_cloud"),
            (StereoCloud, "depth", "head_depth"),
        ]
    )
