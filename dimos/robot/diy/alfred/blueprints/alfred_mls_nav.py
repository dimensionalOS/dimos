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

"""Alfred running MLS planning off the D455 alone, with no Mid-360.

    dimos run alfred-mls-nav

The lidar nav stack this replaces needed the Mid-360 twice over: FastLIO2 for odometry
and registered scans for the costmap. This drops it entirely. The whole processing
stack — ``DimSlam``, ``RayTracingVoxelMap``, MLS planning, Dan's local planner and
controller — lives in ``vis_nav``, shared verbatim with ``alfred-replay``; this
blueprint only adds the live drivers: the D455 and the base.

The mapper is ``RayTracingVoxelMap`` rather than ``VoxelGridMapper`` on purpose. The
latter carves whole (X, Y) columns on every insert, which is last-write-wins: measured
against a lidar reference on drive_2026-08-18_23-05-04.db it kept 26% as many voxels
as ray tracing and ate the walls MLS needs to plan around (IoU 0.076 vs 0.107, recall
0.200 vs 0.292).
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.diy.alfred.blueprints.vis_nav import vis_nav
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel

D455_MOUNT = Transform(
    translation=Vector3(-0.2518, -0.2736, 0.42),
    rotation=Quaternion(0.078360, 0.006348, -0.996712, 0.019616),
)
"""Where the D455 sits on the body, as base_link -> camera_link.

``RealSenseCameraConfig.base_transform`` defaults to identity, which would drop the
depth cloud at the body origin and leave MLS planning against a floor half a metre off.
It has to be set here: the camera module publishes this edge itself and never reads
``alfred.urdf``, so correcting the URDF would not reach this blueprint.

Fit against drive_2026-08-18_23-05-04.db, the first recording taken after the camera was
re-angled. Scored against the lidar map it lands the floor at z=-0.40, matching it
exactly, where the pre-remount angle misses it by 2.2 m.
"""

alfred_mls_nav = (
    autoconnect(
        RealSenseCamera.blueprint(
            # Twice the config default; the tracker wants the frame rate.
            fps=30,
            # cuVSLAM tracks the IR pair; they arrive rectified and are not perturbed by
            # the colour sensor's auto-exposure.
            enable_infrared=True,
            # The projector's dot pattern moves with the camera, so feature trackers latch
            # onto it and bias motion toward zero.
            emitter_enabled=False,
            enable_depth=True,
            # Colour itself is unconsumed, but depth arrives aligned to the colour frame
            # and the tracker's depth path is validated that way.
            enable_color=True,
            # Goes to the fusion filter, not to cuVSLAM: see the DimSlam note.
            enable_imu=True,
            base_transform=D455_MOUNT,
        ),
        AlfredHighLevel.blueprint(),
        vis_nav,
    )
    .remappings(
        [
            # Both imagers onto the one stream; the tracker tells them apart by frame_id.
            (RealSenseCamera, "infrared_left", "image"),
            (RealSenseCamera, "infrared_right", "image"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
            (RealSenseCamera, "infrared_right_camera_info", "camera_info"),
            # Keep the colour info off the stream cuVSLAM reads its rig from.
            (RealSenseCamera, "camera_info", "color_camera_info"),
            # Both odometry sources onto the one stream; the filter tells them apart by
            # frame_id, the same way the tracker tells the two imagers apart.
            (AlfredHighLevel, "wheel_odometry", "source_odometry"),
        ]
    )
    .global_config(n_workers=10, robot_model="alfred", obstacle_avoidance=False)
)
