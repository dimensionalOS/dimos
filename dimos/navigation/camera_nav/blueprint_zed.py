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

"""Camera-nav blueprint for ZED Mini stereo camera (requires ZED SDK / pyzed.sl).

Kept in a separate file so importing ``blueprint_flowbase`` does not pull in
``pyzed`` on machines where the SDK is not installed.

``camera_nav_zed_teleop``
    ZED Mini stereo camera.  Uses hardware stereo depth (much better quality
    than DepthAnythingV2) and the ZED's built-in Visual Inertial Odometry for
    pose — no wheel odometry needed.  ``MonocularDepthModule`` and
    ``FlowBaseOdomModule`` are both removed from this path.

Port autoconnects:
    ZEDCamera.color_image    →  HardwareDepthModule.color_image
    ZEDCamera.depth_image    →  HardwareDepthModule.depth_image
    ZEDCamera.camera_info    →  HardwareDepthModule.camera_info
    HardwareDepthModule.frame_cloud  →  DepthAccumulatorModule.frame_cloud
    HardwareDepthModule.frame_cloud  →  CameraNavRecorder.frame_cloud

TF (ZEDCamera publishes all of these from its VIO + extrinsics):
    world ← base_link           (from ZED positional tracking)
    base_link ← camera_link     (from base_transform config)
    camera_link ← camera_color_optical_frame

Post-traversal correction::

    python -m dimos.navigation.camera_nav.correct_map traversal.db --rerun
"""

from __future__ import annotations

from dimos.control.blueprints.mobile import coordinator_flowbase_keyboard_teleop
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.zed.camera import ZEDCamera
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.camera_nav.recorder import CameraNavRecorder
from dimos.perception.depth.accumulator import DepthAccumulatorModule
from dimos.perception.depth.hardware_depth_module import HardwareDepthModule
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _cloud_points(cloud):
    return cloud.to_rerun(mode="points")


# ---------------------------------------------------------------------------
# Camera mount — tune these two values to match your physical ZED Mini mount.
#
# Coordinate convention for base_link: x=forward, y=left, z=up (ROS standard).
# Measure from the center of the FlowBase platform to the ZED Mini lens.
#
#   translation: (forward_m, lateral_m, height_m)
#   rotation:    from_euler(Vector3(roll, pitch, yaw))
#                pitch is negative to tilt the camera downward
#                e.g. Vector3(0, -0.26, 0)  ≈ 15° downward tilt
# ---------------------------------------------------------------------------
_ZED_MOUNT = Transform(
    translation=Vector3(0.0, 0.0, 0.5),          # adjust to your mount height
    rotation=Quaternion.from_euler(Vector3(0.0, -0.26, 0.0)),  # 15° downward tilt
    frame_id="base_link",
    child_frame_id="camera_link",
)

_RERUN_VIZ = RerunBridgeModule.blueprint(
    pubsubs=[LCM()],
    rerun_open="web",
    visual_override={
        "global_map": _cloud_points,
        "frame_cloud": _cloud_points,
    },
)

camera_nav_zed_teleop = autoconnect(
    coordinator_flowbase_keyboard_teleop,
    # ZED handles depth + VIO — no FlowBaseOdomModule or MonocularDepthModule needed.
    ZEDCamera.blueprint(
        base_transform=_ZED_MOUNT,
        enable_depth=True,
        enable_pointcloud=False,   # we backproject ourselves via HardwareDepthModule
        enable_tracking=True,      # ZED VIO → publishes world ← base_link TF
        enable_imu_fusion=True,
        set_floor_as_origin=True,
    ),
    HardwareDepthModule.blueprint(
        camera_frame="camera_color_optical_frame",  # ZED's left-camera optical frame name
    ),
    DepthAccumulatorModule.blueprint(),
    CameraNavRecorder.blueprint(db_path="traversal.db"),
    _RERUN_VIZ,
)
