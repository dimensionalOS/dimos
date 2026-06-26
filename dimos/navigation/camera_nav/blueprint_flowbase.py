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

"""Camera-nav perception stack wired onto the FlowBase keyboard-teleop blueprint.

Two variants (ZED Mini variant lives in ``blueprint_zed.py`` to keep the ZED
SDK import lazy — importing this file does not require ``pyzed``):

``camera_nav_static_trial``
    No robot required.  Uses the host laptop webcam (index 0) and
    DepthAnythingV2.  Good for verifying the depth → point-cloud →
    accumulation → Rerun pipeline on a dev machine before touching hardware.
    Because there is no odometry the cloud accumulates in camera frame (the
    robot isn't moving, so this is fine for a visual sanity check).

``camera_nav_flowbase_teleop``
    USB/generic webcam.  Uses DepthAnythingV2 for monocular depth estimation.
    Needs ``FlowBaseOdomModule`` for odometry TF.

Post-traversal correction::

    python -m dimos.navigation.camera_nav.correct_map traversal.db --rerun
"""

from __future__ import annotations

from dimos.control.blueprints.mobile import coordinator_flowbase_keyboard_teleop
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.drive_trains.flowbase.odom_tf import FlowBaseOdomModule
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.camera_nav.recorder import CameraNavRecorder
from dimos.perception.depth.accumulator import DepthAccumulatorModule
from dimos.perception.depth.monocular_depth_module import MonocularDepthModule
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _cloud_points(cloud):
    return cloud.to_rerun(mode="points")


def _depth_img(img):
    return img.to_rerun()


# ---------------------------------------------------------------------------
# Static trial — laptop webcam only, no robot required.
#
# Uses camera 0 (the first webcam the OS enumerates).  MonocularDepthModule
# falls back to camera frame when world ← camera TF is unavailable, so the
# point cloud builds up in camera space — good enough to verify the pipeline.
# ---------------------------------------------------------------------------
camera_nav_static_trial = autoconnect(
    CameraModule.blueprint(),
    MonocularDepthModule.blueprint(),
    DepthAccumulatorModule.blueprint(),
    RerunBridgeModule.blueprint(
        pubsubs=[LCM()],
        rerun_open="web",
        visual_override={
            "global_map": _cloud_points,
            "frame_cloud": _cloud_points,
            "depth_image": _depth_img,
        },
    ),
)

# ---------------------------------------------------------------------------
# Camera mount — tune these two values to match your physical setup.
#
# Coordinate convention for base_link: x=forward, y=left, z=up (ROS standard).
# Measure from the center of the FlowBase platform to the camera lens.
#
#   translation: (forward_m, lateral_m, height_m)
#   rotation:    from_euler(Vector3(roll, pitch, yaw))
#                pitch is negative to tilt the camera downward
#                e.g. Vector3(0, -0.26, 0)  ≈ 15° downward tilt
#
# The camera_optical frame (z-forward, x-right, y-down) is added automatically
# by CameraModule via a fixed -90°/+90° rotation on top of camera_link.
# ---------------------------------------------------------------------------
_CAMERA_MOUNT = Transform(
    translation=Vector3(0.0, 0.0, 0.5),         # 0.5 m above base center — adjust to your mount
    rotation=Quaternion.from_euler(Vector3(0.0, -0.26, 0.0)),  # 15° downward tilt
    frame_id="base_link",
    child_frame_id="camera_link",
)

# FlowBase keyboard teleop + camera perception + map recording in one blueprint.
#
# Port autoconnects (all by matching name + type):
#   ControlCoordinator.coordinator_joint_state  →  FlowBaseOdomModule.coordinator_joint_state
#   CameraModule.color_image                    →  MonocularDepthModule.color_image
#   CameraModule.camera_info                    →  MonocularDepthModule.camera_info
#   MonocularDepthModule.frame_cloud            →  DepthAccumulatorModule.frame_cloud
#   MonocularDepthModule.frame_cloud            →  CameraNavRecorder.frame_cloud
#
# TF (published at runtime, consumed by MonocularDepthModule and CameraNavRecorder):
#   FlowBaseOdomModule  →  world ← base_link
#   CameraModule        →  base_link ← camera_link ← camera_optical
camera_nav_flowbase_teleop = autoconnect(
    coordinator_flowbase_keyboard_teleop,
    FlowBaseOdomModule.blueprint(),
    CameraModule.blueprint(transform=_CAMERA_MOUNT),
    MonocularDepthModule.blueprint(),
    DepthAccumulatorModule.blueprint(),
    CameraNavRecorder.blueprint(db_path="traversal.db"),
    # Rerun opens automatically in the browser when the blueprint starts.
    # Shows /global_map (growing point cloud), /depth_image, /color_image live.
    RerunBridgeModule.blueprint(
        pubsubs=[LCM()],
        rerun_open="web",
        visual_override={
            "global_map": _cloud_points,
            "frame_cloud": _cloud_points,
        },
    ),
)

