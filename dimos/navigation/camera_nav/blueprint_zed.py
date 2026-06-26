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

Kept in a separate file so importing blueprint_flowbase does not pull in pyzed.
ZED VIO provides world←base_link TF directly, so FlowBaseOdomModule is not needed.
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


# Tune translation and pitch to match physical ZED Mini mount.
_ZED_MOUNT = Transform(
    translation=Vector3(0.0, 0.0, 0.5),
    rotation=Quaternion.from_euler(Vector3(0.0, -0.26, 0.0)),  # ≈15° downward tilt
    frame_id="base_link",
    child_frame_id="camera_link",
)

_RERUN_VIZ = RerunBridgeModule.blueprint(
    pubsubs=[LCM()],
    rerun_open="web",
    visual_override={
        "world/global_map": _cloud_points,
        "world/frame_cloud": _cloud_points,
    },
)

camera_nav_zed_teleop = autoconnect(
    coordinator_flowbase_keyboard_teleop,
    ZEDCamera.blueprint(
        base_transform=_ZED_MOUNT,
        enable_depth=True,
        enable_pointcloud=False,
        enable_tracking=True,
        enable_imu_fusion=True,
        set_floor_as_origin=True,
    ),
    HardwareDepthModule.blueprint(
        camera_frame="camera_color_optical_frame",
    ),
    DepthAccumulatorModule.blueprint(),
    CameraNavRecorder.blueprint(db_path="traversal.db"),
    _RERUN_VIZ,
)
