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

"""``px4-sitl`` with target following, and what the simulator lacks.

FakeA8Camera and FakeA8 stand in for the A8's camera and gimbal. BrightBlobDetector boxes
the camera's square, so it needs no model and no GPU.

Usage:
    dimos run px4-sitl-follow
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal, a8_camera_info
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.robot.px4.blueprints.basic.px4 import zenoh_transport
from dimos.robot.px4.blueprints.basic.px4_sitl import px4_sitl
from dimos.robot.px4.perception_bridge import PerceptionBridge
from dimos.robot.px4.sitl import BrightBlobDetector, FakeA8, FakeA8Camera

_SIZE = (320, 180)
# The intrinsics of the frames the detector sees (detections carry no image size).
_CAMERA = a8_camera_info(*_SIZE)

# The fake A8 starts 20 deg down and 30 deg right so the target (the middle of the image)
# has a non-trivial line of sight; the gimbal gets a nominal mount. AGL is fixed at 10 m: on
# the ground rel_alt is 0, which the estimator refuses.
px4_sitl_follow = autoconnect(
    px4_sitl,
    FakeA8Camera.blueprint(image_size=_SIZE),
    FakeA8.blueprint(initial_pitch_deg=-20.0, initial_yaw_deg=30.0),
    SiyiA8Gimbal.blueprint(mount_xyz=(0.0, 0.0, -0.08), aim_enabled=True, image_size=_SIZE),
    Detection2DModule.blueprint(
        detector=BrightBlobDetector,
        camera_info=_CAMERA,
        publish_detection_images=False,
        max_freq=10.0,
    ),
    PerceptionBridge.blueprint(
        camera_info=_CAMERA, estimator={"agl_source": "fixed", "fixed_agl_m": 10.0}
    ),
).transports(
    {
        ("camera_info", CameraInfo): zenoh_transport("camera_info", CameraInfo),
        ("detections", Detection2DArray): zenoh_transport(
            "detections", Detection2DArray, latest_wins=True
        ),
    }
)
