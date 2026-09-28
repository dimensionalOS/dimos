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

"""The PX4 blueprints with detection and target following (FOLLOW, YAW_TRACK).

``px4-follow``
    ``px4-drone`` plus Detection2DModule (YOLO with track ids) on the A8's frames and the
    PerceptionBridge that turns the operator's selected track into the connection's target.
``px4-sitl-follow``
    The simulator twin: BrightBlobDetector boxes the synthetic clip's square, so it needs
    no model and no GPU. ``tool_follow_gate.py`` runs this blueprint end to end.

Apart from ``blueprints.py`` because YOLO needs the ``perception`` extra (ultralytics,
torch), which the aircraft does not install for plain flight. Same composition as
``unitree_go2_detection``.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.gimbal.siyi.gimbal import a8_camera_info
from dimos.perception.detection.module2D import Detection2DModule
from dimos.robot.px4.blueprints import px4_drone, px4_sitl
from dimos.robot.px4.perception_bridge import PerceptionBridge, followable
from dimos.robot.px4.sitl import BrightBlobDetector

# The detections carry no image size, so the intrinsics must be those of the frames the
# detector sees: the A8's 1280x720 main stream (if the stream is resized, FOLLOW aims at the
# wrong ground point) and the 320x180 synthetic clip in the twin.
_A8 = a8_camera_info()
_CLIP = a8_camera_info(320, 180)

px4_follow = autoconnect(
    px4_drone,
    Detection2DModule.blueprint(
        camera_info=_A8, filter=followable, publish_detection_images=False, max_freq=10.0
    ),
    # On this airframe the camera sits 0.10 m below the altitude reference.
    PerceptionBridge.blueprint(camera_info=_A8, estimator={"camera_below_ref_m": 0.10}),
)

# The aircraft on the ground gives no altitude, so the estimator uses a fixed 10 m AGL.
px4_sitl_follow = autoconnect(
    px4_sitl,
    Detection2DModule.blueprint(
        detector=BrightBlobDetector,
        camera_info=_CLIP,
        publish_detection_images=False,
        max_freq=25.0,
    ),
    PerceptionBridge.blueprint(
        camera_info=_CLIP, estimator={"agl_source": "fixed", "fixed_agl_m": 10.0}
    ),
)
