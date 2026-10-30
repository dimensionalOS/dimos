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

"""``px4-a8`` with target following (FOLLOW, YAW_TRACK).

Detection2DModule runs YOLO with track ids on the A8's frames; PerceptionBridge turns the
operator's selected track into the connection's target. Same composition as
``unitree_go2_detection``. YOLO needs the ``perception`` extra (ultralytics, torch), which
the aircraft does not install for plain flight.

Usage:
    dimos run px4-a8-follow
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.gimbal.siyi.gimbal import a8_camera_info
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.robot.px4.blueprints.basic.px4 import zenoh_transport
from dimos.robot.px4.blueprints.basic.px4_a8 import px4_a8
from dimos.robot.px4.perception_bridge import PerceptionBridge, followable

# The detections carry no image size, so the intrinsics must be those of the frames the
# detector sees: the A8's 1280x720 main stream. If the stream is resized, FOLLOW aims at the
# wrong ground point.
_A8 = a8_camera_info()

px4_a8_follow = autoconnect(
    px4_a8,
    Detection2DModule.blueprint(
        camera_info=_A8, filter=followable, publish_detection_images=False, max_freq=10.0
    ),
    # The camera's drop below PX4's altitude reference (0.10 m for an A8 under an X500);
    # measure it on your airframe: --perceptionbridge.estimator '{"camera_below_ref_m": ...}'.
    PerceptionBridge.blueprint(camera_info=_A8, estimator={"camera_below_ref_m": 0.10}),
).transports(
    {
        ("detections", Detection2DArray): zenoh_transport(
            "detections", Detection2DArray, latest_wins=True
        ),
    }
)
