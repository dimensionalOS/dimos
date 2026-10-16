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

The camera replays a generated clip and FakeA8 answers as the gimbal. BrightBlobDetector
boxes the clip's square, so it needs no model and no GPU.

Usage:
    dimos run px4-sitl-follow
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.gimbal.siyi.gimbal import a8_camera_info
from dimos.hardware.sensors.camera.rtsp.camera import SYNTHETIC_URL
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.robot.px4.blueprints.basic.px4 import zenoh_transport
from dimos.robot.px4.blueprints.basic.px4_a8 import a8_payload
from dimos.robot.px4.blueprints.basic.px4_sitl import px4_sitl
from dimos.robot.px4.perception_bridge import PerceptionBridge
from dimos.robot.px4.sitl import BrightBlobDetector, FakeA8

# The intrinsics of the 320x180 clip the detector sees (detections carry no image size).
_CLIP = a8_camera_info(320, 180)

# The fake A8 starts 20 deg down and 30 deg right so the synthetic target (parked at the
# image centre) has a non-trivial line of sight. The camera keeps the A8's frame_id and
# capture latency, so frames are stamped as on the aircraft; the gimbal gets a nominal mount.
# AGL is fixed at 10 m: on the ground rel_alt is 0, which the estimator refuses.
px4_sitl_follow = autoconnect(
    px4_sitl,
    a8_payload(
        camera={"url": SYNTHETIC_URL, "color_hz": 25.0},
        gimbal={"mount_xyz": (0.0, 0.0, -0.08), "aim_enabled": True, "image_size": (320, 180)},
    ),
    FakeA8.blueprint(initial_pitch_deg=-20.0, initial_yaw_deg=30.0),
    Detection2DModule.blueprint(
        detector=BrightBlobDetector,
        camera_info=_CLIP,
        publish_detection_images=False,
        max_freq=25.0,
    ),
    PerceptionBridge.blueprint(
        camera_info=_CLIP, estimator={"agl_source": "fixed", "fixed_agl_m": 10.0}
    ),
).transports(
    {
        ("detections", Detection2DArray): zenoh_transport(
            "detections", Detection2DArray, latest_wins=True
        ),
    }
)
