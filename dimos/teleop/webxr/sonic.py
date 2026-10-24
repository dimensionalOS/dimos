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

"""SONIC stick commands alongside the shared enterprise WebXR body stream."""

import math
from typing import Any

from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.teleop.webxr.body_tracking import BodyTrackingMode
from dimos.teleop.webxr.controller_types import WebXRControllerState
from dimos.teleop.webxr.module import WebXRTeleopConfig, WebXRTeleopModule


class SonicWebXRConfig(WebXRTeleopConfig):
    body_tracking_mode: BodyTrackingMode = "required"
    input_timeout_s: float = 0.25


class SonicWebXRTeleopModule(WebXRTeleopModule):
    """Left stick translates, right stick turns; releasing sends one stop."""

    config: SonicWebXRConfig
    tele_cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._was_moving = False

    def _publish_button_state(
        self, left: WebXRControllerState | None, right: WebXRControllerState | None
    ) -> None:
        # Missing controllers must expire the policy's button lease, not look
        # like a continuous stream of freshly released buttons.
        if left is not None and right is not None:
            super()._publish_button_state(left, right)

        def axis(value: float) -> float:
            return max(-1.0, min(1.0, value)) if math.isfinite(value) and abs(value) > 0.15 else 0.0

        vx = -axis(left.thumbstick.y) * 0.5 if left else 0.0
        vy = -axis(left.thumbstick.x) * 0.25 if left else 0.0
        yaw = -axis(right.thumbstick.x) * 0.6 if right else 0.0
        switching = bool(left and right and left.primary and right.primary)
        if switching:
            vx = vy = yaw = 0.0
        moving = bool(vx or vy or yaw)
        if moving or self._was_moving or switching:
            self.tele_cmd_vel.publish(Twist(linear=(vx, vy, 0.0), angular=(0.0, 0.0, yaw)))
        self._was_moving = moving
