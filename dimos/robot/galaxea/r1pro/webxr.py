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

"""CC's WebXR inputs on the shared R1 hardware teleoperation stack."""

import json
from typing import Any

from dimos.control.motion_limits import MotionLimitsSpec
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.std_msgs.String import String
from dimos.teleop.joystick import JoystickTeleopMapper
from dimos.teleop.webxr.extensions import ArmTeleopModule, push_jpeg_bytes


class R1ProWebXR(ArmTeleopModule):
    """Headset control plus the same compressed head RGB used by Desktop/recording."""

    color_image: In[CompressedImage]

    async def handle_color_image(self, image: CompressedImage) -> None:
        push_jpeg_bytes(self, bytes(image.data))


class R1ProWebXRJoystick(JoystickTeleopMapper):
    """Scale normalized sticks by the coordinator's current physical speed settings."""

    _motion_limits: MotionLimitsSpec
    motion_settings: In[String]

    def _apply_motion_settings(self, settings: dict[str, dict[str, Any]]) -> None:
        with self._lock:
            self.config.base_linear_speed = settings["base_speed"]["rate"]
            self.config.base_angular_speed = settings["yaw_speed"]["rate"]

    @rpc
    def start(self) -> None:
        self._apply_motion_settings(self._motion_limits.get_motion_limits())
        super().start()

    async def handle_motion_settings(self, sample: String) -> None:
        self._apply_motion_settings(json.loads(sample.data))
