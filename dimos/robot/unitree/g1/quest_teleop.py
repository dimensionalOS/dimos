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

"""G1-specific Quest teleoperation, including bounded GR00T locomotion."""

from __future__ import annotations

import math
import time
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.teleop.quest.quest_extensions import VideoArmTeleopConfig, VideoArmTeleopModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState


class G1QuestTeleopConfig(VideoArmTeleopConfig):
    """Configuration for bounded G1 Quest joystick locomotion."""

    forward_speed_mps: float = Field(default=0.2, ge=0.0, allow_inf_nan=False)
    lateral_speed_mps: float = Field(default=0.2, ge=0.0, allow_inf_nan=False)
    yaw_speed_rad_s: float = Field(default=0.5, ge=0.0, allow_inf_nan=False)
    deadzone: float = Field(default=0.18, ge=0.0, lt=1.0, allow_inf_nan=False)
    invert_forward: bool = True
    invert_lateral: bool = True
    invert_yaw: bool = True


class G1QuestTeleopModule(VideoArmTeleopModule):
    """Quest arm/video teleop plus bounded planar velocity commands for G1.

    Locomotion starts locked. After every connection, disconnect, stale or
    malformed input, and stop, fresh samples from both controllers must show
    the used axes at neutral before stick commands can flow.
    """

    dedicated_worker = True
    config: G1QuestTeleopConfig

    cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._drive_ready = False
        self._drive_reset_at = time.monotonic()
        self._last_drive_command = Twist.zero()

    @staticmethod
    def _clamp_axis(value: float) -> float:
        return max(-1.0, min(1.0, value))

    def _apply_deadzone(self, value: float) -> float:
        clamped = self._clamp_axis(value)
        return 0.0 if self._axis_is_neutral(clamped) else clamped

    def _axis_is_neutral(self, value: float) -> bool:
        return abs(value) <= self.config.deadzone or math.isclose(
            abs(value), self.config.deadzone, rel_tol=1e-6
        )

    @staticmethod
    def _controller_is_finite(controller: QuestControllerState) -> bool:
        return all(
            math.isfinite(value)
            for value in (
                controller.thumbstick.x,
                controller.thumbstick.y,
                controller.trigger,
                controller.grip,
            )
        )

    def _publish_zero(self) -> None:
        self._last_drive_command = Twist.zero()
        self.cmd_vel.publish(self._last_drive_command)

    def _publish_safe_command(self) -> None:
        with self._lock:
            self._drive_ready = False
            self._drive_reset_at = time.monotonic()
            self._publish_zero()

    def _on_joy_bytes(self, data: bytes) -> bool:
        try:
            valid = super()._on_joy_bytes(data)
        except ValueError:
            self._publish_safe_command()
            raise
        if not valid:
            self._publish_safe_command()
            return False

        with self._lock:
            invalid_hands = [
                hand
                for hand, controller in self._controllers.items()
                if controller is not None and not self._controller_is_finite(controller)
            ]
            for hand in invalid_hands:
                self._controllers[hand] = None
                self._last_controller_update[hand] = None
                self._is_engaged[hand] = False
                self._initial_poses[hand] = None
            if invalid_hands:
                self._publish_safe_command()
                return False
        return True

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        super()._publish_button_state(left, right)

        if left is None or right is None:
            self._publish_zero()
            return

        left_x = left.thumbstick.x
        left_y = left.thumbstick.y
        right_x = right.thumbstick.x
        if not all(math.isfinite(value) for value in (left_x, left_y, right_x)):
            self._publish_safe_command()
            return
        left_x = self._clamp_axis(left_x)
        left_y = self._clamp_axis(left_y)
        right_x = self._clamp_axis(right_x)

        if not self._drive_ready:
            fresh_after_reset = all(
                updated_at is not None and updated_at > self._drive_reset_at
                for updated_at in self._last_controller_update.values()
            )
            axes_neutral = all(self._axis_is_neutral(value) for value in (left_x, left_y, right_x))
            if fresh_after_reset and axes_neutral:
                self._drive_ready = True
            self._publish_zero()
            return

        forward = self._apply_deadzone(left_y) * self.config.forward_speed_mps
        lateral = self._apply_deadzone(left_x) * self.config.lateral_speed_mps
        yaw = self._apply_deadzone(right_x) * self.config.yaw_speed_rad_s
        if self.config.invert_forward:
            forward = -forward
        if self.config.invert_lateral:
            lateral = -lateral
        if self.config.invert_yaw:
            yaw = -yaw
        self._last_drive_command = Twist(linear=[forward, lateral, 0.0], angular=[0.0, 0.0, yaw])
        self.cmd_vel.publish(self._last_drive_command)

    @rpc
    def state_snapshot(self) -> dict[str, Any]:
        """Return Quest drive-gate, raw input, age, and output telemetry."""
        with self._lock:
            now = time.monotonic()
            left = self._controllers[Hand.LEFT]
            right = self._controllers[Hand.RIGHT]

            def age(hand: Hand) -> float | None:
                updated_at = self._last_controller_update[hand]
                return None if updated_at is None else max(0.0, now - updated_at)

            return {
                "drive_ready": self._drive_ready,
                "left_stick_x": left.thumbstick.x if left is not None else None,
                "left_stick_y": left.thumbstick.y if left is not None else None,
                "right_stick_x": right.thumbstick.x if right is not None else None,
                "left_input_age_s": age(Hand.LEFT),
                "right_input_age_s": age(Hand.RIGHT),
                "command": [
                    self._last_drive_command.linear.x,
                    self._last_drive_command.linear.y,
                    self._last_drive_command.angular.z,
                ],
            }
