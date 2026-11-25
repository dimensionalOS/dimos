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

"""Local Quest producer for the tri-mode reference task."""

import math
import time
from typing import Any

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.manipulators.openyam.trimode_tasks import QUEST_FRAME, QUEST_FRESH, QUEST_TIMEOUT
from dimos.teleop.webxr.controller_types import Buttons, Hand, WebXRControllerState
from dimos.teleop.webxr.extensions import ArmTeleopModule


class TrimodeQuest(ArmTeleopModule):
    """Treat a gap in either raw input as a loss of the complete right-hand pair."""

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault("input_timeout_s", QUEST_TIMEOUT)
        super().__init__(**kwargs)
        if self.config.input_timeout_s > QUEST_TIMEOUT:
            raise ValueError("tri-mode Quest input timeout cannot exceed 250 ms")
        self._published_pair: tuple[float, float] | None = None
        self._need_release = True

    def _right_pair(self, now: float) -> tuple[float, float] | None:
        pose = self._last_pose_update[Hand.RIGHT]
        joy = self._last_controller_update[Hand.RIGHT]
        if pose is None or joy is None or now >= min(pose, joy) + self.config.input_timeout_s:
            return None
        return pose, joy

    def _invalidate_right_pair(self) -> None:
        self._last_pose_update[Hand.RIGHT] = self._last_controller_update[Hand.RIGHT] = None
        self._current_poses[Hand.RIGHT] = self._controllers[Hand.RIGHT] = None
        if self._published_pair is not None:
            self._publish_button_state(self._controllers[Hand.LEFT], None)

    def _expire_stale_state(self, now: float) -> None:
        # Run before ingest as well as in the loop, so two updates between
        # iterations cannot erase evidence of a gap. Neither cached half can
        # serve as the new release across a continuity break.
        stamps = (self._last_pose_update[Hand.RIGHT], self._last_controller_update[Hand.RIGHT])
        if any(
            stamp is not None and now >= stamp + self.config.input_timeout_s for stamp in stamps
        ):
            self._invalidate_right_pair()
        super()._expire_stale_state(now)

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._expire_stale_state(time.monotonic())
            super()._on_pose(msg)

    def _on_joy(self, msg: Joy) -> bool:
        with self._lock:
            self._expire_stale_state(time.monotonic())
            valid = super()._on_joy(msg)
            if msg.frame_id == "right":
                right = self._controllers[Hand.RIGHT]
                if right is None or not all(
                    math.isfinite(v) and 0.0 <= v <= 1.0 for v in (right.grip, right.trigger)
                ):
                    self._invalidate_right_pair()
                    return False
            return valid

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        if hand == Hand.RIGHT:
            pair = self._right_pair(time.monotonic())
            if pair is None or self._need_release:
                return
            # Private same-host stream: ts is the raw ingress deadline, not a
            # browser stamp or this loop iteration's publication time.
            output_msg = PoseStamped(
                ts=min(pair) + self.config.input_timeout_s,
                frame_id=QUEST_FRAME,
                position=output_msg.position,
                orientation=output_msg.orientation,
            )
        super()._publish_msg(hand, output_msg)

    def _publish_button_state(
        self, left: WebXRControllerState | None, right: WebXRControllerState | None
    ) -> None:
        pair = self._right_pair(time.monotonic()) if right is not None else None
        live_right = right if pair is not None else None
        buttons = Buttons.from_controllers(left, live_right)
        buttons.pack_analog_triggers(
            left=left.trigger if left else 0.0, right=live_right.trigger if live_right else 0.0
        )
        previous = self._published_pair
        if pair is None:
            self._need_release = True
            self._published_pair = None
            self._publish_buttons(buttons)
        elif previous is None or (pair[0] > previous[0] and pair[1] > previous[1]):
            self._published_pair = pair
            if not buttons.right_grip:
                self._need_release = False
            if not self._need_release:
                buttons.data |= QUEST_FRESH
            self._publish_buttons(buttons)
        self._publish_gripper_commands(left, live_right, buttons)
        self._log_engagement(buttons)
