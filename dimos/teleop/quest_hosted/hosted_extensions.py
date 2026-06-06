#!/usr/bin/env python3
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

"""Hosted teleop subclasses: arm IK and mobile-base twist."""

import time
from typing import Any

from pydantic import Field

from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.teleop.quest_hosted.hosted_teleop_module import (
    Hand,
    HostedTeleopConfig,
    HostedTeleopModule,
)


class HostedArmTeleopConfig(HostedTeleopConfig):
    # task_names maps "left"/"right" → coordinator task name (e.g. "teleop_xarm"),
    # used as frame_id so the coordinator routes to the right TeleopIKTask.
    task_names: dict[str, str] = Field(default_factory=dict)


class HostedArmTeleopModule(HostedTeleopModule):
    """Arm-IK subclass: routes per-hand poses to coordinator tasks + analog triggers."""

    config: HostedArmTeleopConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._task_names: dict[Hand, str] = {
            Hand[k.upper()]: v for k, v in self.config.task_names.items()
        }

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        # Stamp frame_id with the per-hand task name so the coordinator routes.
        task_name = self._task_names.get(hand)
        if task_name:
            output_msg = PoseStamped(
                position=output_msg.position,
                orientation=output_msg.orientation,
                ts=output_msg.ts,
                frame_id=task_name,
            )
        super()._publish_msg(hand, output_msg)

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        # Same as base, plus analog triggers packed into Buttons bits 16-29.
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        self.buttons.publish(buttons)


class HostedTwistTeleopConfig(HostedTeleopConfig):
    # Operator sends normalized [-1, 1] (Shift=2x, Ctrl=0.5x); we scale here.
    linear_speed: float = 0.5
    angular_speed: float = 0.8


class HostedTwistTeleopModule(HostedTeleopModule):
    """Mobile-base subclass. Drives cmd_vel from keyboard TwistStamped or VR Joy."""

    config: HostedTwistTeleopConfig

    cmd_vel: Out[Twist]

    def _publish_twist(
        self, lx: float, ly: float, az: float, ts: float, frame_id: str, seq: int
    ) -> None:
        ls = self.config.linear_speed
        as_ = self.config.angular_speed
        linear = Vector3(lx * ls, ly * ls, 0.0)
        angular = Vector3(0.0, 0.0, az * as_)
        self.cmd_vel.publish(Twist(linear=linear, angular=angular))
        self.cmd_vel_stamped.publish(
            TwistStamped(ts=ts, frame_id=frame_id, seq=seq, linear=linear, angular=angular)
        )

    def _on_twist_bytes(self, data: bytes) -> None:
        # Keyboard/touch path: stamped ts + seq feed the HUD command-plane stats.
        msg = TwistStamped.lcm_decode(data)
        self._record_cmd_arrival(msg.ts, msg.seq)
        self._publish_twist(
            msg.linear.x, msg.linear.y, msg.angular.z, msg.ts, msg.frame_id, msg.seq
        )

    def _on_joy_bytes(self, data: bytes) -> None:
        # VR thumbsticks → base velocity. Left Y = fwd/back, left X = strafe,
        # right X = yaw. Stick conventions are opposite ROS, so negate.
        super()._on_joy_bytes(data)
        with self._lock:
            right = self._controllers.get(Hand.RIGHT)
            left = self._controllers.get(Hand.LEFT)
        fwd = -left.thumbstick.y if left is not None else 0.0
        strafe = -left.thumbstick.x if left is not None else 0.0
        yaw = -right.thumbstick.x if right is not None else 0.0
        self._publish_twist(fwd, strafe, yaw, time.time(), "vr", 0)
