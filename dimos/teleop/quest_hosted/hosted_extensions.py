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

"""Hosted teleop subclasses (WebRTC-via-Cloudflare-Realtime transport).

Mirrors the role of ``dimos/teleop/quest/quest_extensions.py`` but for the
hosted module — small overrides on top of ``HostedTeleopModule``:

  - ``HostedArmTeleopModule``: per-hand task_name routing + analog trigger
    packing (Quest VR mode, arm robots).
  - ``HostedTwistTeleopModule``: scales incoming Twist by configured
    linear/angular speeds (keyboard mode, mobile-base robots like Go2).
"""

from typing import Any

from pydantic import Field

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.quest.quest_teleop_module import Hand
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.teleop.quest_hosted.hosted_teleop_module import (
    HostedTeleopConfig,
    HostedTeleopModule,
)


class HostedArmTeleopConfig(HostedTeleopConfig):
    """Adds ``task_names`` for routing per-hand commands to coordinator tasks.

    ``task_names`` maps lower-case hand names (``"left"``, ``"right"``) to
    the coordinator task name (e.g. ``"teleop_xarm"``). Used to set
    ``frame_id`` on the published ``PoseStamped`` so the coordinator routes
    to the correct ``TeleopIKTask``.
    """

    task_names: dict[str, str] = Field(default_factory=dict)


class HostedArmTeleopModule(HostedTeleopModule):
    """Hosted teleop with per-hand task_name routing + analog trigger packing.

    Same overrides as ``ArmTeleopModule`` but on top of the WebRTC-via-broker
    ``HostedTeleopModule`` instead of the local-WebSocket ``QuestTeleopModule``.
    """

    config: HostedArmTeleopConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._task_names: dict[Hand, str] = {
            Hand[k.upper()]: v for k, v in self.config.task_names.items()
        }

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Stamp ``frame_id`` with the configured task name, then publish."""
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
        """Publish ``Buttons`` with analog triggers packed into bits 16-29."""
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        self.buttons.publish(buttons)


class HostedTwistTeleopConfig(HostedTeleopConfig):
    """Adds ``linear_speed`` / ``angular_speed`` for scaling incoming Twist.

    The operator's keyboard sends normalized commands in [-1, 1] (with
    Shift = 2x, Ctrl = 0.5x). The robot side multiplies by these speeds
    to get final m/s and rad/s. Defaults are reasonable for an indoor Go2.
    """

    linear_speed: float = 0.5
    angular_speed: float = 0.8


class HostedTwistTeleopModule(HostedTeleopModule):
    """Hosted teleop variant for mobile-base robots (Go2, wheeled, etc.).

    Same as ``HostedTeleopModule`` but scales incoming Twist commands by
    the configured ``linear_speed`` / ``angular_speed`` before publishing.
    """

    config: HostedTwistTeleopConfig

    def _on_twist_bytes(self, data: bytes) -> None:
        msg = TwistStamped.lcm_decode(data)
        ls = self.config.linear_speed
        as_ = self.config.angular_speed
        linear = Vector3(msg.linear.x * ls, msg.linear.y * ls, msg.linear.z * ls)
        angular = Vector3(msg.angular.x * as_, msg.angular.y * as_, msg.angular.z * as_)
        self.cmd_vel.publish(Twist(linear=linear, angular=angular))
        self.cmd_vel_stamped.publish(
            TwistStamped(
                ts=msg.ts, frame_id=msg.frame_id, seq=msg.seq, linear=linear, angular=angular
            )
        )
