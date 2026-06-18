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

"""unitree-go2-twitch — Twitch Plays Go2.

Usage::

    export DIMOS_TWITCH_TOKEN=oauth:your_token
    export DIMOS_CHANNEL_NAME=your_channel
    dimos run unitree-go2-twitch --robot-ip 192.168.123.161
"""

from __future__ import annotations

import threading
import time

from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.connection_spec import GO2ConnectionSpec
from dimos.stream.twitch.votes import TwitchChoice, TwitchVotes
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DRIVE_SPEED = 0.3  # m/s
TURN_SPEED = 0.5  # rad/s
COMMAND_DURATION = 1.0  # seconds


class _ChoiceToCmdVel(Module):
    config: ModuleConfig

    chat_vote_choice: In[TwitchChoice]
    cmd_vel: Out[Twist]

    _connection: GO2ConnectionSpec

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._is_sitting = False
        self._exec_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self.chat_vote_choice.subscribe(self._on_choice)

    def _on_choice(self, choice: TwitchChoice) -> None:
        # Run command execution on a separate thread to avoid blocking the callback
        threading.Thread(
            target=self._execute_choice,
            args=(choice,),
            daemon=True,
            name="twitch-exec",
        ).start()

    def _execute_choice(self, choice: TwitchChoice) -> None:
        with self._exec_lock:
            logger.info("[TwitchPlays] Executing: %s", choice.winner)

            if choice.winner == "stop":
                self.cmd_vel.publish(Twist())
                return
            elif choice.winner == "sit":
                self._do_sport_command("Sit")
                self._is_sitting = True
                return
            elif choice.winner == "stand":
                self._do_sport_command("StandUp")
                self._is_sitting = False
                return

            # Auto-stand before any movement command
            if self._is_sitting:
                logger.info("[TwitchPlays] Auto-standing before movement")
                self._do_sport_command("StandUp")
                self._is_sitting = False
                time.sleep(1.0)

            t = Twist()
            if choice.winner == "forward":
                t.linear.x = DRIVE_SPEED
            elif choice.winner == "back":
                t.linear.x = -DRIVE_SPEED
            elif choice.winner == "left":
                t.angular.z = TURN_SPEED
            elif choice.winner == "right":
                t.angular.z = -TURN_SPEED

            end = time.time() + COMMAND_DURATION
            while time.time() < end:
                self.cmd_vel.publish(t)
                time.sleep(0.1)

            self.cmd_vel.publish(Twist())

    def _do_sport_command(self, command_name: str) -> None:
        api_id = SPORT_CMD[command_name]
        logger.info("[TwitchPlays] Sport command: %s (api_id=%d)", command_name, api_id)
        self._connection.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": api_id})


unitree_go2_twitch = autoconnect(
    GO2Connection.blueprint(),
    TwitchVotes.blueprint(
        vote_window_seconds=5.0,
        vote_mode="plurality",
    ),
    _ChoiceToCmdVel.blueprint(),
).global_config(n_workers=4, robot_model="unitree_go2")

__all__ = ["unitree_go2_twitch"]
