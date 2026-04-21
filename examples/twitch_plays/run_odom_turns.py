#!/usr/bin/env python3
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

"""Twitch Plays Go2 with odom-based turn feedback.

Chat votes on categories: ``forward`` / ``back`` / ``turn`` / ``jump`` / ``sit`` / ``stand``
(plurality over a 5-second window).

- forward/back → drive at 0.3 m/s for 1 second
- turn → parse ``turn [left|right] <degrees>`` from each turn vote, average the
  valid angles, floor to 5° increments, and execute using odom feedback

Turn parsing rules:

- ``turn 30``                → right 30  (positive default)
- ``turn -30``               → left 30   (negative = left when no direction)
- ``turn left``              → left 90   (no number → 90°)
- ``turn right 10 degrees``  → right 10
- 0° or >180° is **invalid** — the message still counts as a turn vote but
  contributes no angle
- if every turn message has an invalid angle → default 20° right
- ``left`` / ``right`` tolerate up to 3 edits (Levenshtein), so ``lft`` and
  ``rihgt`` work. ``turn`` / ``forward`` / ``back`` must be exact.

Usage::

    export DIMOS_TWITCH_TOKEN=oauth:your_token
    export DIMOS_CHANNEL_NAME=your_channel
    python examples/twitch_plays/run_odom_turns.py
"""

from __future__ import annotations

from collections import Counter
import math
import re
import threading
import time
from typing import Any

from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection_spec import GO2ConnectionSpec
from dimos.stream.twitch.module import TwitchChat, TwitchMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

VOTE_WINDOW_SECONDS = 1.0
LINEAR_SPEED = 0.5  # m/s
LINEAR_DURATION = 2.0  # seconds
ANGULAR_SPEED = 0.5  # rad/s
MIN_TURN_DEG = 5.0
MAX_TURN_DEG = 180.0
ANGLE_STEP = 5.0  # floor turn angles to this increment
NO_NUMBER_DEFAULT_DEG = 90.0  # "turn left" with no number
FALLBACK_TURN_DEG = 20.0  # used when all turn votes have invalid angles
YAW_MARGIN_DEG = 5.0
LEV_MAX_EDITS = 2
TURN_TIMEOUT_SECONDS = 30.0
CMD_VEL_PUBLISH_HZ = 20.0


# ── Text parsing ──


def _levenshtein(a: str, b: str) -> int:
    if a == b:
        return 0
    if not a or not b:
        return len(a) or len(b)
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        curr = [i]
        for j, cb in enumerate(b, 1):
            curr.append(min(curr[j - 1] + 1, prev[j] + 1, prev[j - 1] + (ca != cb)))
        prev = curr
    return prev[-1]


def _match_direction(word: str) -> str | None:
    """Return 'left'/'right' if word is within LEV_MAX_EDITS of either."""
    if _levenshtein(word, "left") <= LEV_MAX_EDITS:
        return "left"
    if _levenshtein(word, "right") <= LEV_MAX_EDITS:
        return "right"
    return None


def _categorize(content: str) -> str | None:
    """Return 'forward' | 'back' | 'turn' | 'jump' | 'sit' | 'stand' | None."""
    words = set(re.findall(r"[a-zA-Z]+", content.lower()))
    if "turn" in words:
        return "turn"
    if words & {"forward", "forwards", "fwd"}:
        return "forward"
    if words & {"back", "backward", "backwards"}:
        return "back"
    if words & {"jump", "hop", "leap"}:
        return "jump"
    if words & {"sit", "sitdown"}:
        return "sit"
    if words & {"stand", "standup", "getup"}:
        return "stand"
    return None


def _parse_turn_angle(content: str) -> float | None:
    """Parse turn degrees from a message.

    Returns signed degrees (positive=right, negative=left), floored to 5°
    increments with |deg| in [5, 180]. Returns None if the angle is invalid.
    """
    tokens = re.findall(r"-?\d+(?:\.\d+)?|[a-zA-Z]+", content.lower())
    direction: str | None = None
    raw_number: float | None = None
    for tok in tokens:
        if tok.isalpha():
            if direction is None:
                direction = _match_direction(tok)
        elif raw_number is None:
            try:
                raw_number = float(tok)
            except ValueError:
                pass

    if raw_number is None and direction is None:
        return None  # bare "turn" with nothing else — invalid angle

    if raw_number is None:
        angle = NO_NUMBER_DEFAULT_DEG
    else:
        angle = abs(raw_number)
        if direction is None and raw_number < 0:
            direction = "left"

    angle = math.floor(angle / ANGLE_STEP) * ANGLE_STEP
    if angle < MIN_TURN_DEG or angle > MAX_TURN_DEG:
        return None

    return -angle if direction == "left" else angle


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ── Module ──


class TwitchPlaysGo2(Module):
    """Collects chat votes and drives a Go2 robot with odom-based turn feedback.

    Votes are gathered over a ``VOTE_WINDOW_SECONDS`` window and executed in a
    loop; new votes arriving while a command is running accumulate for the
    *next* window.
    """

    config: ModuleConfig

    raw_messages: In[TwitchMessage]
    odom: In[PoseStamped]
    cmd_vel: Out[Twist]

    _connection: GO2ConnectionSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._votes: list[tuple[str, float | None]] = []
        self._latest_yaw: float | None = None
        self._is_sitting = False
        self._stop_event = threading.Event()
        self._vote_thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self.raw_messages.subscribe(self._on_message)
        self.odom.subscribe(self._on_odom)
        self._vote_thread = threading.Thread(
            target=self._vote_loop, daemon=True, name="twitch-go2-votes"
        )
        self._vote_thread.start()
        logger.info("[TwitchPlaysGo2] Started, window=%.1fs", VOTE_WINDOW_SECONDS)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._vote_thread is not None:
            self._vote_thread.join(timeout=2)
            self._vote_thread = None
        super().stop()

    def _on_odom(self, pose: PoseStamped) -> None:
        with self._lock:
            self._latest_yaw = pose.orientation.euler.yaw

    def _on_message(self, msg: TwitchMessage) -> None:
        category = _categorize(msg.content)
        if category is None:
            return
        angle = _parse_turn_angle(msg.content) if category == "turn" else None
        with self._lock:
            self._votes.append((category, angle))

    def _auto_stand_if_sitting(self) -> None:
        """Issue StandUp if the robot is currently sitting."""
        if self._is_sitting:
            logger.info("[TwitchPlaysGo2] Auto-standing before movement")
            self._do_sport_command("StandUp")
            self._is_sitting = False
            time.sleep(1.0)

    def _vote_loop(self) -> None:
        while not self._stop_event.is_set():
            if self._stop_event.wait(VOTE_WINDOW_SECONDS):
                break
            with self._lock:
                votes, self._votes = self._votes, []
            if not votes:
                continue
            counts = Counter(c for c, _ in votes)
            winner = counts.most_common(1)[0][0]
            logger.info("[TwitchPlaysGo2] Winner: %s (tally=%s)", winner, dict(counts))
            if winner == "forward":
                self._auto_stand_if_sitting()
                self._drive_linear(LINEAR_SPEED)
            elif winner == "back":
                self._auto_stand_if_sitting()
                self._drive_linear(-LINEAR_SPEED)
            elif winner == "turn":
                self._auto_stand_if_sitting()
                self._do_turn([a for c, a in votes if c == "turn" and a is not None])
            elif winner == "jump":
                self._auto_stand_if_sitting()
                self._do_sport_command("FrontJump")
            elif winner == "sit":
                self._do_sport_command("Sit")
                self._is_sitting = True
            elif winner == "stand":
                self._do_sport_command("StandUp")
                self._is_sitting = False

    def _do_sport_command(self, command_name: str) -> None:
        api_id = SPORT_CMD[command_name]
        logger.info("[TwitchPlaysGo2] Sport command: %s (api_id=%d)", command_name, api_id)
        try:
            self._connection.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": api_id})
        except Exception:
            logger.exception("[TwitchPlaysGo2] Failed to execute %s", command_name)

    def _drive_linear(self, linear_x: float) -> None:
        t = Twist()
        t.linear.x = linear_x
        period = 1.0 / CMD_VEL_PUBLISH_HZ
        end = time.time() + LINEAR_DURATION
        while time.time() < end and not self._stop_event.is_set():
            self.cmd_vel.publish(t)
            time.sleep(period)
        self.cmd_vel.publish(Twist())

    def _do_turn(self, valid_angles: list[float]) -> None:
        if valid_angles:
            avg = sum(valid_angles) / len(valid_angles)
            sign = 1.0 if avg >= 0 else -1.0
            magnitude = math.floor(abs(avg) / ANGLE_STEP) * ANGLE_STEP
            target_deg = sign * magnitude if magnitude >= MIN_TURN_DEG else FALLBACK_TURN_DEG
        else:
            target_deg = FALLBACK_TURN_DEG

        logger.info(
            "[TwitchPlaysGo2] Turn target: %+.1f° (valid_angles=%d)",
            target_deg,
            len(valid_angles),
        )

        with self._lock:
            yaw = self._latest_yaw
        if yaw is None:
            logger.warning("[TwitchPlaysGo2] No odom yaw yet — skipping turn")
            return

        start_yaw = yaw
        # ROS convention: +angular.z = counter-clockwise (left). Our target_deg
        # convention: positive = right, so flip the sign.
        angular_z = ANGULAR_SPEED if target_deg < 0 else -ANGULAR_SPEED
        abs_target_rad = math.radians(abs(target_deg))
        margin_rad = math.radians(YAW_MARGIN_DEG)
        period = 1.0 / CMD_VEL_PUBLISH_HZ

        t = Twist()
        t.angular.z = angular_z
        deadline = time.time() + TURN_TIMEOUT_SECONDS
        while time.time() < deadline and not self._stop_event.is_set():
            self.cmd_vel.publish(t)
            time.sleep(period)
            with self._lock:
                yaw = self._latest_yaw
            if yaw is None:
                continue
            delta = abs(_wrap_angle(yaw - start_yaw))
            if delta >= abs_target_rad - margin_rad:
                break
        self.cmd_vel.publish(Twist())


# ── Blueprint ──


twitch_plays_go2 = autoconnect(
    unitree_go2_basic,
    TwitchChat.blueprint(),
    TwitchPlaysGo2.blueprint(),
)

if __name__ == "__main__":
    ModuleCoordinator.build(twitch_plays_go2).loop()
