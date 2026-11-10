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

"""Receipt-stamped WebXR operator input for teleoperation control tasks.

The coordinator delivers controller poses and packed buttons as separate
streams.  A policy or IK task needs the last sample of every source plus the
time it was *received*: a goal rebuilt from one live and one stale controller
is not fresh, and a held grip cannot outlive its button message.  This layer
records exactly that.  Deciding what a stale input means (wait, release, fault)
stays with the task.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.teleop.webxr.controller_types import Buttons

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    PoseLike = Pose | PoseStamped

#: Controller sides in the trained/goal layout.
SIDES: tuple[str, str] = ("left", "right")


class TeleopInput:
    """Last sample and receipt time of each WebXR input source."""

    def __init__(self) -> None:
        self._poses: dict[str, tuple[PoseLike, float]] = {}
        self._buttons_at: float | None = None
        self._grips: tuple[bool, bool] = (False, False)

    @property
    def grips(self) -> tuple[bool, bool]:
        return self._grips

    @property
    def deadman(self) -> bool:
        """Both grips held; the caller decides what that means."""
        return self._grips[0] and self._grips[1]

    def on_buttons(self, msg: Buttons, t_now: float) -> None:
        self._grips = (bool(msg.left_grip), bool(msg.right_grip))
        self._buttons_at = t_now

    def on_pose(self, side: str, pose: PoseLike, t_now: float) -> None:
        self._poses[side] = (pose, t_now)

    def pose(self, side: str) -> tuple[PoseLike, float] | None:
        return self._poses.get(side)

    def ages(self, now: float) -> tuple[float, float, float] | None:
        """Ages of buttons, left and right poses, or ``None`` if a source is missing."""
        left = self._poses.get("left")
        right = self._poses.get("right")
        if self._buttons_at is None or left is None or right is None:
            return None
        return (now - self._buttons_at, now - left[1], now - right[1])

    def fresh(self, now: float, timeout: float) -> bool:
        """True while the deadman is held and every source is within ``timeout``."""
        ages = self.ages(now)
        return ages is not None and self.deadman and all(age <= timeout for age in ages)

    def clear(self) -> None:
        """Forget every sample; a later session needs new input, never a replay."""
        self._poses.clear()
        self._buttons_at = None
        self._grips = (False, False)
