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

"""Body-tracking reference ownership, called under the SONIC control lock."""

from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.control.sonic.sonic_pipeline import POLICY_DT, SonicPipeline
from dimos.control.sonic.webxr.retargeting import WebXRSonicPoseStream
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons

TRACKING_TIMEOUT = 0.25
TRANSITION_SECONDS = 1.0


class WebXRReference:
    """A+X requests pose mode; tracking loss requires a new release and press."""

    def __init__(self) -> None:
        self.stream = WebXRSonicPoseStream()
        self.reset()

    def reset(self) -> None:
        self.stream.reset()
        self.mode = "planner"
        self.reason: str | None = None
        self._frame_id: str | None = None
        self._body_at: float | None = None
        self._buttons_at: float | None = None
        self._toggle_ready = False
        self._toggle = False
        self._exit = False
        self._applied_generation = -1

    @property
    def owns_reference(self) -> bool:
        return self.mode != "planner" or self._toggle

    def body(self, snapshot: BodyTrackingSnapshot, now: float) -> None:
        if self._frame_id is not None and snapshot.frame_id != self._frame_id:
            self._lose_tracking("WebXR reference space changed")
        self._frame_id = snapshot.frame_id
        try:
            self.stream.push(snapshot)
        except ValueError as exc:
            self._lose_tracking(str(exc))
            return
        self._body_at = now

    def buttons(self, buttons: Buttons, now: float, *, enabled: bool) -> None:
        self._buttons_at = now
        if not any(
            (
                buttons.left_primary,
                buttons.right_primary,
                buttons.left_secondary,
                buttons.right_secondary,
            )
        ):
            self._toggle_ready = True
        elif buttons.left_primary and buttons.right_primary and self._toggle_ready:
            self._toggle_ready = False
            self._toggle = enabled and not (buttons.left_secondary or buttons.right_secondary)

    def stop(self, reason: str = "Operator stopped pose tracking") -> None:
        self._exit = True
        self._toggle = False
        self._toggle_ready = False
        self.reason = reason

    def _lose_tracking(self, reason: str) -> None:
        self.stop(reason)
        self.stream.reset()
        self._body_at = None

    def tick(
        self, policy: SonicPipeline, command: NDArray[np.float32], now: float
    ) -> NDArray[np.float32]:
        fresh = all(
            t is not None and 0 <= now - t <= TRACKING_TIMEOUT
            for t in (self._body_at, self._buttons_at)
        )
        if self.mode in ("pose", "entering") and not fresh:
            self._lose_tracking("WebXR body or controller input timed out")
        if self._toggle:
            self._toggle = False
            if self.mode == "planner":
                if fresh and self.stream.ready:
                    policy.clear_vr_3point()
                    policy.set_pose_window(self.stream.fields())
                    policy.begin_pose_transition(TRANSITION_SECONDS)
                    self._applied_generation = self.stream.generation
                    self.mode = "entering"
                    self._exit = False
                    self.reason = None
                else:
                    self.reason = "Need a fresh complete body window and controllers"
            else:
                self.stop()
        if self._exit:
            self._exit = False
            if self.mode in ("pose", "entering"):
                policy.return_to_planner(TRANSITION_SECONDS)
                self.mode = "returning"
        if self.mode in ("pose", "entering"):
            if self.stream.generation != self._applied_generation and self.stream.ready:
                policy.set_pose_window(self.stream.fields())
                self._applied_generation = self.stream.generation
            if not policy.reference_transition_active:
                self.mode = "pose"
            policy.apply_heading_increment(float(command[2]) * POLICY_DT)
        elif self.mode == "returning" and not policy.reference_transition_active:
            self.mode = "planner"
            self.stream.reset()
            self._toggle_ready = False
            return np.zeros(3, dtype=np.float32)
        return command if self.mode == "planner" else np.zeros(3, dtype=np.float32)

    def status(self, now: float) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "reason": self.reason,
            "buffered_frames": self.stream.buffered_frames,
            "body_age_ms": None if self._body_at is None else (now - self._body_at) * 1000,
        }
