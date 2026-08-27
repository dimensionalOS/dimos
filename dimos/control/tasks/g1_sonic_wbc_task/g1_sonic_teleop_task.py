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

"""PICO WebXR specialization of the G1 SONIC whole-body task."""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Any, cast

from dimos.control.task import CoordinatorState, JointCommandOutput
from dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task import (
    G1SonicWBCTask,
    G1SonicWBCTaskConfig,
    _create_task,
)
from dimos.control.tasks.g1_sonic_wbc_task.webxr_retargeting import (
    IncompleteBodyPoseError,
    WebXRSonicRetargeter,
)
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.whole_body.spec import WholeBodyAdapter
    from dimos.msgs.geometry_msgs.Twist import Twist

logger = setup_logger()

_BODY_HOLD_SECONDS = 0.15


class G1SonicTeleopTask(G1SonicWBCTask):
    """Drive SONIC from complete WebXR body frames while X+A are held.

    Partial body frames retain the last complete pose for 150 ms. Tracking
    loss, a changed WebXR reference space, a stale complete pose, or release
    of either deadman button returns SONIC to its planner source.
    """

    def __init__(
        self,
        name: str,
        config: G1SonicWBCTaskConfig,
        adapter: WholeBodyAdapter,
    ) -> None:
        super().__init__(name, config, adapter)
        self._teleop_lock = threading.Lock()
        self._retargeter = WebXRSonicRetargeter()
        self._latest_complete: BodyTrackingSnapshot | None = None
        self._latest_complete_time = 0.0
        self._latest_sequence = 0
        self._applied_sequence = 0
        self._stream_frame_index = 0
        self._tracking_frame_id: str | None = None
        self._deadman_held = False
        self._engaged = False
        self._blocked_until_release = False
        self._last_disengage_reason = "not_engaged"
        self._yaw_rate = 0.0
        self._last_yaw_time = 0.0

    def on_body_tracking(self, msg: BodyTrackingSnapshot, t_now: float) -> None:
        with self._teleop_lock:
            if msg.joints is None:
                self._disengage_locked("body_tracking_unavailable", require_release=True)
                self._latest_complete = None
                return

            if not self._retargeter.is_complete(msg):
                return

            if self._tracking_frame_id is not None and msg.frame_id != self._tracking_frame_id:
                self._disengage_locked("tracking_reference_changed", require_release=True)
            self._tracking_frame_id = msg.frame_id
            self._latest_complete = msg
            self._latest_complete_time = t_now
            self._latest_sequence += 1
            self._try_engage_locked(t_now)

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> None:
        held = bool(msg.left_primary and msg.right_primary)
        with self._teleop_lock:
            self._deadman_held = held
            if not held:
                self._blocked_until_release = False
                self._disengage_locked("deadman_released", require_release=False)
                return
            self._try_engage_locked(t_now)

    def on_twist_command(self, msg: Twist, t_now: float) -> None:
        with self._teleop_lock:
            if self._engaged:
                self._yaw_rate = float(msg.angular.z)
                self._last_yaw_time = t_now
                return
        super().on_twist_command(msg, t_now)

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._teleop_lock:
            if self.policy_active:
                self._try_engage_locked(state.t_now)
                self._prepare_teleop_frame_locked(state.t_now, state.dt)
            output = super().compute(state)
            # Initialization may have entered CONTROL in this tick. Select
            # WebXR now so the next policy inference sees the pose reference.
            if self.policy_active:
                self._try_engage_locked(state.t_now)
            return output

    def start(self) -> None:
        with self._teleop_lock:
            self._reset_teleop_locked()
        super().start()

    def stop(self) -> None:
        with self._teleop_lock:
            self._reset_teleop_locked()
        super().stop()

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        with self._teleop_lock:
            self._reset_teleop_locked()
        return super().reset_runtime_state(reactivate)

    def state_snapshot(self) -> dict[str, Any]:
        with self._teleop_lock:
            snapshot = super().state_snapshot()
            last_complete_received_at = None
            if self._latest_complete is not None:
                last_complete_received_at = self._latest_complete_time
            snapshot["webxr_teleop"] = {
                "engaged": self._engaged,
                "deadman_held": self._deadman_held,
                "blocked_until_release": self._blocked_until_release,
                "tracking_frame_id": self._tracking_frame_id,
                "last_complete_received_at": last_complete_received_at,
                "last_disengage_reason": self._last_disengage_reason,
            }
            snapshot["reference_source"] = "webxr_pose" if self._engaged else "planner"
        return snapshot

    def _try_engage_locked(self, t_now: float) -> None:
        if (
            self._engaged
            or not self.policy_active
            or not self._deadman_held
            or self._blocked_until_release
            or self._latest_complete is None
            or (t_now - self._latest_complete_time) > _BODY_HOLD_SECONDS
        ):
            return
        self._engaged = True
        self._last_disengage_reason = ""
        self._retargeter.reset()
        self._select_stream_reference(True)
        self.set_velocity_command(0.0, 0.0, 0.0)
        logger.info("G1 SONIC WebXR teleop engaged", task=self._name)

    def _prepare_teleop_frame_locked(self, t_now: float, dt: float) -> None:
        if not self._engaged:
            return
        if (
            self._latest_complete is None
            or (t_now - self._latest_complete_time) > _BODY_HOLD_SECONDS
        ):
            self._disengage_locked("body_tracking_stale", require_release=True)
            return

        if self._latest_sequence != self._applied_sequence:
            try:
                frame = self._retargeter.retarget(
                    self._latest_complete,
                    frame_index=self._stream_frame_index,
                    t_now=t_now,
                )
            except IncompleteBodyPoseError:
                self._disengage_locked("invalid_body_pose", require_release=True)
                return
            self._select_stream_reference(True)
            result = self._pipeline.apply_pose_message(frame.fields)
            if "error" in result:
                self._disengage_locked("sonic_pose_rejected", require_release=True)
                return
            self._stream_frame_index += 1
            self._applied_sequence = self._latest_sequence

        yaw_is_fresh = self._last_yaw_time > 0.0 and (
            self._config.timeout <= 0.0 or (t_now - self._last_yaw_time) <= self._config.timeout
        )
        if yaw_is_fresh:
            self._pipeline.apply_heading_increment(self._yaw_rate * dt)

    def _disengage_locked(self, reason: str, *, require_release: bool) -> None:
        was_engaged = self._engaged
        self._engaged = False
        self._blocked_until_release = self._blocked_until_release or require_release
        self._last_disengage_reason = reason
        self._yaw_rate = 0.0
        self._last_yaw_time = 0.0
        if was_engaged:
            self._retargeter.reset()
            self._pipeline.clear_vr_3point()
            self._return_to_planner_reference()
            self.set_velocity_command(0.0, 0.0, 0.0)
            logger.info(
                "G1 SONIC WebXR teleop disengaged",
                task=self._name,
                reason=reason,
            )

    def _reset_teleop_locked(self) -> None:
        self._latest_complete = None
        self._latest_complete_time = 0.0
        self._latest_sequence = 0
        self._applied_sequence = 0
        self._stream_frame_index = 0
        self._tracking_frame_id = None
        self._deadman_held = False
        self._engaged = False
        self._blocked_until_release = False
        self._last_disengage_reason = "not_engaged"
        self._yaw_rate = 0.0
        self._last_yaw_time = 0.0
        self._retargeter.reset()


def create_task(cfg: Any, hardware: Any) -> G1SonicTeleopTask:
    return cast("G1SonicTeleopTask", _create_task(cfg, hardware, G1SonicTeleopTask))
