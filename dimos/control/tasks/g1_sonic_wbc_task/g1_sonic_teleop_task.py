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

from collections.abc import Callable
from enum import Enum
import threading
from typing import TYPE_CHECKING, Any, cast

from dimos.control.task import CoordinatorState, JointCommandOutput
from dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task import (
    G1SonicWBCTask,
    G1SonicWBCTaskConfig,
    _create_task,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import WRIST_ONNX_INDICES
from dimos.control.tasks.g1_sonic_wbc_task.webxr_retargeting import (
    IncompleteBodyPoseError,
    PoseStreamError,
    WebXRSonicPoseStream,
    WebXRSonicRetargeter,
)
from dimos.msgs.visualization_msgs.SonicPoseReference import SonicPoseReference
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.whole_body.spec import WholeBodyAdapter
    from dimos.msgs.geometry_msgs.Twist import Twist

logger = setup_logger()

_BODY_HOLD_SECONDS = 1.0


class SonicTeleopMode(str, Enum):
    OFF = "off"
    PLANNER = "planner"
    POSE_TRANSITION = "pose_transition"
    POSE = "pose"
    PLANNER_TRANSITION = "planner_transition"


_POSE_REFERENCE_MODES = frozenset(
    {
        SonicTeleopMode.POSE_TRANSITION,
        SonicTeleopMode.POSE,
    }
)
_POSE_HISTORY_MODES = _POSE_REFERENCE_MODES | {SonicTeleopMode.PLANNER_TRANSITION}


class G1SonicTeleopTask(G1SonicWBCTask):
    """Run smooth planner-to-pose and pose-to-planner handoffs.

    The DimOS policy lifecycle owns OFF -> PLANNER: armed policy control enters
    the balancing planner, including in dry-run, while disarm enters OFF. Exact
    A+X toggles between the planner and the configured full-body POSE stream.
    """

    def __init__(
        self,
        name: str,
        config: G1SonicWBCTaskConfig,
        adapter: WholeBodyAdapter,
    ) -> None:
        super().__init__(name, config, adapter)
        # ZMQ command handling runs inside compute() and can synchronously
        # invoke disarm(), so lifecycle cleanup must be re-entrant here.
        self._teleop_lock = threading.RLock()
        self._pose_stream = WebXRSonicPoseStream(config.sonic_pipeline)
        self._latest_complete: BodyTrackingSnapshot | None = None
        self._latest_complete_time = 0.0
        self._tracking_frame_id: str | None = None
        self._mode = SonicTeleopMode.OFF
        self._previous_ax_combo = False
        self._applied_generation = 0
        self._last_transition_reason = "not_started"
        self._yaw_rate = 0.0
        self._last_yaw_time = 0.0
        self._pose_reference_publisher: Callable[[SonicPoseReference], None] | None = None
        self._pose_reference_visible = False

    def set_pose_reference_publisher(self, publisher: Callable[[SonicPoseReference], None]) -> None:
        """Attach the coordinator-owned diagnostic stream publisher."""
        with self._teleop_lock:
            self._pose_reference_publisher = publisher
            self._publish_pose_reference_locked(SonicPoseReference.clear())

    def on_body_tracking(self, msg: BodyTrackingSnapshot, t_now: float) -> None:
        with self._teleop_lock:
            if msg.joints is None:
                self._latest_complete = None
                if self._mode in _POSE_REFERENCE_MODES:
                    self._enter_planner_locked("body_tracking_unavailable")
                elif self._mode is SonicTeleopMode.PLANNER:
                    self._clear_pose_stream_locked("body_tracking_unavailable")
                return

            if not WebXRSonicRetargeter.is_complete(msg):
                return

            if (
                self._mode is not SonicTeleopMode.OFF
                and self._tracking_frame_id is not None
                and msg.frame_id != self._tracking_frame_id
            ):
                self._latest_complete = msg
                self._latest_complete_time = t_now
                self._enter_planner_locked("tracking_reference_changed")
                return

            self._latest_complete = msg
            self._latest_complete_time = t_now
            if self._mode is SonicTeleopMode.OFF:
                return

            self._tracking_frame_id = msg.frame_id
            try:
                self._pose_stream.push(msg)
            except (IncompleteBodyPoseError, PoseStreamError) as exc:
                logger.warning(
                    "G1 SONIC WebXR pose stream reset",
                    task=self._name,
                    error=str(exc),
                )
                if self._mode in _POSE_REFERENCE_MODES:
                    self._enter_planner_locked("invalid_body_pose")
                else:
                    self._clear_pose_stream_locked("invalid_body_pose")

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> None:
        ax_combo = bool(
            msg.left_primary
            and msg.right_primary
            and not msg.left_secondary
            and not msg.right_secondary
        )
        with self._teleop_lock:
            ax_edge = ax_combo and not self._previous_ax_combo
            self._previous_ax_combo = ax_combo

            if not ax_edge:
                return
            if self._mode is SonicTeleopMode.PLANNER:
                self._enter_pose_locked(t_now)
            elif self._mode in _POSE_REFERENCE_MODES:
                self._enter_planner_locked("operator_planner_toggle")

    def on_twist_command(self, msg: Twist, t_now: float) -> None:
        with self._teleop_lock:
            if self._mode in _POSE_REFERENCE_MODES:
                self._yaw_rate = float(msg.angular.z)
                self._last_yaw_time = t_now
                return
        super().on_twist_command(msg, t_now)

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._teleop_lock:
            self._sync_policy_lifecycle_locked()
            if self.policy_active:
                self._prepare_teleop_locked(state.t_now, state.dt)
            output = super().compute(state)
            self._sync_policy_lifecycle_locked()
            if (
                self._mode is SonicTeleopMode.POSE_TRANSITION
                and not self._pipeline.reference_transition_active
            ):
                self._mode = SonicTeleopMode.POSE
                self._last_transition_reason = "pose_transition_complete"
                logger.info("G1 SONIC WebXR mode", task=self._name, mode=self._mode.value)
            elif (
                self._mode is SonicTeleopMode.PLANNER_TRANSITION
                and not self._pipeline.reference_transition_active
            ):
                self._mode = SonicTeleopMode.PLANNER
                logger.info(
                    "G1 SONIC WebXR mode",
                    task=self._name,
                    mode=self._mode.value,
                    reason=self._last_transition_reason,
                )
            return output

    def start(self) -> None:
        with self._teleop_lock:
            self._reset_teleop_locked()
        super().start()
        logger.info(
            "G1 SONIC WebXR pipeline configured",
            task=self._name,
            sonic_pipeline=self._pose_stream.sonic_pipeline,
            pose_window_frames=self._pose_stream.window_frames,
        )

    def stop(self) -> None:
        with self._teleop_lock:
            self._reset_teleop_locked("task_stopped")
        super().stop()

    def disarm(self) -> bool:
        with self._teleop_lock:
            self._reset_teleop_locked("policy_disarmed")
        return super().disarm()

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        with self._teleop_lock:
            self._reset_teleop_locked("runtime_reset")
        return super().reset_runtime_state(reactivate)

    def set_dry_run(self, enabled: bool) -> None:
        with self._teleop_lock:
            was_dry_run = self._dry_run
            super().set_dry_run(enabled)
            if was_dry_run and not self._dry_run and self._mode in _POSE_HISTORY_MODES:
                self._enter_planner_locked("motor_output_enabled", smooth=False)
                self._reset_policy_state()
            self._sync_policy_lifecycle_locked()

    def state_snapshot(self) -> dict[str, Any]:
        with self._teleop_lock:
            snapshot = super().state_snapshot()
            last_complete_received_at = None
            if self._latest_complete is not None:
                last_complete_received_at = self._latest_complete_time
            snapshot["webxr_teleop"] = {
                "mode": self._mode.value,
                "sonic_pipeline": self._pose_stream.sonic_pipeline,
                "pose_window_frames": self._pose_stream.window_frames,
                "pose_transition_seconds": self._config.pose_transition_seconds,
                "pose_transition_progress": (
                    1.0
                    if self._mode is SonicTeleopMode.POSE
                    else self._pipeline.reference_transition_progress
                    if self._mode
                    in {
                        SonicTeleopMode.POSE_TRANSITION,
                        SonicTeleopMode.PLANNER_TRANSITION,
                    }
                    else 0.0
                ),
                "stream_ready": self._pose_stream.ready,
                "buffered_frames": self._pose_stream.buffered_frames,
                "tracking_frame_id": self._tracking_frame_id,
                "last_complete_received_at": last_complete_received_at,
                "last_transition_reason": self._last_transition_reason,
            }
            if self._mode is SonicTeleopMode.POSE_TRANSITION:
                snapshot["reference_source"] = "planner_to_webxr_pose"
            elif self._mode is SonicTeleopMode.POSE:
                snapshot["reference_source"] = "webxr_pose"
            elif self._mode is SonicTeleopMode.PLANNER_TRANSITION:
                snapshot["reference_source"] = "webxr_pose_to_planner"
            else:
                snapshot["reference_source"] = "planner"
            return snapshot

    def _enter_pose_locked(self, t_now: float) -> None:
        if not self.policy_active:
            self._enter_off_locked("policy_inactive")
            return
        if (
            self._latest_complete is None
            or (t_now - self._latest_complete_time) > _BODY_HOLD_SECONDS
        ):
            self._last_transition_reason = "body_tracking_stale"
            return
        if not self._pose_stream.ready:
            self._last_transition_reason = "pose_buffer_not_ready"
            logger.warning(
                "G1 SONIC WebXR POSE rejected",
                task=self._name,
                buffered_frames=self._pose_stream.buffered_frames,
            )
            return

        # Match the native manager's ordering: pose data reaches SONIC before
        # the planner flag changes, so no empty or previous-session stream can
        # become the active reference.
        self._return_to_planner_reference()
        result = self._apply_pose_stream_locked()
        if "error" in result:
            self._last_transition_reason = "sonic_pose_rejected"
            self._pose_stream.reset()
            return
        self._applied_generation = self._pose_stream.generation
        self.set_velocity_command(0.0, 0.0, 0.0)
        if not self._begin_stream_reference_transition(self._config.pose_transition_seconds):
            self._last_transition_reason = "planner_reference_not_ready"
            self._applied_generation = 0
            self._clear_pose_reference_locked()
            self._return_to_planner_reference()
            return
        self._yaw_rate = 0.0
        self._last_yaw_time = 0.0
        self._mode = SonicTeleopMode.POSE_TRANSITION
        self._last_transition_reason = "operator_pose_toggle"
        logger.info("G1 SONIC WebXR mode", task=self._name, mode=self._mode.value)

    def _enter_planner_locked(self, reason: str, *, smooth: bool = True) -> None:
        was_pose_reference = self._mode in _POSE_REFERENCE_MODES
        was_pose_history = self._mode in _POSE_HISTORY_MODES
        self._clear_pose_reference_locked()
        self._last_transition_reason = reason
        self._yaw_rate = 0.0
        self._last_yaw_time = 0.0
        self._pipeline.clear_vr_3point()
        transition_started = False
        if was_pose_reference:
            self.set_velocity_command(0.0, 0.0, 0.0)
            if smooth:
                transition_started = self._begin_planner_reference_transition(
                    self._config.pose_transition_seconds
                )
        if was_pose_history and not transition_started:
            self._return_to_planner_reference()
        self._mode = (
            SonicTeleopMode.PLANNER_TRANSITION if transition_started else SonicTeleopMode.PLANNER
        )
        self._pose_stream.reset()
        self._applied_generation = 0
        self._tracking_frame_id = None
        if self._latest_complete is not None:
            self._tracking_frame_id = self._latest_complete.frame_id
            try:
                self._pose_stream.push(self._latest_complete)
            except (IncompleteBodyPoseError, PoseStreamError):
                pass
        logger.info(
            "G1 SONIC WebXR mode",
            task=self._name,
            mode=self._mode.value,
            reason=reason,
        )

    def _enter_off_locked(self, reason: str) -> None:
        was_pose = self._mode in _POSE_HISTORY_MODES
        self._clear_pose_reference_locked()
        self._mode = SonicTeleopMode.OFF
        self._last_transition_reason = reason
        self._tracking_frame_id = None
        self._yaw_rate = 0.0
        self._last_yaw_time = 0.0
        self._pipeline.clear_vr_3point()
        if was_pose:
            self._return_to_planner_reference()
            self.set_velocity_command(0.0, 0.0, 0.0)
        self._pose_stream.reset()
        self._applied_generation = 0
        logger.info(
            "G1 SONIC WebXR mode",
            task=self._name,
            mode=self._mode.value,
            reason=reason,
        )

    def _clear_pose_stream_locked(self, reason: str) -> None:
        self._pose_stream.reset()
        self._applied_generation = 0
        self._last_transition_reason = reason

    def _sync_policy_lifecycle_locked(self) -> None:
        if not self.policy_active:
            if self._mode is not SonicTeleopMode.OFF:
                self._enter_off_locked("policy_inactive")
            return
        if self._mode is SonicTeleopMode.OFF:
            self._enter_planner_locked("policy_control_active")

    def _prepare_teleop_locked(self, t_now: float, dt: float) -> None:
        if self._mode in {SonicTeleopMode.OFF, SonicTeleopMode.PLANNER_TRANSITION}:
            return
        if (
            self._latest_complete is None
            or (t_now - self._latest_complete_time) > _BODY_HOLD_SECONDS
        ):
            if self._mode in _POSE_REFERENCE_MODES:
                self._enter_planner_locked("body_tracking_stale")
            else:
                self._clear_pose_stream_locked("body_tracking_stale")
            return

        if (
            self._mode in _POSE_REFERENCE_MODES
            and self._pose_stream.ready
            and self._pose_stream.generation != self._applied_generation
        ):
            result = self._apply_pose_stream_locked()
            if "error" in result:
                self._enter_planner_locked("sonic_pose_rejected")
                return
            self._applied_generation = self._pose_stream.generation

        yaw_is_fresh = self._last_yaw_time > 0.0 and (
            self._config.timeout <= 0.0 or (t_now - self._last_yaw_time) <= self._config.timeout
        )
        if self._mode in _POSE_REFERENCE_MODES and yaw_is_fresh:
            self._pipeline.apply_heading_increment(self._yaw_rate * dt)

    def _apply_pose_stream_locked(self) -> dict[str, Any]:
        fields = self._pose_stream.fields()
        result = self._pipeline.apply_pose_message(fields)
        if "error" not in result:
            self._publish_pose_reference_locked(
                SonicPoseReference.from_arrays(
                    frame_indices=fields["frame_index"],
                    smpl_joints=fields["smpl_joints"],
                    body_quat_w=fields["body_quat_w"],
                    wrist_joint_pos=fields["joint_pos"][:, WRIST_ONNX_INDICES],
                )
            )
        return result

    def _publish_pose_reference_locked(self, reference: SonicPoseReference) -> None:
        if self._pose_reference_publisher is None:
            return
        try:
            self._pose_reference_publisher(reference)
        except Exception:
            logger.warning(
                "G1 SONIC reference visualization publish failed",
                task=self._name,
                exc_info=True,
            )
            return
        self._pose_reference_visible = reference.active

    def _clear_pose_reference_locked(self) -> None:
        if self._pose_reference_visible:
            self._publish_pose_reference_locked(SonicPoseReference.clear())

    def _reset_teleop_locked(self, reason: str = "not_started") -> None:
        self._clear_pose_reference_locked()
        if self._mode in _POSE_HISTORY_MODES:
            self._pipeline.clear_vr_3point()
            self._return_to_planner_reference()
            self.set_velocity_command(0.0, 0.0, 0.0)
        self._latest_complete = None
        self._latest_complete_time = 0.0
        self._tracking_frame_id = None
        self._mode = SonicTeleopMode.OFF
        self._previous_ax_combo = False
        self._applied_generation = 0
        self._last_transition_reason = reason
        self._yaw_rate = 0.0
        self._last_yaw_time = 0.0
        self._pose_stream.reset()


def create_task(cfg: Any, hardware: Any) -> G1SonicTeleopTask:
    return cast("G1SonicTeleopTask", _create_task(cfg, hardware, G1SonicTeleopTask))
