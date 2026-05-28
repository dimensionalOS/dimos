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

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Literal

from dimos_lcm.std_msgs import Bool, String  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.replanning_a_star.module_spec import ReplanningAStarPlannerSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

BehaviorState = Literal[
    "idle",
    "navigating_near",
    "dwelling_near",
    "navigating_far",
    "dwelling_far",
    "returning_near",
    "done",
    "failed",
]
GoalName = Literal["near", "far", "return_near"]

_WORLD_FRAME_ID = "world"


class TargetStandoffBehaviorConfig(ModuleConfig):
    command_hz: float = 5.0
    near_distance: float = 0.5
    far_distance: float = 3.0
    near_dwell_duration_sec: float = 10.0
    far_dwell_duration_sec: float = 10.0
    countdown_log_interval_sec: float = 2.0
    goal_timeout_sec: float = 45.0
    tf_time_tolerance: float = 0.5
    restart_target_distance_m: float = 0.2


class TargetStandoffBehaviorModule(Module):
    """Compute near/far target waypoints and delegate motion to the path planner."""

    config: TargetStandoffBehaviorConfig

    target_pose: In[PoseStamped]
    teleop_active: In[Bool]

    clear_selection_request: Out[Bool]
    behavior_status: Out[String]

    _planner: ReplanningAStarPlannerSpec | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._state: BehaviorState = "idle"
        self._state_started_at = time.monotonic()
        self._target_pose: PoseStamped | None = None
        self._near_pose: PoseStamped | None = None
        self._far_pose: PoseStamped | None = None
        self._current_goal_name: GoalName | None = None
        self._current_goal_pose: PoseStamped | None = None
        self._goal_started_at: float | None = None
        self._last_countdown_log_at = 0.0
        self._last_block_reason: str | None = None
        self._last_geometry_block_reason: str | None = None
        self._planner = None

    @rpc
    def start(self) -> None:
        super().start()
        logger.info(
            "TargetStandoffBehaviorModule: started "
            f"command_hz={self.config.command_hz:.1f} "
            f"near_distance={self.config.near_distance:.3f} "
            f"far_distance={self.config.far_distance:.3f} "
            f"near_dwell_duration_sec={self.config.near_dwell_duration_sec:.3f} "
            f"far_dwell_duration_sec={self.config.far_dwell_duration_sec:.3f} "
            f"countdown_log_interval_sec={self.config.countdown_log_interval_sec:.3f} "
            f"goal_timeout_sec={self.config.goal_timeout_sec:.3f} "
            f"tf_time_tolerance={self.config.tf_time_tolerance:.3f}s"
        )
        self.register_disposable(Disposable(self.target_pose.subscribe(self._on_target_pose)))
        self.register_disposable(Disposable(self.teleop_active.subscribe(self._on_teleop_active)))
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._behavior_loop,
            name="TargetStandoffBehaviorModule",
            daemon=True,
        )
        self._thread.start()
        self._publish_status("idle")

    @rpc
    def stop(self) -> None:
        logger.info("TargetStandoffBehaviorModule: stopping; cancelling planner goal")
        self._stop_event.set()
        self._cancel_planner_goal()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    @rpc
    def start_behavior(self) -> str:
        with self._lock:
            target_pose = self._target_pose
        if target_pose is None:
            return "target standoff behavior has no target pose yet"
        if self._start_sequence(target_pose):
            return "target standoff behavior started"
        return "target standoff behavior could not start"

    @rpc
    def stop_behavior(self) -> str:
        with self._lock:
            previous_state = self._state
            self._reset_sequence_locked(state="idle")
        self._cancel_planner_goal()
        self.clear_selection_request.publish(Bool(data=True))
        logger.info(
            "TargetStandoffBehaviorModule: stop_behavior requested "
            f"state_before={previous_state}"
        )
        self._publish_status("idle")
        return "target standoff behavior stopped"

    def _on_target_pose(self, target_pose: PoseStamped) -> None:
        with self._lock:
            state = self._state
            previous_target = self._target_pose
            self._target_pose = target_pose

        if state == "idle":
            self._start_sequence(target_pose)
            return

        if state in ("done", "failed") and self._is_new_target(target_pose, previous_target):
            logger.info(
                "TargetStandoffBehaviorModule: received new target after terminal state; "
                f"restarting pose={self._pose_summary(target_pose)}"
            )
            self._start_sequence(target_pose)
            return

        logger.debug(
            "TargetStandoffBehaviorModule: ignoring target_pose update during active sequence "
            f"state={state} pose={self._pose_summary(target_pose)}"
        )

    def _on_teleop_active(self, msg: Any) -> None:
        if not bool(getattr(msg, "data", False)):
            return
        with self._lock:
            previous_state = self._state
            if previous_state == "idle":
                return
            self._reset_sequence_locked(state="idle")
        self._cancel_planner_goal()
        self.clear_selection_request.publish(Bool(data=True))
        logger.info(
            "TargetStandoffBehaviorModule: interrupted by teleop "
            f"state_before={previous_state}"
        )
        self._publish_status("idle")

    def _start_sequence(self, target_pose: PoseStamped) -> bool:
        waypoints = self._compute_waypoints(target_pose)
        if waypoints is None:
            self._publish_status("idle", target_source="blocked")
            return False

        target, near, far = waypoints
        with self._lock:
            self._target_pose = target
            self._near_pose = near
            self._far_pose = far
            self._current_goal_name = None
            self._current_goal_pose = None
            self._goal_started_at = None

        logger.info(
            "TargetStandoffBehaviorModule: waypoints computed "
            f"target={self._pose_summary(target)} "
            f"near={self._pose_summary(near)} "
            f"far={self._pose_summary(far)}"
        )
        return self._dispatch_goal("near", near, "navigating_near")

    def _behavior_loop(self) -> None:
        period_sec = 1.0 / max(self.config.command_hz, 1.0)
        while not self._stop_event.wait(period_sec):
            self._tick()

    def _tick(self) -> None:
        with self._lock:
            state = self._state
            state_started_at = self._state_started_at
            near_pose = self._near_pose
            far_pose = self._far_pose
            current_goal_name = self._current_goal_name
            goal_started_at = self._goal_started_at

        if state in ("idle", "done", "failed"):
            self._set_block_reason(None)
            return

        planner = self._planner
        if planner is None:
            self._fail_sequence("missing_planner_spec")
            return

        now = time.monotonic()
        if state in ("navigating_near", "navigating_far", "returning_near"):
            if self._planner_goal_reached(planner):
                if state == "navigating_near":
                    self._transition("dwelling_near", now, reason="near_goal_reached")
                    return
                if state == "navigating_far":
                    self._transition("dwelling_far", now, reason="far_goal_reached")
                    return
                self._complete_sequence()
                return
            if (
                goal_started_at is not None
                and now - goal_started_at > self.config.goal_timeout_sec
            ):
                self._fail_sequence(f"{current_goal_name or 'unknown'}_goal_timeout")
                return
            self._publish_status(state, elapsed_sec=now - state_started_at)
            return

        if state == "dwelling_near":
            elapsed = now - state_started_at
            if elapsed < self.config.near_dwell_duration_sec:
                self._log_countdown(
                    state,
                    self.config.near_dwell_duration_sec,
                    elapsed,
                    now,
                )
                self._publish_status(state, elapsed_sec=elapsed)
                return
            if far_pose is None:
                self._fail_sequence("missing_far_pose")
                return
            self._dispatch_goal("far", far_pose, "navigating_far")
            return

        if state == "dwelling_far":
            elapsed = now - state_started_at
            if elapsed < self.config.far_dwell_duration_sec:
                self._log_countdown(
                    state,
                    self.config.far_dwell_duration_sec,
                    elapsed,
                    now,
                )
                self._publish_status(state, elapsed_sec=elapsed)
                return
            if near_pose is None:
                self._fail_sequence("missing_near_pose")
                return
            self._dispatch_goal("return_near", near_pose, "returning_near")

    def _dispatch_goal(
        self,
        goal_name: GoalName,
        goal_pose: PoseStamped,
        next_state: BehaviorState,
    ) -> bool:
        planner = self._planner
        if planner is None:
            self._fail_sequence("missing_planner_spec")
            return False

        accepted = planner.set_goal(goal_pose)
        if not accepted:
            self._fail_sequence(f"{goal_name}_goal_rejected")
            return False

        now = time.monotonic()
        with self._lock:
            old_state = self._state
            self._state = next_state
            self._state_started_at = now
            self._current_goal_name = goal_name
            self._current_goal_pose = goal_pose
            self._goal_started_at = now
            self._last_countdown_log_at = 0.0
        logger.info(
            "TargetStandoffBehaviorModule: navigation goal set "
            f"name={goal_name} state={old_state}->{next_state} "
            f"goal={self._pose_summary(goal_pose)}"
        )
        self._publish_status(next_state, goal_name=goal_name)
        return True

    def _compute_waypoints(
        self,
        target_pose: PoseStamped,
    ) -> tuple[PoseStamped, PoseStamped, PoseStamped] | None:
        if target_pose.frame_id and target_pose.frame_id != _WORLD_FRAME_ID:
            self._set_geometry_block_reason(f"target_frame_is_{target_pose.frame_id!r}")
            return None
        robot_tf = self._get_robot_transform()
        if robot_tf is None:
            self._set_geometry_block_reason("missing_world_to_base_link_tf")
            return None

        target_x = float(target_pose.position.x)
        target_y = float(target_pose.position.y)
        robot_x = float(robot_tf.translation.x)
        robot_y = float(robot_tf.translation.y)
        ux, uy = self._unit_from_target_to_robot(target_x, target_y, robot_x, robot_y)
        self._set_geometry_block_reason(None)

        target = self._make_pose(target_x, target_y, 0.0, yaw=robot_tf.rotation.to_euler().yaw)
        near = self._make_nav_pose_facing_target(
            target_x + ux * self.config.near_distance,
            target_y + uy * self.config.near_distance,
            target_x,
            target_y,
        )
        far = self._make_nav_pose_facing_target(
            target_x + ux * self.config.far_distance,
            target_y + uy * self.config.far_distance,
            target_x,
            target_y,
        )
        return target, near, far

    def _make_nav_pose_facing_target(
        self,
        x: float,
        y: float,
        target_x: float,
        target_y: float,
    ) -> PoseStamped:
        yaw = math.atan2(target_y - y, target_x - x)
        return self._make_pose(x, y, 0.0, yaw=yaw)

    @staticmethod
    def _make_pose(x: float, y: float, z: float, *, yaw: float) -> PoseStamped:
        return PoseStamped(
            ts=time.time(),
            frame_id=_WORLD_FRAME_ID,
            position=Vector3(x, y, z),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
        )

    def _get_robot_transform(self) -> Transform | None:
        try:
            return self.tf.get(
                _WORLD_FRAME_ID,
                "base_link",
                time_tolerance=self.config.tf_time_tolerance,
            )
        except RuntimeError as exc:
            logger.debug(f"TargetStandoffBehaviorModule: robot TF lookup failed: {exc}")
            return None

    def _complete_sequence(self) -> None:
        with self._lock:
            target_pose = self._target_pose
            near_pose = self._near_pose
            far_pose = self._far_pose
            self._state = "done"
            self._state_started_at = time.monotonic()
            self._current_goal_name = None
            self._current_goal_pose = None
            self._goal_started_at = None
            self._last_countdown_log_at = 0.0
        logger.info(
            "TargetStandoffBehaviorModule: behavior complete "
            f"target={self._pose_summary(target_pose)} "
            f"near={self._pose_summary(near_pose)} "
            f"far={self._pose_summary(far_pose)}"
        )
        self.clear_selection_request.publish(Bool(data=True))
        self._publish_status("done")

    def _fail_sequence(self, reason: str) -> None:
        with self._lock:
            previous_state = self._state
            self._state = "failed"
            self._state_started_at = time.monotonic()
            self._current_goal_name = None
            self._current_goal_pose = None
            self._goal_started_at = None
        self._cancel_planner_goal()
        self.clear_selection_request.publish(Bool(data=True))
        self._set_block_reason(reason)
        logger.info(
            "TargetStandoffBehaviorModule: behavior failed "
            f"state_before={previous_state} reason={reason}"
        )
        self._publish_status("failed", target_source=reason)

    def _reset_sequence_locked(self, *, state: BehaviorState) -> None:
        self._state = state
        self._state_started_at = time.monotonic()
        self._target_pose = None
        self._near_pose = None
        self._far_pose = None
        self._current_goal_name = None
        self._current_goal_pose = None
        self._goal_started_at = None
        self._last_countdown_log_at = 0.0

    def _cancel_planner_goal(self) -> None:
        if self._planner is not None:
            self._planner.cancel_goal()

    def _planner_goal_reached(self, planner: ReplanningAStarPlannerSpec) -> bool:
        try:
            return planner.is_goal_reached()
        except RuntimeError as exc:
            self._set_block_reason(f"planner_goal_check_failed:{exc}")
            return False

    def _set_block_reason(self, reason: str | None) -> None:
        if reason == self._last_block_reason:
            return
        self._last_block_reason = reason
        if reason is not None:
            logger.info(f"TargetStandoffBehaviorModule: blocked reason={reason}")

    def _set_geometry_block_reason(self, reason: str | None) -> None:
        if reason == self._last_geometry_block_reason:
            return
        previous = self._last_geometry_block_reason
        self._last_geometry_block_reason = reason
        if reason is None:
            if previous is not None:
                logger.info(
                    "TargetStandoffBehaviorModule: target geometry recovered "
                    f"previous_reason={previous}"
                )
            return
        logger.info(f"TargetStandoffBehaviorModule: target geometry blocked reason={reason}")

    def _transition(
        self,
        state: BehaviorState,
        started_at: float,
        *,
        reason: str,
    ) -> None:
        with self._lock:
            old_state = self._state
            self._state = state
            self._state_started_at = started_at
            self._current_goal_name = None
            self._current_goal_pose = None
            self._goal_started_at = None
            self._last_countdown_log_at = 0.0
        logger.info(
            "TargetStandoffBehaviorModule: action transition "
            f"completed={old_state} entered={state} reason={reason}"
        )
        self._publish_status(state)

    def _log_countdown(
        self,
        state: BehaviorState,
        duration_sec: float,
        elapsed_sec: float,
        now: float,
    ) -> None:
        with self._lock:
            last_logged_at = self._last_countdown_log_at
            if (
                last_logged_at > 0.0
                and now - last_logged_at < self.config.countdown_log_interval_sec
            ):
                return
            self._last_countdown_log_at = now
        remaining_sec = max(duration_sec - elapsed_sec, 0.0)
        logger.info(
            "TargetStandoffBehaviorModule: dwell countdown "
            f"state={state} elapsed_sec={elapsed_sec:.1f} remaining_sec={remaining_sec:.1f}"
        )

    def _publish_status(
        self,
        state: BehaviorState,
        *,
        elapsed_sec: float | None = None,
        goal_name: str | None = None,
        target_source: str = "target_pose",
    ) -> None:
        with self._lock:
            payload: dict[str, Any] = {
                "state": state,
                "target_source": target_source,
                "elapsed_sec": 0.0 if elapsed_sec is None else elapsed_sec,
                "goal_name": goal_name or self._current_goal_name,
                "target_pose": self._pose_to_dict(self._target_pose),
                "near_pose": self._pose_to_dict(self._near_pose),
                "far_pose": self._pose_to_dict(self._far_pose),
                "current_goal_pose": self._pose_to_dict(self._current_goal_pose),
            }
        self.behavior_status.publish(String(json.dumps(payload, ensure_ascii=True)))

    @staticmethod
    def _unit_from_target_to_robot(
        target_x: float,
        target_y: float,
        robot_x: float,
        robot_y: float,
    ) -> tuple[float, float]:
        dx = robot_x - target_x
        dy = robot_y - target_y
        norm = math.sqrt(dx * dx + dy * dy)
        if norm < 1e-6:
            return (-1.0, 0.0)
        return dx / norm, dy / norm

    def _is_new_target(
        self,
        target_pose: PoseStamped,
        previous_target: PoseStamped | None,
    ) -> bool:
        if previous_target is None:
            return True
        dx = float(target_pose.position.x) - float(previous_target.position.x)
        dy = float(target_pose.position.y) - float(previous_target.position.y)
        return math.sqrt(dx * dx + dy * dy) > self.config.restart_target_distance_m

    @staticmethod
    def _pose_to_dict(pose: PoseStamped | None) -> dict[str, Any] | None:
        if pose is None:
            return None
        return {
            "frame_id": pose.frame_id,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "ts": float(pose.ts or 0.0),
        }

    @staticmethod
    def _pose_summary(pose: PoseStamped | None) -> str:
        if pose is None:
            return "None"
        return (
            f"frame={pose.frame_id!r} "
            f"x={float(pose.position.x):.3f} "
            f"y={float(pose.position.y):.3f} "
            f"z={float(pose.position.z):.3f} "
            f"ts={float(pose.ts or 0.0):.3f}"
        )

__all__ = [
    "TargetStandoffBehaviorConfig",
    "TargetStandoffBehaviorModule",
]
