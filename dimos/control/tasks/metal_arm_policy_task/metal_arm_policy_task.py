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

"""Coordinator-owned task for the trained dual Metal bimanual PPO policy.

The task is deliberately small: it owns the 12 arm joints, samples measured
feedback and intent from the coordinator clock, runs the pure runtime at the
checkpoint's 20 Hz, and stops issuing commands the moment its session is no
longer valid. It never switches to IK, never replays a dropped command and
never starts a session without fresh, complete feedback and a fresh goal.

Activation is explicit. ``engage()`` only *requests* a session; the next
coordinator tick opens one when every condition holds, seeding the command
state from the measured joints so an engagement at an arbitrary pose does not
jump toward the training home. A release, fault, timeout, preemption or e-stop
closes the session and clears the pending request, so recovery always needs a
new goal and a new ``engage()``.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
import threading
import time
from typing import TYPE_CHECKING, Any, NamedTuple, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.hardware.manipulators.spec import ControlMode
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.manipulators.dual_makermods_metal.policy.contract import (
    JOINT_ORDER,
    PolicyManifest,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.runtime import (
    MetalArmPolicyRuntime,
)
from dimos.robot.manipulators.dual_makermods_metal.policy.targets import (
    ControllerPose,
    DualHandGoal,
    HandGoal,
    WebXRGoalMapper,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.control.connection_source import ConnectionSource
    from dimos.control.coordinator import TaskConfig
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.teleop.webxr.controller_types import Buttons

logger = setup_logger()


class PolicyRuntime(Protocol):
    """The slice of :class:`MetalArmPolicyRuntime` the task depends on."""

    manifest: PolicyManifest

    def reset(
        self, q_measured: NDArray[np.float32] | NDArray[np.float64] | None = None
    ) -> None: ...

    def observe(
        self,
        q: NDArray[np.float32] | NDArray[np.float64],
        dq: NDArray[np.float32] | NDArray[np.float64],
        goal_pose24: NDArray[np.float32] | NDArray[np.float64],
    ) -> NDArray[np.float32]: ...

    def act(self) -> NDArray[np.float32]: ...

    @property
    def issued(self) -> NDArray[np.float32]: ...


class StationKinematics(Protocol):
    """Station-frame FK used to anchor teleoperation goals."""

    def pose24(self, q: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float64]: ...


class SessionState(str, Enum):
    """Lifecycle of one policy session."""

    DISENGAGED = "disengaged"
    ACTIVE = "active"


class _PolicyTick(NamedTuple):
    """What one coordinator tick must run outside the session lock.

    ``epoch`` is the session generation the tick was prepared under; the
    inference result is dropped unless it is still current when it returns.
    """

    epoch: int
    q: NDArray[np.float32]
    dq: NDArray[np.float32]
    goal: NDArray[np.float64]
    t_now: float
    reset: bool


class PolicyTaskFault(str, Enum):
    """Why the last session ended; kept for diagnostics, never auto-cleared."""

    NONE = "none"
    FEEDBACK = "feedback"
    INTENT = "intent"
    INFERENCE = "inference"
    PREEMPTED = "preempted"
    ESTOP = "estop"
    RELEASED = "released"
    STOPPED = "stopped"
    RESET = "reset"


@dataclass(frozen=True)
class MetalArmPolicyTaskConfig:
    """Everything the task needs to open a session and run the runtime.

    ``joint_names`` must be the coordinator's names for the checkpoint's
    12-joint order. The task claims exactly these joints; each jaw stays with
    its own gripper task.
    """

    joint_names: tuple[str, ...]
    model_dir: str
    manifest_path: str
    priority: int = 40
    #: Complete q/dq must be observable when a session opens, and every active
    #: tick re-checks the injected connection freshness source.
    feedback_timeout_s: float = 0.2
    #: A goal older than this cannot open a session. Once open, an absolute
    #: goal stays valid; ``goal_timeout_s`` adds a staleness bound for streams.
    intent_timeout_s: float = 0.5
    goal_timeout_s: float | None = None
    teleop: bool = False
    dry_run: bool = False

    def __post_init__(self) -> None:
        if len(self.joint_names) != len(JOINT_ORDER):
            raise ValueError(
                f"metal_arm_policy requires {len(JOINT_ORDER)} joint names "
                f"({', '.join(JOINT_ORDER)}), got {len(self.joint_names)}"
            )
        suffixes = tuple(name.rsplit("/", 1)[-1] for name in self.joint_names)
        if suffixes != JOINT_ORDER:
            raise ValueError(
                "joint_names must carry the checkpoint joint order "
                f"{list(JOINT_ORDER)}, got {list(suffixes)}"
            )
        robots = {name.split("/", 1)[0] for name in self.joint_names if "/" in name}
        if len(robots) != 1:
            raise ValueError("all policy joints must belong to one robot, e.g. 'arm/<joint>'")
        if self.feedback_timeout_s <= 0.0:
            raise ValueError("feedback_timeout_s must be positive")
        if self.intent_timeout_s <= 0.0:
            raise ValueError("intent_timeout_s must be positive")
        if self.goal_timeout_s is not None and self.goal_timeout_s <= 0.0:
            raise ValueError("goal_timeout_s must be positive when set")


class MetalArmPolicyTask(BaseControlTask):
    """Run the accepted Actor at 20 Hz inside the coordinator's tick loop."""

    def __init__(
        self,
        name: str,
        config: MetalArmPolicyTaskConfig,
        *,
        runtime: PolicyRuntime,
        kinematics: StationKinematics,
        feedback_fresh: Callable[[], bool] | None = None,
        teleop_mapper: WebXRGoalMapper | None = None,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        manifest = runtime.manifest
        if manifest.policy_hz <= 0.0:
            raise ValueError("policy manifest has a non-positive policy_hz")
        self._name = name
        self._config = config
        self._runtime = runtime
        self._kinematics = kinematics
        self._feedback_fresh = feedback_fresh or _always_fresh
        self._joint_names = list(config.joint_names)
        self._joint_set = frozenset(config.joint_names)
        self._robot = config.joint_names[0].split("/", 1)[0]
        self._period_s = 1.0 / manifest.policy_hz
        self._dry_run = config.dry_run
        # Session state: only ever held for short, non-blocking sections.
        self._lock = threading.Lock()
        # Runtime ownership: one observe/act (or reset) at a time. This lock
        # is never taken while holding ``_lock``, so release/e-stop never wait
        # for a blocking actor.
        self._inference_lock = threading.Lock()
        # Monotonic clock used to stamp inputs on receipt. In deployment it is
        # the coordinator's ``time.perf_counter``; tests inject a fake one.
        self._clock = clock

        self._session: SessionState = SessionState.DISENGAGED
        self._fault = PolicyTaskFault.NONE
        self._epoch = 0
        self._estop = False

        self._goal: NDArray[np.float64] | None = None
        self._goal_received_at: float | None = None
        self._engage_requested = False
        self._engage_requested_at: float | None = None

        self._last_command: list[float] | None = None
        self._next_tick: float | None = None
        self._policy_ticks = 0

        self._mapper = teleop_mapper
        if config.teleop and self._mapper is None:
            self._mapper = WebXRGoalMapper()
        self._controller_poses: dict[str, tuple[ControllerPose, float]] = {}
        self._grips = (False, False)
        self._buttons_time: float | None = None

    # ----------------------------------------------------------- coordinator API
    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        """Claim the 12 arm joints only; jaws belong to the gripper tasks."""
        return ResourceClaim(
            joints=self._joint_set,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Active while a session runs *or* a requested session may still open.

        The coordinator only calls ``compute`` for active tasks, so a pending
        ``engage()`` must keep the task in the loop until it can decide.
        """
        with self._lock:
            return self._session is SessionState.ACTIVE or self._engage_requested

    def measured_frame_poses(self, state: CoordinatorState) -> dict[str, PoseStamped]:
        """No published TF frames: the TCP sites are internal policy state."""
        return {}

    # ------------------------------------------------------------- goal entries
    def set_goal_pose24(self, pose24: list[float], t_now: float | None = None) -> bool:
        """Set absolute paired TCP goals in the trained 24-value layout."""
        goal = DualHandGoal.from_pose24(np.asarray(pose24, dtype=np.float64))
        return self._set_goal(goal.to_pose24(), t_now)

    def set_goal(
        self,
        left_xyz: list[float],
        left_quaternion_xyzw: list[float],
        right_xyz: list[float],
        right_quaternion_xyzw: list[float],
        t_now: float | None = None,
    ) -> bool:
        """Set absolute paired TCP goals from positions and (x, y, z, w) quaternions."""
        goal = DualHandGoal(
            HandGoal.from_quaternion(
                np.asarray(left_xyz, dtype=np.float64), _quaternion(left_quaternion_xyzw)
            ),
            HandGoal.from_quaternion(
                np.asarray(right_xyz, dtype=np.float64), _quaternion(right_quaternion_xyzw)
            ),
        )
        return self._set_goal(goal.to_pose24(), t_now)

    def _set_goal(self, pose24: NDArray[np.float64], t_now: float | None) -> bool:
        with self._lock:
            if self._estop:
                return False
            self._goal = np.asarray(pose24, dtype=np.float64).reshape(24).copy()
            # Stamp receipt, never first consumption: a goal that waited in a
            # queue cannot become fresh when a tick finally looks at it.
            self._goal_received_at = t_now if t_now is not None else self._clock()
            return True

    def engage(self, t_now: float | None = None) -> bool:
        """Request a session; the next tick opens it only if every check passes.

        ``t_now`` must be the coordinator's monotonic timestamp for the
        request (the same clock as ``CoordinatorState.t_now``). When omitted
        the task stamps receipt with its own monotonic clock, which is that
        same ``time.perf_counter`` domain in deployment.
        """
        with self._lock:
            if self._estop:
                return False
            if self._goal is None and not self._config.teleop:
                return False
            self._engage_requested = True
            self._engage_requested_at = t_now if t_now is not None else self._clock()
            return True

    def release(self, t_now: float | None = None) -> bool:
        """Close the session and drop pending intent; a new engage is required."""
        del t_now
        with self._lock:
            self._invalidate_locked(PolicyTaskFault.RELEASED)
            return True

    def disarm(self, t_now: float | None = None) -> bool:
        """Coordinator activation hook: same effect as :meth:`release`."""
        return self.release()

    def set_dry_run(self, enabled: bool) -> None:
        """Run the runtime but withhold commands, for pre-go inspection."""
        with self._lock:
            self._dry_run = bool(enabled)

    def set_estop(self, estopped: bool) -> None:
        """Latch or clear the emergency stop; clearing never resumes a session."""
        with self._lock:
            if estopped:
                self._estop = True
                self._invalidate_locked(PolicyTaskFault.ESTOP)
            else:
                self._estop = False

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        """Drop runtime state after a discontinuity such as a simulator respawn.

        A reset is not an engagement: even with ``reactivate=True`` the task
        asks for a new session, which still needs a fresh goal and complete
        feedback at the next tick. This keeps reset and arbitrary-pose
        engagement separate.
        """
        with self._lock:
            had_state = (
                self._session is SessionState.ACTIVE
                or self._last_command is not None
                or self._goal is not None
                or self._engage_requested
            )
            saved_goal = self._goal
            saved_at = self._goal_received_at
            self._invalidate_locked(PolicyTaskFault.RESET)
            if reactivate and not self._estop and saved_goal is not None:
                # Keep the intent for a reactivation, but still require complete
                # feedback at the next tick and seed from the measured pose.
                self._goal = saved_goal
                self._goal_received_at = saved_at
                self._engage_requested = True
                self._engage_requested_at = self._clock()
            return had_state

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """A higher-priority task took arm joints: never keep the old session."""
        if joints & self._joint_set:
            with self._lock:
                self._invalidate_locked(PolicyTaskFault.PREEMPTED)
            logger.warning(
                "MetalArmPolicyTask preempted and session invalidated",
                task=self._name,
                by_task=by_task,
                joints=sorted(joints),
            )

    def state_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "task": self._name,
                "session": self._session.value,
                "fault": self._fault.value,
                "estop": self._estop,
                "epoch": self._epoch,
                "engage_requested": self._engage_requested,
                "goal_set": self._goal is not None,
                "policy_ticks": self._policy_ticks,
                "last_command": None if self._last_command is None else list(self._last_command),
                "issued": self._runtime.issued.tolist(),
                "teleop": self._config.teleop,
                "mapper_engaged": bool(self._mapper and self._mapper.engaged),
            }

    def start(self) -> None:
        """Inert until :meth:`engage`; start never opens a session."""

    def stop(self) -> None:
        with self._lock:
            self._invalidate_locked(PolicyTaskFault.STOPPED)

    # -------------------------------------------------------------- teleop entry
    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        """Deadman: both grips held requests engagement; either release closes."""
        if not self._config.teleop:
            return False
        grips = (bool(msg.left_grip), bool(msg.right_grip))
        with self._lock:
            if self._estop:
                return True
            self._grips = grips
            self._buttons_time = t_now
            if not (grips[0] and grips[1]):
                if self._session is SessionState.ACTIVE or self._engage_requested:
                    self._invalidate_locked(PolicyTaskFault.RELEASED)
                return True
            if self._session is SessionState.DISENGAGED and not self._engage_requested:
                self._engage_requested = True
                self._engage_requested_at = t_now
        return True

    def on_left_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the left controller pose; it only matters while mapping is engaged."""
        return self._on_controller_pose("left", pose, t_now)

    def on_right_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the right controller pose; it only matters while mapping is engaged."""
        return self._on_controller_pose("right", pose, t_now)

    @staticmethod
    def _controller_pose(pose: Pose | PoseStamped) -> ControllerPose:
        quaternion = pose.orientation
        return ControllerPose.from_pose(
            tuple(pose.position),
            (
                float(quaternion.x),
                float(quaternion.y),
                float(quaternion.z),
                float(quaternion.w),
            ),
        )

    def _on_controller_pose(self, side: str, pose: Pose | PoseStamped, t_now: float) -> bool:
        if not self._config.teleop:
            return False
        sample = self._controller_pose(pose)
        with self._lock:
            if self._estop:
                return True
            self._controller_poses[side] = (sample, t_now)
            if self._mapper is not None and self._mapper.engaged:
                self._refresh_teleop_goal_locked(t_now)
        return True

    def _input_timeout_s(self) -> float:
        """Freshness bound for WebXR samples: the streamed-goal timeout."""
        timeout = self._config.goal_timeout_s
        return timeout if timeout is not None else self._config.intent_timeout_s

    def _teleop_input_ages_locked(self, now: float) -> tuple[float, float, float] | None:
        """Ages of buttons and both poses, or ``None`` if any source is missing."""
        left = self._controller_poses.get("left")
        right = self._controller_poses.get("right")
        if self._buttons_time is None or left is None or right is None:
            return None
        return (now - self._buttons_time, now - left[1], now - right[1])

    def _teleop_inputs_fresh_locked(self, now: float) -> bool:
        """Every WebXR source must be fresh: both poses, buttons and both grips.

        One live controller can never reauthorize a stale one, and a held grip
        cannot outlive its button message.
        """
        ages = self._teleop_input_ages_locked(now)
        if ages is None or not (self._grips[0] and self._grips[1]):
            return False
        bound = self._input_timeout_s()
        return all(age <= bound for age in ages)

    def _refresh_teleop_goal_locked(self, now: float) -> None:
        mapper = self._mapper
        if mapper is None or not mapper.engaged:
            return
        if not self._teleop_inputs_fresh_locked(now):
            # Never rebuild a goal from a mixture of live and stale sources.
            return
        left = self._controller_poses.get("left")
        right = self._controller_poses.get("right")
        if left is None or right is None:
            return
        try:
            goal = mapper.update(left[0], right[0])
        except ValueError:
            return
        self._goal = goal.to_pose24()
        # The older source bounds the goal's age; a live side cannot mask it.
        self._goal_received_at = min(left[1], right[1])

    # ------------------------------------------------------------------ compute
    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Decide this tick under the short session lock, then run the actor.

        The actor is never called with the session lock held: release, e-stop,
        preemption and session invalidation stay responsive while inference is
        in flight, and their generation change makes the late result unusable.
        """
        with self._lock:
            plan: _PolicyTick | JointCommandOutput | None
            if self._session is SessionState.DISENGAGED:
                plan = self._prepare_activation_locked(state)
            else:
                plan = self._prepare_tick_locked(state)
        if plan is None or isinstance(plan, JointCommandOutput):
            return plan
        return self._run_policy(plan)

    def _read_feedback(
        self, state: CoordinatorState
    ) -> tuple[NDArray[np.float32], NDArray[np.float32]] | None:
        q = np.empty(len(self._joint_names), dtype=np.float32)
        dq = np.empty(len(self._joint_names), dtype=np.float32)
        for index, joint in enumerate(self._joint_names):
            position = state.joints.get_position(joint)
            velocity = state.joints.get_velocity(joint)
            if position is None or velocity is None:
                return None
            q[index] = position
            dq[index] = velocity
        if not np.isfinite(q).all() or not np.isfinite(dq).all():
            return None
        return q, dq

    def _prepare_activation_locked(self, state: CoordinatorState) -> _PolicyTick | None:
        """Open a session if all preconditions hold; the actor runs outside the lock."""
        if not self._engage_requested or self._estop:
            return None
        now = state.t_now
        if (
            self._engage_requested_at is not None
            and now - self._engage_requested_at > self._config.intent_timeout_s
        ):
            # An old engage request is not reauthorization; drop it explicitly.
            self._engage_requested = False
            self._engage_requested_at = None
            self._fault = PolicyTaskFault.INTENT
            return None
        if self._goal is None and not self._config.teleop:
            return None
        if self._goal is not None:
            if self._goal_received_at is None:  # defensive: every goal is stamped
                self._goal_received_at = self._clock()
            if now - self._goal_received_at > self._config.intent_timeout_s:
                self._engage_requested = False
                self._engage_requested_at = None
                self._fault = PolicyTaskFault.INTENT
                return None
        feedback = self._read_feedback(state)
        if feedback is None or not self._feedback_fresh():
            return None
        q, dq = feedback
        goal = self._goal
        if self._config.teleop:
            ages = self._teleop_input_ages_locked(now)
            if ages is None or not (self._grips[0] and self._grips[1]):
                # An incomplete deadman is not a fault yet: wait for it, and
                # let the engage request's own age bound expire it.
                return None
            if any(age > self._input_timeout_s() for age in ages):
                self._invalidate_locked(PolicyTaskFault.INTENT)
                return None
            anchored = self._anchor_teleop_goal_locked(q, now)
            if anchored is None:
                return None
            goal = anchored
        if goal is None:
            return None

        self._session = SessionState.ACTIVE
        self._fault = PolicyTaskFault.NONE
        self._epoch += 1
        self._engage_requested = False
        self._engage_requested_at = None
        self._last_command = None
        self._policy_ticks = 0
        self._next_tick = now
        q_home = np.asarray(self._runtime.manifest.home, dtype=np.float32)
        logger.info(
            "MetalArmPolicyTask session opened",
            task=self._name,
            epoch=self._epoch,
            non_home_engagement=bool(np.abs(q - q_home).max() > 1e-3),
            dq_max=float(np.abs(dq).max()),
        )
        return _PolicyTick(self._epoch, q, dq, goal, now, True)

    def _anchor_teleop_goal_locked(
        self, q: NDArray[np.float32], t_now: float
    ) -> NDArray[np.float64] | None:
        mapper = self._mapper
        left = self._controller_poses.get("left")
        right = self._controller_poses.get("right")
        if mapper is None or left is None or right is None:
            return None
        if not self._teleop_inputs_fresh_locked(t_now):
            return None
        try:
            current = DualHandGoal.from_pose24(self._kinematics.pose24(q))
            mapper.engage(current, left[0], right[0])
        except (ValueError, RuntimeError):
            return None
        anchor = current.to_pose24()
        self._goal = anchor
        self._goal_received_at = min(left[1], right[1])
        return anchor

    def _prepare_tick_locked(
        self, state: CoordinatorState
    ) -> _PolicyTick | JointCommandOutput | None:
        """Check freshness/intent and either hold or request one policy tick."""
        feedback = self._read_feedback(state)
        if feedback is None or not self._feedback_fresh():
            self._invalidate_locked(PolicyTaskFault.FEEDBACK)
            return None
        q, dq = feedback
        now = state.t_now

        if self._config.teleop:
            # Checked on every coordinator tick, including between inferences.
            if not self._teleop_inputs_fresh_locked(now):
                self._invalidate_locked(PolicyTaskFault.INTENT)
                return None
            self._refresh_teleop_goal_locked(now)

        timeout = self._config.goal_timeout_s
        if (
            timeout is not None
            and self._goal_received_at is not None
            and now - self._goal_received_at > timeout
        ):
            self._invalidate_locked(PolicyTaskFault.INTENT)
            return None

        goal = self._goal
        if goal is None:
            self._invalidate_locked(PolicyTaskFault.INTENT)
            return None

        next_tick = self._next_tick
        if next_tick is not None and now + 1e-9 < next_tick:
            return self._hold_locked()
        return _PolicyTick(self._epoch, q, dq, goal, now, False)

    def _run_policy(self, plan: _PolicyTick) -> JointCommandOutput | None:
        """Run the actor under runtime ownership, then hand off only if still current."""
        with self._inference_lock:
            with self._lock:
                if self._epoch != plan.epoch or self._session is not SessionState.ACTIVE:
                    return None
            try:
                if plan.reset:
                    self._runtime.reset(plan.q)
                    with self._lock:
                        # A release/e-stop/preemption may have landed during reset.
                        if self._epoch != plan.epoch or self._session is not SessionState.ACTIVE:
                            return None
                self._runtime.observe(plan.q, plan.dq, plan.goal)
                command = self._runtime.act()
            except Exception:
                logger.exception("MetalArmPolicyTask inference failed", task=self._name)
                with self._lock:
                    if self._epoch == plan.epoch:
                        self._invalidate_locked(PolicyTaskFault.INFERENCE)
                return None

        # Output handoff boundary: a command from an invalidated or superseded
        # session is dropped, never emitted and never replayed.
        with self._lock:
            if self._epoch != plan.epoch or self._session is not SessionState.ACTIVE:
                return None
            if not self._feedback_fresh():
                self._invalidate_locked(PolicyTaskFault.FEEDBACK)
                return None

            values = np.asarray(command, dtype=np.float32).reshape(-1)
            if values.shape != (len(self._joint_names),) or not np.isfinite(values).all():
                logger.error("MetalArmPolicyTask produced an invalid command", task=self._name)
                self._invalidate_locked(PolicyTaskFault.INFERENCE)
                return None

            self._policy_ticks += 1
            values_list = values.tolist()
            self._last_command = values_list
            next_tick = self._next_tick
            now = plan.t_now
            scheduled = (next_tick if next_tick is not None else now) + self._period_s
            if scheduled <= now:
                scheduled = now + self._period_s
            self._next_tick = scheduled

            if self._dry_run:
                return None
            return JointCommandOutput(
                joint_names=list(self._joint_names),
                positions=list(values_list),
                mode=ControlMode.SERVO_POSITION,
            )

    def _hold_locked(self) -> JointCommandOutput | None:
        if self._dry_run or self._last_command is None:
            return None
        return JointCommandOutput(
            joint_names=list(self._joint_names),
            positions=list(self._last_command),
            mode=ControlMode.SERVO_POSITION,
        )

    def _invalidate_locked(self, reason: PolicyTaskFault) -> None:
        was_active = self._session is SessionState.ACTIVE
        self._session = SessionState.DISENGAGED
        if reason is not PolicyTaskFault.NONE:
            self._fault = reason
        if was_active:
            self._epoch += 1
        self._goal = None
        self._goal_received_at = None
        self._engage_requested = False
        self._engage_requested_at = None
        self._last_command = None
        self._next_tick = None
        self._policy_ticks = 0
        if self._config.teleop:
            # A new session needs new samples: never replay the ones that were
            # live when this one was invalidated.
            self._controller_poses.clear()
            self._grips = (False, False)
            self._buttons_time = None
        if self._mapper is not None:
            self._mapper.release()


class MetalArmPolicyTaskParams(BaseConfig):
    """Serialized task parameters carried by ``TaskConfig.params``."""

    model_dir: str
    manifest_path: str
    feedback_timeout_s: float = 0.2
    intent_timeout_s: float = 0.5
    goal_timeout_s: float | None = None
    teleop: bool = False
    dry_run: bool = False


def create_task(
    cfg: TaskConfig,
    hardware: Mapping[str, object],
    connections: Mapping[str, ConnectionSource] | None = None,
) -> MetalArmPolicyTask:
    """Build the task from its registry configuration.

    The manifest is the deployment contract: unknown schema keys, wrong joint
    order and artifact hash mismatches fail here, before the coordinator runs.
    """
    del hardware
    params = MetalArmPolicyTaskParams.model_validate(cfg.params)
    manifest = PolicyManifest.load(params.manifest_path)
    runtime = MetalArmPolicyRuntime(manifest, params.model_dir)

    robot = cfg.joint_names[0].split("/", 1)[0] if cfg.joint_names else ""
    sources = [source for name, source in (connections or {}).items() if name == robot]
    freshness: Callable[[], bool] | None = None
    if sources:

        def _fresh() -> bool:
            return all(bool(source.ready_for_control()) for source in sources)

        freshness = _fresh

    config = MetalArmPolicyTaskConfig(
        joint_names=tuple(cfg.joint_names),
        model_dir=params.model_dir,
        manifest_path=params.manifest_path,
        priority=cfg.priority,
        feedback_timeout_s=params.feedback_timeout_s,
        intent_timeout_s=params.intent_timeout_s,
        goal_timeout_s=params.goal_timeout_s,
        teleop=params.teleop,
        dry_run=params.dry_run,
    )
    return MetalArmPolicyTask(
        cfg.name,
        config,
        runtime=runtime,
        kinematics=runtime.kinematics,
        feedback_fresh=freshness,
    )


def _always_fresh() -> bool:
    return True


def _quaternion(values: list[float]) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError("quaternion must have 4 values (x, y, z, w)")
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))
