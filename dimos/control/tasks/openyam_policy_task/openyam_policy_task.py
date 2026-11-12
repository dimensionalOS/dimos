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

"""Coordinator-owned task for the trained OpenYAM history policy.

The task owns six arm joints, samples measured feedback on the coordinator
clock, and runs the pure runtime at 30 Hz. What it does *not* do is as
important as what it does:

* it never opens a session without complete, fresh feedback and a fresh
  reference input, and it never starts one at an arbitrary pose - the Quest
  reference is anchored to the measured pose, so engagement adds no motion;
* it never invents a reference: a refused IK step, a stale controller sample or
  a wrong ``dt`` closes the session and holds instead of zero-filling a row;
* it never keeps emitting after a release, e-stop, preemption, feedback loss or
  the end of the trained episode.

The trained decode law (``q_des = q_ref + 0.1 * raw``, no clamp) lives in the
runtime; this task only decides when a tick may run and what reference row it
sees.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from enum import Enum
import time
import traceback
from typing import TYPE_CHECKING, Any, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import model_validator

from dimos.control.task import CoordinatorState, JointCommandOutput
from dimos.control.tasks.policy_execution import PolicyTask, PolicyTick
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.manipulators.openyam.policy.contract import (
    EPISODE_TICKS,
    JOINT_ORDER,
    PolicyContractError,
    load_manifest,
)
from dimos.robot.manipulators.openyam.policy.reference import ReferenceRow
from dimos.robot.manipulators.openyam.policy.runtime import OpenYamPolicyRuntime
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.control.connection_source import ConnectionSource
    from dimos.control.coordinator import TaskConfig
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.teleop.webxr.controller_types import Buttons

logger = setup_logger()


class PolicyTaskFault(str, Enum):
    """Why the last session ended; kept for diagnostics, never auto-cleared."""

    NONE = "none"
    FEEDBACK = "feedback"
    INTENT = "intent"
    REFERENCE = "reference"
    INFERENCE = "inference"
    PREEMPTED = "preempted"
    ESTOP = "estop"
    RELEASED = "released"
    STOPPED = "stopped"
    RESET = "reset"
    EPISODE = "episode"


class ReferenceSource(Protocol):
    """What the task needs from a reference generator (Quest or recorded clip)."""

    @property
    def anchored(self) -> bool: ...

    def anchor(self, q_measured: NDArray[np.float32], pose: Any, t_now: float) -> ReferenceRow: ...

    def update(self, pose: Any, t_now: float, *, tick_index: int) -> ReferenceRow: ...

    def release(self) -> None: ...


class OpenYamPolicyTaskConfig(BaseConfig):
    """Everything the task needs to open a session and run the runtime."""

    joint_names: tuple[str, ...]
    model_dir: str
    manifest_path: str
    priority: int = 40
    feedback_timeout_s: float = 0.2
    intent_timeout_s: float = 0.5
    #: A controller sample older than this cannot open or sustain a session.
    input_timeout_s: float = 0.25
    #: ``teleop`` drives the reference from WebXR, ``clip`` replays a recorded
    #: reference clip for the training-chain comparison.
    reference_source: str = "teleop"
    clip_path: str | None = None
    clip_anchor_index: int | None = None
    #: Run the actor every N coordinator ticks instead of on a wall-clock
    #: deadline. A deadline is quantized to coordinator ticks anyway, so when
    #: the two rates are commensurate a fixed decimation is the only cadence
    #: that does not slip a tick; ``None`` uses the manifest's policy rate.
    every_n_ticks: int | None = None
    max_episode_ticks: int | None = EPISODE_TICKS
    dry_run: bool = False

    @model_validator(mode="after")
    def _validate(self) -> OpenYamPolicyTaskConfig:
        if len(self.joint_names) != len(JOINT_ORDER):
            raise ValueError(
                f"openyam_policy requires {len(JOINT_ORDER)} joint names "
                f"({', '.join(JOINT_ORDER)}), got {len(self.joint_names)}"
            )
        suffixes = tuple(name.rsplit("/", 1)[-1] for name in self.joint_names)
        if suffixes != JOINT_ORDER:
            raise ValueError(
                f"joint_names must carry the checkpoint joint order {list(JOINT_ORDER)}, "
                f"got {list(suffixes)}"
            )
        robots = {name.split("/", 1)[0] for name in self.joint_names if "/" in name}
        if len(robots) != 1:
            raise ValueError("all policy joints must belong to one robot, e.g. 'arm/yam_joint1'")
        if self.reference_source not in ("teleop", "clip"):
            raise ValueError("reference_source must be 'teleop' or 'clip'")
        if self.reference_source == "clip" and not self.clip_path:
            raise ValueError("reference_source='clip' requires clip_path")
        for name in ("feedback_timeout_s", "intent_timeout_s", "input_timeout_s"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.every_n_ticks is not None and self.every_n_ticks < 1:
            raise ValueError("every_n_ticks must be >= 1 when set")
        if self.max_episode_ticks is not None and self.max_episode_ticks <= 0:
            raise ValueError("max_episode_ticks must be positive when set")
        return self


class OpenYamPolicyTask(PolicyTask):
    """Run the exported OpenYAM actor at 30 Hz inside the coordinator tick loop."""

    def __init__(
        self,
        name: str,
        config: OpenYamPolicyTaskConfig,
        *,
        runtime: OpenYamPolicyRuntime,
        reference: ReferenceSource,
        feedback_fresh: Callable[[], bool] | None = None,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        manifest = runtime.manifest
        if manifest.timing.policy_hz <= 0.0:
            raise ValueError("policy manifest has a non-positive policy_hz")
        self._config = config
        self._runtime = runtime
        self._reference = reference
        self._feedback_fresh = feedback_fresh or _always_fresh
        self._robot = config.joint_names[0].split("/", 1)[0]
        cadence: dict[str, Any] = (
            {"every_n_ticks": config.every_n_ticks}
            if config.every_n_ticks is not None
            else {"hz": manifest.timing.policy_hz}
        )
        super().__init__(
            name,
            config.joint_names,
            priority=config.priority,
            dry_run=config.dry_run,
            **cadence,
        )
        self._clock = clock
        self._fault = PolicyTaskFault.NONE
        self._estop = False
        self._pose: Pose | PoseStamped | None = None
        self._pose_at: float | None = None
        self._buttons_at: float | None = None
        self._deadman = False
        #: A session that ended for any reason needs a released deadman before
        #: another one may open, so a held grip can never re-arm in a loop.
        self._rearm_required = False
        self._last_reference: ReferenceRow | None = None

    # -------------------------------------------------------------- operator
    @property
    def fault(self) -> PolicyTaskFault:
        return self._fault

    def on_right_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the latest right controller pose; stamped at receipt, never at use."""
        with self._lock:
            if self._estop:
                return True
            self._pose = pose
            self._pose_at = float(t_now)
            return True

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        """Right grip is the deadman: hold requests a session, release closes it."""
        with self._lock:
            if self._estop:
                return True
            self._deadman = bool(msg.right_grip)
            self._buttons_at = float(t_now)
            if not self._deadman:
                self._rearm_required = False
                if self.session_active or self._session_requested:
                    self._invalidate_locked(PolicyTaskFault.RELEASED)
                return True
            if not self._rearm_required and not self.session_active and not self._session_requested:
                self.request_session(float(t_now))
            return True

    def _input_fresh_locked(self, now: float, timeout: float) -> bool:
        if self._pose_at is None or self._buttons_at is None:
            return False
        return (
            self._deadman and now - self._pose_at <= timeout and now - self._buttons_at <= timeout
        )

    def _pose_locked(self) -> Pose | PoseStamped:
        if self._pose is None:
            raise PolicyContractError("no controller pose has been received")
        return self._pose

    # --------------------------------------------------------------- session
    def engage(self, t_now: float | None = None) -> bool:
        """Request a session; the next tick opens it only if every check passes."""
        with self._lock:
            if self._estop:
                return False
            if self._config.reference_source == "teleop" and self._pose is None:
                return False
            self.request_session(t_now if t_now is not None else self._clock())
            return True

    def release(self, t_now: float | None = None) -> bool:
        del t_now
        with self._lock:
            self._invalidate_locked(PolicyTaskFault.RELEASED)
            return True

    def disarm(self, t_now: float | None = None) -> bool:
        return self.release()

    def set_estop(self, estopped: bool) -> None:
        """Latch or clear the emergency stop; clearing never resumes a session."""
        with self._lock:
            if estopped:
                self._estop = True
                self._invalidate_locked(PolicyTaskFault.ESTOP)
            else:
                self._estop = False

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        """Drop session state after a discontinuity such as a simulator respawn."""
        with self._lock:
            had_state = self.session_active or self._session_requested
            self._invalidate_locked(PolicyTaskFault.RESET)
            if reactivate and not self._estop:
                self.request_session(self._clock())
            return had_state

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_set:
            with self._lock:
                self._invalidate_locked(PolicyTaskFault.PREEMPTED)
            logger.warning(
                "OpenYamPolicyTask preempted and session invalidated",
                task=self._name,
                by_task=by_task,
                joints=sorted(joints),
            )

    def state_snapshot(self) -> dict[str, Any]:
        with self._lock:
            diagnostics = getattr(self._reference, "diagnostics", None)
            row = self._last_reference
            return {
                "task": self._name,
                "session": "active" if self.session_active else "disengaged",
                "fault": self._fault.value,
                "estop": self._estop,
                "epoch": self.session_epoch,
                "engage_requested": self._session_requested,
                "deadman": self._deadman,
                "rearm_required": self._rearm_required,
                "pose_age_s": None if self._pose_at is None else self._clock() - self._pose_at,
                "policy_ticks": self.policy_ticks,
                "last_command": self.held_positions,
                "issued": None if not self.session_active else self._runtime.issued.tolist(),
                "reference_source": self._config.reference_source,
                "reference_index": None if row is None else row.index,
                "reference_time_s": None if row is None else row.time_s,
                #: Goal TCP in the *trained plant's* frame: what the actor saw.
                "goal_position": None if row is None else np.asarray(row.goal_position).tolist(),
                "goal_quaternion_wxyz": (
                    None if row is None else np.asarray(row.goal_quaternion_wxyz).tolist()
                ),
                "reference": None if diagnostics is None else diagnostics.as_dict(),
            }

    def start(self) -> None:
        """Inert until the operator engages; start never opens a session."""

    def stop(self) -> None:
        self._fault = PolicyTaskFault.STOPPED
        super().stop()

    # ---------------------------------------------------------------- compute
    def prepare(self, state: CoordinatorState) -> PolicyTick | JointCommandOutput | None:
        if not self.session_active:
            return self._prepare_activation_locked(state)
        return self._prepare_tick_locked(state)

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

    def _prepare_activation_locked(self, state: CoordinatorState) -> PolicyTick | None:
        if not self._session_requested or self._estop:
            return None
        now = state.t_now
        if (
            self._requested_at is not None
            and now - self._requested_at > self._config.intent_timeout_s
        ):
            self.clear_session_request()
            self._fault = PolicyTaskFault.INTENT
            return None
        feedback = self._read_feedback(state)
        if feedback is None or not self._feedback_fresh():
            return None
        q, dq = feedback
        teleop = self._config.reference_source == "teleop"
        if teleop and not self._input_fresh_locked(now, self._config.input_timeout_s):
            return None
        try:
            row = (
                self._reference.anchor(q, self._pose_locked(), now)
                if teleop
                else self._reference.anchor(q, None, now)
            )
        except PolicyContractError as exc:
            self._fault = PolicyTaskFault.REFERENCE
            self.clear_session_request()
            logger.warning("reference anchor refused", task=self._name, error=str(exc))
            return None

        epoch = self.begin_session(now)
        self._fault = PolicyTaskFault.NONE
        self._last_reference = row
        logger.info(
            "OpenYamPolicyTask session opened",
            task=self._name,
            epoch=epoch,
            source=self._config.reference_source,
            reference_index=row.index,
        )
        return PolicyTick(epoch, now, reset=True, payload=(q, dq, row))

    def _prepare_tick_locked(
        self, state: CoordinatorState
    ) -> PolicyTick | JointCommandOutput | None:
        feedback = self._read_feedback(state)
        if feedback is None or not self._feedback_fresh():
            self._invalidate_locked(PolicyTaskFault.FEEDBACK)
            return None
        q, dq = feedback
        now = state.t_now
        teleop = self._config.reference_source == "teleop"
        if teleop and not self._input_fresh_locked(now, self._config.input_timeout_s):
            self._invalidate_locked(PolicyTaskFault.INTENT)
            return None
        if not self._cadence_due(now):
            return self._held_command()
        if (
            self._config.max_episode_ticks is not None
            and self.policy_ticks >= self._config.max_episode_ticks
        ):
            self._invalidate_locked(PolicyTaskFault.EPISODE)
            return None
        try:
            # ``observe`` advances the runtime tick before ``act`` runs, so the
            # reference row this tick consumes is the *next* index. Its time
            # stamp must equal the newest observed frame's, exactly as in
            # training.
            next_index = self._runtime.tick + 1
            row = (
                self._reference.update(self._pose_locked(), now, tick_index=next_index)
                if teleop
                else self._reference.update(None, now, tick_index=next_index)
            )
        except PolicyContractError as exc:
            logger.warning(
                "reference refused; holding and closing the session",
                task=self._name,
                error=str(exc),
            )
            self._invalidate_locked(PolicyTaskFault.REFERENCE)
            return None
        self._last_reference = row
        return PolicyTick(self.session_epoch, now, payload=(q, dq, row))

    def reset_policy(self, tick: PolicyTick) -> None:
        q, dq, row = tick.payload
        try:
            self._runtime.reset(q, dq, row)
        except Exception as exc:
            logger.error(
                "openyam policy reset failed",
                task=self._name,
                error=repr(exc),
                detail=traceback.format_exc(),
            )
            raise

    def act(self, tick: PolicyTick) -> NDArray[np.float32]:
        q, dq, row = tick.payload
        # The reset tick's history was just seeded by ``reset``; observing again
        # would duplicate the anchor frame. Every other tick appends the frame
        # for the interval that ended, then infers for the new tick index.
        try:
            if not tick.reset:
                self._runtime.observe(q, dq)
            return self._runtime.act(row)
        except Exception as exc:
            logger.error(
                "openyam policy tick failed",
                task=self._name,
                error=repr(exc),
                detail=traceback.format_exc(),
            )
            raise

    def commit_guard(
        self, tick: PolicyTick, command: NDArray[np.float32]
    ) -> NDArray[np.float32] | None:
        """Feedback may have gone stale, or the operator may have let go mid-inference."""
        del tick
        if not self._feedback_fresh():
            self._invalidate_locked(PolicyTaskFault.FEEDBACK)
            return None
        return command

    def on_inference_error(self, tick: PolicyTick) -> None:
        del tick
        self._fault = PolicyTaskFault.INFERENCE
        self.end_session()

    def on_invalid_command(self) -> None:
        self._fault = PolicyTaskFault.INFERENCE
        self.end_session()

    def _invalidate_locked(self, reason: PolicyTaskFault) -> None:
        if reason is not PolicyTaskFault.NONE:
            self._fault = reason
        self.end_session()

    def on_session_ended(self) -> None:
        """Drop everything tied to the closed session; runs under the lock."""
        # Whatever ended the session - release, e-stop, fault, episode end or
        # preemption - the operator must let go and press again.
        self._rearm_required = True
        self._pose = None
        self._pose_at = None
        self._buttons_at = None
        self._deadman = False
        self._last_reference = None
        self._reference.release()


def _always_fresh() -> bool:
    return True


def create_task(
    cfg: TaskConfig,
    hardware: Mapping[str, object],
    connections: Mapping[str, ConnectionSource] | None = None,
) -> OpenYamPolicyTask:
    """Build the task from its registry configuration.

    The manifest is the deployment contract: a wrong joint order or a hash
    mismatch fails here, before the coordinator runs.
    """
    del hardware
    params = OpenYamPolicyTaskConfig(
        joint_names=tuple(cfg.joint_names), priority=cfg.priority, **cfg.params
    )
    manifest = load_manifest(params.manifest_path)
    runtime = OpenYamPolicyRuntime(manifest, params.model_dir)
    reference = _build_reference(params, manifest, runtime)

    robot = cfg.joint_names[0].split("/", 1)[0] if cfg.joint_names else ""
    sources = [source for name, source in (connections or {}).items() if name == robot]
    freshness: Callable[[], bool] | None = None
    if sources:

        def _fresh() -> bool:
            return all(bool(source.ready_for_control()) for source in sources)

        freshness = _fresh

    return OpenYamPolicyTask(
        cfg.name,
        params,
        runtime=runtime,
        reference=reference,
        feedback_fresh=freshness,
    )


def _build_reference(
    params: OpenYamPolicyTaskConfig, manifest: Any, runtime: OpenYamPolicyRuntime
) -> ReferenceSource:
    if params.reference_source == "clip":
        from dimos.robot.manipulators.openyam.policy.reference import (
            ClipReference,
            ReferenceClip,
        )

        assert params.clip_path is not None
        clip = ReferenceClip(params.clip_path, hz=manifest.timing.policy_hz)
        return ClipReference(
            clip, anchor_index=params.clip_anchor_index, hz=manifest.timing.policy_hz
        )

    from dimos.control.tasks.pose_target_ik import PoseTargetIKTaskConfig
    from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
    from dimos.robot.manipulators.openyam.config import make_openyam_model_config
    from dimos.robot.manipulators.openyam.policy.quest_reference import (
        TOOL_FRAME,
        QuestReferenceGenerator,
    )
    from dimos.robot.manipulators.openyam.policy.tcp import MjcfTcpFk
    from dimos.robot.manipulators.openyam.teleop_ik import OpenYamPinkPoseTargetSolver

    model = make_openyam_model_config(robot="arm")
    pose_config = PoseTargetIKTaskConfig(
        joint_names=tuple(model.joint_names),
        robot_model=model,
        target_frames=(TOOL_FRAME,),
        # The same streaming IK tuning the Quest teleoperation blueprint uses.
        pink=PinkKinematicsConfig(
            dt=manifest.tick_period_s,
            position_cost=8.0,
            orientation_cost=2.0,
            posture_cost=0.01,
            joint_limit_posture_margin=0.3,
            lm_damping=0.01,
            gain=1.0,
        ),
        max_joint_velocity_rad_s=2.0,
        max_command_tracking_error_deg=10.0,
        joint_command_filter_cutoff_hz=30.0,
    )
    solver = OpenYamPinkPoseTargetSolver(pose_config)

    plant_xml = _plant_path(manifest)
    goal_fk = MjcfTcpFk(plant_xml)
    return QuestReferenceGenerator(
        manifest,
        solver=solver,
        goal_fk=goal_fk,
        joint_names=tuple(JOINT_ORDER),
        solver_joint_names=tuple(model.joint_names),
        tool_frame=TOOL_FRAME,
        max_reference_velocity_rad_s=2.0,
    )


def _plant_path(manifest: Any):
    plant = manifest.metadata.get("plant_model") if hasattr(manifest, "metadata") else None
    if not plant:
        raise PolicyContractError(
            "manifest does not pin the training plant model (metadata.plant_model); "
            "the Quest reference generator must compute its goal in the trained TCP frame"
        )
    return plant
