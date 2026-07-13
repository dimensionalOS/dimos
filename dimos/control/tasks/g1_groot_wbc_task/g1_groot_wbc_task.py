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

"""GR00T whole-body-control task for the Unitree G1 humanoid.

Runs the two-model GR00T WBC locomotion policy (balance + walk) inside
the coordinator tick loop.  Claims the 15 legs+waist joints at high
priority; arm joints are left to lower-priority tasks in the blueprint.

Observation, action, and model-selection semantics are part of the
bundled ONNX policy contract. Changing them can drift this task away
from the behavior the policies were trained for.

"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.control.components import make_humanoid_joints
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.whole_body.spec import WholeBodyAdapter
    from dimos.msgs.geometry_msgs.Twist import Twist

logger = setup_logger()


# The 29 DDS motor names plus the per-joint kp/kd values used with the
# GR00T-trained policies. Diverging from these on real hardware risks
# instability because the ONNX models were trained against this control
# contract.
g1_joints = make_humanoid_joints("g1")
g1_legs_waist = g1_joints[:15]  # indices 0..14 - legs (12) + waist (3)
g1_arms = g1_joints[15:]  # indices 15..28 - left arm (7) + right arm (7)

G1_GROOT_KP: list[float] = [
    150.0,
    150.0,
    150.0,
    200.0,
    40.0,
    40.0,  # left leg
    150.0,
    150.0,
    150.0,
    200.0,
    40.0,
    40.0,  # right leg
    250.0,
    250.0,
    250.0,  # waist
    100.0,
    100.0,
    40.0,
    40.0,
    20.0,
    20.0,
    20.0,  # left arm
    100.0,
    100.0,
    40.0,
    40.0,
    20.0,
    20.0,
    20.0,  # right arm
]
G1_GROOT_KD: list[float] = [
    2.0,
    2.0,
    2.0,
    4.0,
    2.0,
    2.0,  # left leg
    2.0,
    2.0,
    2.0,
    4.0,
    2.0,
    2.0,  # right leg
    5.0,
    5.0,
    5.0,  # waist
    5.0,
    5.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,  # left arm
    5.0,
    5.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,  # right arm
]

# Relaxed arms-down pose. The policy treats all 14 arm defaults as zero.
# Operators can override at runtime by publishing joint targets on the
# arms via the coordinator's joint_command transport.
ARM_DEFAULT_POSE: list[float] = [0.0] * 14


# Default joint angles for all 29 G1 joints. The policy treats these as
# its zero-offset pose.
_DEFAULT_POSITIONS_29 = [
    -0.1,
    0.0,
    0.0,
    0.3,
    -0.2,
    0.0,  # left leg
    -0.1,
    0.0,
    0.0,
    0.3,
    -0.2,
    0.0,  # right leg
    0.0,
    0.0,
    0.0,  # waist
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # left arm (not driven by policy)
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # right arm (not driven by policy)
]

_SINGLE_OBS_DIM = 86
_OBS_HISTORY_LEN = 6
_NUM_ACTIONS = 15
_NUM_MOTORS = 29


def _preferred_onnx_providers() -> list[str]:
    available = ort.get_available_providers()
    providers: list[str] = []

    if "CUDAExecutionProvider" in available:
        preload_dlls = getattr(ort, "preload_dlls", None)
        if preload_dlls is not None:
            try:
                preload_dlls(cuda=True, cudnn=True, msvc=False)
            except Exception as exc:
                logger.warning(
                    "Failed to preload ONNXRuntime CUDA/cuDNN libraries",
                    error=repr(exc),
                )
        providers.append("CUDAExecutionProvider")

    providers.append("CPUExecutionProvider")
    return providers


@dataclass
class G1GrootWBCTaskConfig:
    """Configuration for the GR00T WBC task.

    Attributes:
        balance_onnx: Path to the balance ONNX model.  Used when
            ``||cmd|| <= cmd_norm_threshold``.
        walk_onnx: Path to the walk ONNX model.  Used otherwise.
        joint_names: The 15 coordinator joint names this task claims
            (legs 0-11 + waist 12-14, in DDS order).
        all_joint_names: All 29 coordinator joint names in DDS order
            (legs 0-11 + waist 12-14 + arms 15-28).  Required to build
            the observation, which feeds all 29 joint states.
        default_positions_29: Default joint angles for all 29 joints
            (DDS order).  First 15 are the policy's zero-offset pose.
        priority: Arbitration priority (higher wins).  50 is the
            recommended WBC priority per the task.py conventions.
        decimation: Run inference every N ticks.  At 500 Hz tick /
            50 Hz policy -> decimation=10.
        action_scale: Multiplier on raw policy output before adding
            defaults.
        obs_ang_vel_scale: Scale for base angular velocity in obs.
        obs_dof_pos_scale: Scale for joint position offset in obs.
        obs_dof_vel_scale: Scale for joint velocity in obs.
        cmd_scale: Per-axis scale applied to (vx, vy, wz) in obs.
        cmd_norm_threshold: ||cmd|| below this selects the balance
            model, otherwise walk.
        height_cmd: Fixed height command slot in obs.
        timeout: Seconds without a velocity command before zeroing it.
        auto_arm: Arm the policy automatically on ``start()``.  Default
            False - safe for real hardware; the blueprint sets True for
            simulation.
        auto_dry_run: Enter dry-run mode on ``start()``.  Policy still
            runs but outputs are not emitted to the adapter - useful for
            verifying on real hardware without commanding motors.
        default_ramp_seconds: Duration of the arming ramp (current pose
            -> ``default_15``) when ``arm()`` is called without an
            explicit duration. Set to 0 in simulation (no ramp needed);
            10 s on real hardware gives operators time to verify the ramp.
    """

    balance_onnx: str | Path
    walk_onnx: str | Path
    joint_names: list[str]
    all_joint_names: list[str]
    default_positions_29: list[float] = field(default_factory=lambda: list(_DEFAULT_POSITIONS_29))
    priority: int = 50
    decimation: int = 10
    action_scale: float = 0.25
    obs_ang_vel_scale: float = 0.5
    obs_dof_pos_scale: float = 1.0
    obs_dof_vel_scale: float = 0.05
    cmd_scale: tuple[float, float, float] = (2.0, 2.0, 0.5)
    cmd_norm_threshold: float = 0.05
    height_cmd: float = 0.74
    timeout: float = 1.0
    auto_arm: bool = False
    auto_dry_run: bool = False
    default_ramp_seconds: float = 10.0


class G1GrootWBCTask(BaseControlTask):
    """Runs the GR00T balance / walk ONNX policies inside the coordinator tick loop.

    Observation vector (86 dims, built each inference tick, replicates
    ``groot_wbc_backend.GrootWBCBackend._compute_obs`` verbatim):

        [0:3]    cmd_vel * cmd_scale                # scaled velocity command
        [3]      height_cmd                         # fixed slot (0.74)
        [4:7]    (0, 0, 0)                          # rpy_cmd, zeros
        [7:10]   gyro * obs_ang_vel_scale           # body-frame ang vel
        [10:13]  projected_gravity(quat)            # gravity in body frame
        [13:42]  (q_29 - default_29) * dof_pos_scale
        [42:71]  dq_29 * dof_vel_scale
        [71:86]  last_action (15 dims)

    The observation is stacked into a 6-frame history buffer (516 dims)
    before being fed to ONNX.

    Action (15 dims, legs + waist only):

        target_q_15 = action * action_scale + default_15

    Arms are NOT driven by this task - the blueprint pairs this task
    with a lower-priority servo task scoped to the 14 arm joints.
    """

    def __init__(
        self,
        name: str,
        config: G1GrootWBCTaskConfig,
        adapter: WholeBodyAdapter,
    ) -> None:
        if len(config.joint_names) != _NUM_ACTIONS:
            raise ValueError(
                f"G1GrootWBCTask '{name}' requires exactly {_NUM_ACTIONS} joint names "
                f"(legs + waist), got {len(config.joint_names)}"
            )
        if len(config.all_joint_names) != _NUM_MOTORS:
            raise ValueError(
                f"G1GrootWBCTask '{name}' requires exactly {_NUM_MOTORS} all_joint_names "
                f"(full 29-DOF G1), got {len(config.all_joint_names)}"
            )
        if len(config.default_positions_29) != _NUM_MOTORS:
            raise ValueError(
                f"G1GrootWBCTask '{name}' requires exactly {_NUM_MOTORS} "
                f"default_positions_29, got {len(config.default_positions_29)}"
            )
        if config.decimation < 1:
            raise ValueError(f"G1GrootWBCTask '{name}' requires decimation >= 1")

        self._name = name
        self._config = config
        self._adapter = adapter
        self._joint_names_list = list(config.joint_names)
        self._joint_names_set = frozenset(config.joint_names)
        self._all_joint_names = list(config.all_joint_names)

        providers = _preferred_onnx_providers()
        self._balance_session = ort.InferenceSession(str(config.balance_onnx), providers=providers)
        self._walk_session = ort.InferenceSession(str(config.walk_onnx), providers=providers)
        self._balance_input = self._balance_session.get_inputs()[0].name
        self._walk_input = self._walk_session.get_inputs()[0].name
        logger.info(
            "G1GrootWBCTask loaded ONNX models",
            task=name,
            balance=str(config.balance_onnx),
            walk=str(config.walk_onnx),
            requested_providers=providers,
            balance_providers=self._balance_session.get_providers(),
            walk_providers=self._walk_session.get_providers(),
        )

        self._default_29 = np.asarray(config.default_positions_29, dtype=np.float32)
        self._default_15 = self._default_29[:_NUM_ACTIONS]
        self._cmd_scale = np.asarray(config.cmd_scale, dtype=np.float32)

        # Inference state
        self._last_action = np.zeros(_NUM_ACTIONS, dtype=np.float32)
        self._obs_buf = np.zeros((1, _SINGLE_OBS_DIM * _OBS_HISTORY_LEN), dtype=np.float32)
        self._first_inference = True
        self._tick_count = 0
        self._last_targets: list[float] | None = None

        # Last-known-good state caches. compute() falls back to these
        # whenever a joint is missing from CoordinatorState (transient
        # packet drop, late publisher, etc) instead of substituting 0.0
        # - feeding a zero pose to the policy makes it think the robot
        # is at the URDF zero (legs straight) and command a snap-back,
        # which on real hardware tips the robot over. ``_state_seen``
        # tracks whether we've ever observed a fully-populated state;
        # until then compute() returns None rather than running on
        # half-cached defaults.
        self._cached_q_29 = self._default_29.copy()
        self._cached_dq_29 = np.zeros(_NUM_MOTORS, dtype=np.float32)
        self._cached_q_15 = self._default_15.copy()
        self._state_seen = False

        self._active = False
        self._armed = False
        self._arming = False
        self._arm_pending = False
        self._dry_run = bool(config.auto_dry_run)
        self._arming_duration = 0.0
        self._arming_start_t = 0.0
        self._ramp_start: NDArray[np.float32] | None = None
        self._last_dry_run_log_t: float = 0.0

        self._cmd_lock = threading.Lock()
        self._cmd = np.zeros(3, dtype=np.float32)
        self._last_cmd_time: float = 0.0

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names_set,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        return self._active

    def _refresh_state_caches(self, state: CoordinatorState) -> bool:
        """Pull current q/dq for the full 29 from ``CoordinatorState``.

        Updates last-known-good caches and returns True iff the full 29
        came back populated this tick. The 15 claimed-joint q cache is
        derived from the first 15 entries of the same full-state cache,
        so ramp/hold behavior and policy observations cannot diverge on
        partial packets.

        On a missing joint we keep the cached value rather than dropping
        in 0.0 - the policy interprets 0.0 as "at URDF zero / legs
        straight" and commands a recovery, which tips the robot.
        """
        all_present = True
        for i, jname in enumerate(self._all_joint_names):
            pos = state.joints.get_position(jname)
            vel = state.joints.get_velocity(jname)
            if pos is None:
                all_present = False
            else:
                self._cached_q_29[i] = pos
            if vel is None:
                all_present = False
            else:
                self._cached_dq_29[i] = vel
        self._cached_q_15[:] = self._cached_q_29[:_NUM_ACTIONS]
        if all_present:
            self._state_seen = True
        return all_present

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self._active:
            return None

        # Refresh the last-known-good state caches. If we've never seen
        # a fully-populated state and this tick is also incomplete, hold
        # off - emitting a command from defaults would snap the robot.
        fresh = self._refresh_state_caches(state)
        if not self._state_seen and not fresh:
            return None

        current_15 = self._cached_q_15.copy()

        # arm() was called - snapshot the ramp start and enter arming /
        # armed state (ramp=0 arms immediately).
        if self._arm_pending:
            self._ramp_start = current_15.copy()
            self._arming_start_t = state.t_now
            if self._arming_duration > 0.0:
                self._arming = True
                self._armed = False
                logger.info(
                    "G1GrootWBCTask arming: ramp to default_15",
                    task=self._name,
                    ramp_seconds=self._arming_duration,
                )
            else:
                self._arming = False
                self._armed = True
                self._reset_policy_state()
                logger.info("G1GrootWBCTask armed (no ramp)", task=self._name)
            self._arm_pending = False

        # Unarmed & not arming: echo current joint positions.  With the
        # component's kp/kd applied downstream, q_tgt == q_actual yields
        # pure damping (tau = -kd * dq), which mirrors the reference
        # backend's inactive "hold current pose" behaviour.
        if not self._armed and not self._arming:
            self._last_targets = current_15.tolist()
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=self._last_targets,
                mode=ControlMode.SERVO_POSITION,
            )

        # Arming: lerp ramp_start -> default_15 over arming_duration.
        if self._arming:
            assert self._ramp_start is not None
            elapsed = state.t_now - self._arming_start_t
            alpha = (
                1.0 if self._arming_duration <= 0.0 else min(1.0, elapsed / self._arming_duration)
            )
            target = self._ramp_start + alpha * (self._default_15 - self._ramp_start)
            self._last_targets = target.tolist()
            if alpha >= 1.0:
                self._arming = False
                self._armed = True
                self._reset_policy_state()
                logger.info(
                    "G1GrootWBCTask ramp complete - policy armed",
                    task=self._name,
                    mode="dry-run" if self._dry_run else "live",
                )
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=self._last_targets,
                mode=ControlMode.SERVO_POSITION,
            )

        # Armed: run the policy.  In dry-run mode we still compute (so
        # the obs buffer stays hot), but return None so no command goes
        # downstream. A throttled log line shows what WOULD have been
        # sent so operators can verify pre-go behavior.
        self._tick_count += 1

        # Decimation: only run inference every N ticks.  Between inference
        # ticks, re-emit the last target so the coordinator keeps driving
        # the joints (or nothing, in dry-run).
        if self._tick_count % self._config.decimation != 0:
            if self._dry_run or self._last_targets is None:
                return None
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=self._last_targets,
                mode=ControlMode.SERVO_POSITION,
            )

        # State was refreshed up top (with fall-back-to-last-good on
        # missing joints). Snapshot the caches now so concurrent state
        # updates don't tear the obs vector.
        q_29 = self._cached_q_29.copy()
        dq_29 = self._cached_dq_29.copy()

        # Prefer IMU from CoordinatorState (populated by the coordinator
        # each tick from every whole-body adapter); fall back to the
        # adapter-direct read if state.imu is empty (e.g. unit tests
        # that build a bare CoordinatorState). The state path is what
        # decouples this task from the WholeBodyAdapter Protocol.
        if state.imu:
            # Single whole-body adapter is the common case - take any.
            imu = next(iter(state.imu.values()))
        else:
            imu = self._adapter.read_imu()
        gyro = np.asarray(imu.gyroscope, dtype=np.float32)
        gravity = self._projected_gravity(imu.quaternion)

        # Velocity command (with timeout -> zero).
        with self._cmd_lock:
            if (
                self._config.timeout > 0.0
                and self._last_cmd_time > 0.0
                and (state.t_now - self._last_cmd_time) > self._config.timeout
            ):
                cmd = np.zeros(3, dtype=np.float32)
            else:
                cmd = self._cmd.copy()

        obs = self._build_obs(cmd=cmd, gyro=gyro, gravity=gravity, q=q_29, dq=dq_29)

        # History buffer: first inference fills all slots with the current
        # obs (warm-start); subsequent ticks roll the window.
        if self._first_inference:
            tiled = np.tile(obs, _OBS_HISTORY_LEN)
            self._obs_buf[0, :] = tiled
            self._first_inference = False
        else:
            self._obs_buf[0, : _SINGLE_OBS_DIM * (_OBS_HISTORY_LEN - 1)] = self._obs_buf[
                0, _SINGLE_OBS_DIM:
            ]
            self._obs_buf[0, _SINGLE_OBS_DIM * (_OBS_HISTORY_LEN - 1) :] = obs

        # Model selection: balance when near-stationary, walk otherwise.
        cmd_norm = float(np.linalg.norm(cmd))
        if cmd_norm <= self._config.cmd_norm_threshold:
            raw = self._balance_session.run(None, {self._balance_input: self._obs_buf})[0]
        else:
            raw = self._walk_session.run(None, {self._walk_input: self._obs_buf})[0]

        action = raw[0, :_NUM_ACTIONS].astype(np.float32)
        self._last_action[:] = action

        target_q_15 = action * self._config.action_scale + self._default_15
        self._last_targets = target_q_15.tolist()

        if self._dry_run:
            # Throttled peek at the commanded pose so the operator can
            # decide whether it looks sane before flipping dry-run off.
            if (state.t_now - self._last_dry_run_log_t) >= 1.0:
                max_delta = float(np.max(np.abs(target_q_15 - current_15)))
                logger.info(
                    "G1GrootWBCTask DRY-RUN",
                    task=self._name,
                    max_dq_rad=max_delta,
                    model="walk" if cmd_norm > self._config.cmd_norm_threshold else "balance",
                )
                self._last_dry_run_log_t = state.t_now
            return None

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=self._last_targets,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names_set:
            logger.warning(
                "G1GrootWBCTask preempted", task=self._name, by_task=by_task, joints=joints
            )

    # Velocity command input

    def set_velocity_command(self, vx: float, vy: float, yaw_rate: float, t_now: float) -> None:
        """Set the (vx, vy, yaw_rate) commanded to the policy.

        Called by the coordinator's twist_command dispatcher and by
        external Python callers.  Thread-safe.
        """
        with self._cmd_lock:
            self._cmd[:] = [vx, vy, yaw_rate]
            self._last_cmd_time = t_now

    def on_twist(self, msg: Twist, t_now: float) -> bool:
        """Accept a Twist message, e.g. from an LCM cmd_vel transport."""
        self.set_velocity_command(
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
            t_now,
        )
        return True

    # Lifecycle

    def start(self) -> None:
        """Enter the coordinator tick loop.

        Starts in "active but unarmed" - compute() echoes current joint
        positions every tick, which (combined with the component's
        kp/kd) produces damping-only behaviour on real hardware (the
        robot sits quietly in dev mode).

        If ``config.auto_arm`` is set, schedules an immediate
        ``arm()`` using ``config.default_ramp_seconds`` - this is how
        the simulation blueprint bypasses the activation ritual.
        If ``config.auto_dry_run`` is set, starts in dry-run mode.
        """
        self._active = True
        self._armed = False
        self._arming = False
        self._arm_pending = False
        self._dry_run = bool(self._config.auto_dry_run)
        self._last_targets = None
        self._reset_policy_state()
        with self._cmd_lock:
            self._cmd[:] = 0.0
            self._last_cmd_time = 0.0
        logger.info("G1GrootWBCTask started", task=self._name, armed=False, dry_run=self._dry_run)
        if self._config.auto_arm:
            self.arm(self._config.default_ramp_seconds)

    def stop(self) -> None:
        """Leave the tick loop.  Re-activation resets policy state."""
        self._active = False
        self._armed = False
        self._arming = False
        self._arm_pending = False
        self._last_targets = None
        logger.info("G1GrootWBCTask stopped", task=self._name)

    # Arming / dry-run (RPC-callable via coordinator.task_invoke)

    def arm(self, ramp_seconds: float | None = None) -> bool:
        """Begin the arming sequence.

        ``compute()`` will snapshot the current joint positions on the
        next tick, lerp toward ``default_15`` over ``ramp_seconds``,
        then flip ``_armed`` true and hand control to the ONNX policy.
        A ramp of 0 arms immediately with no interpolation, which is what
        sim uses when the MJCF already starts near the policy default pose.

        Safe to call redundantly; calls while already armed or arming
        are ignored.  No-op if the task is not ``_active``.
        """
        if not self._active:
            logger.warning("G1GrootWBCTask arm() called before start(); ignoring", task=self._name)
            return False
        if self._armed:
            logger.info("G1GrootWBCTask already armed; arm() ignored", task=self._name)
            return False
        if self._arming or self._arm_pending:
            logger.info("G1GrootWBCTask arm in progress; arm() ignored", task=self._name)
            return False
        ramp = ramp_seconds if ramp_seconds is not None else self._config.default_ramp_seconds
        self._arming_duration = max(0.0, float(ramp))
        self._arm_pending = True
        logger.info(
            "G1GrootWBCTask arm requested", task=self._name, ramp_seconds=self._arming_duration
        )
        return True

    def disarm(self) -> bool:
        """Stop emitting policy outputs; fall back to hold-current-pose.

        Called either from an operator ``Disarm`` button or from
        safety watchdogs.  Resets obs history so the next ``arm()``
        starts with a clean buffer.
        """
        if not self._armed and not self._arming and not self._arm_pending:
            return False
        self._armed = False
        self._arming = False
        self._arm_pending = False
        self._ramp_start = None
        self._reset_policy_state()
        logger.info("G1GrootWBCTask disarmed (holding current pose)", task=self._name)
        return True

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        """Clear runtime policy state after a simulation discontinuity.

        ``reactivate=None`` preserves whether the task was armed/arming before
        reset. Passing ``True`` forces a clean immediate re-arm on the next
        coordinator tick, which is useful after MuJoCo respawn.
        """
        was_armed = self._armed or self._arming or self._arm_pending
        should_reactivate = was_armed if reactivate is None else bool(reactivate)

        self._armed = False
        self._arming = False
        self._arm_pending = False
        self._ramp_start = None
        self._arming_start_t = 0.0
        self._last_targets = None
        self._state_seen = False
        self._cached_q_29[:] = self._default_29
        self._cached_dq_29[:] = 0.0
        self._cached_q_15[:] = self._default_15
        self._reset_policy_state()
        with self._cmd_lock:
            self._cmd[:] = 0.0
            self._last_cmd_time = 0.0

        if self._active and should_reactivate:
            self._arming_duration = 0.0
            self._arm_pending = True

        logger.info(
            "G1GrootWBCTask runtime state reset",
            task=self._name,
            reactivate=should_reactivate,
        )
        return True

    def set_dry_run(self, enabled: bool) -> None:
        """Enable/disable dry-run.

        In dry-run the policy still runs (obs history stays hot) but
        ``compute()`` returns ``None``, so the coordinator forwards no
        command to the adapter.  Use to verify policy sanity on real
        hardware before committing motor torques.
        """
        new_val = bool(enabled)
        if new_val == self._dry_run:
            return
        self._dry_run = new_val
        self._last_dry_run_log_t = 0.0
        logger.info("G1GrootWBCTask dry_run changed", task=self._name, dry_run=new_val)

    def state_snapshot(self) -> dict[str, Any]:
        """Return the current state-machine flags for UI / telemetry."""
        return {
            "active": self._active,
            "armed": self._armed,
            "arming": self._arming,
            "arm_pending": self._arm_pending,
            "dry_run": self._dry_run,
            "arming_duration": self._arming_duration,
        }

    # Internal helpers

    def _reset_policy_state(self) -> None:
        """Clear inference state - obs history, last action, tick count."""
        self._last_action[:] = 0.0
        self._obs_buf[:] = 0.0
        self._first_inference = True
        self._tick_count = 0

    def _build_obs(
        self,
        cmd: NDArray[np.float32],
        gyro: NDArray[np.float32],
        gravity: NDArray[np.float32],
        q: NDArray[np.float32],
        dq: NDArray[np.float32],
    ) -> NDArray[np.float32]:
        """Build the 86-dim GR00T observation.  Layout matches
        ``groot_wbc_backend.py`` exactly."""
        obs = np.zeros(_SINGLE_OBS_DIM, dtype=np.float32)
        obs[0:3] = cmd * self._cmd_scale
        obs[3] = self._config.height_cmd
        obs[4:7] = 0.0
        obs[7:10] = gyro * self._config.obs_ang_vel_scale
        obs[10:13] = gravity
        obs[13:42] = (q - self._default_29) * self._config.obs_dof_pos_scale
        obs[42:71] = dq * self._config.obs_dof_vel_scale
        obs[71:86] = self._last_action
        return obs

    @staticmethod
    def _projected_gravity(quaternion: tuple[float, ...]) -> NDArray[np.float32]:
        """Project world gravity into body frame.

        Uses Unitree DDS quaternion order (w, x, y, z).  Formula matches
        ``groot_wbc_backend._get_gravity_orientation`` and is
        algebraically equivalent to the Go2 RLPolicyTask helper.
        """
        w, x, y, z = quaternion
        gx = 2.0 * (-x * z + w * y)
        gy = 2.0 * (-y * z - w * x)
        gz = -(w * w - x * x - y * y + z * z)
        return np.array([gx, gy, gz], dtype=np.float32)


class G1GrootWBCTaskParams(BaseConfig):
    model_path: str | Path
    hardware_id: str
    auto_arm: bool = False
    auto_dry_run: bool = False
    default_ramp_seconds: float = 10.0
    decimation: int | None = None


def create_task(cfg: Any, hardware: Any) -> G1GrootWBCTask:
    params = G1GrootWBCTaskParams.model_validate(cfg.params)
    hw = hardware.get(params.hardware_id) if hardware else None
    if hw is None:
        raise ValueError(
            f"G1GrootWBCTask {cfg.name!r} references unknown hardware "
            f"{params.hardware_id!r}. Declare the hardware before the task "
            f"in the blueprint config."
        )
    if not isinstance(hw, ConnectedWholeBody):
        raise TypeError(
            f"G1GrootWBCTask {cfg.name!r} requires a WHOLE_BODY hardware "
            f"component for {params.hardware_id!r}, got {type(hw).__name__}. "
            f"Set hardware_type=HardwareType.WHOLE_BODY."
        )

    model_dir = Path(params.model_path)
    kwargs: dict[str, Any] = dict(
        balance_onnx=model_dir / "balance.onnx",
        walk_onnx=model_dir / "walk.onnx",
        joint_names=cfg.joint_names,
        all_joint_names=hw.joint_names,
        priority=cfg.priority,
        auto_arm=params.auto_arm,
        auto_dry_run=params.auto_dry_run,
        default_ramp_seconds=params.default_ramp_seconds,
    )
    if params.decimation is not None:
        kwargs["decimation"] = params.decimation
    return G1GrootWBCTask(
        cfg.name,
        G1GrootWBCTaskConfig(**kwargs),
        adapter=hw.adapter,
    )
