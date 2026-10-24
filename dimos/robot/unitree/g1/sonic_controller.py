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

"""G1 SONIC runtime: feedback, policy clock, activation and latched stops."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from enum import Enum
import math
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.components import make_humanoid_joints
from dimos.control.sonic.models import sonic_model_directory
from dimos.control.sonic.sonic_pipeline import (
    DEFAULT_ANGLES_DDS,
    LOCOMOTION_MODES,
    NUM_JOINTS,
    SONIC_KD,
    SONIC_KP,
    SonicPipeline,
)
from dimos.control.sonic.streamed_motion import StreamedMotion
from dimos.control.sonic.webxr.reference import WebXRReference
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.std_msgs.String import String
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.unitree.g1.control_safety import (
    FEEDBACK_TIMEOUT_SECONDS,
    JOINT_VELOCITY_LIMIT,
    G1ControlError,
    check_joint_velocities,
    damping_commands,
)
from dimos.robot.unitree.g1.jetson_clocks import require_jetson_clocks
from dimos.simulation.adapters.whole_body.g1 import SimMujocoG1WholeBodyAdapter
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Match the former coordinator's arm-reference limit, in radians per second.
_ARM_REFERENCE_VELOCITY_LIMIT = 1.0


class SonicControllerConfig(BaseConfig):
    webxr_enabled: bool = False
    encoder_onnx: Path = Field(
        default_factory=lambda: sonic_model_directory() / "sonic_v1_1/model_encoder.onnx"
    )
    decoder_onnx: Path = Field(
        default_factory=lambda: sonic_model_directory() / "sonic_v1_1/model_decoder.onnx"
    )
    planner_onnx: Path = Field(
        default_factory=lambda: sonic_model_directory() / "planner_sonic.onnx"
    )
    simulation_address: Path | None = None
    cpu_simulation: bool = False
    joint_names: list[str] = Field(
        default_factory=lambda: make_humanoid_joints("g1"), min_length=29, max_length=29
    )
    tick_rate: float = Field(default=50.0, gt=0.0, allow_inf_nan=False)
    decimation: int = Field(default=1, ge=1)
    timeout: float = Field(default=1.0, gt=0.0, allow_inf_nan=False)
    feedback_timeout: float = Field(default=FEEDBACK_TIMEOUT_SECONDS, gt=0.0, allow_inf_nan=False)
    auto_arm: bool = False
    auto_dry_run: bool = True
    default_ramp_seconds: float = Field(default=3.0, ge=0.0, allow_inf_nan=False)
    joint_velocity_limit: float = Field(default=JOINT_VELOCITY_LIMIT, gt=0.0, allow_inf_nan=False)


class SonicControlState(str, Enum):
    STOPPED = "stopped"
    UNARMED = "unarmed"
    INITIALIZING = "initializing"
    CONTROL = "control"
    FAULT = "fault"


class SonicController:
    """Internal controller used by G1SonicPolicyModule; owns no module transports."""

    def __init__(
        self,
        config: SonicControllerConfig,
        *,
        publish_joint_state: Callable[[JointState], None],
        publish_imu: Callable[[Imu], None],
        publish_motor_command: Callable[[MotorCommandArray], None],
        publish_fault: Callable[[String], None],
    ) -> None:
        self.config = config
        if config.webxr_enabled and not math.isclose(config.tick_rate / config.decimation, 50.0):
            raise ValueError("SONIC WebXR references require a 50 Hz policy loop")
        self._publish_joint_state = publish_joint_state
        self._publish_imu = publish_imu
        self._publish_motor_command = publish_motor_command
        self._publish_fault = publish_fault
        self._pipeline: SonicPipeline | None = None
        self._webxr = WebXRReference() if config.webxr_enabled else None
        self._adapter: SimMujocoG1WholeBodyAdapter | None = None
        self._control_lock = threading.RLock()
        self._fault_lock = threading.RLock()
        self._feedback_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._feedback: JointState | None = None
        self._imu_feedback: Imu | None = None
        self._feedback_at = self._imu_at = 0.0
        self._joint_names_list = list(self.config.joint_names)
        self._default_29 = DEFAULT_ANGLES_DDS.copy()
        self._arm_reference = self._default_29[15:].copy()
        self._arm_reference_output: NDArray[np.float32] | None = None
        self._cached_q_29 = self._default_29.copy()
        self._cached_dq_29 = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._state_seen = False
        self._active = False
        self._control_state = SonicControlState.STOPPED
        self._fault_reason: str | None = None
        self._arm_pending = False
        self._dry_run = self.config.auto_dry_run
        self._arming_duration = self.config.default_ramp_seconds
        self._initialization_start_t = 0.0
        self._ramp_start: NDArray[np.float32] | None = None
        self._stream_source_requested = False
        self._tick_count = 0
        self._last_targets: list[float] | None = None
        self._hold_targets: list[float] | None = None
        self._cmd = np.zeros(3, dtype=np.float32)
        self._last_cmd_time = 0.0
        self._last_dry_run_log_t = self._last_diag_log_t = 0.0
        self._policy_durations_ms: deque[float] = deque(maxlen=500)
        self._policy_intervals_ms: deque[float] = deque(maxlen=500)
        self._last_policy_started_at: float | None = None

    @property
    def _policy(self) -> SonicPipeline:
        if self._pipeline is None:
            raise RuntimeError("SONIC has not started")
        return self._pipeline

    @property
    def fault_reason(self) -> str | None:
        with self._fault_lock:
            return self._fault_reason

    def _require_hardware_clocks(self) -> None:
        if self.config.simulation_address is None:
            require_jetson_clocks()

    def start(self) -> bool:
        """Load models and start the policy clock; return whether newly started."""
        if self.fault_reason is not None:
            raise RuntimeError("SONIC damping stop is latched; restart the stack to recover")
        if self._active:
            return False
        if self.config.cpu_simulation and self.config.simulation_address is None:
            raise ValueError("SONIC CPU inference is for simulation, not hardware control")
        self._require_hardware_clocks()
        paths = (self.config.encoder_onnx, self.config.decoder_onnx, self.config.planner_onnx)
        missing = [str(path) for path in paths if not path.is_file()]
        if missing:
            raise FileNotFoundError(
                f"SONIC models missing: {missing}. Run dimos-sonic-models first."
            )
        try:
            self._pipeline = SonicPipeline(
                encoder_path=paths[0],
                decoder_path=paths[1],
                planner_path=paths[2],
                cpu_simulation=self.config.cpu_simulation,
            )
            if self.config.simulation_address is not None:
                self._adapter = SimMujocoG1WholeBodyAdapter(address=self.config.simulation_address)
                if not self._adapter.connect() or not self._adapter.activate():
                    raise RuntimeError("Cannot connect SONIC to MuJoCo")
            self._active = True
            self._control_state = SonicControlState.UNARMED
            self._reset_policy_state()
            if self.config.auto_arm:
                self.arm()
            self._thread = threading.Thread(target=self._run, name="g1-sonic-policy", daemon=True)
            self._thread.start()
            return True
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
        was_active = self._active
        self._active = False
        self._stop_event.set()
        if was_active:
            self.latch_fault("SONIC module stopped")
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._thread is None or not self._thread.is_alive():
            self._close_policy()
        else:
            logger.error("SONIC policy thread has not stopped; damping remains latched")

    def _close_policy(self) -> None:
        if self._adapter is not None:
            self._adapter.disconnect()
            self._adapter = None
        if self._pipeline is not None:
            self._pipeline.close()
            self._pipeline = None

    def _run(self) -> None:
        period = 1.0 / self.config.tick_rate
        next_at = time.perf_counter()
        try:
            while not self._stop_event.is_set():
                self._step(time.perf_counter())
                next_at = max(next_at + period, time.perf_counter())
                self._stop_event.wait(max(0.0, next_at - time.perf_counter()))
        finally:
            self._close_policy()

    def set_joint_reference(self, msg: JointState) -> None:
        arms = self._joint_names_list[15:]
        if (
            not msg.name
            or len(msg.name) != len(msg.position)
            or len(set(msg.name)) != len(msg.name)
        ):
            raise ValueError("position_command needs unique joint names and one position per name")
        if set(msg.name) - set(arms):
            raise ValueError(f"SONIC position_command only accepts arm joints: {arms}")
        if not np.isfinite(msg.position).all():
            raise ValueError("Arm references must be finite")
        with self._control_lock:
            targets = self._arm_reference.copy()
            for name, position in zip(msg.name, msg.position, strict=True):
                targets[arms.index(name)] = position
            self.set_upper_body(targets.tolist())

    def update_motor_states(self, msg: JointState) -> None:
        with self._feedback_lock:
            self._feedback = msg
            self._feedback_at = time.perf_counter()

    def update_imu(self, msg: Imu) -> None:
        with self._feedback_lock:
            self._imu_feedback = msg
            self._imu_at = time.perf_counter()

    def _read_state(self, t_now: float) -> tuple[list[MotorState], IMUState] | None:
        if self._adapter is not None:
            if not self._adapter.has_motor_states():
                return None
            return self._adapter.read_motor_states(), self._adapter.read_imu()
        with self._feedback_lock:
            msg, imu = self._feedback, self._imu_feedback
            if msg is None or imu is None:
                return None
            if t_now - min(self._feedback_at, self._imu_at) >= self.config.feedback_timeout:
                raise G1ControlError("SONIC feedback timeout")
        if any(len(values) != NUM_JOINTS for values in (msg.position, msg.velocity, msg.effort)):
            raise G1ControlError("incomplete robot joint feedback")
        return (
            [
                MotorState(q=q, dq=dq, tau=tau)
                for q, dq, tau in zip(msg.position, msg.velocity, msg.effort, strict=True)
            ],
            IMUState(
                quaternion=(
                    imu.orientation.w,
                    imu.orientation.x,
                    imu.orientation.y,
                    imu.orientation.z,
                ),
                gyroscope=(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z),
                accelerometer=(
                    imu.linear_acceleration.x,
                    imu.linear_acceleration.y,
                    imu.linear_acceleration.z,
                ),
            ),
        )

    def _step(self, t_now: float) -> list[float] | None:
        with self._control_lock:
            if not self._active:
                return None
            if self.fault_reason is not None:
                self._write_damping()
            try:
                state = self._read_state(t_now)
                if state is None:
                    if self._state_seen:
                        raise G1ControlError("robot feedback lost")
                    return None
                motors, imu = state
                if len(motors) != NUM_JOINTS:
                    raise G1ControlError("incomplete robot joint feedback")
                self._cached_q_29[:] = [m.q for m in motors]
                self._cached_dq_29[:] = [m.dq for m in motors]
                self._state_seen = True
                if not np.isfinite(self._cached_q_29).all():
                    raise G1ControlError("non-finite robot joint positions")
                ts = time.time()
                self._publish_joint_state(
                    JointState(
                        name=self._joint_names_list,
                        position=[m.q for m in motors],
                        velocity=[m.dq for m in motors],
                        effort=[m.tau for m in motors],
                        frame_id="g1_pelvis",
                        ts=ts,
                    )
                )
                w, x, y, z = imu.quaternion
                self._publish_imu(
                    Imu(
                        orientation=Quaternion(x, y, z, w),
                        angular_velocity=Vector3(*imu.gyroscope),
                        linear_acceleration=Vector3(*imu.accelerometer),
                        frame_id="g1_pelvis",
                        ts=ts,
                    )
                )
                if self.fault_reason is not None:
                    return None
                check_joint_velocities(
                    self._cached_dq_29.tolist(), self.config.joint_velocity_limit
                )
                targets = self._compute_policy(t_now, imu)
                # An E-stop can arrive during inference. Serialize the last
                # check and motor write with the stop, without delaying E-stop
                # behind the policy lock.
                with self._fault_lock:
                    if self._fault_reason is not None or not self._active:
                        return None
                    if targets is not None:
                        self._write_commands(
                            [
                                MotorCommand(q=q, dq=0.0, kp=float(kp), kd=float(kd), tau=0.0)
                                for q, kp, kd in zip(targets, SONIC_KP, SONIC_KD, strict=True)
                            ]
                        )
                return targets
            except Exception as exc:
                if self.fault_reason is None:
                    logger.exception("SONIC control fault")
                    self.latch_fault(str(exc))
                return None

    def _write_commands(self, commands: list[MotorCommand]) -> None:
        if self._adapter is not None:
            if not self._adapter.write_motor_commands(commands):
                raise G1ControlError("MuJoCo rejected SONIC motor commands")
        else:
            self._publish_motor_command(
                MotorCommandArray(
                    q=[c.q for c in commands],
                    dq=[c.dq for c in commands],
                    kp=[c.kp for c in commands],
                    kd=[c.kd for c in commands],
                    tau=[c.tau for c in commands],
                )
            )

    def latch_fault(self, reason: str) -> None:
        with self._fault_lock:
            if self._fault_reason is not None:
                return
            self._fault_reason = (reason or "SONIC control failure")[:256]
            self._arm_pending = False
            logger.error("SONIC damping stop latched; restart required", reason=self._fault_reason)
            self._write_damping()

    def _write_damping(self) -> None:
        with self._fault_lock:
            if self._adapter is not None:
                self._write_commands(damping_commands(NUM_JOINTS))
            elif self._fault_reason is not None:
                # The DDS module owns takeover and the final latch. An ordinary
                # damping MotorCommandArray here could itself trigger takeover.
                self._publish_fault(String(self._fault_reason))

    def _compute_policy(self, t_now: float, imu: IMUState) -> list[float] | None:
        if not self._active:
            return None

        current_29 = self._cached_q_29.copy()

        if self._control_state is SonicControlState.UNARMED:
            if not self._arm_pending:
                self._last_targets = current_29.tolist()
                self._hold_targets = self._last_targets.copy()
                return self._last_targets
            self._arm_pending = False
            self._control_state = SonicControlState.INITIALIZING

        if self._control_state is SonicControlState.INITIALIZING:
            if self._ramp_start is None:
                self._ramp_start = current_29.copy()
                self._initialization_start_t = t_now
                logger.info(
                    "G1SonicPolicyModule initializing to SONIC default pose",
                    connection="g1",
                    ramp_seconds=self._arming_duration,
                )

            assert self._ramp_start is not None
            elapsed = t_now - self._initialization_start_t
            alpha = (
                1.0 if self._arming_duration <= 0.0 else min(1.0, elapsed / self._arming_duration)
            )
            target = self._ramp_start + alpha * (self._default_29 - self._ramp_start)
            self._last_targets = target.tolist()
            self._hold_targets = self._last_targets.copy()
            if alpha >= 1.0:
                self._control_state = SonicControlState.CONTROL
                self._reset_policy_state()
                self._policy.set_source_stream(self._stream_source_requested)
                logger.info(
                    "SONIC policy control active",
                    reference_source="stream" if self._stream_source_requested else "planner",
                    mode="dry-run" if self._dry_run else "live",
                )
            return self._last_targets

        if self._control_state is not SonicControlState.CONTROL:
            return None

        # CONTROL: run the balancing policy continuously at the decimated rate.
        self._tick_count += 1
        if self._tick_count % self.config.decimation != 0:
            if self._dry_run:
                return self._hold_output()
            if self._last_targets is None:
                return None
            return self._last_targets

        q_29 = self._cached_q_29.copy()
        dq_29 = self._cached_dq_29.copy()

        if self._arm_reference_output is not None:
            max_delta = (
                _ARM_REFERENCE_VELOCITY_LIMIT * self.config.decimation / self.config.tick_rate
            )
            self._arm_reference_output += np.clip(
                self._arm_reference - self._arm_reference_output, -max_delta, max_delta
            )
            self._policy.set_upper_body(self._arm_reference_output.copy())

        gyro = np.asarray(imu.gyroscope, dtype=np.float32)
        quat = np.asarray(imu.quaternion, dtype=np.float64)
        gravity = self._projected_gravity(imu.quaternion)

        with self._cmd_lock:
            if (
                self.config.timeout > 0.0
                and self._last_cmd_time > 0.0
                and (t_now - self._last_cmd_time) > self.config.timeout
            ):
                cmd = np.zeros(3, dtype=np.float32)
            else:
                cmd = self._cmd.copy()
        if self._webxr is not None:
            previous_mode = self._webxr.mode
            cmd = self._webxr.tick(self._policy, cmd, t_now)
            if self._webxr.mode != previous_mode:
                self.set_velocity_command(0.0, 0.0, 0.0)
        self._policy.set_velocity(float(cmd[0]), float(cmd[1]), float(cmd[2]))

        policy_started_at = time.perf_counter()
        targets_29 = self._policy.step(
            q_dds=q_29,
            dq_dds=dq_29,
            base_quat_wxyz=quat,
            gyro_body=gyro,
            gravity_body=gravity,
        )
        self._record_policy_timing(time.perf_counter() - policy_started_at, policy_started_at)
        if targets_29.shape != (NUM_JOINTS,) or not np.isfinite(targets_29).all():
            raise G1ControlError("invalid SONIC motor targets")
        self._last_targets = targets_29.tolist()

        if (t_now - self._last_diag_log_t) >= 5.0:
            logger.info("G1SonicPolicyModule", connection="g1", **self._policy.snapshot())
            self._last_diag_log_t = t_now

        if self._dry_run:
            if (t_now - self._last_dry_run_log_t) >= 1.0:
                max_delta = float(np.max(np.abs(targets_29 - current_29)))
                logger.info(
                    "G1SonicPolicyModule DRY-RUN",
                    connection="g1",
                    max_dq_rad=max_delta,
                )
                self._last_dry_run_log_t = t_now
            # Continue publishing the fixed hold so the hardware watchdog can
            # distinguish healthy dry-run inference from a stalled task.
            return self._hold_output()

        self._hold_targets = self._last_targets.copy()
        return self._last_targets

    def _hold_output(self) -> list[float]:
        if self._hold_targets is None:
            raise G1ControlError("dry-run has no prepared hold target")
        return self._hold_targets.copy()

    def set_estop(self, estopped: bool) -> None:
        if estopped:
            self.latch_fault("operator stop")
        elif self.fault_reason is not None:
            raise RuntimeError("SONIC damping stop is latched; restart the stack to recover")

    def set_velocity_command(
        self, vx: float, vy: float, yaw_rate: float, t_now: float | None = None
    ) -> None:
        if not all(math.isfinite(v) for v in (vx, vy, yaw_rate)):
            raise ValueError("Velocity commands must be finite")
        if t_now is None:
            t_now = time.perf_counter()
        with self._cmd_lock:
            self._cmd[:] = [vx, vy, yaw_rate]
            self._last_cmd_time = t_now

    def play_motion_clip(self, name: str) -> dict[str, Any]:
        with self._control_lock:
            self._require_no_webxr_reference()
            motions = Path(self.config.planner_onnx).parent / "motions"
            if name not in self.list_motion_clips():
                raise ValueError(f"unknown or uninstalled motion clip: {name}")
            clip_dir = motions / name
            if not clip_dir.is_dir():
                raise FileNotFoundError(f"no such clip: {name} ({clip_dir})")
            jp = np.loadtxt(clip_dir / "joint_pos.csv", delimiter=",", dtype=np.float32, skiprows=1)
            jv = np.loadtxt(clip_dir / "joint_vel.csv", delimiter=",", dtype=np.float32, skiprows=1)
            bq = np.loadtxt(clip_dir / "body_quat.csv", delimiter=",", dtype=np.float32, skiprows=1)
            motion = StreamedMotion(
                joint_pos=jp,
                joint_vel=jv,
                root_quat=bq[:, :4],
                smpl_joints=None,
                smpl_pose=None,
                encode_mode=0,
                timesteps=len(jp),
            )
            self._policy.play_clip(motion)
            self._stream_source_requested = True
            logger.info(
                "G1SonicPolicyModule playing clip",
                connection="g1",
                clip=name,
                frames=len(jp),
                seconds=round(len(jp) / 50.0, 1),
            )
            return {"clip": name, "frames": len(jp), "seconds": len(jp) / 50.0}

    def set_vr_3point(
        self,
        positions: list[float],
        orientations: list[float],
        t_now: float | None = None,
    ) -> dict[str, Any]:
        with self._control_lock:
            self._require_no_webxr_reference()
            self._policy.set_vr_3point(
                np.asarray(positions, dtype=np.float32),
                np.asarray(orientations, dtype=np.float32),
                t_now=t_now,
            )
            return {"vr_active": True}

    def clear_vr_3point(self) -> bool:
        with self._control_lock:
            self._policy.clear_vr_3point()
            return True

    def stop_motion_clip(self) -> bool:
        with self._control_lock:
            if self._webxr is not None and self._webxr.owns_reference:
                self._webxr.stop()
                return True
            self._stream_source_requested = False
            self._policy.stop_clip()
            return True

    def list_motion_clips(self) -> list[str]:
        with self._control_lock:
            motions = Path(self.config.planner_onnx).parent / "motions"
            if not motions.is_dir():
                return []
            return sorted(p.name for p in motions.iterdir() if p.is_dir())

    def set_locomotion_mode(self, mode: int | str | None) -> dict[str, Any]:
        with self._control_lock:
            self._require_no_webxr_reference()
            applied = self._policy.set_mode(mode)
            logger.info(
                "G1SonicPolicyModule locomotion mode",
                connection="g1",
                requested=mode,
                applied=applied,
            )
            return {"mode_override": applied}

    def list_locomotion_modes(self) -> dict[str, int]:
        with self._control_lock:
            return dict(LOCOMOTION_MODES)

    def set_base_height(self, height: float) -> None:
        with self._control_lock:
            self._require_no_webxr_reference()
            if not math.isfinite(height):
                raise ValueError("Height must be finite")
            self._policy.set_base_height(float(height))

    def set_upper_body(self, positions: list[float]) -> bool:
        with self._control_lock:
            self._require_no_webxr_reference()
            if len(positions) != 14:
                raise ValueError(f"set_upper_body expects 14 values, got {len(positions)}")
            if not np.isfinite(positions).all():
                raise ValueError("Arm references must be finite")
            self._arm_reference = np.asarray(positions, dtype=np.float32)
            if self._arm_reference_output is None:
                self._arm_reference_output = self._cached_q_29[15:].copy()
            return True

    def clear_upper_body(self) -> None:
        with self._control_lock:
            self._arm_reference = self._default_29[15:].copy()
            if self._arm_reference_output is None:
                self._arm_reference_output = self._cached_q_29[15:].copy()

    def activate(self, ramp_seconds: float | None = None) -> bool:
        """Request live control atomically, after checking feedback and lifecycle state."""
        with self._control_lock:
            if self.fault_reason is not None:
                raise RuntimeError("SONIC damping stop is latched; restart the stack to recover")
            if not self._active:
                raise RuntimeError("SONIC is not running")
            if self._control_state is not SonicControlState.UNARMED or self._arm_pending:
                raise RuntimeError("Deactivate the policy before requesting activation")
            if self._read_state(time.perf_counter()) is None:
                raise RuntimeError("Waiting for joint and IMU feedback")
            # arm() validates the ramp before changing either activation or dry-run state.
            accepted = self.arm(ramp_seconds)
            if accepted:
                # arm() has checked clocks. Do not introduce another fallible
                # operation after it has accepted the activation request.
                self._dry_run = False
                self._last_dry_run_log_t = 0.0
            return accepted

    def stop_motion(self) -> None:
        """Clear operator references and request stationary balancing, without disarming."""
        with self._control_lock:
            self.set_velocity_command(0.0, 0.0, 0.0)
            self.stop_motion_clip()
            self.clear_vr_3point()
            self.clear_upper_body()

    def arm(self, ramp_seconds: float | None = None) -> bool:
        with self._control_lock:
            if ramp_seconds is not None and (not math.isfinite(ramp_seconds) or ramp_seconds < 0):
                raise ValueError("ramp_seconds must be finite and nonnegative")
            if self.fault_reason is not None:
                raise RuntimeError("SONIC damping stop is latched; restart the stack to recover")
            if not self._active:
                logger.warning(
                    "G1SonicPolicyModule arm() before start(); ignoring", connection="g1"
                )
                return False
            if (
                self._control_state
                in (
                    SonicControlState.INITIALIZING,
                    SonicControlState.CONTROL,
                )
                or self._arm_pending
            ):
                return False
            self._require_hardware_clocks()
            self._arming_duration = (
                self.config.default_ramp_seconds if ramp_seconds is None else ramp_seconds
            )
            self._arm_pending = True
            logger.info(
                "G1SonicPolicyModule arm requested",
                connection="g1",
                control_state=self._control_state.value,
            )
            return True

    def disarm(self) -> bool:
        with self._control_lock:
            if self.fault_reason is not None:
                return False
            if not self._arm_pending and self._control_state not in (
                SonicControlState.INITIALIZING,
                SonicControlState.CONTROL,
            ):
                return False
            self._arm_pending = False
            self._stream_source_requested = False
            self._control_state = SonicControlState.UNARMED
            self._ramp_start = None
            self._last_targets = None
            self._reset_policy_state()
            logger.info(
                "G1SonicPolicyModule policy stopped",
                connection="g1",
                control_state=self._control_state.value,
            )
            return True

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        with self._control_lock:
            if self.fault_reason is not None:
                return False
            was_armed = self._arm_pending or self._control_state in (
                SonicControlState.INITIALIZING,
                SonicControlState.CONTROL,
            )
            should_reactivate = was_armed if reactivate is None else bool(reactivate)
            if self._active and should_reactivate:
                self._require_hardware_clocks()

            self._control_state = (
                SonicControlState.UNARMED if self._active else SonicControlState.STOPPED
            )
            self._arm_pending = self._active and should_reactivate
            self._ramp_start = None
            self._initialization_start_t = 0.0
            self._last_targets = None
            self._state_seen = False
            self._stream_source_requested = False
            self._cached_q_29[:] = self._default_29
            self._cached_dq_29[:] = 0.0
            self._reset_policy_state()

            logger.info(
                "G1SonicPolicyModule runtime state reset",
                connection="g1",
                reactivate=should_reactivate,
            )
            return True

    def set_dry_run(self, enabled: bool) -> None:
        with self._control_lock:
            new_val = bool(enabled)
            if new_val == self._dry_run:
                return
            if not new_val:
                self._require_hardware_clocks()
            if self._webxr is not None and self._webxr.owns_reference:
                raise RuntimeError("Stop WebXR pose mode before changing dry-run")
            self._dry_run = new_val
            self._last_dry_run_log_t = 0.0
            logger.info("G1SonicPolicyModule dry_run changed", connection="g1", dry_run=new_val)

    def status(self) -> dict[str, Any]:
        with self._control_lock:
            control_state = (
                SonicControlState.FAULT if self.fault_reason is not None else self._control_state
            )
            snap: dict[str, Any] = {
                "active": self._active,
                "armed": control_state is SonicControlState.CONTROL,
                "arming": control_state is SonicControlState.INITIALIZING,
                "arm_pending": self._arm_pending,
                "arming_duration": self._arming_duration,
                "control_state": control_state.value,
                "dry_run": self._dry_run,
                "fault_reason": self.fault_reason,
                "simulation": self._adapter is not None,
                "model": self.config.encoder_onnx.parent.name,
                "feedback_timeout_ms": self.config.feedback_timeout * 1000.0,
            }
            now = time.perf_counter()
            with self._feedback_lock:
                snap["feedback_received"] = (
                    self._state_seen if self._adapter is not None else self._feedback is not None
                )
                snap["feedback_age_ms"] = (
                    (now - self._feedback_at) * 1000.0 if self._feedback is not None else None
                )
                snap["imu_age_ms"] = (
                    (now - self._imu_at) * 1000.0 if self._imu_feedback is not None else None
                )
            if self._pipeline is not None:
                snap.update(self._pipeline.snapshot())
            snap["reference_source"] = "stream" if snap.get("stream_active") else "planner"
            if self._webxr is not None:
                snap["webxr"] = self._webxr.status(now)
                if self._webxr.owns_reference:
                    snap["reference_source"] = "webxr"
            snap["debug_q_leg"] = [round(float(v), 4) for v in self._cached_q_29[:6]]
            snap["debug_dq_leg"] = [round(float(v), 4) for v in self._cached_dq_29[:6]]
            snap["policy_timing"] = self._policy_timing_snapshot()
            return snap

    def _reset_policy_state(self) -> None:
        # Activation must not replay references received before or during the pose ramp.
        with self._cmd_lock:
            self._cmd[:] = 0.0
            self._last_cmd_time = 0.0
        self._policy.reset()
        if self._webxr is not None:
            self._webxr.reset()
        self._arm_reference = self._default_29[15:].copy()
        self._arm_reference_output = None
        self._tick_count = 0
        self._policy_durations_ms.clear()
        self._policy_intervals_ms.clear()
        self._last_policy_started_at = None

    def _require_no_webxr_reference(self) -> None:
        if self._webxr is not None and self._webxr.owns_reference:
            raise RuntimeError("Stop WebXR pose mode before selecting another reference")

    def update_body_tracking(self, snapshot: BodyTrackingSnapshot) -> None:
        with self._control_lock:
            if self._webxr is not None:
                self._webxr.body(snapshot, time.perf_counter())

    def update_teleop_buttons(self, buttons: Buttons) -> None:
        if self._webxr is None:
            return
        # Damping must not wait for inference or retargeting to release the lock.
        if all(
            (
                buttons.left_primary,
                buttons.right_primary,
                buttons.left_secondary,
                buttons.right_secondary,
            )
        ):
            self.set_estop(True)
            return
        with self._control_lock:
            self._webxr.buttons(
                buttons,
                time.perf_counter(),
                enabled=self._control_state is SonicControlState.CONTROL
                and not self._dry_run
                and self.fault_reason is None
                and not self._stream_source_requested,
            )

    def _record_policy_timing(self, duration_seconds: float, started_at: float) -> None:
        duration_ms = duration_seconds * 1000.0
        self._policy_durations_ms.append(duration_ms)
        if self._last_policy_started_at is not None:
            self._policy_intervals_ms.append((started_at - self._last_policy_started_at) * 1000.0)
        self._last_policy_started_at = started_at

    def _policy_timing_snapshot(self) -> dict[str, Any]:
        def summary(samples: deque[float]) -> dict[str, float | int]:
            if not samples:
                return {"samples": 0, "mean": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0}
            values = np.asarray(samples, dtype=np.float64)
            return {
                "samples": len(samples),
                "mean": round(float(np.mean(values)), 3),
                "p95": round(float(np.percentile(values, 95)), 3),
                "p99": round(float(np.percentile(values, 99)), 3),
                "max": round(float(np.max(values)), 3),
            }

        return {
            "target_hz": self.config.tick_rate / self.config.decimation,
            "last_step_age_ms": (
                (time.perf_counter() - self._last_policy_started_at) * 1000.0
                if self._last_policy_started_at is not None
                else None
            ),
            "step_ms": summary(self._policy_durations_ms),
            "start_interval_ms": summary(self._policy_intervals_ms),
        }

    @staticmethod
    def _projected_gravity(quaternion: tuple[float, ...]) -> NDArray[np.float32]:
        w, x, y, z = quaternion
        gx = 2.0 * (-x * z + w * y)
        gy = 2.0 * (-y * z - w * x)
        gz = -(w * w - x * x - y * y + z * z)
        return np.array([gx, gy, gz], dtype=np.float32)
