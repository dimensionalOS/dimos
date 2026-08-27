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

"""SONIC whole-body-control task for the Unitree G1 humanoid.

Runs the GEAR-SONIC planner+encoder+decoder pipeline inside the
coordinator tick loop. Unlike the GR00T decoupled task, SONIC is a
UNIFIED policy: it claims all 29 joints at WBC priority. Precise arm
servoing is not this task's job (upper-body targets are encoder hints,
per sonic-notebook DECISIONS.md D3) - pair with the decoupled task and
hot-swap when manipulation accuracy matters.

Locomotion modes (the 27 GEAR modes: squat, kneel, crawl, boxing, dances,
carrying, jump...) are RPC-reachable via coordinator.task_invoke:

    task_invoke("sonic_wbc", "set_locomotion_mode", {"mode": "HAPPY_DANCE_WALK"})
    task_invoke("sonic_wbc", "set_locomotion_mode", {"mode": None})  # speed-auto
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray

from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import (
    DEFAULT_ANGLES_DDS,
    LOCOMOTION_MODES,
    NUM_JOINTS,
    SonicPipeline,
)
from dimos.control.tasks.g1_sonic_wbc_task.zmq_wire import (
    CommandUpdate,
    PlannerUpdate,
    decode,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.whole_body.spec import WholeBodyAdapter
    from dimos.msgs.geometry_msgs.Twist import Twist

logger = setup_logger()


@dataclass
class G1SonicWBCTaskConfig:
    """Configuration for the SONIC WBC task.

    joint_names must be all 29 G1 coordinator joint names in DDS order -
    SONIC is a unified whole-body policy and claims every joint.
    """

    encoder_onnx: str | Path
    decoder_onnx: str | Path
    planner_onnx: str | Path
    joint_names: list[str]
    priority: int = 50
    decimation: int = 1
    timeout: float = 1.0
    zmq_enabled: bool = True
    zmq_sub_endpoint: str = "tcp://127.0.0.1:5556"
    zmq_pub_endpoint: str = "tcp://*:5557"
    auto_arm: bool = False
    auto_dry_run: bool = False
    default_ramp_seconds: float = 3.0


class SonicControlState(str, Enum):
    STOPPED = "stopped"
    UNARMED = "unarmed"
    INITIALIZING = "initializing"
    READY = "ready"
    CONTROL = "control"


class G1SonicWBCTask(BaseControlTask):
    """GEAR-SONIC unified 29-DOF whole-body policy as a coordinator task.

    Startup holds the measured pose. arm() snapshots that pose on the next
    control tick, ramps to SONIC's default, then runs the balancing policy.
    """

    def __init__(
        self,
        name: str,
        config: G1SonicWBCTaskConfig,
        adapter: WholeBodyAdapter,
    ) -> None:
        if len(config.joint_names) != NUM_JOINTS:
            raise ValueError(
                f"G1SonicWBCTask '{name}' requires exactly {NUM_JOINTS} joint "
                f"names (unified whole-body policy), got {len(config.joint_names)}"
            )
        if config.decimation < 1:
            raise ValueError(f"G1SonicWBCTask '{name}' requires decimation >= 1")

        self._name = name
        self._config = config
        self._adapter = adapter
        self._joint_names_list = list(config.joint_names)
        self._joint_names_set = frozenset(config.joint_names)

        self._pipeline = SonicPipeline(
            encoder_path=config.encoder_onnx,
            decoder_path=config.decoder_onnx,
            planner_path=config.planner_onnx,
        )

        self._default_29 = DEFAULT_ANGLES_DDS.copy()

        self._tick_count = 0
        self._last_targets: list[float] | None = None

        # Last-known-good caches; same missing-joint policy as the GR00T
        # task - never substitute 0.0, a zero pose reads as "legs straight"
        # and provokes a snap-back.
        self._cached_q_29 = self._default_29.copy()
        self._cached_dq_29 = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._state_seen = False

        self._active = False
        self._control_state = SonicControlState.STOPPED
        self._arm_pending = False
        self._dry_run = bool(config.auto_dry_run)
        self._arming_duration = max(0.0, float(config.default_ramp_seconds))
        self._initialization_start_t = 0.0
        self._initialization_started = False
        self._ramp_start: NDArray[np.float32] | None = None
        self._stream_source_requested = False
        self._last_dry_run_log_t = 0.0
        self._last_diag_log_t = 0.0

        self._cmd_lock = threading.Lock()
        self._cmd = np.zeros(3, dtype=np.float32)
        self._last_cmd_time = 0.0

        # ZMQ wire compatibility (D2): SONIC's native command/planner/pose
        # protocol. Sockets are created lazily on start() and polled
        # non-blocking from compute() - the task stays passive (no threads).
        self._zmq_sub: Any = None
        self._zmq_pub: Any = None
        self._zmq_started = False
        self._zmq_failed = False
        self._left_hand: NDArray[Any] | None = None
        self._right_hand: NDArray[Any] | None = None
        self._last_pose_msg_t = 0.0
        self._last_planner_msg_t = 0.0
        self._zmq_stats = {"command": 0, "planner": 0, "pose": 0, "errors": 0}

    # -- ControlTask protocol ----------------------------------------------

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names_set,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        return self._active

    @property
    def control_state(self) -> SonicControlState:
        return self._control_state

    @property
    def policy_active(self) -> bool:
        return self._control_state is SonicControlState.CONTROL

    def _refresh_state_caches(self, state: CoordinatorState) -> bool:
        all_present = True
        for i, jname in enumerate(self._joint_names_list):
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
        if all_present:
            self._state_seen = True
        return all_present

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self._active:
            return None

        fresh = self._refresh_state_caches(state)
        if not self._state_seen and not fresh:
            return None

        self._zmq_start()
        self._zmq_poll(state.t_now)

        current_29 = self._cached_q_29.copy()

        if self._control_state is SonicControlState.UNARMED:
            if not self._arm_pending:
                self._last_targets = current_29.tolist()
                return JointCommandOutput(
                    joint_names=self._joint_names_list,
                    positions=self._last_targets,
                    mode=ControlMode.SERVO_POSITION,
                )
            self._arm_pending = False
            self._control_state = SonicControlState.INITIALIZING

        if self._control_state is SonicControlState.INITIALIZING:
            if not self._initialization_started:
                self._initialization_started = True
                self._ramp_start = current_29.copy()
                self._initialization_start_t = state.t_now
                logger.info(
                    "G1SonicWBCTask initializing to SONIC default pose",
                    task=self._name,
                    ramp_seconds=self._arming_duration,
                )

            assert self._ramp_start is not None
            elapsed = state.t_now - self._initialization_start_t
            alpha = (
                1.0 if self._arming_duration <= 0.0 else min(1.0, elapsed / self._arming_duration)
            )
            target = self._ramp_start + alpha * (self._default_29 - self._ramp_start)
            self._last_targets = target.tolist()
            if alpha >= 1.0:
                self._control_state = SonicControlState.READY
                self._reset_policy_state()
                logger.info("G1SonicWBCTask initialization complete", task=self._name)
                self._enter_control()
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=self._last_targets,
                mode=ControlMode.SERVO_POSITION,
            )

        if self._control_state is SonicControlState.READY:
            self._last_targets = self._default_29.tolist()
            self._enter_control()
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=self._last_targets,
                mode=ControlMode.SERVO_POSITION,
            )

        if self._control_state is not SonicControlState.CONTROL:
            return None

        # CONTROL: run the balancing policy continuously at the decimated rate.
        self._tick_count += 1
        if self._tick_count % self._config.decimation != 0:
            if self._dry_run or self._last_targets is None:
                return None
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=self._last_targets,
                mode=ControlMode.SERVO_POSITION,
            )

        q_29 = self._cached_q_29.copy()
        dq_29 = self._cached_dq_29.copy()

        if state.imu:
            imu = next(iter(state.imu.values()))
        else:
            imu = self._adapter.read_imu()
        gyro = np.asarray(imu.gyroscope, dtype=np.float32)
        quat = np.asarray(imu.quaternion, dtype=np.float64)
        gravity = self._projected_gravity(imu.quaternion)

        with self._cmd_lock:
            if (
                self._config.timeout > 0.0
                and self._last_cmd_time > 0.0
                and (state.t_now - self._last_cmd_time) > self._config.timeout
            ):
                cmd = np.zeros(3, dtype=np.float32)
            else:
                cmd = self._cmd.copy()
        self._pipeline.set_velocity(float(cmd[0]), float(cmd[1]), float(cmd[2]))

        targets_29 = self._pipeline.step(
            q_dds=q_29,
            dq_dds=dq_29,
            base_quat_wxyz=quat,
            gyro_body=gyro,
            gravity_body=gravity,
        )
        self._last_targets = targets_29.tolist()
        self._zmq_publish_state(state.t_now, q_29, dq_29, quat, gyro, targets_29)

        if (state.t_now - self._last_diag_log_t) >= 5.0:
            logger.info("G1SonicWBCTask", task=self._name, **self._pipeline.snapshot())
            self._last_diag_log_t = state.t_now

        if self._dry_run:
            if (state.t_now - self._last_dry_run_log_t) >= 1.0:
                max_delta = float(np.max(np.abs(targets_29 - current_29)))
                logger.info(
                    "G1SonicWBCTask DRY-RUN",
                    task=self._name,
                    max_dq_rad=max_delta,
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
                "G1SonicWBCTask preempted",
                task=self._name,
                by_task=by_task,
                joints=joints,
            )

    # -- command inputs ------------------------------------------------------

    def set_velocity_command(
        self, vx: float, vy: float, yaw_rate: float, t_now: float | None = None
    ) -> None:
        if t_now is None:
            t_now = time.perf_counter()
        with self._cmd_lock:
            self._cmd[:] = [vx, vy, yaw_rate]
            self._last_cmd_time = t_now

    def on_twist_command(self, msg: Twist, t_now: float) -> None:
        self.set_velocity_command(
            float(msg.linear.x), float(msg.linear.y), float(msg.angular.z), t_now
        )

    def play_motion_clip(self, name: str) -> dict[str, Any]:
        """Play a reference motion clip from the sonic data dir by name.

        Clips are 50 Hz CSVs in SONIC's reference layout (joint_pos.csv,
        joint_vel.csv, body_quat.csv - IsaacLab joint order, header row).
        """
        import numpy as np

        from dimos.control.tasks.g1_sonic_wbc_task.streamed_motion import StreamedMotion
        from dimos.utils.data import get_data

        clip_dir = Path(get_data("sonic")) / "motions" / name
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
        self._pipeline.play_clip(motion)
        self._stream_source_requested = True
        logger.info(
            "G1SonicWBCTask playing clip",
            task=self._name,
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
        """VR 3-point teleop targets (SONIC encoder mode 1).

        positions: 9 floats - [left wrist, right wrist, head] xyz, root-relative
        (world minus pelvis, rotated into the pelvis frame). orientations: 12
        floats - the same three points as quat wxyz, root-relative
        (quat_inv(root) * q_world). The C++ deploy stack's wrist offsets
        [0.18, -/+0.025, 0] and head offset [0, 0, 0.35] must already be
        applied by the caller. Targets are encoder HINTS through the policy
        latent - expect coordinated whole-body following, not servo-accurate
        end-effector tracking. Stale data (> 0.5 s) reverts to planner obs;
        re-send at teleop rate.
        """
        import numpy as np

        self._pipeline.set_vr_3point(
            np.asarray(positions, dtype=np.float32),
            np.asarray(orientations, dtype=np.float32),
            t_now=t_now,
        )
        return {"vr_active": True}

    def clear_vr_3point(self) -> bool:
        self._pipeline.clear_vr_3point()
        return True

    def stop_motion_clip(self) -> bool:
        self._return_to_planner_reference()
        return True

    def list_motion_clips(self) -> list[str]:
        from dimos.utils.data import get_data

        motions = Path(get_data("sonic")) / "motions"
        if not motions.is_dir():
            return []
        return sorted(p.name for p in motions.iterdir() if p.is_dir())

    def set_locomotion_mode(self, mode: int | str | None) -> dict[str, Any]:
        """Force one of the 27 GEAR locomotion modes; None = speed-auto."""
        applied = self._pipeline.set_mode(mode)
        logger.info(
            "G1SonicWBCTask locomotion mode",
            task=self._name,
            requested=mode,
            applied=applied,
        )
        return {"mode_override": applied}

    def list_locomotion_modes(self) -> dict[str, int]:
        return dict(LOCOMOTION_MODES)

    def set_base_height(self, height: float) -> None:
        self._pipeline.set_base_height(float(height))

    def set_upper_body(self, positions: list[float]) -> bool:
        """14 arm-joint encoder hints, DDS order (indices 15-28)."""
        if len(positions) != 14:
            raise ValueError(f"set_upper_body expects 14 values, got {len(positions)}")
        self._pipeline.set_upper_body(np.asarray(positions, dtype=np.float32))
        return True

    def clear_upper_body(self) -> None:
        self._pipeline.set_upper_body(DEFAULT_ANGLES_DDS[15:].copy())

    # -- ZMQ wire endpoint (D2) ------------------------------------------------

    def _zmq_start(self) -> None:
        if self._zmq_started or self._zmq_failed or not self._config.zmq_enabled:
            return
        try:
            import zmq

            ctx = zmq.Context.instance()
            sub = ctx.socket(zmq.SUB)
            sub.connect(self._config.zmq_sub_endpoint)
            for topic in (b"command", b"planner", b"pose"):
                sub.setsockopt(zmq.SUBSCRIBE, topic)
            pub = ctx.socket(zmq.PUB)
            pub.bind(self._config.zmq_pub_endpoint)
            self._zmq_sub = sub
            self._zmq_pub = pub
            self._zmq_started = True
            logger.info(
                "G1SonicWBCTask ZMQ endpoint up",
                task=self._name,
                sub=self._config.zmq_sub_endpoint,
                pub=self._config.zmq_pub_endpoint,
            )
        except Exception as exc:
            # Give up permanently: retrying (and logging) from the 50 Hz
            # compute tick starves the control loop badly enough to drop the
            # robot. One warning, then the wire stays off for this run.
            logger.warning(
                "G1SonicWBCTask ZMQ unavailable, wire disabled for this run",
                task=self._name,
                error=repr(exc),
            )
            self._zmq_failed = True
            self._zmq_started = False

    def _zmq_poll(self, t_now: float) -> None:
        """Drain pending wire messages; called once per compute() tick."""
        if not self._zmq_started or self._zmq_sub is None:
            return
        import zmq

        cmd = CommandUpdate()
        got_cmd = False
        for _ in range(64):  # bounded drain per tick
            try:
                raw = self._zmq_sub.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception as exc:
                self._zmq_stats["errors"] += 1
                logger.warning("ZMQ recv failed", task=self._name, error=repr(exc))
                break
            try:
                msg = decode(raw)
            except Exception as exc:
                self._zmq_stats["errors"] += 1
                logger.warning("ZMQ decode failed", task=self._name, error=repr(exc))
                continue
            if msg.topic == "command":
                cmd.merge(msg)
                got_cmd = True
                self._zmq_stats["command"] += 1
            elif msg.topic == "planner":
                self._on_wire_planner(PlannerUpdate.from_message(msg), t_now)
                self._zmq_stats["planner"] += 1
            elif msg.topic == "pose":
                summary = self._pipeline.apply_pose_message(msg.fields)
                self._last_pose_msg_t = t_now
                self._zmq_stats["pose"] += 1
                if "error" in summary:
                    self._zmq_stats["errors"] += 1
                # Pico pose messages also carry VR 3-point targets and the
                # operator's joystick yaw (heading_increment) - C++ consumes
                # both from this topic as well as the planner topic.
                vr_p = msg.get("vr_position")
                vr_o = msg.get("vr_orientation")
                if vr_p is not None and vr_o is not None:
                    self._pipeline.set_vr_3point(
                        vr_p.astype("float64").ravel(),
                        vr_o.astype("float64").ravel(),
                        t_now=t_now,
                    )
                hi = msg.get("heading_increment")
                if hi is not None:
                    self._pipeline.apply_heading_increment(float(hi.flat[0]))
        if got_cmd:
            self._on_wire_command(cmd)

    def _on_wire_command(self, cmd: CommandUpdate) -> None:
        # C++ semantics: start/stop pulses OR-accumulated; planner flag
        # selects planner vs streamed-motion source.
        self._select_stream_reference(not cmd.planner)
        if cmd.stop:
            self.disarm()
        elif cmd.start:
            self.arm()
        if cmd.delta_heading is not None:
            # C++ command-topic semantics: incremental yaw pulses folded into
            # HeadingState.delta_heading (gamepad delta_left/right are +/-0.1).
            self._pipeline.apply_heading_increment(float(cmd.delta_heading))

    def _on_wire_planner(self, upd: PlannerUpdate, t_now: float) -> None:
        self._pipeline.set_planner_command(
            mode=upd.mode,
            movement=upd.movement,
            facing=upd.facing,
            speed=upd.speed,
            height=upd.height,
        )
        self._last_planner_msg_t = t_now
        self._pipeline.set_upper_body_wire17(upd.upper_body_position, upd.upper_body_velocity)
        if upd.left_hand_joints is not None:
            self._left_hand = upd.left_hand_joints
        if upd.right_hand_joints is not None:
            self._right_hand = upd.right_hand_joints
        if upd.vr_position is not None and upd.vr_orientation is not None:
            self._pipeline.set_vr_3point(upd.vr_position, upd.vr_orientation, t_now=t_now)

    def _zmq_publish_state(
        self,
        t_now: float,
        q: NDArray[Any],
        dq: NDArray[Any],
        quat: NDArray[Any],
        gyro: NDArray[Any],
        targets: NDArray[Any],
    ) -> None:
        if not self._zmq_started or self._zmq_pub is None:
            return
        try:
            import msgpack  # type: ignore[import-untyped]

            payload = msgpack.packb(
                {
                    "timestamp": t_now,
                    "joint_pos": q.tolist(),
                    "joint_vel": dq.tolist(),
                    "base_quat": quat.tolist(),
                    "base_ang_vel": gyro.tolist(),
                    "position_targets": targets.tolist(),
                    **{
                        k: v
                        for k, v in self._pipeline.snapshot().items()
                        if not k.startswith("debug_")
                    },
                }
            )
            self._zmq_pub.send(b"g1_debug" + payload)
        except Exception:
            pass

    # -- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        self._active = True
        self._control_state = SonicControlState.UNARMED
        self._arm_pending = False
        self._dry_run = bool(self._config.auto_dry_run)
        self._arming_duration = max(0.0, float(self._config.default_ramp_seconds))
        self._initialization_start_t = 0.0
        self._initialization_started = False
        self._ramp_start = None
        self._stream_source_requested = False
        self._last_targets = None
        self._state_seen = False
        self._reset_policy_state()
        with self._cmd_lock:
            self._cmd[:] = 0.0
            self._last_cmd_time = 0.0
        if self._config.auto_arm:
            self.arm()
        logger.info(
            "G1SonicWBCTask started",
            task=self._name,
            control_state=self._control_state.value,
            auto_arm=self._config.auto_arm,
            dry_run=self._dry_run,
        )

    def stop(self) -> None:
        self._active = False
        self._control_state = SonicControlState.STOPPED
        self._arm_pending = False
        self._initialization_started = False
        self._ramp_start = None
        self._stream_source_requested = False
        self._last_targets = None
        logger.info("G1SonicWBCTask stopped", task=self._name)

    def arm(self, ramp_seconds: float | None = None) -> bool:
        if not self._active:
            logger.warning("G1SonicWBCTask arm() before start(); ignoring", task=self._name)
            return False
        if (
            self._control_state
            in (
                SonicControlState.INITIALIZING,
                SonicControlState.READY,
                SonicControlState.CONTROL,
            )
            or self._arm_pending
        ):
            return False
        if ramp_seconds is not None:
            self._arming_duration = max(0.0, float(ramp_seconds))
        else:
            self._arming_duration = max(0.0, float(self._config.default_ramp_seconds))
        self._arm_pending = True
        logger.info(
            "G1SonicWBCTask arm requested",
            task=self._name,
            control_state=self._control_state.value,
        )
        return True

    def disarm(self) -> bool:
        if not self._arm_pending and self._control_state not in (
            SonicControlState.INITIALIZING,
            SonicControlState.READY,
            SonicControlState.CONTROL,
        ):
            return False
        self._arm_pending = False
        self._stream_source_requested = False
        self._control_state = SonicControlState.UNARMED
        self._initialization_started = False
        self._ramp_start = None
        self._last_targets = None
        self._reset_policy_state()
        logger.info(
            "G1SonicWBCTask policy stopped",
            task=self._name,
            control_state=self._control_state.value,
        )
        return True

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        was_armed = self._arm_pending or self._control_state in (
            SonicControlState.INITIALIZING,
            SonicControlState.READY,
            SonicControlState.CONTROL,
        )
        should_reactivate = was_armed if reactivate is None else bool(reactivate)

        self._control_state = (
            SonicControlState.UNARMED if self._active else SonicControlState.STOPPED
        )
        self._arm_pending = self._active and should_reactivate
        self._ramp_start = None
        self._initialization_start_t = 0.0
        self._initialization_started = False
        self._last_targets = None
        self._state_seen = False
        self._stream_source_requested = False
        self._cached_q_29[:] = self._default_29
        self._cached_dq_29[:] = 0.0
        self._reset_policy_state()
        with self._cmd_lock:
            self._cmd[:] = 0.0
            self._last_cmd_time = 0.0

        logger.info(
            "G1SonicWBCTask runtime state reset",
            task=self._name,
            reactivate=should_reactivate,
        )
        return True

    def set_dry_run(self, enabled: bool) -> None:
        new_val = bool(enabled)
        if new_val == self._dry_run:
            return
        self._dry_run = new_val
        self._last_dry_run_log_t = 0.0
        logger.info("G1SonicWBCTask dry_run changed", task=self._name, dry_run=new_val)

    def state_snapshot(self) -> dict[str, Any]:
        snap: dict[str, Any] = {
            "active": self._active,
            "armed": self._control_state is SonicControlState.CONTROL,
            "arming": self._control_state is SonicControlState.INITIALIZING,
            "arm_pending": self._arm_pending,
            "arming_duration": self._arming_duration,
            "control_state": self._control_state.value,
            "dry_run": self._dry_run,
        }
        snap.update(self._pipeline.snapshot())
        snap["reference_source"] = "stream" if snap.get("stream_active") else "planner"
        snap["zmq"] = dict(self._zmq_stats)
        snap["debug_q_leg"] = [round(float(v), 4) for v in self._cached_q_29[:6]]
        snap["debug_dq_leg"] = [round(float(v), 4) for v in self._cached_dq_29[:6]]
        try:
            imu = self._adapter.read_imu()
            snap["debug_quat"] = [round(float(v), 4) for v in imu.quaternion]
            snap["debug_gyro"] = [round(float(v), 4) for v in imu.gyroscope]
        except Exception:
            pass
        return snap

    # -- internal ------------------------------------------------------------

    def _reset_policy_state(self) -> None:
        self._pipeline.reset()
        self._tick_count = 0

    def _enter_control(self) -> None:
        self._control_state = SonicControlState.CONTROL
        self._reset_policy_state()
        self._pipeline.set_source_stream(self._stream_source_requested)
        logger.info(
            "G1SonicWBCTask policy control active",
            task=self._name,
            reference_source="stream" if self._stream_source_requested else "planner",
            mode="dry-run" if self._dry_run else "live",
        )

    def _select_stream_reference(self, use_stream: bool) -> None:
        self._stream_source_requested = bool(use_stream)
        if self.policy_active:
            self._pipeline.set_source_stream(self._stream_source_requested)

    def _return_to_planner_reference(self) -> None:
        self._stream_source_requested = False
        self._pipeline.stop_clip()

    @staticmethod
    def _projected_gravity(quaternion: tuple[float, ...]) -> NDArray[np.float32]:
        w, x, y, z = quaternion
        gx = 2.0 * (-x * z + w * y)
        gy = 2.0 * (-y * z - w * x)
        gz = -(w * w - x * x - y * y + z * z)
        return np.array([gx, gy, gz], dtype=np.float32)


class G1SonicWBCTaskParams(BaseConfig):
    encoder_onnx: str | Path
    decoder_onnx: str | Path
    planner_onnx: str | Path
    hardware_id: str
    auto_arm: bool = False
    auto_dry_run: bool = False
    default_ramp_seconds: float = 3.0
    decimation: int | None = None
    zmq_enabled: bool = True


def _create_task(
    cfg: Any,
    hardware: Any,
    task_class: type[G1SonicWBCTask],
) -> G1SonicWBCTask:
    params = G1SonicWBCTaskParams.model_validate(cfg.params)
    hw = hardware.get(params.hardware_id) if hardware else None
    if hw is None:
        raise ValueError(
            f"G1SonicWBCTask {cfg.name!r} references unknown hardware "
            f"{params.hardware_id!r}. Declare the hardware before the task "
            f"in the blueprint config."
        )
    if not isinstance(hw, ConnectedWholeBody):
        raise TypeError(
            f"G1SonicWBCTask {cfg.name!r} requires a WHOLE_BODY hardware "
            f"component for {params.hardware_id!r}, got {type(hw).__name__}."
        )

    kwargs: dict[str, Any] = dict(
        encoder_onnx=params.encoder_onnx,
        decoder_onnx=params.decoder_onnx,
        planner_onnx=params.planner_onnx,
        joint_names=cfg.joint_names,
        priority=cfg.priority,
        auto_arm=params.auto_arm,
        auto_dry_run=params.auto_dry_run,
        default_ramp_seconds=params.default_ramp_seconds,
        zmq_enabled=params.zmq_enabled,
    )
    if params.decimation is not None:
        kwargs["decimation"] = params.decimation
    return task_class(
        cfg.name,
        G1SonicWBCTaskConfig(**kwargs),
        adapter=hw.adapter,
    )


def create_task(cfg: Any, hardware: Any) -> G1SonicWBCTask:
    return _create_task(cfg, hardware, G1SonicWBCTask)
