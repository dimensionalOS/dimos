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

"""Official MicroDuck policy set as one passive ControlCoordinator task."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.microduck_policy_task.head_kinematics import (
    HEAD_COMMAND_LOWER,
    HEAD_COMMAND_UPPER,
    look_at as solve_look_at,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.pollen.microduck.config import (
    MICRODUCK_HOME,
    MICRODUCK_POSITION_LOWER,
    MICRODUCK_POSITION_UPPER,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Twist import Twist

logger = setup_logger()

OBS_LEN = 61
ACTION_LEN = 14
COMMAND_LEN = 13
CONTROL_HZ = 50

_REQUIRED_POLICIES = frozenset(
    {"walk", "stand", "sitstand", "ground_pick", "roulade", "kick_left", "kick_right"}
)
_PUBLIC_SKILL_ORDER = ("ground_pick", "kick_left", "kick_right", "roulade")
_POLICY_NAME_ALIASES = {
    "alpha_walking": "walk",
    "alpha_stand": "stand",
    "alpha_sitstand": "sitstand",
    "alpha_ground_pick": "ground_pick",
}

_HEAD_SLICE = slice(5, 9)
_TWIST_LIMITS = np.asarray((0.4, 0.3, 1.0), dtype=np.float32)
_BODY_LOWER = np.asarray((-0.025, -0.26, -0.26), dtype=np.float32)
_BODY_UPPER = np.asarray((0.010, 0.26, 0.26), dtype=np.float32)

SessionFactory = Callable[[Path, list[str]], Any]


def _preferred_onnx_providers() -> list[str]:
    available = ort.get_available_providers()
    providers: list[str] = []
    if "CUDAExecutionProvider" in available:
        preload_dlls = getattr(ort, "preload_dlls", None)
        if preload_dlls is not None:
            try:
                preload_dlls(cuda=True, cudnn=True, msvc=False)
            except Exception as exc:
                logger.warning("Failed to preload ONNX Runtime CUDA libraries", error=repr(exc))
        providers.append("CUDAExecutionProvider")
    providers.append("CPUExecutionProvider")
    return providers


def _default_session_factory(path: Path, providers: list[str]) -> ort.InferenceSession:
    return ort.InferenceSession(str(path), providers=providers)


@dataclass(frozen=True)
class PolicyDefinition:
    name: str
    path: Path
    kind: str
    duration_s: float = 0.0
    chainable: bool = False
    action_scale: float | None = None
    command: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class MicroDuckPolicyTaskConfig:
    policy_dir: str | Path
    joint_names: list[str]
    hardware_id: str = "microduck"
    priority: int = 50
    auto_arm: bool = True
    timeout: float = 0.5
    standing_threshold: float = 0.05
    command_alpha: float = 0.2
    head_alpha: float = 0.2
    body_alpha: float = 0.2
    walking_action_scale: float = 0.9
    standing_action_scale: float = 1.0
    head_target_alpha: float = 0.5
    leg_target_alpha: float = 0.7
    session_factory: SessionFactory = _default_session_factory


class MicroDuckPolicyTask(BaseControlTask):
    """Run all walking-mode MicroDuck policies in one shared state machine."""

    def __init__(self, name: str, config: MicroDuckPolicyTaskConfig) -> None:
        if len(config.joint_names) != ACTION_LEN:
            raise ValueError(
                f"MicroDuckPolicyTask {name!r} requires {ACTION_LEN} joints, "
                f"got {len(config.joint_names)}"
            )
        if config.timeout <= 0.0:
            raise ValueError("MicroDuck velocity timeout must be positive")
        for field_name in ("command_alpha", "head_alpha", "body_alpha"):
            value = float(getattr(config, field_name))
            if not 0.0 < value <= 1.0:
                raise ValueError(f"{field_name} must be in (0, 1]")

        self._name = name
        self._config = config
        self._joint_names = list(config.joint_names)
        self._joint_set = frozenset(config.joint_names)
        self._home = np.asarray(MICRODUCK_HOME, dtype=np.float32)
        self._position_lower = np.asarray(MICRODUCK_POSITION_LOWER, dtype=np.float32)
        self._position_upper = np.asarray(MICRODUCK_POSITION_UPPER, dtype=np.float32)
        self._lock = threading.RLock()

        self._definitions, self._sessions, self._io_names = self._load_policy_set(
            Path(config.policy_dir)
        )
        self._sitstand_rise_s = self._manifest_number("sitstand", "unwind_s", fallback=1.0)
        ground_command = self._definitions["ground_pick"].command
        self._ground_period_s = self._finite_positive(
            ground_command.get("period_s", 4.0), "ground_pick command.period_s"
        )
        self._ground_end_phase = self._finite_positive(
            ground_command.get("end_phase", 0.7), "ground_pick command.end_phase"
        )

        self._active = False
        self._armed = False
        self._estopped = False
        self._desired_twist = np.zeros(3, dtype=np.float32)
        self._applied_twist = np.zeros(3, dtype=np.float32)
        self._desired_head = np.zeros(4, dtype=np.float32)
        self._applied_head = np.zeros(4, dtype=np.float32)
        self._desired_body = np.zeros(3, dtype=np.float32)
        self._applied_body = np.zeros(3, dtype=np.float32)
        self._body_active = False
        self._last_twist_time: float | None = None
        self._last_tick_time: float | None = None
        self._last_action = np.zeros(ACTION_LEN, dtype=np.float32)
        self._previous_targets: NDArray[np.float32] | None = None
        self._current_policy: str | None = None
        self._last_error: str | None = None
        self._posture = "standing"
        self._rise_remaining = 0.0
        self._active_skill: str | None = None
        self._skill_remaining = 0.0
        self._skill_chain_window = 0.0
        self._ground_phase: float | None = None

    def _load_policy_set(
        self, policy_dir: Path
    ) -> tuple[dict[str, PolicyDefinition], dict[str, Any], dict[str, tuple[str, str]]]:
        manifest_path = policy_dir / "manifest.json"
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise ValueError(f"Failed to read MicroDuck manifest {manifest_path}: {exc}") from exc
        if not isinstance(manifest, dict):
            raise ValueError(f"{manifest_path}: root must be an object")

        expected = {
            "schema_version": 2,
            "model_api": 1,
            "obs_len": OBS_LEN,
            "action_len": ACTION_LEN,
        }
        for key, wanted in expected.items():
            got = manifest.get(key)
            if got != wanted:
                raise ValueError(f"{manifest_path}: {key} is {got!r}, expected {wanted!r}")
        robot = manifest.get("robot")
        if not isinstance(robot, dict):
            raise ValueError(f"{manifest_path}: robot must be an object")
        robot_expected: dict[str, Any] = {"model": "microduck", "control_hz": CONTROL_HZ}
        for key, wanted in robot_expected.items():
            got = robot.get(key)
            if got != wanted:
                raise ValueError(f"{manifest_path}: robot.{key} is {got!r}, expected {wanted!r}")
        entries = manifest.get("policies")
        if not isinstance(entries, list):
            raise ValueError(f"{manifest_path}: policies must be a list")

        definitions: dict[str, PolicyDefinition] = {}
        entry_data: dict[str, Mapping[str, Any]] = {}
        for raw in entries:
            if not isinstance(raw, dict):
                raise ValueError(f"{manifest_path}: each policy must be an object")
            if raw.get("mode") == "roller":
                continue
            filename = raw.get("file")
            if not isinstance(filename, str) or not filename.endswith(".onnx"):
                raise ValueError(f"{manifest_path}: invalid policy file {filename!r}")
            stem = Path(filename).stem
            explicit_name = raw.get("name")
            if explicit_name is not None and not isinstance(explicit_name, str):
                raise ValueError(f"{manifest_path}: name for {filename} must be a string")
            policy_name = explicit_name or _POLICY_NAME_ALIASES.get(stem, stem)
            if policy_name in definitions:
                raise ValueError(f"{manifest_path}: duplicate walking policy {policy_name!r}")
            kind = raw.get("kind", "episodic")
            if kind not in ("perpetual", "scripted", "episodic"):
                raise ValueError(f"{manifest_path}: invalid kind {kind!r} for {filename}")
            duration = float(raw.get("duration_s", 0.0))
            if not math.isfinite(duration) or duration < 0.0:
                raise ValueError(f"{manifest_path}: invalid duration_s for {filename}")
            scale_raw = raw.get("action_scale")
            scale = None if scale_raw is None else float(scale_raw)
            if scale is not None and (not math.isfinite(scale) or scale <= 0.0):
                raise ValueError(f"{manifest_path}: invalid action_scale for {filename}")
            command = raw.get("command", {})
            if not isinstance(command, dict):
                raise ValueError(f"{manifest_path}: command for {filename} must be an object")
            definitions[policy_name] = PolicyDefinition(
                name=policy_name,
                path=policy_dir / filename,
                kind=kind,
                duration_s=duration,
                chainable=bool(raw.get("chain", False)),
                action_scale=scale,
                command=command,
            )
            entry_data[policy_name] = raw

        missing = sorted(_REQUIRED_POLICIES - definitions.keys())
        extra = sorted(definitions.keys() - _REQUIRED_POLICIES)
        if missing or extra:
            raise ValueError(
                f"{manifest_path}: walking policy set mismatch; missing={missing}, extra={extra}"
            )

        # Keep the fields not represented by PolicyDefinition for manifest-derived
        # sit/rise timing without turning the entire schema into runtime state.
        self._manifest_entries = entry_data
        providers = _preferred_onnx_providers()
        sessions: dict[str, Any] = {}
        io_names: dict[str, tuple[str, str]] = {}
        for policy_name, definition in definitions.items():
            if not definition.path.is_file():
                raise FileNotFoundError(f"MicroDuck policy is missing: {definition.path}")
            try:
                session = self._config.session_factory(definition.path, providers)
                input_name, output_name = self._validate_and_warm_session(session, definition.path)
            except Exception as exc:
                raise ValueError(
                    f"Failed to load MicroDuck policy {definition.path}: {exc}"
                ) from exc
            sessions[policy_name] = session
            io_names[policy_name] = (input_name, output_name)
        logger.info(
            "MicroDuck policy set loaded",
            task=self._name,
            policy_dir=str(policy_dir),
            policies=sorted(sessions),
            requested_providers=providers,
        )
        return definitions, sessions, io_names

    @staticmethod
    def _validate_and_warm_session(session: Any, path: Path) -> tuple[str, str]:
        inputs = session.get_inputs()
        outputs = session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError(
                f"{path}: expected one input and one output, got {len(inputs)} and {len(outputs)}"
            )
        input_meta, output_meta = inputs[0], outputs[0]
        if list(input_meta.shape) != [1, OBS_LEN]:
            raise ValueError(f"{path}: input shape is {input_meta.shape}, expected [1, {OBS_LEN}]")
        if list(output_meta.shape) != [1, ACTION_LEN]:
            raise ValueError(
                f"{path}: output shape is {output_meta.shape}, expected [1, {ACTION_LEN}]"
            )
        for label, meta in (("input", input_meta), ("output", output_meta)):
            if getattr(meta, "type", "tensor(float)") != "tensor(float)":
                raise ValueError(f"{path}: {label} type is {meta.type}, expected tensor(float)")
        zero = np.zeros((1, OBS_LEN), dtype=np.float32)
        raw = session.run([output_meta.name], {input_meta.name: zero})[0]
        action = np.asarray(raw, dtype=np.float32)
        if action.shape != (1, ACTION_LEN) or not np.all(np.isfinite(action)):
            raise ValueError(
                f"{path}: warm-up result must be finite [1, {ACTION_LEN}], got {action.shape}"
            )
        return str(input_meta.name), str(output_meta.name)

    def _manifest_number(self, policy: str, field_name: str, *, fallback: float) -> float:
        return self._finite_positive(
            self._manifest_entries[policy].get(field_name, fallback),
            f"{policy}.{field_name}",
        )

    @staticmethod
    def _finite_positive(raw: Any, label: str) -> float:
        value = float(raw)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{label} must be finite and positive, got {raw!r}")
        return value

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_set,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        with self._lock:
            return self._active

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            self._last_tick_time = state.t_now
            if not self._active or not self._armed or self._estopped:
                return None

            measured = self._read_policy_state(state)
            if measured is None:
                return None
            q, dq, gyro, gravity = measured

            stale = (
                self._last_twist_time is None
                or state.t_now - self._last_twist_time > self._config.timeout
            )
            if stale:
                twist_target = np.zeros(3, dtype=np.float32)
                self._applied_twist[:] = 0.0
            else:
                twist_target = self._desired_twist
                self._ema(self._applied_twist, twist_target, self._config.command_alpha)
            if self._body_active:
                self._applied_twist[:] = 0.0

            self._ema(self._applied_head, self._desired_head, self._config.head_alpha)
            if self._body_active:
                self._ema(self._applied_body, self._desired_body, self._config.body_alpha)
            else:
                self._applied_body[:] = 0.0

            self._expire_windows()
            policy_name, label, command = self._select_policy(stale=stale)
            observation = self._build_observation(q, dq, gyro, gravity, command)
            if not np.all(np.isfinite(observation)):
                self._inference_failure("non-finite MicroDuck observation")
                return None

            try:
                input_name, output_name = self._io_names[policy_name]
                raw = self._sessions[policy_name].run(
                    [output_name], {input_name: observation.reshape(1, OBS_LEN)}
                )[0]
                action = np.asarray(raw, dtype=np.float32)
                if action.shape != (1, ACTION_LEN):
                    raise ValueError(
                        f"runtime output shape {action.shape}, expected (1, {ACTION_LEN})"
                    )
                action = action[0]
                if not np.all(np.isfinite(action)):
                    raise ValueError("runtime output contains non-finite values")
                scale = self._action_scale(policy_name, command[:3])
                targets = self._home + scale * action
                targets = self._filter_targets(targets)
                targets = np.clip(targets, self._position_lower, self._position_upper)
                if not np.all(np.isfinite(targets)):
                    raise ValueError("filtered targets contain non-finite values")
            except Exception as exc:
                self._inference_failure(f"{policy_name} inference failed: {exc}")
                return None

            self._last_action[:] = action
            self._previous_targets = targets.copy()
            self._current_policy = label
            self._advance_windows(max(0.0, state.dt))
            return JointCommandOutput(
                joint_names=self._joint_names,
                positions=[float(value) for value in targets],
                mode=ControlMode.SERVO_POSITION,
            )

    def _read_policy_state(
        self, state: CoordinatorState
    ) -> (
        tuple[
            NDArray[np.float32],
            NDArray[np.float32],
            NDArray[np.float32],
            NDArray[np.float32],
        ]
        | None
    ):
        positions: list[float] = []
        velocities: list[float] = []
        for joint_name in self._joint_names:
            position = state.joints.get_position(joint_name)
            velocity = state.joints.get_velocity(joint_name)
            if position is None or velocity is None:
                return None
            positions.append(float(position))
            velocities.append(float(velocity))
        imu = state.imu.get(self._config.hardware_id)
        if imu is None:
            return None
        q = np.asarray(positions, dtype=np.float32)
        dq = np.asarray(velocities, dtype=np.float32)
        gyro = np.asarray(imu.gyroscope, dtype=np.float32)
        quaternion = np.asarray(imu.quaternion, dtype=np.float64)
        if (
            not np.all(np.isfinite(q))
            or not np.all(np.isfinite(dq))
            or gyro.shape != (3,)
            or not np.all(np.isfinite(gyro))
            or quaternion.shape != (4,)
            or not np.all(np.isfinite(quaternion))
        ):
            self._inference_failure("non-finite or malformed MicroDuck state")
            return None
        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-6:
            self._inference_failure("MicroDuck IMU quaternion has zero norm")
            return None
        quaternion /= norm
        gravity = self._projected_gravity(
            (
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            )
        )
        return q, dq, gyro, gravity

    def _select_policy(self, *, stale: bool) -> tuple[str, str, NDArray[np.float32]]:
        command = np.zeros(COMMAND_LEN, dtype=np.float32)
        if self._active_skill is not None:
            return self._active_skill, self._active_skill, command
        if self._ground_phase is not None:
            angle = math.tau * self._ground_phase
            command[:3] = (math.cos(angle), math.sin(angle), 0.0)
            return "ground_pick", "ground_pick", command

        command[3:7] = self._applied_head
        command[9:12] = self._applied_body
        if self._posture == "sitting":
            command[:3] = (1.0, 0.0, 0.0)
            return "sitstand", "sit", command
        if self._posture == "rising":
            return "sitstand", "rise", command

        if not stale and not self._body_active:
            command[:3] = self._applied_twist
        magnitude = float(np.linalg.norm(command[:3]))
        if self._body_active or stale or magnitude <= self._config.standing_threshold:
            return "stand", "stand", command
        return "walk", "walk", command

    def _build_observation(
        self,
        q: NDArray[np.float32],
        dq: NDArray[np.float32],
        gyro: NDArray[np.float32],
        gravity: NDArray[np.float32],
        command: NDArray[np.float32],
    ) -> NDArray[np.float32]:
        observation = np.empty(OBS_LEN, dtype=np.float32)
        observation[0:3] = gyro
        observation[3:6] = gravity
        observation[6:20] = q - self._home
        observation[20:34] = dq
        observation[34:48] = self._last_action
        observation[48:61] = command
        return observation

    @staticmethod
    def _projected_gravity(quaternion: tuple[float, float, float, float]) -> NDArray[np.float32]:
        w, x, y, z = quaternion
        return np.asarray(
            (
                2.0 * (-x * z + w * y),
                2.0 * (-y * z - w * x),
                -(w * w - x * x - y * y + z * z),
            ),
            dtype=np.float32,
        )

    def _action_scale(self, policy_name: str, effective_twist: NDArray[np.float32]) -> float:
        override = self._definitions[policy_name].action_scale
        if override is not None:
            return override
        if policy_name in ("stand", "sitstand"):
            return self._config.standing_action_scale
        if policy_name in ("roulade", "kick_left", "kick_right"):
            if float(np.linalg.norm(effective_twist)) <= self._config.standing_threshold:
                return self._config.standing_action_scale
        if policy_name == "ground_pick":
            return 1.0
        return self._config.walking_action_scale

    def _filter_targets(self, targets: NDArray[np.float32]) -> NDArray[np.float32]:
        previous = self._previous_targets
        if previous is None:
            return targets
        filtered = targets.copy()
        filtered[_HEAD_SLICE] = (
            self._config.head_target_alpha * targets[_HEAD_SLICE]
            + (1.0 - self._config.head_target_alpha) * previous[_HEAD_SLICE]
        )
        leg_indices = np.asarray((0, 1, 2, 3, 4, 9, 10, 11, 12, 13))
        filtered[leg_indices] = (
            self._config.leg_target_alpha * targets[leg_indices]
            + (1.0 - self._config.leg_target_alpha) * previous[leg_indices]
        )
        return filtered

    @staticmethod
    def _ema(current: NDArray[np.float32], target: NDArray[np.float32], alpha: float) -> None:
        current += alpha * (target - current)

    def _expire_windows(self) -> None:
        if self._active_skill is not None and self._skill_remaining <= 0.0:
            definition = self._definitions[self._active_skill]
            if definition.chainable and self._skill_chain_window > 0.0:
                self._skill_remaining = definition.duration_s
                self._skill_chain_window = 0.0
            else:
                self._active_skill = None
                self._skill_remaining = 0.0
                self._skill_chain_window = 0.0
        if self._posture == "rising" and self._rise_remaining <= 0.0:
            self._posture = "standing"
            self._rise_remaining = 0.0

    def _advance_windows(self, dt: float) -> None:
        if self._ground_phase is not None:
            self._ground_phase += dt / self._ground_period_s
            if self._ground_phase >= self._ground_end_phase:
                self._ground_phase = None
        if self._active_skill is not None:
            self._skill_remaining -= dt
            self._skill_chain_window = max(0.0, self._skill_chain_window - dt)
        if self._posture == "rising":
            self._rise_remaining -= dt

    def _inference_failure(self, reason: str) -> None:
        self._last_error = reason
        self._armed = False
        self._clear_transient_state(clear_intents=True)
        logger.error("MicroDuck policy task disarmed", task=self._name, error=reason)
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_set:
            logger.warning(
                "MicroDuck policy task preempted", task=self._name, by_task=by_task, joints=joints
            )

    def on_twist_command(self, msg: Twist, t_now: float) -> bool:
        """Validate, clamp, and latch a velocity command without running inference."""

        values = np.asarray(
            (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)), dtype=np.float32
        )
        if not np.all(np.isfinite(values)) or not math.isfinite(t_now):
            return False
        with self._lock:
            if not self._active or not self._armed or self._estopped:
                return False
            self._desired_twist[:] = np.clip(values, -_TWIST_LIMITS, _TWIST_LIMITS)
            self._last_twist_time = t_now
            return True

    def start(self) -> None:
        """Activate the passive task and apply the simulation auto-arm setting."""

        with self._lock:
            self._active = True
            self._armed = False
            self._last_error = None
            self._clear_transient_state(clear_intents=True)
            if self._config.auto_arm and not self._estopped:
                self._armed = True
        logger.info("MicroDuck policy task started", task=self._name, armed=self._armed)

    def stop(self) -> None:
        """Deactivate the task and clear every latched intent."""

        with self._lock:
            self._active = False
            self._armed = False
            self._clear_transient_state(clear_intents=True)

    def arm(self) -> dict[str, Any]:
        """Idempotently enable policy output with a zero velocity command."""

        with self._lock:
            if not self._active:
                return self._intent(False, "task is not started")
            if self._estopped:
                return self._intent(False, "E-stop is latched")
            if self._armed:
                return self._intent(True)
            self._clear_transient_state(clear_intents=True)
            self._last_error = None
            self._armed = True
            return self._intent(True)

    def disarm(self) -> dict[str, Any]:
        """Idempotently stop output and clear pending movements."""

        with self._lock:
            self._armed = False
            self._clear_transient_state(clear_intents=True)
            return self._intent(True)

    def stop_motion(self) -> dict[str, Any]:
        """Immediately clear requested and smoothed velocity, preserving one-shots."""

        with self._lock:
            self._desired_twist[:] = 0.0
            self._applied_twist[:] = 0.0
            self._last_twist_time = None
            return self._intent(True)

    def set_head_pose(
        self,
        neck_pitch: float,
        head_pitch: float,
        head_yaw: float,
        head_roll: float,
    ) -> dict[str, Any]:
        """Latch four head command offsets in radians."""

        values = np.asarray((neck_pitch, head_pitch, head_yaw, head_roll), dtype=np.float32)
        if not np.all(np.isfinite(values)):
            return self._intent(False, "head pose must contain only finite values")
        if np.any(values < HEAD_COMMAND_LOWER) or np.any(values > HEAD_COMMAND_UPPER):
            return self._intent(False, "head pose is outside the trained command envelope")
        with self._lock:
            self._desired_head[:] = values
            return self._intent(True)

    def look_at(self, x: float, y: float, z: float, neck_pitch: float = 0.0) -> dict[str, Any]:
        """Solve and latch a camera gaze for a point in the trunk frame."""

        values = (float(x), float(y), float(z), float(neck_pitch))
        if not all(math.isfinite(value) for value in values):
            return self._look_result(False, "look target must contain only finite values")
        gaze = solve_look_at((values[0], values[1], values[2]), values[3])
        with self._lock:
            self._desired_head[:] = gaze.joints
        return self._look_result(True, None, clamped=gaze.clamped, head=gaze.joints)

    def set_body_pose(
        self,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        active: bool = True,
    ) -> dict[str, Any]:
        """Enable or clear the standing body-pose command."""

        with self._lock:
            if not active:
                self._body_active = False
                self._desired_body[:] = 0.0
                self._applied_body[:] = 0.0
                return self._intent(True)
        values = np.asarray((z, roll, pitch), dtype=np.float32)
        if not np.all(np.isfinite(values)):
            return self._intent(False, "body pose must contain only finite values")
        if np.any(values < _BODY_LOWER) or np.any(values > _BODY_UPPER):
            return self._intent(False, "body pose is outside the trained command envelope")
        with self._lock:
            if self._posture != "standing" or self._is_busy():
                return self._intent(False, "body pose requires an idle standing robot")
            self._desired_twist[:] = 0.0
            self._applied_twist[:] = 0.0
            self._last_twist_time = None
            self._desired_body[:] = values
            self._body_active = True
            return self._intent(True)

    def set_posture(self, posture: str) -> dict[str, Any]:
        """Request the idempotent ``sit`` or ``stand`` posture."""

        if posture not in ("sit", "stand"):
            return self._intent(False, "posture must be 'sit' or 'stand'")
        with self._lock:
            if not self._armed or self._estopped:
                return self._intent(False, "task must be armed and not E-stopped")
            if posture == "sit" and self._body_active:
                return self._intent(False, "clear body-pose mode before sitting")
            if self._active_skill is not None or self._ground_phase is not None:
                return self._intent(False, "a scripted motion is already running")
            if posture == "sit":
                if self._posture == "sitting":
                    return self._intent(True)
                if self._posture == "rising":
                    return self._intent(False, "robot is already standing up")
                self._body_active = False
                self._desired_body[:] = 0.0
                self._applied_body[:] = 0.0
                self.stop_motion()
                self._posture = "sitting"
                return self._intent(True)
            if self._posture in ("standing", "rising"):
                return self._intent(True)
            self._posture = "rising"
            self._rise_remaining = self._sitstand_rise_s
            return self._intent(True)

    def run_skill(self, name: str) -> dict[str, Any]:
        """Start one manifest-defined walking-mode one-shot policy."""

        if name not in _PUBLIC_SKILL_ORDER:
            return self._intent(False, f"unknown skill {name!r}")
        with self._lock:
            if name not in self._sessions:
                return self._intent(False, f"skill {name!r} is unavailable")
            if not self._active or not self._armed or self._estopped:
                return self._intent(False, "task must be armed and not E-stopped")
            if self._posture != "standing":
                return self._intent(False, "skills require a standing robot")
            if name == self._active_skill and self._definitions[name].chainable:
                self._skill_chain_window = 0.15
                return self._intent(True)
            if self._is_busy():
                return self._intent(False, "a scripted motion is already running")
            self._body_active = False
            self._desired_body[:] = 0.0
            self._applied_body[:] = 0.0
            self.stop_motion()
            if name == "ground_pick":
                self._ground_phase = 0.0
            else:
                self._active_skill = name
                self._skill_remaining = self._definitions[name].duration_s
                self._skill_chain_window = 0.0
            return self._intent(True)

    def list_skills(self) -> list[dict[str, Any]]:
        """List the exact public one-shot set and manifest-derived metadata."""

        with self._lock:
            return [
                {
                    "name": name,
                    "duration_s": self._skill_duration(name),
                    "chainable": self._definitions[name].chainable,
                    "required_mode": "walk",
                }
                for name in _PUBLIC_SKILL_ORDER
                if name in self._definitions
            ]

    def get_status(self) -> dict[str, Any]:
        """Return a lock-consistent, JSON-serializable task snapshot."""

        with self._lock:
            age = None
            if self._last_tick_time is not None and self._last_twist_time is not None:
                age = max(0.0, self._last_tick_time - self._last_twist_time)
            active_skill = "ground_pick" if self._ground_phase is not None else self._active_skill
            return {
                "active": self._active,
                "armed": self._armed,
                "estopped": self._estopped,
                "busy": self._is_busy(),
                "current_policy": self._current_policy,
                "posture": self._posture,
                "active_skill": active_skill,
                "available_skills": [item["name"] for item in self.list_skills()],
                "applied_twist": {
                    "vx": float(self._applied_twist[0]),
                    "vy": float(self._applied_twist[1]),
                    "yaw_rate": float(self._applied_twist[2]),
                },
                "command_age_s": age,
                "last_error": self._last_error,
            }

    def set_estop(self, estopped: bool) -> None:
        """Latch or clear the coordinator-owned E-stop."""

        with self._lock:
            self._estopped = bool(estopped)
            if self._estopped:
                self._armed = False
                self._clear_transient_state(clear_intents=True)

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        """Clear histories after a simulator discontinuity and optionally re-arm."""

        with self._lock:
            was_armed = self._armed
            self._armed = False
            self._clear_transient_state(clear_intents=True)
            self._last_error = None
            should_reactivate = was_armed if reactivate is None else bool(reactivate)
            if self._active and should_reactivate and not self._estopped:
                self._armed = True
            return True

    def _clear_transient_state(self, *, clear_intents: bool) -> None:
        self._last_action[:] = 0.0
        self._previous_targets = None
        self._current_policy = None
        self._active_skill = None
        self._skill_remaining = 0.0
        self._skill_chain_window = 0.0
        self._ground_phase = None
        self._posture = "standing"
        self._rise_remaining = 0.0
        self._last_tick_time = None
        if clear_intents:
            self._desired_twist[:] = 0.0
            self._applied_twist[:] = 0.0
            self._desired_head[:] = 0.0
            self._applied_head[:] = 0.0
            self._desired_body[:] = 0.0
            self._applied_body[:] = 0.0
            self._body_active = False
            self._last_twist_time = None

    def _is_busy(self) -> bool:
        return (
            self._active_skill is not None
            or self._ground_phase is not None
            or self._posture == "rising"
        )

    def _skill_duration(self, name: str) -> float:
        if name == "ground_pick":
            return self._ground_period_s * self._ground_end_phase
        return self._definitions[name].duration_s

    @staticmethod
    def _intent(accepted: bool, reason: str | None = None) -> dict[str, Any]:
        return {"accepted": accepted, "reason": reason}

    @staticmethod
    def _look_result(
        accepted: bool,
        reason: str | None,
        *,
        clamped: bool = False,
        head: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
    ) -> dict[str, Any]:
        return {
            "accepted": accepted,
            "reason": reason,
            "clamped": clamped,
            "head": dict(
                zip(
                    ("neck_pitch", "head_pitch", "head_yaw", "head_roll"),
                    (float(value) for value in head),
                    strict=True,
                )
            ),
        }


class MicroDuckPolicyTaskParams(BaseConfig):
    policy_dir: str | Path
    hardware_id: str = "microduck"
    auto_arm: bool = True


def create_task(cfg: Any, hardware: Any) -> MicroDuckPolicyTask:
    """Construct a MicroDuck task from its registry envelope."""

    params = MicroDuckPolicyTaskParams.model_validate(cfg.params)
    connected = hardware.get(params.hardware_id) if hardware else None
    if connected is None:
        raise ValueError(
            f"MicroDuckPolicyTask {cfg.name!r} references unknown hardware {params.hardware_id!r}"
        )
    if not isinstance(connected, ConnectedWholeBody):
        raise TypeError(
            f"MicroDuckPolicyTask {cfg.name!r} requires WHOLE_BODY hardware "
            f"{params.hardware_id!r}, got {type(connected).__name__}"
        )
    if list(cfg.joint_names) != connected.joint_names:
        raise ValueError(
            f"MicroDuckPolicyTask {cfg.name!r} joint order must equal hardware order; "
            f"task={cfg.joint_names}, hardware={connected.joint_names}"
        )
    return MicroDuckPolicyTask(
        cfg.name,
        MicroDuckPolicyTaskConfig(
            policy_dir=params.policy_dir,
            joint_names=list(cfg.joint_names),
            hardware_id=params.hardware_id,
            priority=cfg.priority,
            auto_arm=params.auto_arm,
        ),
    )
