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

"""Unitree Go2 velocity policy for the DimOS ControlCoordinator."""

from __future__ import annotations

from dataclasses import dataclass
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
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Twist import Twist

logger = setup_logger()

GO2_JOINT_SUFFIXES = (
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
)
GO2_DEFAULT_JOINT_POSITIONS = (
    -0.1,
    0.9,
    -1.8,
    0.1,
    0.9,
    -1.8,
    -0.1,
    0.9,
    -1.8,
    0.1,
    0.9,
    -1.8,
)
GO2_POLICY_KP = (20.0, 20.0, 40.0) * 4
GO2_POLICY_KD = (1.0, 1.0, 2.0) * 4

_ACTION_SCALE = 0.5
_COMMAND_SCALE = np.asarray((1.0, 1.0, 1.5), dtype=np.float32)
_COMMAND_MIN = np.asarray((-1.0, -1.0, -1.0), dtype=np.float32)
_COMMAND_MAX = np.asarray((2.0, 1.0, 1.0), dtype=np.float32)
_DEFAULT_POSITION = np.asarray(GO2_DEFAULT_JOINT_POSITIONS, dtype=np.float32)
_INPUT_SIZE = 45
_ACTION_SIZE = 12


@dataclass(frozen=True)
class Go2VelocityPolicyTaskConfig:
    model_path: str | Path
    hardware_id: str
    joint_names: list[str]
    priority: int = 50
    timeout: float = 0.5


class Go2VelocityPolicyTask(BaseControlTask):
    """Convert body velocity commands into Go2 joint position targets."""

    def __init__(self, name: str, config: Go2VelocityPolicyTaskConfig) -> None:
        expected_names = [f"{config.hardware_id}/{suffix}" for suffix in GO2_JOINT_SUFFIXES]
        if config.joint_names != expected_names:
            raise ValueError(
                f"Go2VelocityPolicyTask {name!r} requires the Go2 policy joint order; "
                f"expected {expected_names}, got {config.joint_names}"
            )
        if config.timeout <= 0.0:
            raise ValueError("Go2 velocity command timeout must be positive")

        model_path = Path(config.model_path).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"Go2 velocity policy not found: {model_path}")
        session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        inputs = session.get_inputs()
        outputs = session.get_outputs()
        if len(inputs) != 1 or inputs[0].shape != [1, _INPUT_SIZE]:
            raise ValueError(f"Go2 policy must have one [1, {_INPUT_SIZE}] input, got {inputs}")
        if len(outputs) != 1 or outputs[0].shape != [1, _ACTION_SIZE]:
            raise ValueError(f"Go2 policy must have one [1, {_ACTION_SIZE}] output, got {outputs}")

        self._name = name
        self._config = config
        self._joint_names = list(config.joint_names)
        self._joint_set = frozenset(config.joint_names)
        self._session = session
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name
        self._last_action = np.zeros(_ACTION_SIZE, dtype=np.float32)
        self._command = np.zeros(3, dtype=np.float32)
        self._last_command_time: float | None = None
        self._command_lock = threading.Lock()
        self._active = False

        logger.info(
            "Go2VelocityPolicyTask loaded ONNX model",
            task=name,
            model=str(model_path),
            providers=session.get_providers(),
        )

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_set,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        return self._active

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self._active:
            return None

        joint_position = self._joint_vector(state, velocity=False)
        joint_velocity = self._joint_vector(state, velocity=True)
        imu = state.imu.get(self._config.hardware_id)
        if joint_position is None or joint_velocity is None or imu is None:
            return None

        with self._command_lock:
            timed_out = (
                self._last_command_time is not None
                and state.t_now - self._last_command_time > self._config.timeout
            )
            command = np.zeros(3, dtype=np.float32) if timed_out else self._command.copy()

        scaled_command = np.clip(
            command * _COMMAND_SCALE,
            _COMMAND_MIN,
            _COMMAND_MAX,
        )
        policy_observation = np.concatenate(
            (
                np.asarray(imu.gyroscope, dtype=np.float32),
                _projected_gravity(imu.quaternion),
                scaled_command,
                joint_position - _DEFAULT_POSITION,
                joint_velocity,
                self._last_action,
            ),
            dtype=np.float32,
        )
        prediction = np.asarray(
            self._session.run(
                [self._output_name],
                {self._input_name: policy_observation.reshape(1, _INPUT_SIZE)},
            )[0],
            dtype=np.float32,
        )
        if prediction.shape != (1, _ACTION_SIZE) or not np.all(np.isfinite(prediction)):
            raise ValueError("Go2 policy output must contain one finite 12-element action vector")
        action = prediction[0]
        self._last_action[:] = action
        targets = action * _ACTION_SCALE + _DEFAULT_POSITION
        return JointCommandOutput(
            joint_names=self._joint_names,
            positions=targets.astype(float).tolist(),
            mode=ControlMode.SERVO_POSITION,
        )

    def on_twist_command(self, msg: Twist, t_now: float) -> None:
        command = np.asarray(
            (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)),
            dtype=np.float32,
        )
        if not np.all(np.isfinite(command)):
            raise ValueError("Go2 velocity command must be finite")
        with self._command_lock:
            self._command[:] = command
            self._last_command_time = t_now

    def start(self) -> None:
        self._active = True
        self._reset_policy_state()
        logger.info("Go2VelocityPolicyTask started", task=self._name)

    def stop(self) -> None:
        self._active = False
        logger.info("Go2VelocityPolicyTask stopped", task=self._name)

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        was_active = self._active
        self._reset_policy_state()
        if reactivate is not None:
            self._active = bool(reactivate)
        return was_active

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_set:
            logger.warning(
                "Go2VelocityPolicyTask preempted",
                task=self._name,
                by_task=by_task,
                joints=joints,
            )

    def _reset_policy_state(self) -> None:
        self._last_action.fill(0.0)
        with self._command_lock:
            self._command.fill(0.0)
            self._last_command_time = None

    def _joint_vector(
        self,
        state: CoordinatorState,
        *,
        velocity: bool,
    ) -> NDArray[np.float32] | None:
        getter = state.joints.get_velocity if velocity else state.joints.get_position
        values = [getter(name) for name in self._joint_names]
        if any(value is None for value in values):
            return None
        result = np.asarray(values, dtype=np.float32)
        if not np.all(np.isfinite(result)):
            raise ValueError("Go2 joint state must be finite")
        return result


class Go2VelocityPolicyTaskParams(BaseConfig):
    model_path: str | Path
    hardware_id: str
    timeout: float = 0.5


def create_task(cfg: Any, hardware: Any) -> Go2VelocityPolicyTask:
    params = Go2VelocityPolicyTaskParams.model_validate(cfg.params)
    whole_body = hardware.get(params.hardware_id) if hardware else None
    if whole_body is None:
        raise ValueError(
            f"Go2VelocityPolicyTask {cfg.name!r} references unknown hardware {params.hardware_id!r}"
        )
    if not isinstance(whole_body, ConnectedWholeBody):
        raise TypeError(
            f"Go2VelocityPolicyTask {cfg.name!r} requires WHOLE_BODY hardware "
            f"for {params.hardware_id!r}, got {type(whole_body).__name__}"
        )
    if list(cfg.joint_names) != whole_body.joint_names:
        raise ValueError(
            f"Go2VelocityPolicyTask {cfg.name!r} must claim all joints of "
            f"hardware {params.hardware_id!r} in hardware order"
        )
    return Go2VelocityPolicyTask(
        cfg.name,
        Go2VelocityPolicyTaskConfig(
            model_path=params.model_path,
            hardware_id=params.hardware_id,
            joint_names=list(cfg.joint_names),
            priority=cfg.priority,
            timeout=params.timeout,
        ),
    )


def _projected_gravity(
    quaternion_wxyz: tuple[float, float, float, float],
) -> NDArray[np.float32]:
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float32)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("Go2 IMU quaternion is invalid")
    w, x, y, z = quaternion / norm
    rotation = np.asarray(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=np.float32,
    )
    return rotation.T @ np.asarray((0.0, 0.0, -1.0), dtype=np.float32)


__all__ = [
    "GO2_DEFAULT_JOINT_POSITIONS",
    "GO2_JOINT_SUFFIXES",
    "GO2_POLICY_KD",
    "GO2_POLICY_KP",
    "Go2VelocityPolicyTask",
    "Go2VelocityPolicyTaskConfig",
    "create_task",
]
