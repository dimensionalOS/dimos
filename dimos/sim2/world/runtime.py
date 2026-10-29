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

"""Command, clock and message bridge around a robosuite environment.

The environment owns model construction, physics, controllers, observations,
reset and task semantics. This bridge owns wall-clock command expiry and the
DimOS messages/snapshots published to existing applications.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from pathlib import Path
import tempfile
import threading
import time
from typing import Any, cast
from uuid import uuid4

import mujoco
import numpy as np
from numpy.typing import NDArray
from robosuite.environments.base import MujocoEnv
from robosuite.wrappers.wrapper import Wrapper

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.control import MotorController
from dimos.sim2.environment import MotorScene
from dimos.sim2.messages import EnvironmentStep, FrameMetadata, StateFrame
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.observations import ObservationFrame, collect_observations
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world.scene_types import SceneDescription

STATE = mujoco.mjtState.mjSTATE_INTEGRATION
FeedbackCallback = Callable[[str, JointState, Imu | None, PoseStamped], None]


@dataclass
class RobotBinding:
    config: SimulatedRobot
    controller: MotorController
    root: int
    imu: tuple[int, int, int] | None
    idle_position: NDArray[np.float64]
    action: NDArray[np.float64] | None = None
    action_expires: float = 0.0
    received_command: bool = False
    command_active: bool = False


class SimulationRuntime:
    def __init__(
        self,
        environment: Callable[[], MujocoEnv | Wrapper],
        robots: dict[str, SimulatedRobot] | None = None,
        sensors: tuple[tuple[str | None, Sensor], ...] = (),
        snapshot_hz: float = 60.0,
    ) -> None:
        self.environment = environment()
        self.base_environment = (
            self.environment.unwrapped
            if isinstance(self.environment, Wrapper)
            else self.environment
        )
        self.lock = threading.RLock()
        self.world_id = uuid4().hex
        self._directory = tempfile.TemporaryDirectory(prefix="dimos-robosuite-")
        self._robot_configs = robots or {}
        self.sensors = sensors
        self.snapshot_hz = snapshot_hz
        self.robots: dict[str, RobotBinding] = {}
        self.episode = 0
        self.tick = 0
        self.paused = False
        self._sequence = 0
        self._clock_origin = time.time()
        self._reset_wall_time = 0.0
        self._closed = False
        self._observed: dict[str, tuple[int, float]] = {}
        self._action: NDArray[np.float64] | None = None
        self.publish_feedback: FeedbackCallback | None = None
        self.publish_snapshot: Callable[[StateFrame], None] | None = None
        self.publish_observation: Callable[[ObservationFrame], None] | None = None
        self.publish_environment: Callable[[EnvironmentStep], None] | None = None
        self.last_snapshot: StateFrame | None = None
        self.result: EnvironmentStep | None = None
        self.description = (
            self.base_environment.description
            if isinstance(self.base_environment, MotorScene)
            else SceneDescription(id=type(self.environment).__name__)
        )
        try:
            self.reset()
        except BaseException:
            self.close()
            raise

    @property
    def model(self) -> mujoco.MjModel:
        return cast("mujoco.MjModel", self.environment.sim.model._model)

    @property
    def data(self) -> mujoco.MjData:
        return cast("mujoco.MjData", self.environment.sim.data._data)

    @property
    def step_seconds(self) -> float:
        return float(self.environment.control_timestep)

    def describe(self) -> dict[str, Any]:
        prefixes = (
            {
                name: model.naming_prefix
                for name, model in self.base_environment.robot_models.items()
            }
            if isinstance(self.base_environment, MotorScene)
            else {}
        )
        roots = (
            {
                name: model.root_body.removeprefix(model.naming_prefix)
                for name, model in self.base_environment.robot_models.items()
            }
            if isinstance(self.base_environment, MotorScene)
            else {}
        )
        return dict(
            model=str(self._model_path),
            physics_dt=float(self.model.opt.timestep),
            snapshot_stride=self.snapshot_stride,
            snapshot_slots=self.snapshot_slots,
            model_prefixes=prefixes,
            root_bodies=roots,
        )

    def _bind(self) -> None:
        env = self.base_environment
        if self._robot_configs and not isinstance(env, MotorScene):
            raise TypeError(
                "motor device bindings require MotorScene; native tasks use their own actions"
            )
        self.robots.clear()
        if isinstance(env, MotorScene):
            if set(env.robot_models) != set(self._robot_configs):
                raise ValueError("motor bindings must match the environment's robot models")
            for name, model in env.robot_models.items():
                config = self._robot_configs[name]
                controller = MotorController(
                    env.sim,
                    model,
                    scale=config.position_scale or None,
                    offset=config.position_offset or None,
                )
                env.controllers[name] = controller
                if len(controller.actuators) != len(config.joint_names):
                    raise ValueError(f"{name}: hardware motor names do not match model actuators")
                position_only = {
                    joint
                    for joint, servo in zip(
                        config.joint_names, controller.position_servo, strict=True
                    )
                    if servo
                }
                if position_only != set(config.position_only_joints):
                    raise ValueError(
                        f"{name}: position_only_joints must match the model's native servos: {sorted(position_only)}"
                    )
                imu = None
                if config.control_imu is not None:
                    addresses = tuple(
                        int(
                            self.model.sensor(
                                model.correct_naming(f"{config.control_imu}_{suffix}")
                            ).adr[0]
                        )
                        for suffix in ("gyro", "accel", "quat")
                    )
                    imu = (addresses[0], addresses[1], addresses[2])
                self.robots[name] = RobotBinding(
                    config,
                    controller,
                    self.model.body(model.root_body).id,
                    imu,
                    controller.positions.copy(),
                )
        self.waiting_for_control = {
            name for name, binding in self.robots.items() if binding.config.wait_for_first_command
        }
        rate = self.snapshot_hz
        history = 0.0
        for _, device in self.sensors:
            if isinstance(device, Lidar):
                rate = max(rate, device.acquisition.sample_hz)
                history = max(history, device.acquisition.history_seconds)
                if device.acquisition.sample_hz > 1 / self.step_seconds + 1e-9:
                    raise ValueError(
                        "rolling sensor acquisition cannot exceed the environment step rate"
                    )
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("snapshot rate must be finite and positive")
        cadence = max(1, math.floor(1 / rate / self.step_seconds))
        self.snapshot_stride = cadence * max(
            1, round(self.step_seconds / float(self.model.opt.timestep))
        )
        self._snapshot_seconds = self.snapshot_stride * float(self.model.opt.timestep)
        self._last_snapshot_time = -math.inf
        self.snapshot_slots = max(2, math.ceil(history / self._snapshot_seconds) + 2)
        self._state = np.empty(mujoco.mj_stateSize(self.model, STATE))
        self._model_path = Path(self._directory.name) / f"world-{self.episode}.mjb"
        mujoco.mj_saveModel(self.model, str(self._model_path), None)

    def reset(self) -> None:
        with self.lock:
            self._require_live()
            observations = self.environment.reset()
            self.episode += 1
            self._reset_wall_time = time.time()
            self._clock_origin = self._reset_wall_time - float(self.data.time)
            self._observed.clear()
            self._action = None
            self._bind()
            self._publish(observations, force_snapshot=True, environment_result=True)

    def accept_command(self, robot_id: str, command: MotorCommandArray) -> bool:
        with self.lock:
            binding = self.robots[robot_id]
            rows = (command.q, command.dq, command.kp, command.kd, command.tau)
            if any(len(row) != len(binding.config.joint_names) for row in rows):
                return False
            values = np.asarray(rows, dtype=np.float64)
            if not np.isfinite(values).all() or np.any(values[2:4] < 0):
                return False
            if np.any(values[1:, binding.controller.position_servo] != 0):
                return False
            issued = command.timestamp
            if issued is not None and (not math.isfinite(issued) or issued < self._reset_wall_time):
                return False
            binding.action = values
            binding.action_expires = time.monotonic() + binding.config.command_timeout
            binding.received_command = True
            return True

    def accept_action(self, action: list[float]) -> None:
        """Set an action in the original upstream task's action space."""
        with self.lock:
            if self.robots:
                raise ValueError("motor-bound environments accept motor_command messages")
            values = np.asarray(action, dtype=np.float64)
            lower, upper = self.environment.action_spec
            if values.shape != np.shape(lower) or not np.isfinite(values).all():
                raise ValueError("action must have the environment's dimension and finite values")
            if np.any(values < lower) or np.any(values > upper):
                raise ValueError("action is outside the environment's action_spec")
            self._action = values

    def set_paused(self, paused: bool) -> None:
        with self.lock:
            self._require_live()
            resumed = self.paused and not paused
            self.paused = paused
            if resumed:
                self.episode += 1
                self._clock_origin = time.time() - float(self.data.time)
                self._reset_wall_time = time.time()
                self._observed.clear()
                self._action = None
                for binding in self.robots.values():
                    binding.action = None
                    binding.idle_position = binding.controller.positions.copy()
                    binding.command_active = False
                self._publish({}, force_snapshot=True)

    def _motor_action(self, binding: RobotBinding, now: float) -> NDArray[np.float64]:
        if binding.action is not None and now <= binding.action_expires:
            binding.command_active = True
            return binding.action.ravel()
        if binding.command_active:
            binding.idle_position = binding.controller.positions.copy()
        binding.command_active = False
        binding.action = None
        count = len(binding.config.joint_names)
        action = np.zeros((5, count))
        action[0] = binding.idle_position
        action[2] = binding.config.startup_kp or (0.0,) * count
        action[3] = binding.config.startup_kd or (0.0,) * count
        if binding.received_command and binding.config.halt_damping is not None:
            action[2] = 0
            action[3] = binding.config.halt_damping
        action[1:, binding.controller.position_servo] = 0
        return action.ravel()

    def step(self) -> None:
        with self.lock:
            self._require_live()
            if self.paused or self.environment.done:
                self._publish({}, force_snapshot=True)
                return
            if self.robots:
                now = time.monotonic()
                action = np.concatenate(
                    [self._motor_action(binding, now) for binding in self.robots.values()]
                )
                self.waiting_for_control.difference_update(
                    name for name, binding in self.robots.items() if binding.command_active
                )
                if self.waiting_for_control:
                    self._clock_origin = time.time() - float(self.data.time)
                    self._publish({}, force_snapshot=True)
                    return
            else:
                action = (
                    self._action
                    if self._action is not None
                    else np.zeros_like(self.environment.action_spec[0], dtype=np.float64)
                )
            before = float(self.data.time)
            observations, reward, done, info = self.environment.step(action)
            self.environment.sim.forward()
            self.tick += round((float(self.data.time) - before) / float(self.model.opt.timestep))
            self._publish(
                observations,
                reward=float(reward),
                done=bool(done),
                info=info,
                environment_result=True,
            )

    def _publish(
        self,
        observations: dict[str, Any],
        *,
        force_snapshot: bool = False,
        reward: float = 0.0,
        done: bool = False,
        info: dict[str, Any] | None = None,
        environment_result: bool = False,
    ) -> None:
        timestamp = self._clock_origin + float(self.data.time)
        self._sequence += 1
        metadata = FrameMetadata(self._sequence, self.episode, self.tick, float(self.data.time))
        if self.publish_feedback is not None:
            for name, binding in self.robots.items():
                controller = binding.controller
                joints = JointState(
                    name=list(binding.config.joint_names),
                    position=controller.positions.tolist(),
                    velocity=controller.velocities.tolist(),
                    effort=self.data.qfrc_actuator[controller.dofs].tolist(),
                    ts=timestamp,
                    frame_id=binding.config.frame_id,
                )
                imu = None
                if binding.imu is not None:
                    g, a, q = binding.imu
                    w, x, y, z = self.data.sensordata[q : q + 4]
                    imu = Imu(
                        orientation=Quaternion(x, y, z, w),
                        angular_velocity=Vector3(*self.data.sensordata[g : g + 3]),
                        linear_acceleration=Vector3(*self.data.sensordata[a : a + 3]),
                        frame_id=binding.config.frame_id,
                        ts=timestamp,
                    )
                w, x, y, z = self.data.xquat[binding.root]
                odom = PoseStamped(
                    position=Vector3(*self.data.xpos[binding.root]),
                    orientation=Quaternion(x, y, z, w),
                    frame_id="world",
                    ts=timestamp,
                )
                self.publish_feedback(name, joints, imu, odom)
        if self.publish_observation is not None:
            for frame in collect_observations(
                observations, self.sensors, metadata, self._clock_origin, self._observed
            ):
                self.publish_observation(frame)
        if environment_result:
            self.result = EnvironmentStep(metadata, observations, reward, done, info or {})
            if self.publish_environment is not None:
                self.publish_environment(self.result)
        if (
            force_snapshot
            or float(self.data.time) - self._last_snapshot_time >= self._snapshot_seconds - 1e-9
        ):
            self._last_snapshot_time = float(self.data.time)
            mujoco.mj_getState(self.model, self.data, self._state, STATE)
            snapshot = StateFrame(
                metadata,
                {"state": self._state.copy(), "wall_time": np.array([timestamp])},
                str(self._model_path),
            )
            self.last_snapshot = snapshot
            if self.publish_snapshot is not None:
                self.publish_snapshot(snapshot)

    def _require_live(self) -> None:
        if self._closed:
            raise RuntimeError("sim2 environment is closed")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.environment.close()
        finally:
            self._directory.cleanup()

    def __enter__(self) -> SimulationRuntime:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
