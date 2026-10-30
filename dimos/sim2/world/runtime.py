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

"""Command ownership, clocks and publications above a replaceable physics backend."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
import math
import threading
import time
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.messages import EnvironmentStep, FrameMetadata, StateFrame
from dimos.sim2.physics import (
    BackendStep,
    CommandBatch,
    CommandReport,
    JointDrive,
    PhysicsBackend,
    PhysicsCommand,
    WorldState,
)
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.observations import ObservationFrame, collect_observations
from dimos.sim2.spec import RobotInstanceSpec, VelocityController
from dimos.sim2.world.scene_types import SceneDescription

FeedbackCallback = Callable[[str, JointState, Imu | None, PoseStamped], None]


@dataclass
class RobotBinding:
    config: RobotInstanceSpec
    idle_position: NDArray[np.float64]
    action: NDArray[np.float64] | None = None
    action_expires: float = 0.0
    received_command: bool = False
    command_active: bool = False
    velocity_controller: VelocityController | None = None
    velocity: Twist | None = None


@dataclass
class ActiveBatch:
    batch: CommandBatch
    expires: float
    applied: bool = False


class SimulationRuntime:
    def __init__(
        self,
        environment: Callable[[], Any] | None = None,
        robots: dict[str, RobotInstanceSpec] | None = None,
        sensors: tuple[tuple[str | None, Sensor], ...] = (),
        snapshot_hz: float = 60.0,
        *,
        backend: PhysicsBackend | None = None,
    ) -> None:
        if backend is None:
            if environment is None:
                raise ValueError("supply a physics backend or a native robosuite task factory")
            # Native task compatibility loads this optional engine only when selected.
            from dimos.sim2.backends.mujoco import MujocoBackend

            backend = MujocoBackend(environment, robots or {}, sensors, snapshot_hz)
        self.backend: PhysicsBackend = backend
        self.lock = threading.RLock()
        self._robot_configs = robots or {}
        self.sensors = sensors
        if not math.isfinite(snapshot_hz) or snapshot_hz <= 0:
            raise ValueError("snapshot rate must be finite and positive")
        self._snapshot_hz = max(
            [snapshot_hz]
            + [sensor.acquisition.sample_hz for _, sensor in sensors if isinstance(sensor, Lidar)]
        )
        self._snapshot_seconds = 1 / self._snapshot_hz
        self._last_snapshot_time = -math.inf
        self.robots: dict[str, RobotBinding] = {}
        self.episode = 0
        self.tick = 0
        self.paused = False
        self.waiting_for_control: set[str] = set()
        self._sequence = 0
        self._clock_origin = time.time()
        self._reset_wall_time = 0.0
        self._closed = False
        self._built = False
        self._observed: dict[str, tuple[int, float]] = {}
        self._batches: dict[str, ActiveBatch] = {}
        self.publish_feedback: FeedbackCallback | None = None
        self.publish_snapshot: Callable[[StateFrame], None] | None = None
        self.publish_state: Callable[[WorldState], None] | None = None
        self.publish_report: Callable[[CommandReport], None] | None = None
        self.publish_observation: Callable[[ObservationFrame], None] | None = None
        self.publish_environment: Callable[[EnvironmentStep], None] | None = None
        self.last_snapshot: StateFrame | None = None
        self.last_state = WorldState(FrameMetadata(0, 0, 0, 0.0), {}, {}, {})
        self.result: EnvironmentStep | None = None

    def build(self) -> None:
        with self.lock:
            if self._built:
                return
            self._require_live()
            try:
                self.backend.build()
                self._snapshot_seconds = (
                    max(1, math.floor(1 / self._snapshot_hz / self.step_seconds))
                    * self.step_seconds
                )
                self._built = True
                self.reset()
            except BaseException:
                self.close()
                raise

    @property
    def step_seconds(self) -> float:
        return self.backend.step_seconds

    @property
    def sim_time(self) -> float:
        return self.backend.sim_time

    @property
    def description(self) -> SceneDescription:
        return self.backend.description.scene

    def describe(self) -> dict[str, Any]:
        """Engine-private sensor checkpoint description; applications use WorldState."""
        return self.backend.sensor_description()

    def _metadata(self) -> FrameMetadata:
        return FrameMetadata(self._sequence, self.episode, self.tick, self.sim_time)

    def reset(self) -> None:
        with self.lock:
            self._require_live()
            result = self.backend.reset()
            self.episode += 1
            self._reset_wall_time = time.time()
            self._clock_origin = self._reset_wall_time - self.sim_time
            self._observed.clear()
            self._batches.clear()
            self._last_snapshot_time = -math.inf
            initial = self.backend.read_state(self._metadata())
            self.robots = {}
            for name, config in self._robot_configs.items():
                state = initial.robots[name]
                policy = (
                    config.velocity_controller(self.step_seconds)
                    if config.velocity_controller
                    else None
                )
                if policy is not None:
                    policy.reset()
                self.robots[name] = RobotBinding(
                    config, np.asarray(state.position), velocity_controller=policy
                )
            self.waiting_for_control = {
                name
                for name, binding in self.robots.items()
                if binding.config.wait_for_first_command
            }
            self._publish(result, force_snapshot=True, task_result=True)

    @staticmethod
    def _command_values(
        binding: RobotBinding, command: MotorCommandArray
    ) -> NDArray[np.float64] | None:
        rows = (command.q, command.dq, command.kp, command.kd, command.tau)
        if any(len(row) != len(binding.config.joint_names) for row in rows):
            return None
        values = np.asarray(rows, dtype=np.float64)
        if not np.isfinite(values).all() or np.any(values[2:4] < 0):
            return None
        servo = [name in binding.config.position_only_joints for name in binding.config.joint_names]
        if np.any(values[1:, servo] != 0):
            return None
        return values

    def accept_command(self, robot_id: str, command: MotorCommandArray) -> bool:
        with self.lock:
            binding = self.robots[robot_id]
            if binding.config.command_kind != "motor":
                return False
            values = self._command_values(binding, command)
            if values is None:
                return False
            if not math.isfinite(command.timestamp) or command.timestamp < self._reset_wall_time:
                return False
            binding.action = values
            binding.action_expires = time.monotonic() + binding.config.command_timeout
            binding.received_command = True
            return True

    def accept_velocity(self, robot_id: str, command: Twist) -> bool:
        with self.lock:
            binding = self.robots[robot_id]
            if binding.config.command_kind != "velocity":
                return False
            if not all(
                map(math.isfinite, (*command.linear.to_tuple(), *command.angular.to_tuple()))
            ):
                return False
            if command.linear.z != 0 or command.angular.x != 0 or command.angular.y != 0:
                return False
            binding.velocity = Twist(command)
            binding.action_expires = time.monotonic() + binding.config.command_timeout
            binding.received_command = True
            return True

    def accept_action(self, action: list[float]) -> None:
        with self.lock:
            self.backend.set_task_action(action)

    def submit(self, batch: CommandBatch) -> CommandReport:
        with self.lock:
            self._require_live()
            self._expire_batches()
            reason = ""
            if batch.epoch != self.episode:
                reason = "command belongs to a different reset epoch"
            elif not batch.command_id or batch.command_id in self._batches:
                reason = "command_id must be nonempty and unique among active commands"
            elif not math.isfinite(batch.duration) or batch.duration < 0:
                reason = "duration must be finite and nonnegative"
            else:
                try:
                    active = tuple(
                        command
                        for entry in self._batches.values()
                        for command in entry.batch.commands
                    )
                    self.backend.validate((*active, *batch.commands))
                except (ValueError, NotImplementedError) as error:
                    reason = str(error)
            status: Literal["rejected", "accepted"] = "rejected" if reason else "accepted"
            report = CommandReport(batch.command_id, self.episode, self.tick, status, reason)
            if not reason:
                self._batches[batch.command_id] = ActiveBatch(batch, self.sim_time + batch.duration)
            if self.publish_report is not None:
                self.publish_report(report)
            return report

    def _expire_batches(self) -> None:
        for command_id, entry in list(self._batches.items()):
            if entry.applied and self.sim_time >= entry.expires - 1e-10:
                del self._batches[command_id]
                if self.publish_report is not None:
                    self.publish_report(
                        CommandReport(command_id, self.episode, self.tick, "expired")
                    )

    def set_paused(self, paused: bool) -> None:
        with self.lock:
            self._require_live()
            resumed = self.paused and not paused
            self.paused = paused
            if resumed:
                self.episode += 1
                self._clock_origin = time.time() - self.sim_time
                self._reset_wall_time = time.time()
                self._observed.clear()
                self._batches.clear()
                state = self.backend.read_state(self._metadata())
                for name, binding in self.robots.items():
                    binding.action = None
                    binding.velocity = None
                    binding.idle_position = np.asarray(state.robots[name].position)
                    binding.command_active = False
                    if binding.velocity_controller is not None:
                        binding.velocity_controller.reset()
                self._publish(BackendStep(), force_snapshot=True)

    def _motor_drives(self, name: str, binding: RobotBinding, now: float) -> tuple[JointDrive, ...]:
        config = binding.config
        state = self.last_state.robots[name]
        values = None
        if binding.velocity_controller is not None:
            binding.command_active = binding.velocity is not None and now <= binding.action_expires
            command = binding.velocity if binding.command_active else None
            if command is None:
                binding.velocity = None
            values = self._command_values(
                binding, binding.velocity_controller.step(state, command or Twist())
            )
            if values is None:
                raise ValueError(f"{name}: locomotion policy returned invalid motor commands")
        elif binding.action is not None and now <= binding.action_expires:
            binding.command_active = True
            values = binding.action
        else:
            if binding.command_active:
                binding.idle_position = np.asarray(state.position)
            binding.command_active = False
            binding.action = None
            count = len(config.joint_names)
            values = np.zeros((5, count))
            values[0] = binding.idle_position
            values[2] = config.startup_kp or (0.0,) * count
            values[3] = config.startup_kd or (0.0,) * count
            if binding.received_command and config.halt_damping is not None:
                values[2] = 0
                values[3] = config.halt_damping
            servo = [joint in config.position_only_joints for joint in config.joint_names]
            values[1:, servo] = 0
        scale = config.position_scale or (1.0,) * len(config.joint_names)
        offset = config.position_offset or (0.0,) * len(config.joint_names)
        return tuple(
            JointDrive(joint, float(q * s + o), float(dq * s), float(kp), float(kd), float(tau))
            for joint, (q, dq, kp, kd, tau), s, o in zip(
                self.backend.description.robot_joints[name], values.T, scale, offset, strict=True
            )
        )

    def step(self) -> None:
        with self.lock:
            self._require_live()
            if self.paused or self.backend.done:
                self._publish(BackendStep(), force_snapshot=True)
                return
            now = time.monotonic()
            self.waiting_for_control.difference_update(
                name
                for name, binding in self.robots.items()
                if (binding.action is not None or binding.velocity is not None)
                and now <= binding.action_expires
            )
            if self.waiting_for_control:
                self._clock_origin = time.time() - self.sim_time
                self._publish(BackendStep(), force_snapshot=True)
                return
            drives = {
                drive.joint: drive
                for name, binding in self.robots.items()
                for drive in self._motor_drives(name, binding, now)
            }
            other: list[PhysicsCommand] = []
            active: list[ActiveBatch] = []
            self._expire_batches()
            for entry in self._batches.values():
                active.append(entry)
                for command in entry.batch.commands:
                    if isinstance(command, JointDrive):
                        drives[command.joint] = command
                    else:
                        other.append(command)
            before = self.sim_time
            result = self.backend.step((*drives.values(), *other))
            self.tick += round((self.sim_time - before) / self.backend.physics_dt)
            for entry in active:
                if not entry.applied and self.publish_report is not None:
                    self.publish_report(
                        CommandReport(entry.batch.command_id, self.episode, self.tick, "applied")
                    )
                entry.applied = True
            self._publish(result, task_result=True)

    def _publish(
        self, result: BackendStep, *, force_snapshot: bool = False, task_result: bool = False
    ) -> None:
        timestamp = self._clock_origin + self.sim_time
        self._sequence += 1
        metadata = self._metadata()
        self.last_state = replace(self.backend.read_state(metadata), wall_time=timestamp)
        if self.publish_state is not None:
            self.publish_state(self.last_state)
        if self.publish_feedback is not None:
            for name, state in self.last_state.robots.items():
                config = self._robot_configs[name]
                joints = JointState(
                    name=list(state.joint_names),
                    position=list(state.position),
                    velocity=list(state.velocity),
                    effort=list(state.effort),
                    ts=timestamp,
                    frame_id=config.frame_id,
                )
                sensor = state.imu
                imu = (
                    None
                    if sensor is None
                    else Imu(
                        orientation=Quaternion(*sensor.orientation),
                        angular_velocity=Vector3(*sensor.angular_velocity),
                        linear_acceleration=Vector3(*sensor.linear_acceleration),
                        frame_id=config.frame_id,
                        ts=timestamp,
                    )
                )
                odom = PoseStamped(
                    position=Vector3(*state.base.position),
                    orientation=Quaternion(*state.base.orientation),
                    frame_id="world",
                    ts=timestamp,
                )
                self.publish_feedback(name, joints, imu, odom)
        if self.publish_observation is not None:
            for frame in collect_observations(
                result.observations, self.sensors, metadata, self._clock_origin, self._observed
            ):
                self.publish_observation(frame)
        if task_result:
            self.result = EnvironmentStep(
                metadata, result.observations, result.reward, result.done, result.info
            )
            if self.publish_environment is not None:
                self.publish_environment(self.result)
        if (
            force_snapshot
            or self.sim_time - self._last_snapshot_time >= self._snapshot_seconds - 1e-9
        ):
            self._last_snapshot_time = self.sim_time
            snapshot = self.backend.checkpoint(metadata, timestamp)
            self.last_snapshot = snapshot
            if snapshot is not None and self.publish_snapshot is not None:
                self.publish_snapshot(snapshot)

    def _require_live(self) -> None:
        if self._closed:
            raise RuntimeError("simulation is closed")

    def close(self) -> None:
        with self.lock:
            if self._closed:
                return
            self._closed = True
            self.backend.close()

    def __enter__(self) -> SimulationRuntime:
        self.build()
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
