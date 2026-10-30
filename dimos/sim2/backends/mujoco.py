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

"""MuJoCo implementation of the public rigid-body physics contract.

robosuite owns environment stepping, native controllers and task semantics.
Only this adapter and MuJoCo sensor implementations handle native model/state
indices. Public commands and state use stable scene and instance/link names.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
import tempfile
from typing import TYPE_CHECKING, Any, Literal, cast

import mujoco
import numpy as np
from numpy.typing import NDArray
from robosuite.environments.base import MujocoEnv
from robosuite.wrappers.wrapper import Wrapper

from dimos.sim2.control import MotorController
from dimos.sim2.environment import MotorScene
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.physics import (
    BackendStep,
    BodyDescription,
    BodyState,
    BodyWrench,
    InertialState,
    JointDescription,
    JointDrive,
    JointState,
    KinematicTarget,
    PhysicsCommand,
    Quat,
    RobotState,
    Vec3,
    WorldDescription,
    WorldState,
)
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.spec import RobotInstanceSpec
from dimos.sim2.world.scene_types import SceneDescription

if TYPE_CHECKING:
    from dimos.sim2.visuals import VisualScene

STATE = mujoco.mjtState.mjSTATE_INTEGRATION


@dataclass(frozen=True)
class _ActuatorGroup:
    actuators: Sequence[str]
    naming_prefix: str = ""


@dataclass(frozen=True)
class _RobotBinding:
    config: RobotInstanceSpec
    controller: MotorController
    root: int
    imu: tuple[int, int, int, int] | None


def read_body_states(
    model: mujoco.MjModel, data: mujoco.MjData, body_names: dict[int, str]
) -> dict[str, BodyState]:
    """Copy body-origin poses and world velocities from one forwarded native state."""
    bodies = {}
    velocity = np.empty(6)
    for index, name in body_names.items():
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, index, velocity, 0)
        bodies[name] = BodyState(
            cast("Vec3", tuple(data.xpos[index].tolist())),
            cast("Quat", tuple(np.roll(data.xquat[index], -1).tolist())),
            cast("Vec3", tuple(velocity[3:].tolist())),
            cast("Vec3", tuple(velocity[:3].tolist())),
        )
    return bodies


class MujocoBackend:
    name = "mujoco"
    description: WorldDescription

    def __init__(
        self,
        environment: Callable[[], MujocoEnv | Wrapper],
        robots: dict[str, RobotInstanceSpec] | None = None,
        sensors: tuple[tuple[str | None, Sensor], ...] = (),
        snapshot_hz: float = 60.0,
    ) -> None:
        self._factory = environment
        self._robot_configs = robots or {}
        self._sensors = sensors
        self._snapshot_hz = snapshot_hz
        self._environment: MujocoEnv | Wrapper | None = None
        self._directory: tempfile.TemporaryDirectory[str] | None = None
        self._revision = 0
        self._visuals: VisualScene | None = None
        self._task_action: NDArray[np.float64] | None = None
        self._robots: dict[str, _RobotBinding] = {}
        self._bodies: dict[str, int] = {}
        self._joints: dict[str, int] = {}
        self._drives: dict[str, tuple[MotorController, int]] = {}
        self._scene_controller: MotorController | None = None

    @property
    def environment(self) -> MujocoEnv | Wrapper:
        """Native environment for backend-specific diagnostics and tests."""
        if self._environment is None:
            raise RuntimeError("MuJoCo backend is not built")
        return self._environment

    @property
    def base_environment(self) -> MujocoEnv:
        env = self.environment
        return cast("MujocoEnv", env.unwrapped if isinstance(env, Wrapper) else env)

    @property
    def model(self) -> mujoco.MjModel:
        return cast("mujoco.MjModel", self.environment.sim.model._model)

    @property
    def data(self) -> mujoco.MjData:
        return cast("mujoco.MjData", self.environment.sim.data._data)

    @property
    def step_seconds(self) -> float:
        return float(self.environment.control_timestep)

    @property
    def physics_dt(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def sim_time(self) -> float:
        return float(self.data.time)

    @property
    def done(self) -> bool:
        return bool(self.environment.done)

    def build(self) -> None:
        if self._environment is not None:
            return
        self._directory = tempfile.TemporaryDirectory(prefix="dimos-mujoco-")
        try:
            self._environment = self._factory()
            self._bind()
        except BaseException:
            self.close()
            raise

    def _bind(self) -> None:
        env = self.base_environment
        if self._robot_configs and not isinstance(env, MotorScene):
            raise TypeError("robot bindings require MotorScene; native tasks own their controls")
        self._robots.clear()
        prefixes: dict[str, str] = {}
        roots: dict[str, str] = {}
        scene = (
            env.description
            if isinstance(env, MotorScene)
            else SceneDescription(id=type(env).__name__)
        )
        if isinstance(env, MotorScene):
            if set(env.robot_models) != set(self._robot_configs):
                raise ValueError("robot bindings must match the environment's robot models")
            for robot_id, robot_model in env.robot_models.items():
                config = self._robot_configs[robot_id]
                controller = env.controllers[robot_id]
                controller.set_motor_units(
                    scale=config.position_scale or None, offset=config.position_offset or None
                )
                if len(controller.actuators) != len(config.joint_names):
                    raise ValueError(
                        f"{robot_id}: hardware motor names do not match model actuators"
                    )
                servos = {
                    name
                    for name, servo in zip(
                        config.joint_names, controller.position_servo, strict=True
                    )
                    if servo
                }
                if servos != set(config.position_only_joints):
                    raise ValueError(
                        f"{robot_id}: position_only_joints must match native servos: {sorted(servos)}"
                    )
                imu = None
                if config.control_imu is not None:
                    addresses = [
                        int(
                            self.model.sensor(
                                robot_model.correct_naming(f"{config.control_imu}_{suffix}")
                            ).adr[0]
                        )
                        for suffix in ("gyro", "accel", "quat")
                    ]
                    site = self.model.site(robot_model.correct_naming(config.control_imu)).id
                    imu = (addresses[0], addresses[1], addresses[2], site)
                prefixes[robot_id] = robot_model.naming_prefix
                roots[robot_id] = robot_model.root_body.removeprefix(robot_model.naming_prefix)
                self._robots[robot_id] = _RobotBinding(
                    config, controller, self.model.body(robot_model.root_body).id, imu
                )
        self._prefixes, self._root_names = prefixes, roots
        body_aliases = {entry.body: key for key, entry in scene.entities.items()}
        joint_aliases = {entry.joint: key for key, entry in scene.joints.items()}
        self._bodies = {}
        body_ids: dict[int, str] = {}
        descriptions: dict[str, BodyDescription] = {}
        for index in range(self.model.nbody):
            native_name = self.model.body(index).name
            parent = int(self.model.body_parentid[index])
            local = int(np.count_nonzero(self.model.body_parentid[:index] == parent))
            name = (
                self._public_name(native_name)
                if native_name
                else f"{body_ids[parent]}/body_{local}"
            )
            name = body_aliases.get(native_name, name)
            if name in self._bodies:
                raise ValueError(f"duplicate public body name {name!r}")
            self._bodies[name] = index
            body_ids[index] = name
            entity = next(
                (robot for robot, prefix in prefixes.items() if native_name.startswith(prefix)),
                name,
            )
            descriptions[name] = BodyDescription(entity, self._motion(index))
        self._body_names = body_ids
        self._visuals = None
        self._joints = {}
        joint_ids: dict[int, str] = {}
        for index in range(self.model.njnt):
            if self.model.jnt_type[index] == mujoco.mjtJoint.mjJNT_FREE:
                continue  # Floating roots are represented by world-space BodyState.
            native_name = self.model.joint(index).name
            body = int(self.model.jnt_bodyid[index])
            local = index - int(self.model.body_jntadr[body])
            name = (
                self._public_name(native_name) if native_name else f"{body_ids[body]}/joint_{local}"
            )
            name = joint_aliases.get(native_name, name)
            if name in self._joints:
                raise ValueError(f"duplicate public joint name {name!r}")
            self._joints[name] = index
            joint_ids[index] = name
        self._drives = {}
        self._scene_controller = None
        claimed: set[int] = set()
        for binding in self._robots.values():
            self._register_drives(binding.controller, joint_ids)
            claimed.update(int(v) for v in binding.controller.actuators)
        if isinstance(env, MotorScene):
            # Scene actuators use exactly the same native servo/motor conversion.
            remaining = [
                self.model.actuator(i).name for i in range(self.model.nu) if i not in claimed
            ]
            if remaining:
                self._scene_controller = MotorController(env.sim, _ActuatorGroup(remaining))
                self._register_drives(self._scene_controller, joint_ids)
        kinds: dict[int, Literal["revolute", "prismatic", "spherical", "fixed"]] = {
            mujoco.mjtJoint.mjJNT_HINGE: "revolute",
            mujoco.mjtJoint.mjJNT_SLIDE: "prismatic",
            mujoco.mjtJoint.mjJNT_BALL: "spherical",
        }
        actuated = {
            int(self.model.actuator_trnid[index, 0])
            for index in range(self.model.nu)
            if self.model.actuator_trntype[index] == mujoco.mjtTrn.mjTRN_JOINT
        }
        joints: dict[str, JointDescription] = {}
        for name, index in self._joints.items():
            body = int(self.model.jnt_bodyid[index])
            joints[name] = JointDescription(
                body_ids[int(self.model.body_parentid[body])],
                body_ids[body],
                kinds[int(self.model.jnt_type[index])],
                index in actuated,
            )
        capabilities = {"world_state", "body_wrench", "kinematic_target", "mujoco_checkpoint"}
        capabilities.add("joint_drive" if isinstance(env, MotorScene) else "task_action")
        self.description = WorldDescription(
            scene=scene,
            bodies=descriptions,
            joints=joints,
            robot_roots={name: body_ids[binding.root] for name, binding in self._robots.items()},
            robot_joints={
                name: tuple(
                    joint_ids[int(self.model.actuator_trnid[a, 0])]
                    for a in binding.controller.actuators
                )
                for name, binding in self._robots.items()
            },
            capabilities=frozenset(capabilities),
        )
        rate, history = self._snapshot_hz, 0.0
        for _, sensor in self._sensors:
            if isinstance(sensor, Lidar):
                rate = max(rate, sensor.acquisition.sample_hz)
                history = max(history, sensor.acquisition.history_seconds)
                if sensor.acquisition.sample_hz > 1 / self.step_seconds + 1e-9:
                    raise ValueError(
                        "rolling sensor acquisition cannot exceed the environment step rate"
                    )
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("snapshot rate must be finite and positive")
        cadence = max(1, math.floor(1 / rate / self.step_seconds))
        self._snapshot_stride = cadence * max(1, round(self.step_seconds / self.physics_dt))
        self._snapshot_slots = max(
            2, math.ceil(history / (self._snapshot_stride * self.physics_dt)) + 2
        )
        self._applied_effort: NDArray[np.float64] | None = (
            np.zeros(self.model.nv) if isinstance(env, MotorScene) else None
        )
        self._state = np.empty(mujoco.mj_stateSize(self.model, STATE))
        assert self._directory is not None
        self._model_path = Path(self._directory.name) / f"world-{self._revision}.mjb"
        mujoco.mj_saveModel(self.model, str(self._model_path), None)

    def robot_controller(self, robot_id: str) -> MotorController:
        """Native actuator binding for MuJoCo-specific diagnostics."""
        return self._robots[robot_id].controller

    def _register_drives(self, controller: MotorController, joint_ids: dict[int, str]) -> None:
        for offset, actuator in enumerate(controller.actuators):
            name = joint_ids[int(self.model.actuator_trnid[actuator, 0])]
            if name in self._drives:
                raise ValueError(f"multiple actuators drive public joint {name!r}")
            self._drives[name] = controller, offset

    def _public_name(self, native: str) -> str:
        for robot, prefix in self._prefixes.items():
            if native.startswith(prefix):
                return f"{robot}/{native.removeprefix(prefix)}"
        return native

    def _motion(self, body: int) -> Literal["dynamic", "static", "kinematic"]:
        while body:
            if self.model.body_mocapid[body] >= 0:
                return "kinematic"
            if self.model.body_jntnum[body] > 0:
                return "dynamic"
            body = int(self.model.body_parentid[body])
        return "static"

    def reset(self) -> BackendStep:
        observations = self.environment.reset()
        self._revision += 1
        self._task_action = None
        self._bind()
        self.data.qfrc_applied[:] = 0
        self.data.xfrc_applied[:] = 0
        return BackendStep(observations=observations)

    def validate(self, commands: tuple[PhysicsCommand, ...]) -> None:
        seen: set[tuple[type, str]] = set()
        for command in commands:
            key: tuple[type, str]
            if isinstance(command, JointDrive):
                if command.joint not in self._drives:
                    raise ValueError(f"joint {command.joint!r} has no supported scalar drive")
                key = (JointDrive, command.joint)
                if (
                    not all(
                        math.isfinite(v)
                        for v in (
                            command.position,
                            command.velocity,
                            command.kp,
                            command.kd,
                            command.effort,
                        )
                    )
                    or min(command.kp, command.kd) < 0
                ):
                    raise ValueError("joint drive values must be finite and gains nonnegative")
                controller, offset = self._drives[command.joint]
                if controller.position_servo[offset] and any(
                    (command.velocity, command.kp, command.kd, command.effort)
                ):
                    raise ValueError("native position servos accept position targets only")
            elif isinstance(command, BodyWrench):
                if (
                    command.body not in self._bodies
                    or self.description.bodies[command.body].motion != "dynamic"
                ):
                    raise ValueError(f"body {command.body!r} is not dynamic")
                if command.frame not in ("world", "body"):
                    raise ValueError("wrench frame must be world or body")
                values = (*command.force, *command.torque, *(command.point or ()))
                if (
                    len(command.force) != 3
                    or len(command.torque) != 3
                    or (command.point is not None and len(command.point) != 3)
                    or not all(map(math.isfinite, values))
                ):
                    raise ValueError("wrench vectors must contain three finite values")
                continue  # Wrenches on the same body add physically.
            elif isinstance(command, KinematicTarget):
                if (
                    command.body not in self._bodies
                    or self.model.body_mocapid[self._bodies[command.body]] < 0
                ):
                    raise ValueError(f"body {command.body!r} is not an authored kinematic body")
                key = (KinematicTarget, command.body)
                if (
                    len(command.position) != 3
                    or len(command.orientation) != 4
                    or not all(map(math.isfinite, (*command.position, *command.orientation)))
                ):
                    raise ValueError(
                        "kinematic target requires a finite position and xyzw quaternion"
                    )
                if not np.isclose(np.linalg.norm(command.orientation), 1.0, rtol=0, atol=1e-6):
                    raise ValueError("kinematic target quaternion must have unit length")
            else:
                raise TypeError(f"unsupported physics command {type(command).__name__}")
            if key in seen:
                raise ValueError(f"duplicate command target {key[1]!r}")
            seen.add(key)

    def step(self, commands: tuple[PhysicsCommand, ...]) -> BackendStep:
        self.validate(commands)
        if self.done:
            raise RuntimeError("native task has terminated; reset before stepping")
        data, model = self.data, self.model
        data.xfrc_applied[:] = 0
        data.qfrc_applied[:] = 0
        goals: dict[MotorController, NDArray[np.float64]] = {}
        for controller, _ in self._drives.values():
            if controller not in goals:
                target = np.zeros((5, len(controller.actuators)))
                # Uncommanded native position servos hold their current position.
                target[0] = controller.positions
                goals[controller] = target
        for command in commands:
            if isinstance(command, KinematicTarget):
                index = int(model.body_mocapid[self._bodies[command.body]])
                data.mocap_pos[index] = command.position
                data.mocap_quat[index] = np.roll(command.orientation, 1)
        mujoco.mj_forward(model, data)
        for command in commands:
            if isinstance(command, JointDrive):
                controller, offset = self._drives[command.joint]
                goals[controller][:, offset] = (
                    (command.position - controller.offset[offset]) / controller.scale[offset],
                    command.velocity / controller.scale[offset],
                    command.kp,
                    command.kd,
                    command.effort,
                )
            elif isinstance(command, BodyWrench):
                index = self._bodies[command.body]
                rotation = data.xmat[index].reshape(3, 3)
                force, torque = np.asarray(command.force), np.asarray(command.torque)
                if command.frame == "body":
                    force, torque = rotation @ force, rotation @ torque
                point = data.xipos[index] if command.point is None else np.asarray(command.point)
                if command.point is not None and command.frame == "body":
                    point = data.xpos[index] + rotation @ point
                data.xfrc_applied[index, :3] += force
                data.xfrc_applied[index, 3:] += torque + np.cross(point - data.xipos[index], force)
        env = self.base_environment
        if isinstance(env, MotorScene):
            if self._scene_controller is not None:
                controller = self._scene_controller
                controller.set_goal(goals[controller])
                data.ctrl[controller.actuators] = controller.run_controller()
            action = (
                np.concatenate([goals[env.controllers[name]].ravel() for name in env.robot_models])
                if env.robot_models
                else np.empty(0)
            )
        else:
            action = (
                self._task_action
                if self._task_action is not None
                else np.zeros_like(self.environment.action_spec[0], dtype=np.float64)
            )
        observations, reward, done, info = self.environment.step(action)
        if isinstance(env, MotorScene):
            assert env.last_actuator_effort is not None
            self._applied_effort = env.last_actuator_effort.copy()
        self.environment.sim.forward()
        return BackendStep(observations, float(reward), bool(done), info)

    def read_state(self, metadata: FrameMetadata) -> WorldState:
        bodies = read_body_states(self.model, self.data, self._body_names)
        joints: dict[str, JointState] = {}
        for name, index in self._joints.items():
            qpos, dof = int(self.model.jnt_qposadr[index]), int(self.model.jnt_dofadr[index])
            ball = self.model.jnt_type[index] == mujoco.mjtJoint.mjJNT_BALL
            position = (
                np.roll(self.data.qpos[qpos : qpos + 4], -1)
                if ball
                else self.data.qpos[qpos : qpos + 1]
            )
            width = 3 if ball else 1
            joints[name] = JointState(
                tuple(position.tolist()),
                tuple(self.data.qvel[dof : dof + width].tolist()),
                None
                if self._applied_effort is None
                else tuple(self._applied_effort[dof : dof + width].tolist()),
            )
        robots: dict[str, RobotState] = {}
        for name, binding in self._robots.items():
            assert self._applied_effort is not None
            controller = binding.controller
            imu = None
            if binding.imu is not None:
                gyro, accel, quat, site = binding.imu
                velocity = np.empty(6)
                mujoco.mj_objectVelocity(
                    self.model, self.data, mujoco.mjtObj.mjOBJ_SITE, site, velocity, 1
                )
                imu = InertialState(
                    cast(
                        "Quat", tuple(np.roll(self.data.sensordata[quat : quat + 4], -1).tolist())
                    ),
                    cast("Vec3", tuple(self.data.sensordata[gyro : gyro + 3].tolist())),
                    cast("Vec3", tuple(velocity[3:].tolist())),
                    cast("Vec3", tuple(self.data.sensordata[accel : accel + 3].tolist())),
                )
            robots[name] = RobotState(
                binding.config.joint_names,
                tuple(controller.positions.tolist()),
                tuple(controller.velocities.tolist()),
                tuple(self._applied_effort[controller.dofs].tolist()),
                bodies[self.description.robot_roots[name]],
                imu,
            )
        return WorldState(metadata, bodies, joints, robots, self._revision)

    def set_task_action(self, action: list[float]) -> None:
        if self._robots:
            raise ValueError("motor-bound environments accept joint drives")
        values = np.asarray(action, dtype=np.float64)
        lower, upper = self.environment.action_spec
        if values.shape != np.shape(lower) or not np.isfinite(values).all():
            raise ValueError("action must have the environment's dimension and finite values")
        if np.any(values < lower) or np.any(values > upper):
            raise ValueError("action is outside the environment's action_spec")
        self._task_action = values.copy()

    def describe_visuals(self) -> VisualScene:
        # Visual asset parsing is optional for headless worlds without a renderer.
        from dimos.sim2.backends.mjcf_visuals import load_visuals

        if self._visuals is None:
            self._visuals = load_visuals(
                self.model, self._body_names, self._public_name, self._revision
            )
        return self._visuals

    def sensor_description(self) -> dict[str, Any]:
        return dict(
            backend=self.name,
            model=str(self._model_path),
            physics_dt=self.physics_dt,
            snapshot_stride=self._snapshot_stride,
            snapshot_slots=self._snapshot_slots,
            model_prefixes=self._prefixes,
            root_bodies=self._root_names,
        )

    def checkpoint(self, metadata: FrameMetadata, wall_time: float) -> StateFrame:
        mujoco.mj_getState(self.model, self.data, self._state, STATE)
        return StateFrame(
            metadata,
            {"state": self._state.copy(), "wall_time": np.array([wall_time])},
            str(self._model_path),
        )

    def close(self) -> None:
        environment, self._environment = self._environment, None
        directory, self._directory = self._directory, None
        try:
            if environment is not None:
                environment.close()
        finally:
            if directory is not None:
                directory.cleanup()

    def __enter__(self) -> MujocoBackend:
        self.build()
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
