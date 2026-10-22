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

"""Single mutable MuJoCo world; control and snapshots never wait for rendering."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any
from uuid import uuid4

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.messages import (
    FrameMetadata,
    MotorFrame,
    StreamDescriptor,
    robot_descriptor,
)
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.spec import RobotConfig, WorldConfig
from dimos.sim2.streams import SimStream
from dimos.sim2.world.scene import describe_scene, load_scene
from dimos.sim2.world.scene_types import EntityState, RegionState, SceneState, SceneUpdate

STATE = mujoco.mjtState.mjSTATE_INTEGRATION


@dataclass
class RobotBinding:
    config: RobotConfig
    channel: SimStream
    qpos: NDArray[np.int32]
    dofs: NDArray[np.int32]
    actuators: NDArray[np.int64]
    root: int
    scale: NDArray[np.float64]
    offset: NDArray[np.float64]
    imu: tuple[int, int, int] | None
    idle_position: NDArray[np.float64]
    hold_kp: NDArray[np.float64]
    hold_kd: NDArray[np.float64]
    enabled: bool = True
    action: MotorFrame | None = None
    command_active: bool = False
    applied_action_sequence: int = 0


class SimulationRuntime:
    def __init__(self, config: WorldConfig, sim_id: str) -> None:
        self.config = config
        self.world_id = uuid4().hex
        self.description = describe_scene(config.scene)
        self.model = load_scene(config, self.description)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self.episode = 0
        self.generation = 0
        self.tick = 0
        self._clock_origin = time.time()
        self._closed = False
        self.paused = False
        self.waiting_for_control = {
            name for name, robot in config.robots.items() if robot.wait_for_first_command
        }
        self._bodies = {
            key: self.model.body(e.body).id for key, e in self.description.entities.items()
        }
        self._joints = {
            key: self.model.joint(j.joint).id for key, j in self.description.joints.items()
        }
        self._region_bodies = {
            key: self.model.body(r.body).id for key, r in self.description.regions.items()
        }
        if set(self._bodies) & set(config.robots):
            raise ValueError("scene entity IDs and robot IDs must be disjoint")
        # Bind ownership once; the physics loop does no semantic/name traversal.
        owners = {body: key for key, body in self._bodies.items()}
        self._geom_owners: list[str] = []
        for body in self.model.geom_bodyid:
            ancestor = int(body)
            while ancestor and ancestor not in owners:
                ancestor = int(self.model.body_parentid[ancestor])
            self._geom_owners.append(owners.get(ancestor, self.model.body(int(body)).name))
        self._entity_geoms = {
            key: np.array(
                [i for i, owner in enumerate(self._geom_owners) if owner == key], dtype=int
            )
            for key in self._bodies
        }
        self.robots: dict[str, RobotBinding] = {}
        nstate = mujoco.mj_stateSize(self.model, STATE)
        self.state = np.empty(nstate)
        snapshot_hz = config.snapshot_hz
        history_seconds = 0.0
        for _, sensor in config.devices:
            acquisition = sensor.acquisition
            if acquisition.sample_hz > 1 / config.timestep:
                raise ValueError("sensor acquisition rate cannot exceed the physics rate")
            snapshot_hz = max(snapshot_hz, acquisition.sample_hz)
            history_seconds = max(history_seconds, acquisition.history_seconds)
        ticks_per_snapshot = 1 / snapshot_hz / config.timestep
        self._snapshot_stride = max(1, math.floor(ticks_per_snapshot))
        snapshot_slots = max(
            2, math.ceil(history_seconds / (self._snapshot_stride * config.timestep)) + 2
        )
        self.snapshot_descriptor = StreamDescriptor(
            sim_id=sim_id,
            robot_id="world",
            generation=self.world_id,
            physics_dt=config.timestep,
            control_decimation=self._snapshot_stride,
            observation_slots=snapshot_slots,
        )
        self.snapshots = SimStream.create(self.snapshot_descriptor)
        try:
            for robot_id, instance in config.robots.items():
                definition = instance.config
                joints = [
                    self.model.joint(robot_id + "/" + j.model_name).id for j in definition.joints
                ]
                actuators = np.array(
                    [self.model.actuator(robot_id + "/" + j.actuator).id for j in definition.joints]
                )
                imus = [
                    s
                    for s in definition.mounted_sensors
                    if isinstance(s, Imu) and s.name == instance.control_imu
                ]
                imu = None
                if imus:
                    g, a, q = (
                        int(self.model.sensor(f"{robot_id}/sensor/{imus[0].name}/{s}").adr[0])
                        for s in ("gyro", "accel", "quat")
                    )
                    imu = (g, a, q)
                positions = self.data.qpos[self.model.jnt_qposadr[joints]].copy()
                for i, joint in enumerate(definition.joints):
                    if joint.name in instance.initial_positions:
                        positions[i] = (
                            instance.initial_positions[joint.name] * joint.scale + joint.offset
                        )
                hold_gains = [
                    instance.hold_gains.get(j.name, (j.kp, j.kd)) for j in definition.joints
                ]
                self.robots[robot_id] = RobotBinding(
                    definition,
                    SimStream.create(
                        robot_descriptor(sim_id, robot_id, self.world_id, instance, config.timestep)
                    ),
                    self.model.jnt_qposadr[joints],
                    self.model.jnt_dofadr[joints],
                    actuators,
                    self.model.body(robot_id + "/" + definition.root_body).id,
                    np.array([j.scale for j in definition.joints]),
                    np.array([j.offset for j in definition.joints]),
                    imu,
                    positions,
                    np.array([g[0] for g in hold_gains]),
                    np.array([g[1] for g in hold_gains]),
                )
            mujoco.mj_resetData(self.model, self.data)
            for binding in self.robots.values():
                self.data.qpos[binding.qpos] = binding.idle_position
                public_positions = (binding.idle_position - binding.offset) / binding.scale
                for joint, aid, position in zip(
                    binding.config.joints, binding.actuators, public_positions, strict=True
                ):
                    self.data.ctrl[aid] = (
                        position * joint.ctrl_scale + joint.ctrl_offset
                        if joint.mode == "position"
                        else 0.0
                    )
            self._apply_update(self.description.initial)
            self._baseline = np.empty(nstate)
            mujoco.mj_getState(self.model, self.data, self._baseline, STATE)
            self.reset()
        except BaseException:
            self.close()
            raise

    def reset(self, initial: SceneUpdate | None = None) -> SceneState:
        with self.lock:
            self._require_live()
            update = initial if initial is not None else SceneUpdate()
            self._validate_update(update)
            mujoco.mj_setState(self.model, self.data, self._baseline, STATE)
            self._apply_update(update)
            self.episode += 1
            return self._finish_change()

    def _finish_change(self, *, enable: bool = True) -> SceneState:
        self.generation += 1
        self._clock_origin = time.time() - float(self.data.time)
        self.snapshots.set_episode(self.generation)
        mujoco.mj_forward(self.model, self.data)
        for binding in self.robots.values():
            binding.channel.set_episode(self.generation)
            if enable:
                binding.enabled = True
            binding.action = None
            binding.command_active = False
            binding.applied_action_sequence = 0
            binding.idle_position = self.data.qpos[binding.qpos].copy()
        self._publish(force_snapshot=True)
        for binding in self.robots.values():
            binding.channel.set_lifecycle("ready")
        self.snapshots.set_lifecycle("ready")
        return self.scene_state()

    def _validate_update(self, update: SceneUpdate) -> None:
        for key in update.poses:
            if key in self.robots:
                body = self.robots[key].root
            else:
                entity = self.description.entities[key]
                if not entity.movable:
                    raise ValueError(f"{key!r} is fixed geometry; edit the scene file instead")
                body = self._bodies[key]
            joint = int(self.model.body_jntadr[body])
            if self.model.body_mocapid[body] < 0 and (
                joint < 0 or self.model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE
            ):
                raise ValueError(f"{key!r} has no movable root")
        for key, value in update.joints.items():
            joint = self._joints[key]
            if self.model.jnt_type[joint] not in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                raise ValueError(f"{key!r} is not a scalar fixture joint")
            if self.model.jnt_limited[joint] and not (
                self.model.jnt_range[joint, 0] <= value <= self.model.jnt_range[joint, 1]
            ):
                raise ValueError(
                    f"{key!r}: {value} is outside joint limits {self.model.jnt_range[joint]}"
                )

    def _apply_update(self, update: SceneUpdate) -> None:
        self._validate_update(update)
        for key, pose in update.poses.items():
            body = self.robots[key].root if key in self.robots else self._bodies[key]
            xyz = pose.position.to_tuple()
            x, y, z, w = pose.orientation.to_tuple()
            mid = int(self.model.body_mocapid[body])
            if mid >= 0:
                self.data.mocap_pos[mid] = xyz
                self.data.mocap_quat[mid] = (w, x, y, z)
            else:
                jid = int(self.model.body_jntadr[body])
                q, v = int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid])
                self.data.qpos[q : q + 7] = (*xyz, w, x, y, z)
                self.data.qvel[v : v + 6] = 0
        for key, value in update.joints.items():
            jid = self._joints[key]
            self.data.qpos[self.model.jnt_qposadr[jid]] = value
            self.data.qvel[self.model.jnt_dofadr[jid]] = 0

    def set_scene_state(self, update: SceneUpdate) -> SceneState:
        with self.lock:
            self._require_live()
            self._apply_update(update)
            return self._finish_change(enable=False)

    def set_paused(self, paused: bool) -> None:
        with self.lock:
            self._require_live()
            resumed = self.paused and not paused
            self.paused = paused
            if resumed:
                # Re-anchor sensor clocks and discard pre-pause commands/history.
                self._finish_change(enable=False)

    def _body_pose(self, body: int) -> Pose:
        w, x, y, z = self.data.xquat[body]
        return Pose(self.data.xpos[body], (x, y, z, w))

    def scene_state(self) -> SceneState:
        with self.lock:
            entities: dict[str, EntityState] = {}
            for key, body in self._bodies.items():
                geoms = self._entity_geoms[key]
                if len(geoms):
                    rotation = self.data.geom_xmat[geoms].reshape(-1, 3, 3)
                    local = self.model.geom_aabb[geoms]
                    centers = self.data.geom_xpos[geoms] + np.einsum(
                        "nij,nj->ni", rotation, local[:, :3]
                    )
                    half = np.einsum("nij,nj->ni", np.abs(rotation), local[:, 3:])
                    lo, hi = np.min(centers - half, axis=0), np.max(centers + half, axis=0)
                else:
                    lo = hi = self.data.xpos[body]
                velocity = np.empty(6)
                mujoco.mj_objectVelocity(
                    self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0
                )
                entities[key] = EntityState(
                    pose=self._body_pose(body),
                    velocity=velocity[3:].tolist(),
                    angular_velocity=velocity[:3].tolist(),
                    bounds_min=lo.tolist(),
                    bounds_max=hi.tolist(),
                )
            regions: dict[str, RegionState] = {}
            for key, region in self.description.regions.items():
                body = self._region_bodies[key]
                rotation = Rotation.from_matrix(self.data.xmat[body].reshape(3, 3))
                xyz = self.data.xpos[body] + rotation.apply(region.pose.position.to_tuple())
                quat = rotation * Rotation.from_quat(region.pose.orientation.to_tuple())
                regions[key] = RegionState(pose=Pose(xyz, quat.as_quat()), size=region.size)
            contacts = {
                tuple(sorted((self._geom_owners[c.geom1], self._geom_owners[c.geom2])))
                for c in self.data.contact
                if c.geom1 >= 0
                and c.geom2 >= 0
                and c.dist <= 0.001
                and self._geom_owners[c.geom1] != self._geom_owners[c.geom2]
            }
            return SceneState(
                world_id=self.world_id,
                scene_id=self.description.id,
                episode=self.episode,
                generation=self.generation,
                tick=self.tick,
                sim_time=float(self.data.time),
                ts=time.time(),
                entities=entities,
                robots={k: self._body_pose(b.root) for k, b in self.robots.items()},
                joints={
                    k: float(self.data.qpos[self.model.jnt_qposadr[j]])
                    for k, j in self._joints.items()
                },
                regions=regions,
                contacts=tuple(sorted(contacts)),
            )

    def set_spawn(
        self, robot_id: str, xyz: tuple[float, float, float], rpy: tuple[float, float, float]
    ) -> None:
        self.set_scene_state(
            SceneUpdate(poses={robot_id: Pose(xyz, Rotation.from_euler("xyz", rpy).as_quat())})
        )

    def step(self) -> None:
        with self.lock:
            self._require_live()
            if self.paused:
                self._publish(force_snapshot=True)
                return
            if self.tick == 0 and self.waiting_for_control:
                self._clock_origin = time.time() - float(self.data.time)
            for binding in self.robots.values():
                self._apply(binding)
            self.waiting_for_control.difference_update(
                name
                for name, binding in self.robots.items()
                if binding.command_active and binding.enabled
            )
            if self.waiting_for_control:
                self._publish(force_snapshot=True)
                return
            mujoco.mj_step(self.model, self.data)
            # mj_step integrates qpos/qvel last. Refresh derived poses and sensors
            # before publishing a sample labelled with that post-step time.
            mujoco.mj_forward(self.model, self.data)
            self.tick += 1
            self._publish()

    def _apply(self, b: RobotBinding) -> None:
        b.action = b.channel.read_action()
        action = b.action
        q = self.data.qpos[b.qpos]
        dq = self.data.qvel[b.dofs]
        if (
            action is not None
            and action.metadata.episode_id == self.generation
            and time.monotonic() <= float(action.values["expires_at"][0])
        ):
            b.command_active = True
            b.applied_action_sequence = action.metadata.sequence
            values = action.values
            enabled = bool(values["enabled"][0])
            target = values["position"] * b.scale + b.offset
            velocity = values["velocity"] * b.scale
            kp, kd, ff = values["kp"], values["kd"], values["effort"]
        else:
            if b.command_active:
                b.idle_position = q.copy()
            b.command_active = False
            b.applied_action_sequence = 0
            enabled = b.enabled
            target = b.idle_position
            velocity = np.zeros_like(dq)
            kp, kd = b.hold_kp, b.hold_kd
            ff = np.zeros_like(dq)
        torque = kp * (target - q) + kd * (velocity - dq) + ff
        b.enabled = enabled
        for i, (joint, aid) in enumerate(zip(b.config.joints, b.actuators, strict=True)):
            if joint.mode == "position":
                public_target = (
                    (target[i] - b.offset[i]) / b.scale[i]
                    if enabled
                    else (q[i] - b.offset[i]) / b.scale[i]
                )
                value = public_target * joint.ctrl_scale + joint.ctrl_offset
            else:
                value = torque[i] if enabled else 0.0
            if self.model.actuator_ctrllimited[aid]:
                value = np.clip(value, *self.model.actuator_ctrlrange[aid])
            if joint.actuator_time_constant and enabled:
                alpha = self.model.opt.timestep / (
                    joint.actuator_time_constant + self.model.opt.timestep
                )
                value = self.data.ctrl[aid] + alpha * (value - self.data.ctrl[aid])
            self.data.ctrl[aid] = value

    def _publish(self, *, force_snapshot: bool = False) -> None:
        meta = FrameMetadata(0, self.generation, self.tick, 0, float(self.data.time))
        timestamp = self._clock_origin + float(self.data.time)
        for b in self.robots.values():
            values: dict[str, Any] = {
                "position": (self.data.qpos[b.qpos] - b.offset) / b.scale,
                "velocity": self.data.qvel[b.dofs] / b.scale,
                "effort": self.data.qfrc_actuator[b.dofs],
                "enabled": [int(b.enabled)],
                "wall_time": [timestamp],
                "monotonic_time": [time.monotonic()],
                "root_position": self.data.xpos[b.root],
                "root_quaternion": self.data.xquat[b.root],
                "root_linear_velocity": self.data.cvel[b.root, 3:],
                "root_angular_velocity": self.data.cvel[b.root, :3],
            }
            if b.imu is not None:
                g, a, r = b.imu
                gyro, accel, quat = (
                    self.data.sensordata[g : g + 3],
                    self.data.sensordata[a : a + 3],
                    self.data.sensordata[r : r + 4],
                )
                values.update(
                    imu_quaternion=quat,
                    imu_gyroscope=gyro,
                    imu_accelerometer=accel,
                    imu_rpy=Rotation.from_quat(quat, scalar_first=True).as_euler("xyz"),
                )
            b.channel.publish_observation(
                values,
                FrameMetadata(
                    0,
                    self.generation,
                    self.tick,
                    0,
                    float(self.data.time),
                    b.applied_action_sequence,
                ),
            )
        if force_snapshot or self.tick % self._snapshot_stride == 0:
            mujoco.mj_getState(self.model, self.data, self.state, STATE)
            self.snapshots.publish_observation(
                {"state": self.state, "wall_time": [timestamp]}, meta
            )

    def _require_live(self) -> None:
        if self._closed:
            raise RuntimeError("sim2 world is closed")
        if self.snapshots.lifecycle == "faulted":
            raise RuntimeError("sim2 world is faulted")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for channel in [*(b.channel for b in self.robots.values()), self.snapshots]:
            channel.set_lifecycle("closed")
            channel.close()

    def __enter__(self) -> SimulationRuntime:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
