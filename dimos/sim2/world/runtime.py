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

"""Single mutable MuJoCo world. Robot IO is standard messages; snapshots feed sensor workers."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import threading
import time
from typing import Any
from uuid import uuid4

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu as ImuMessage
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.spec import RobotConfig, WorldConfig
from dimos.sim2.world.scene import describe_scene, load_scene
from dimos.sim2.world.scene_types import SceneUpdate

STATE = mujoco.mjtState.mjSTATE_INTEGRATION

FeedbackCallback = Callable[[str, JointState, ImuMessage | None, PoseStamped], None]
SnapshotCallback = Callable[[StateFrame], None]


@dataclass
class RobotBinding:
    """One robot's indices into the compiled model and its current command."""

    config: RobotConfig
    frame_id: str
    command_timeout: float
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
    halt_damping: float | None = None
    received_command: bool = False
    # Rows: position, velocity, kp, kd, effort, in public joint units.
    action: NDArray[np.float64] | None = None
    # Monotonic time after which the action no longer applies and the robot holds.
    action_expires: float = 0.0
    command_active: bool = False


class SimulationRuntime:
    """Compose, step and observe one world. ``SimulationModule`` owns the instance.

    Feedback and snapshots leave through two callbacks the owner sets:
    ``publish_feedback(robot_id, joints, imu, odom)`` after every step and
    ``publish_snapshot(frame)`` every snapshot stride. Commands arrive through
    ``accept_command``. Nothing here waits on a consumer.
    """

    def __init__(self, config: WorldConfig) -> None:
        self.config = config
        self.world_id = uuid4().hex
        self.description = describe_scene(config.scene)
        self.model = load_scene(config, self.description)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self.episode = 0
        self.tick = 0
        self.paused = False
        self.publish_feedback: FeedbackCallback | None = None
        self.publish_snapshot: SnapshotCallback | None = None
        self.last_snapshot: StateFrame | None = None
        self._sequence = 0
        self._clock_origin = time.time()
        self._reset_wall_time = 0.0
        self._closed = False
        self.waiting_for_control = {
            name for name, robot in config.robots.items() if robot.wait_for_first_command
        }
        self._bodies = {
            key: self.model.body(e.body).id for key, e in self.description.entities.items()
        }
        self._joints = {
            key: self.model.joint(j.joint).id for key, j in self.description.joints.items()
        }
        if set(self._bodies) & set(config.robots):
            raise ValueError("scene entity IDs and robot IDs must be disjoint")
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
        self.snapshot_stride = max(1, math.floor(ticks_per_snapshot))
        self.snapshot_slots = max(
            2, math.ceil(history_seconds / (self.snapshot_stride * config.timestep)) + 2
        )
        for robot_id, instance in config.robots.items():
            definition = instance.config
            joints = [self.model.joint(robot_id + "/" + j.model_name).id for j in definition.joints]
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
            hold_gains = [instance.hold_gains.get(j.name, (j.kp, j.kd)) for j in definition.joints]
            self.robots[robot_id] = RobotBinding(
                definition,
                instance.frame_id or f"{robot_id}/{definition.root_body}",
                instance.command_timeout,
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
                halt_damping=instance.halt_damping,
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

    def describe(self) -> dict[str, Any]:
        """Snapshot timing a sensor worker needs to rebuild history from frames."""
        return {
            "physics_dt": self.config.timestep,
            "snapshot_stride": self.snapshot_stride,
            "snapshot_slots": self.snapshot_slots,
        }

    def accept_command(self, robot_id: str, command: MotorCommandArray) -> bool:
        """Keep one complete, finite PD/feedforward target for a robot until it expires.

        Native position servos accept positions only; velocity, gain and effort
        overrides for them are rejected. A command stamped before the last reset
        is rejected. Returns whether the command was accepted.
        """
        with self.lock:
            binding = self.robots[robot_id]
            dof = len(binding.config.joints)
            rows = (command.q, command.dq, command.kp, command.kd, command.tau)
            if any(len(row) != dof for row in rows):
                return False
            values = np.asarray(rows, dtype=np.float64)
            if not np.isfinite(values).all() or np.any(values[2:4] < 0):
                return False
            servos = np.array([j.mode == "position" for j in binding.config.joints], dtype=bool)
            if np.any(values[1:, servos] != 0):
                return False
            issued = command.timestamp
            if issued is not None and (not math.isfinite(issued) or issued < self._reset_wall_time):
                return False
            binding.action = values
            binding.action_expires = time.monotonic() + binding.command_timeout
            binding.received_command = True
            return True

    def reset(self) -> None:
        """Restore the authored state and start a new episode; held commands are dropped."""
        with self.lock:
            self._require_live()
            mujoco.mj_setState(self.model, self.data, self._baseline, STATE)
            self._reset_wall_time = time.time()
            self._new_episode()

    def set_paused(self, paused: bool) -> None:
        """Pause or resume physics. Resuming re-anchors clocks and starts a new episode."""
        with self.lock:
            self._require_live()
            resumed = self.paused and not paused
            self.paused = paused
            if resumed:
                self._new_episode()

    def _new_episode(self) -> None:
        self.episode += 1
        self._clock_origin = time.time() - float(self.data.time)
        mujoco.mj_forward(self.model, self.data)
        for binding in self.robots.values():
            binding.action = None
            binding.command_active = False
            binding.received_command = False
            binding.idle_position = self.data.qpos[binding.qpos].copy()
        self._publish(force_snapshot=True)

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
        """Place the sidecar's authored initial poses and fixture joint values."""
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

    def _body_pose(self, body: int) -> Pose:
        w, x, y, z = self.data.xquat[body]
        return Pose(self.data.xpos[body], (x, y, z, w))

    def step(self) -> None:
        with self.lock:
            self._require_live()
            if self.paused:
                self._publish(force_snapshot=True)
                return
            if self.tick == 0 and self.waiting_for_control:
                self._clock_origin = time.time() - float(self.data.time)
            now = time.monotonic()
            for binding in self.robots.values():
                self._apply(binding, now)
            self.waiting_for_control.difference_update(
                name for name, binding in self.robots.items() if binding.command_active
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

    def _apply(self, b: RobotBinding, now: float) -> None:
        q = self.data.qpos[b.qpos]
        dq = self.data.qvel[b.dofs]
        if b.action is not None and now <= b.action_expires:
            b.command_active = True
            target = b.action[0] * b.scale + b.offset
            velocity = b.action[1] * b.scale
            kp, kd, ff = b.action[2], b.action[3], b.action[4]
        else:
            # No live command: hold where the last command left the robot.
            if b.command_active:
                b.idle_position = q.copy()
            b.command_active = False
            b.action = None
            target = b.idle_position
            velocity = np.zeros_like(dq)
            kp, kd = b.hold_kp, b.hold_kd
            if b.received_command and b.halt_damping is not None:
                kp = np.zeros_like(kp)
                kd = np.full_like(kd, b.halt_damping)
            ff = np.zeros_like(dq)
        torque = kp * (target - q) + kd * (velocity - dq) + ff
        for i, (joint, aid) in enumerate(zip(b.config.joints, b.actuators, strict=True)):
            if joint.mode == "position":
                public_target = (target[i] - b.offset[i]) / b.scale[i]
                value = public_target * joint.ctrl_scale + joint.ctrl_offset
            else:
                value = torque[i]
            if self.model.actuator_ctrllimited[aid]:
                value = np.clip(value, *self.model.actuator_ctrlrange[aid])
            if joint.actuator_time_constant:
                alpha = self.model.opt.timestep / (
                    joint.actuator_time_constant + self.model.opt.timestep
                )
                value = self.data.ctrl[aid] + alpha * (value - self.data.ctrl[aid])
            self.data.ctrl[aid] = value

    def _publish(self, *, force_snapshot: bool = False) -> None:
        timestamp = self._clock_origin + float(self.data.time)
        if self.publish_feedback is not None:
            for robot_id, b in self.robots.items():
                joints = JointState(
                    name=[j.name for j in b.config.joints],
                    position=((self.data.qpos[b.qpos] - b.offset) / b.scale).tolist(),
                    velocity=(self.data.qvel[b.dofs] / b.scale).tolist(),
                    effort=self.data.qfrc_actuator[b.dofs].tolist(),
                    ts=timestamp,
                    frame_id=b.frame_id,
                )
                imu = None
                if b.imu is not None:
                    g, a, r = b.imu
                    gyro, accel, quat = (
                        self.data.sensordata[g : g + 3],
                        self.data.sensordata[a : a + 3],
                        self.data.sensordata[r : r + 4],
                    )
                    w, x, y, z = quat
                    imu = ImuMessage(
                        orientation=Quaternion(x, y, z, w),
                        angular_velocity=Vector3(*gyro),
                        linear_acceleration=Vector3(*accel),
                        frame_id=b.frame_id,
                        ts=timestamp,
                    )
                pose = self._body_pose(b.root)
                odom = PoseStamped(
                    position=pose.position,
                    orientation=pose.orientation,
                    frame_id="world",
                    ts=timestamp,
                )
                self.publish_feedback(robot_id, joints, imu, odom)
        if force_snapshot or self.tick % self.snapshot_stride == 0:
            mujoco.mj_getState(self.model, self.data, self.state, STATE)
            self._sequence += 1
            frame = StateFrame(
                FrameMetadata(self._sequence, self.episode, self.tick, float(self.data.time)),
                {"state": self.state.copy(), "wall_time": np.array([timestamp])},
            )
            self.last_snapshot = frame
            if self.publish_snapshot is not None:
                self.publish_snapshot(frame)

    def _require_live(self) -> None:
        if self._closed:
            raise RuntimeError("sim2 world is closed")

    def close(self) -> None:
        self._closed = True

    def __enter__(self) -> SimulationRuntime:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
