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

"""A WorldView drawn by MuJoCo: its own model of the described world, never the backend's, posed from world_change."""

from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world import compile_world
from dimos.sim2.world_change import JOINT, OBJECT, ROBOT, Entity, WorldMirror
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster

_GRAVITY = np.array([0.0, 0.0, -9.81])


class MujocoView:
    def __init__(self, description: Mapping[str, Any]) -> None:
        # Its own model of the world: nothing here reads the backend.
        self.model = compile_world(Path(description["scene"]), description["simulated"])
        self._robots = {
            d.robot_id for d in description["simulated"] if isinstance(d, SimulatedRobot)
        }
        self._joints: dict[str, int] = {}
        self.data = mujoco.MjData(self.model)
        mujoco.mj_kinematics(self.model, self.data)
        self._frames: dict[str, tuple[int, int]] = {}
        self._velocities_current = False
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}
        # An IMU's previous world velocity and the time of it, for its acceleration.
        self._previous: dict[str, tuple[float, NDArray[np.float64]]] = {}

    @property
    def time(self) -> float:
        return float(self.data.time)

    def update(self, world: WorldMirror) -> None:
        model, data = self.model, self.data
        with world.lock:
            for entity in world.entities.values():
                if ROBOT in entity.tags:
                    self._pose_entity(entity, self._root(entity.path))
                elif OBJECT in entity.tags:
                    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, entity.path)
                    # An object spawned after the world was described is not in this view.
                    if body >= 0:
                        self._pose_entity(entity, body)
                elif JOINT in entity.tags and entity.joint is not None:
                    owner, _, name = entity.path.partition("/")
                    # A robot's joints are named "<robot>/<joint>" in this model too; an object's are its own.
                    joint = self._joint(entity.path if owner in self._robots else name)
                    if joint >= 0:
                        data.qpos[model.jnt_qposadr[joint]] = entity.joint[0]
                        data.qvel[model.jnt_dofadr[joint]] = entity.joint[1]
            data.time = world.time
        mujoco.mj_kinematics(self.model, self.data)
        self._velocities_current = False

    def pose(self, frame: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        kind, index = self._frame(frame)
        if kind == mujoco.mjtObj.mjOBJ_BODY:
            return self.data.xpos[index].copy(), self.data.xquat[index].copy()
        matrix = (self.data.site_xmat if kind == mujoco.mjtObj.mjOBJ_SITE else self.data.cam_xmat)[
            index
        ]
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, matrix)
        position = (
            self.data.site_xpos if kind == mujoco.mjtObj.mjOBJ_SITE else self.data.cam_xpos
        )[index]
        return position.copy(), quat

    def velocity(self, frame: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Linear and angular velocity in the frame's own axes."""
        kind, index = self._frame(frame)
        # mjOBJ_BODY would be the body's inertial frame; mjOBJ_XBODY is the body's own.
        kind = mujoco.mjtObj.mjOBJ_XBODY if kind == mujoco.mjtObj.mjOBJ_BODY else kind
        if not self._velocities_current:
            mujoco.mj_comPos(self.model, self.data)
            mujoco.mj_comVel(self.model, self.data)
            self._velocities_current = True
        angular_linear = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, kind, index, angular_linear, 1)
        return angular_linear[3:], angular_linear[:3]

    def render(self, camera: str, width: int, height: int) -> NDArray[np.uint8]:
        if (width, height) not in self._renderers:
            self._renderers[width, height] = mujoco.Renderer(self.model, height, width)
        renderer = self._renderers[width, height]
        # Camera poses, which kinematics leaves stale.
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)
        renderer.update_scene(self.data, camera=camera)
        return renderer.render().copy()

    def raycaster(self, groups: Sequence[int]) -> MujocoRaycaster:
        mask = np.isin(np.arange(mujoco.mjNGROUP), groups).astype(np.uint8)
        return MujocoRaycaster(self.model, self.data, mask)

    def close(self) -> None:
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()

    def imu(
        self, sensor: str
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        _, quat = self.pose(sensor)
        to_sensor = np.zeros(9)
        mujoco.mju_quat2Mat(to_sensor, quat)
        world_from_sensor = to_sensor.reshape(3, 3)
        linear, angular = self.velocity(sensor)
        velocity = world_from_sensor @ linear
        before = self._previous.get(sensor)
        self._previous[sensor] = (self.time, velocity)
        elapsed = self.time - before[0] if before else 0.0
        acceleration = (velocity - before[1]) / elapsed if before and elapsed > 0 else np.zeros(3)
        # An accelerometer measures specific force: at rest, the floor pushing up against gravity.
        return angular, world_from_sensor.T @ (acceleration - _GRAVITY), quat

    def _frame(self, frame: str) -> tuple[int, int]:
        if frame not in self._frames:
            for kind in (
                mujoco.mjtObj.mjOBJ_BODY,
                mujoco.mjtObj.mjOBJ_SITE,
                mujoco.mjtObj.mjOBJ_CAMERA,
            ):
                index = mujoco.mj_name2id(self.model, kind, frame)
                if index >= 0:
                    self._frames[frame] = (kind, index)
                    break
            else:
                raise KeyError(f"no body, site or camera named {frame!r}")
        return self._frames[frame]

    def _joint(self, name: str) -> int:
        if name not in self._joints:
            self._joints[name] = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return self._joints[name]

    def _root(self, robot_id: str) -> int:
        bodies = [b for b in range(1, self.model.nbody) if self.model.body_parentid[b] == 0]
        return next(b for b in bodies if self.model.body(b).name.startswith(f"{robot_id}/"))

    def _pose_entity(self, entity: Entity, body: int) -> None:
        model, data = self.model, self.data
        if entity.pose is not None:
            position, (x, y, z, w) = entity.pose[:3], entity.pose[3:]
            quat_wxyz = np.array([w, x, y, z])
            joint = model.body_jntadr[body]
            if model.body_mocapid[body] >= 0:
                data.mocap_pos[model.body_mocapid[body]] = position
                data.mocap_quat[model.body_mocapid[body]] = quat_wxyz
            elif joint >= 0 and model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
                qpos, dof = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
                data.qpos[qpos : qpos + 7] = [*position, *quat_wxyz]
                velocity = np.zeros(6) if entity.velocity is None else entity.velocity
                # The world's velocities are world-aligned; a free joint's angular velocity is the body's.
                rotation = np.zeros(9)
                mujoco.mju_quat2Mat(rotation, quat_wxyz)
                data.qvel[dof : dof + 3] = velocity[:3]
                data.qvel[dof + 3 : dof + 6] = rotation.reshape(3, 3).T @ velocity[3:]
