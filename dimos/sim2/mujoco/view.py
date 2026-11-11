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


from collections.abc import Iterable, Sequence
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.sim2.mujoco.names import resolved
from dimos.sim2.mujoco.world import SensorMount, compile_world, mount_sensor
from dimos.sim2.world_observations import WORLD, Entity, WorldMirror
from dimos.simulation.sensors.mujoco_raycaster import NO_BODY_EXCLUDED, MujocoRaycaster

_GRAVITY = np.array([0.0, 0.0, -9.81])


class MujocoView:
    def __init__(
        self, scene: Path, ids: Iterable[str] = (), mount: SensorMount | None = None
    ) -> None:
        self._robots = set(ids)
        spec = compile_world(scene, ids=self._robots)
        if mount is not None:
            mount_sensor(spec, mount)
        self.model = spec.compile()
        self._joints: dict[str, int] = {}
        self._robot_roots: dict[str, int] = {}
        self.data = mujoco.MjData(self.model)
        mujoco.mj_kinematics(self.model, self.data)
        self._frames: dict[str, tuple[int, int]] = {}
        self._hidden: list[tuple[int, int, NDArray[np.uint8]]] = []
        self._names: dict[int, list[str]] = {}
        self._velocities_current = False
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}
        self._previous: dict[str, tuple[float, NDArray[np.float64]]] = {}
        self._offsets: dict[int, tuple[NDArray[np.float64], Rotation]] = {}

    @property
    def time(self) -> float:
        return float(self.data.time)

    def update(self, world: WorldMirror) -> None:
        model, data = self.model, self.data
        with world.lock:
            for entity_id, entity in world.entities.items():
                owner, joint_of, name = entity.path.partition("/")
                if entity_id == WORLD:
                    continue
                if joint_of:
                    if entity.joint is None:
                        continue
                    joint = self._joint(entity.path if owner in self._robots else name)
                    if joint >= 0:
                        data.qpos[model.jnt_qposadr[joint]] = entity.joint[0]
                        data.qvel[model.jnt_dofadr[joint]] = entity.joint[1]
                elif entity.path in self._robots:
                    self._pose_entity(entity, self._root(entity.path))
                else:
                    body = resolved(self._named(mujoco.mjtObj.mjOBJ_BODY), entity.path)
                    if body is not None:
                        self._pose_entity(
                            entity, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
                        )
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
        kind, index = self._frame(frame)
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
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)
        renderer.update_scene(self.data, camera=camera)
        return renderer.render().copy()

    def raycaster(
        self,
        groups: Sequence[int],
        mounted_at: str | None = None,
        blind_to: Sequence[str] = (),
    ) -> MujocoRaycaster:
        mask = np.isin(np.arange(mujoco.mjNGROUP), groups).astype(np.uint8)
        self._show_hidden()
        for frame in blind_to or ():
            self._hide_from_rays(self._mount(frame), groups)
        mount = NO_BODY_EXCLUDED if mounted_at is None else self._mount(mounted_at)
        return MujocoRaycaster(self.model, self.data, mask, mount)

    def _hide_from_rays(self, body: int, groups: Sequence[int]) -> None:
        spare = next((g for g in range(mujoco.mjNGROUP) if g not in groups), None)
        if spare is None:
            raise ValueError(
                f"a cast over every geom group ({sorted(groups)}) leaves nowhere to put a frame "
                f"it is meant to be blind to"
            )
        start = int(self.model.body_geomadr[body])
        count = int(self.model.body_geomnum[body])
        self._hidden.append((start, count, self.model.geom_group[start : start + count].copy()))
        self.model.geom_group[start : start + count] = spare

    def _mount(self, frame: str) -> int:
        kind, index = self._frame(frame)
        if kind == mujoco.mjtObj.mjOBJ_SITE:
            body = int(self.model.site_bodyid[index])
        elif kind == mujoco.mjtObj.mjOBJ_CAMERA:
            body = int(self.model.cam_bodyid[index])
        else:
            body = int(index)
        while self.model.body_geomnum[body] == 0:
            parent = int(self.model.body_parentid[body])
            if parent == 0:
                return body
            body = parent
        return body

    def _show_hidden(self) -> None:
        for start, count, groups in reversed(self._hidden):
            self.model.geom_group[start : start + count] = groups
        self._hidden.clear()

    def close(self) -> None:
        self._show_hidden()
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
        return angular, world_from_sensor.T @ (acceleration - _GRAVITY), quat

    def _frame(self, frame: str) -> tuple[int, int]:
        if frame not in self._frames:
            for kind in (
                mujoco.mjtObj.mjOBJ_BODY,
                mujoco.mjtObj.mjOBJ_SITE,
                mujoco.mjtObj.mjOBJ_CAMERA,
            ):
                match = resolved(self._named(kind), frame)
                if match is not None:
                    self._frames[frame] = (kind, mujoco.mj_name2id(self.model, kind, match))
                    break
            else:
                raise KeyError(f"no body, site or camera named {frame!r}")
        return self._frames[frame]

    def owner(self, frame: str) -> str | None:
        kind, index = self._frame(frame)
        name = mujoco.mj_id2name(self.model, kind, index) or ""
        robot, _, _ = name.partition("/")
        return robot if robot in self._robots else None

    def _named(self, kind: int) -> list[str]:
        if kind not in self._names:
            count = {
                mujoco.mjtObj.mjOBJ_BODY: self.model.nbody,
                mujoco.mjtObj.mjOBJ_SITE: self.model.nsite,
                mujoco.mjtObj.mjOBJ_CAMERA: self.model.ncam,
                mujoco.mjtObj.mjOBJ_JOINT: self.model.njnt,
            }[kind]
            self._names[kind] = [
                mujoco.mj_id2name(self.model, kind, index) or "" for index in range(count)
            ]
        return self._names[kind]

    def _joint(self, name: str) -> int:
        if name not in self._joints:
            match = resolved(self._named(mujoco.mjtObj.mjOBJ_JOINT), name)
            self._joints[name] = (
                -1
                if match is None
                else mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, match)
            )
        return self._joints[name]

    def _moved_by(
        self, body: int, position: NDArray[np.float64], quat_wxyz: NDArray[np.float64]
    ) -> tuple[int, NDArray[np.float64], NDArray[np.float64]]:
        model = self.model
        holder = body
        while model.body_jntadr[holder] < 0 and model.body_mocapid[holder] < 0:
            parent = int(model.body_parentid[holder])
            if parent == 0:
                return body, position, quat_wxyz
            holder = parent
        if holder == body:
            return body, position, quat_wxyz
        if body not in self._offsets:
            reference = mujoco.MjData(model)
            mujoco.mj_kinematics(model, reference)
            holder_rotation = Rotation.from_quat(reference.xquat[holder], scalar_first=True)
            self._offsets[body] = (
                holder_rotation.inv().apply(reference.xpos[body] - reference.xpos[holder]),
                holder_rotation.inv()
                * Rotation.from_quat(reference.xquat[body], scalar_first=True),
            )
        offset_position, offset_rotation = self._offsets[body]
        rotation = Rotation.from_quat(quat_wxyz, scalar_first=True) * offset_rotation.inv()
        return (
            holder,
            np.asarray(position) - rotation.apply(offset_position),
            np.asarray(rotation.as_quat(scalar_first=True)),
        )

    def _root(self, robot_id: str) -> int:
        if robot_id not in self._robot_roots:
            model, prefix = self.model, f"{robot_id}/"
            under = [
                body
                for body in range(1, model.nbody)
                if (model.body(body).name or "").startswith(prefix)
                and not (model.body(int(model.body_parentid[body])).name or "").startswith(prefix)
            ]
            if len(under) != 1:
                raise KeyError(
                    f"{robot_id!r} has {len(under)} root bodies in this world: "
                    f"{[model.body(b).name for b in under]}"
                )
            self._robot_roots[robot_id] = under[0]
        return self._robot_roots[robot_id]

    def _pose_entity(self, entity: Entity, body: int) -> None:
        model, data = self.model, self.data
        if entity.pose is not None:
            position, (x, y, z, w) = entity.pose[:3], entity.pose[3:]
            quat_wxyz = np.array([w, x, y, z])
            body, position, quat_wxyz = self._moved_by(body, position, quat_wxyz)
            joint = model.body_jntadr[body]
            if model.body_mocapid[body] >= 0:
                data.mocap_pos[model.body_mocapid[body]] = position
                data.mocap_quat[model.body_mocapid[body]] = quat_wxyz
            elif joint >= 0 and model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
                qpos, dof = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
                data.qpos[qpos : qpos + 7] = [*position, *quat_wxyz]
                velocity = np.zeros(6) if entity.velocity is None else entity.velocity
                rotation = np.zeros(9)
                mujoco.mju_quat2Mat(rotation, quat_wxyz)
                data.qvel[dof : dof + 3] = velocity[:3]
                data.qvel[dof + 3 : dof + 6] = rotation.reshape(3, 3).T @ velocity[3:]
