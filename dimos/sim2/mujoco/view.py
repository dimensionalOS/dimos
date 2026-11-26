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
import copy
from pathlib import Path
import sys
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.sim2.mujoco.lights import dome_sky
from dimos.sim2.mujoco.look import (
    ambient_headroom,
    blank_textures,
    display,
    light_headroom,
    linear_textures,
    merged_light,
    scale_light,
    show_sky,
    stage_tone_map,
    tone_mapped,
)
from dimos.sim2.mujoco.names import resolved
from dimos.sim2.mujoco.ray_scene import RayScene
from dimos.sim2.mujoco.renders import RenderProcess, RenderQuality, Renders
from dimos.sim2.mujoco.world import USD_SUFFIXES, SensorMount, compile_world, mount_sensor
from dimos.sim2.world_observations import WORLD, Entity, WorldMirror

_GRAVITY = np.array([0.0, 0.0, -9.81])


def _open_stage(scene: Path) -> Any:
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    return load_pxr().Usd.Stage.Open(str(scene))


class MujocoView:
    def __init__(
        self,
        scene: Path,
        ids: Iterable[str] = (),
        mount: SensorMount | None = None,
        quality: RenderQuality = RenderQuality(),
    ) -> None:
        self._quality = quality
        self._robots = set(ids)
        # Held while the view is built, so each reader of the scene below shares its parsed layers instead of reparsing them.
        stage = _open_stage(Path(scene)) if Path(scene).suffix.lower() in USD_SUFFIXES else None
        spec = compile_world(scene, ids=self._robots)
        # Only a camera draws the world, and decoding a scene's textures can take gigabytes a view.
        self._draws = mount is None or mount.kind == "camera"
        if not self._draws:
            blank_textures(spec)
        if mount is not None:
            mount_sensor(spec, mount)
        self._tone_map = (
            (quality.tone_map or stage_tone_map(Path(scene)))
            if stage is not None and self._draws
            else None
        )
        if self._tone_map is not None:
            linear_textures(spec, Path(scene))
        self.model = spec.compile()
        quality.apply_to_model(self.model)
        self._headroom = self._full_headroom = 1.0
        if self._tone_map is not None:
            # MuJoCo writes linear light to 8 bits and clips at 1, so render it scaled to fit.
            sky = dome_sky(Path(scene))
            skies = [] if sky is None else [float(sky.max())]
            # Fitting a sun into 8 bits leaves the shade a code value or two, so the shade gets its own exposure.
            self._headroom = max(ambient_headroom(self.model), *skies)
            self._full_headroom = max(light_headroom(self.model), *skies)
            scale_light(self.model, 1.0 / self._headroom)
            if sky is not None:
                show_sky(self.model, sky / self._headroom)
        self._joints: dict[str, int] = {}
        self._robot_roots: dict[str, int] = {}
        self._object_bodies: dict[str, int] = {}
        self.data = mujoco.MjData(self.model)
        mujoco.mj_kinematics(self.model, self.data)
        self._frames: dict[str, tuple[int, int]] = {}
        self._names: dict[int, list[str]] = {}
        self._velocities_current = False
        self._renders: Renders | RenderProcess | None = None
        self._previous: dict[str, tuple[float, NDArray[np.float64]]] = {}
        self._offsets: dict[int, tuple[NDArray[np.float64], Rotation]] = {}

    @property
    def time(self) -> float:
        return float(self.data.time)

    def update(self, world: WorldMirror) -> None:
        model, data = self.model, self.data
        with world.lock:
            for entity_id, entity in world.entities.items():
                if entity_id == WORLD:
                    continue
                if entity.path in self._robots:
                    root = self._root(entity.path)
                    if root >= 0:
                        self._pose_entity(entity, root)
                elif "/" in entity.path:
                    if entity.joint is None:
                        continue
                    # A robot's joint keeps its robot's id (a namespace, which may itself hold "/"); another's drops its owner.
                    owned = self._robot_of(entity.path) is not None
                    joint = self._joint(entity.path if owned else entity.path.partition("/")[2])
                    if joint >= 0:
                        data.qpos[model.jnt_qposadr[joint]] = entity.joint[0]
                        data.qvel[model.jnt_dofadr[joint]] = entity.joint[1]
                else:
                    body = self._object_body(entity.path)
                    if body >= 0:
                        self._pose_entity(entity, body)
            data.time = world.time
        mujoco.mj_kinematics(self.model, self.data)
        self._velocities_current = False

    def _object_body(self, path: str) -> int:
        """The body an object entity moves, or -1; resolved once, since names do not change."""
        if path not in self._object_bodies:
            body = resolved(self._named(mujoco.mjtObj.mjOBJ_BODY), path)
            self._object_bodies[path] = (
                -1
                if body is None
                else mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
            )
        return self._object_bodies[path]

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
        if not self._draws:
            raise ValueError("a lidar or imu view has no textures to render; open a camera view")
        if self._renders is None:
            # A worker is forked, and macOS's Metal compiles no shader there (see renders.py).
            self._renders = (RenderProcess if sys.platform == "darwin" else Renders)(
                self.model, self._quality
            )
        # Placed here as well as where it renders, as pose() reads the cameras this places.
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)
        if self._tone_map is None or self._full_headroom <= self._headroom:
            rgb = self._renders.render(self.data, camera, width, height)
            return rgb if self._tone_map is None else display(rgb, self._headroom, self._tone_map)
        # MuJoCo clips each light's color at 1, so the shade renders alone at its own scale and the direct light at the full one.
        dimmer = self._headroom / self._full_headroom
        shade, lit = self._renders.render_each(
            self.data, camera, width, height, [(1.0, False), (dimmer, True)]
        )
        linear = merged_light(shade, self._headroom, lit, self._full_headroom)
        return tone_mapped(linear, self._tone_map)

    def raycaster(
        self,
        groups: Sequence[int],
        mounted_at: str | None = None,
        blind_to: Sequence[str] = (),
        blind_to_robots: Sequence[str] = (),
    ) -> RayScene:
        """A cast against the geoms in `groups` as this view poses them; build a new one when the world's geometry changes."""
        blind: set[int] = set()
        # The hardware a sensor is mounted in is as invisible to it as the robot link it rides.
        for frame in [*blind_to, *([] if mounted_at is None else [mounted_at])]:
            blind |= {self._mount(frame), *self._carried_by(frame)}
        for robot in blind_to_robots:
            if self._root(robot) >= 0:
                blind |= set(self._under(self._root(robot)))
        return RayScene(self.model, self.data, groups, blind)

    def _under(self, root: int) -> list[int]:
        """A body and every body below it, jointed or not: a whole articulated robot from its root."""
        under = {root}
        for body in range(root + 1, self.model.nbody):
            if int(self.model.body_parentid[body]) in under:
                under.add(body)
        return sorted(under)

    def _carried_by(self, frame: str) -> list[int]:
        """The bodies welded (no joint between) under a body frame, such as a sensor's housing."""
        kind, index = self._frame(frame)
        if kind != mujoco.mjtObj.mjOBJ_BODY:
            return []
        model, carried = self.model, []
        for body in range(index + 1, model.nbody):
            holder = body
            while holder > index and model.body_jntnum[holder] == 0:
                holder = int(model.body_parentid[holder])
            if holder == index:
                carried.append(body)
        return carried

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

    def twin(self) -> "MujocoView":
        """This view's compiled world with poses of its own, which a loop can update while a thread casts against this one's."""
        twin = copy.copy(self)
        # The name and offset lookups are the model's, so the two share them.
        twin.data = mujoco.MjData(self.model)
        mujoco.mj_kinematics(self.model, twin.data)
        twin._velocities_current, twin._renders, twin._previous = False, None, {}
        return twin

    def close(self) -> None:
        if self._renders is not None:
            self._renders.close()
            self._renders = None

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
        return self._robot_of(mujoco.mj_id2name(self.model, kind, index) or "")

    def _robot_of(self, name: str) -> str | None:
        """The robot whose namespace ``name`` is under, or None; the deepest, as one robot's namespace may hold another's."""
        under = [robot for robot in self._robots if name.startswith(f"{robot}/")]
        return max(under, key=len) if under else None

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
            # A view of part of the world, as of one robot alone, holds none of the others.
            if len(under) > 1:
                raise KeyError(
                    f"{robot_id!r} has {len(under)} root bodies in this world: "
                    f"{[model.body(b).name for b in under]}"
                )
            self._robot_roots[robot_id] = under[0] if under else -1
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
