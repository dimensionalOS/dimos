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

"""A WorldView drawn by MuJoCo: its own model of the composed scene, posed from world_observations.

Never the engine's model. This compiles the same layer the engine did -- the one the manager
composed, which `get_scene` answers with -- adds whatever the sensor needs at its own mount, and
poses it from the observations. So a camera draws the furniture that is really there, a lidar
ranges against it, and neither reads a physics engine or needs the simulation to tell it
anything.
"""

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
        # Its own model of the composed scene, shortened for the same robots the engine
        # shortened for, so the entity ids in the observations name bodies in here.
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
        self._names: dict[int, list[str]] = {}
        self._velocities_current = False
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}
        # An IMU's previous world velocity and the time of it, for its acceleration.
        self._previous: dict[str, tuple[float, NDArray[np.float64]]] = {}
        # Per posed body: where the body that actually moves it sits relative to it.
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
                    # A robot's joints are named "<robot>/<joint>" in this model too; an object's are its own.
                    joint = self._joint(entity.path if owner in self._robots else name)
                    if joint >= 0:
                        data.qpos[model.jnt_qposadr[joint]] = entity.joint[0]
                        data.qvel[model.jnt_dofadr[joint]] = entity.joint[1]
                elif entity.path in self._robots:
                    self._pose_entity(entity, self._root(entity.path))
                else:
                    name = resolved(self._named(mujoco.mjtObj.mjOBJ_BODY), entity.path)
                    # An object spawned after the world was composed is not in this view.
                    if name is not None:
                        self._pose_entity(
                            entity, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
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

    def raycaster(
        self,
        groups: Sequence[int],
        mounted_at: str | None = None,
        blind_to: Sequence[str] = (),
    ) -> MujocoRaycaster:
        """Rays against ``groups``; ``mounted_at`` names a frame whose own body they pass through.

        A ranging sensor is bolted to something, and in a model it sits inside that something's
        collision hull rather than proud of its surface -- a hull is a few convex shapes around
        a shape that is neither. Cast against collision geometry without saying so and every
        ray, in every direction, returns the mount a couple of centimetres away; a sensor that
        drops returns inside its blind range then reports nothing at all.

        Excluding the one body the sensor is mounted on says only "a sensor does not see its own
        housing", which is true of every sensor on every robot and needs no distance picked by
        hand. Everything else stays visible, the arms and legs hanging off that mount included.

        ``blind_to`` names any further frames to ignore, for the cases that rule does not cover:
        a radome, a mast the sensor looks past, a wall a test wants out of the way. MuJoCo can
        exclude one body from a cast, so these are moved instead to a geom group the cast does
        not read -- in this view's own model, and a geom's group is a label it selects and draws
        by, never one it simulates by.
        """
        mask = np.isin(np.arange(mujoco.mjNGROUP), groups).astype(np.uint8)
        for frame in blind_to or ():
            self._hide_from_rays(self._mount(frame), groups)
        mount = NO_BODY_EXCLUDED if mounted_at is None else self._mount(mounted_at)
        return MujocoRaycaster(self.model, self.data, mask, mount)

    def _hide_from_rays(self, body: int, groups: Sequence[int]) -> None:
        """Move ``body``'s geoms to a group ``groups`` leaves out, so rays pass through it."""
        spare = next((g for g in range(mujoco.mjNGROUP) if g not in groups), None)
        if spare is None:
            raise ValueError(
                f"a cast over every geom group ({sorted(groups)}) leaves nowhere to put a frame "
                f"it is meant to be blind to"
            )
        start = int(self.model.body_geomadr[body])
        self.model.geom_group[start : start + int(self.model.body_geomnum[body])] = spare

    def _mount(self, frame: str) -> int:
        """The body ``frame`` is attached to: the nearest one at or above it that has any geometry.

        A mount frame used to be a site, whose body is whatever it was placed on. Described in
        USD it is a link of its own, welded to the thing it is bolted to -- the G1's mid360_link
        hangs off torso_link rather than marking a spot on it -- and a frame carries no geometry,
        so taking it at face value excluded a body with nothing in it. The housing stayed visible,
        every ray came back 22 mm away inside the sensor's blind range, and the scan came out
        empty: the exact failure this exclusion exists to prevent.

        Walking up stops short of the world, whose geometry is the floor and the last thing a
        ranging sensor should be blind to.
        """
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
        """A body, site or camera, by the name the world calls it.

        Resolved rather than looked up, because off a USD layer MuJoCo names everything by its
        prim path: the crate the observations call "crate" is "/scene/Geometry/crate" in here,
        and the entities are published under the short name on purpose -- nobody can address a
        prim path. `resolve` is the same rule the rest of this engine uses for the same reason.
        """
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
        """The robot whose subtree this frame is in, by the id the world announced; None for the room's.

        A sensor bolted to a robot sometimes needs to know which one -- the L1 publishes a map
        centred on the robot it rides -- and a frame's own name says so, because a robot's parts
        are all called "<id>/<something>".
        """
        kind, index = self._frame(frame)
        name = mujoco.mj_id2name(self.model, kind, index) or ""
        robot, _, _ = name.partition("/")
        return robot if robot in self._robots else None

    def _named(self, kind: int) -> list[str]:
        """Every name of this kind in the model, for resolving a short one against."""
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
        """The body whose joint moves ``body``, and where that body has to go to put it here.

        A robot grafted into a scene hangs off a mount frame, and the free joint is on the frame
        rather than on the robot's own root: MuJoCo takes a free joint only on a top-level body.
        So a view told "this robot is at x" has to move the frame, not the root, and the two sit
        a fixed distance apart -- the offset the graft put between them.

        Rigid, so the offset is read once off the model's reference pose and never changes. Where
        the posed body is itself the one with the joint, this is the identity and says so.
        """
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
            # From the holder to the body, in the holder's own axes, at the reference pose.
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
        """The body everything else on this robot hangs off, found in the model rather than named.

        The robot's bodies are all called "<id>/<something>"; its root is the one whose *parent*
        is not. That holds whether the robot was attached to the worldbody or arrived composed
        into a room -- where nothing is top-level but the room itself, which is what broke the
        older rule of "the top-level body whose name starts with the id".

        It also means a view needs no description of the robot at all: which body is the root is
        a fact about the world it is looking at, and reading it there cannot disagree with it.

        Cached like _joint and _frame: the model is fixed once compiled, and an IMU reading this
        200 times a second would otherwise rescan every body in the scene each time.
        """
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
                # The world's velocities are world-aligned; a free joint's angular velocity is the body's.
                rotation = np.zeros(9)
                mujoco.mju_quat2Mat(rotation, quat_wxyz)
                data.qvel[dof : dof + 3] = velocity[:3]
                data.qvel[dof + 3 : dof + 6] = rotation.reshape(3, 3).T @ velocity[3:]
