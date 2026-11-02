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


"""The MuJoCo backend: one compiled world, stepped in real time, each frame a WorldChange of what moved."""

import asyncio
import base64
from collections.abc import Mapping, Sequence
import json
from pathlib import Path
from threading import Lock
import time
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET

from dimos_lcm.world_msgs import WorldChange
import mujoco
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.sim2.actions import (
    Action,
    ApplyWrench,
    Attach,
    Detach,
    Interact,
    JointCommand,
    Pause,
    Remove,
    Reset,
    Resume,
    SetJointPositions,
    SetPose,
    SetVelocity,
    Spawn,
    Step,
)
from dimos.sim2.backend import Publisher
from dimos.sim2.spec import Simulated, SimulatedRobot
from dimos.sim2.world import compile_world, keyframe
from dimos.sim2.world_change import (
    EPOCH,
    OBJECT,
    PAUSED,
    ROBOT,
    WorldWriter,
    cbor_update,
    pose_update,
    tag_update,
    velocity_update,
)

if TYPE_CHECKING:
    from dimos.sim2.module import SimulationConfig

# Physics catches up to the wall clock in bursts, publishing after each, since macOS oversleeps short
# waits. Lag past one burst is dropped: a controller's command must not land a whole control period late.
_MAX_BURST_SECONDS = 0.01

_SINGLE_DOF = (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
# Less than this since it was last sent is not a change: a resting body's solver jitter is not motion.
_MOVED = 1e-6

# Geom types a dimos.scene_data.v1 viewer draws (a plane being its own ground grid).
_SCENE_GEOM_TYPES = {
    int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_ELLIPSOID): "ellipsoid",
    int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
    int(mujoco.mjtGeom.mjGEOM_MESH): "mesh",
}


def scene_bodies(
    model: mujoco.MjModel, data: mujoco.MjData, skip_roots: set[int], movers: set[int]
) -> tuple[list[dict[str, Any]], dict[str, dict[str, str]]]:
    """dimos.scene_data.v1 bodies and meshes: every body outside skip_roots with a drawable geom.

    A mover (a root that moves) is "mocap", which a viewer moves by its TF frame, and carries its mass.
    """
    bodies, meshes = [], {}
    for body in range(model.nbody):
        if model.body_rootid[body] in skip_roots:
            continue
        geoms = []
        for geom in range(
            model.body_geomadr[body], model.body_geomadr[body] + model.body_geomnum[body]
        ):
            # Group 3 and up hold collision-only geoms, drawn by nothing.
            kind = _SCENE_GEOM_TYPES.get(int(model.geom_type[geom]))
            if kind is None or model.geom_group[geom] >= 3:
                continue
            material = model.geom_matid[geom]
            rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[geom]
            entry = {
                "name": model.geom(geom).name or f"geom{geom}",
                "type": kind,
                "size": model.geom_size[geom].tolist(),
                "pos": model.geom_pos[geom].tolist(),
                "quat": _xyzw(model.geom_quat[geom]),
                "rgba": rgba.tolist(),
            }
            if kind == "mesh":
                mesh = int(model.geom_dataid[geom])
                entry["mesh"] = name = model.mesh(mesh).name or f"mesh{mesh}"
                meshes.setdefault(name, _mesh_data(model, mesh))
            geoms.append(entry)
        if geoms:
            mover = body in movers
            bodies.append(
                {
                    "name": model.body(body).name or "world",
                    "label": model.body(body).name or "world",
                    "pos": data.xpos[body].tolist(),
                    "quat": _xyzw(data.xquat[body]),
                    "mocap": bool(model.body_mocapid[body] >= 0) or mover,
                    "geoms": geoms,
                    "lights": [],
                    **({"mass": float(model.body_subtreemass[body])} if mover else {}),
                }
            )
    return bodies, meshes


def _xyzw(wxyz: np.ndarray) -> list[float]:
    w, x, y, z = wxyz
    return [float(x), float(y), float(z), float(w)]


def _mesh_data(model: mujoco.MjModel, mesh: int) -> dict[str, str]:
    vertices = model.mesh_vert[
        model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]
    ]
    faces = model.mesh_face[
        model.mesh_faceadr[mesh] : model.mesh_faceadr[mesh] + model.mesh_facenum[mesh]
    ]
    return {
        "vertices": base64.b64encode(vertices.astype("<f4").tobytes()).decode(),
        "faces": base64.b64encode(faces.astype("<u4").tobytes()).decode(),
    }


class MujocoBackend:
    def __init__(
        self, config: "SimulationConfig", simulated: Mapping[str, Simulated], directory: Path
    ) -> None:
        self._config = config
        self._scene = config.scene or Path(__file__).with_name("logistics.xml")
        self._robots = {d.robot_id: d for d in simulated.values() if isinstance(d, SimulatedRobot)}
        self.model = compile_world(self._scene, simulated.values())
        # Robots whose controllers the world steps in lockstep with, and the actions each sent since its last tick.
        self._controllers = [r for r in self._robots.values() if r.control_period is not None]
        self._next_ticks = [0.0] * len(self._controllers)
        self._acted = {robot.robot_id: 0 for robot in self._controllers}
        self.data = mujoco.MjData(self.model)
        self._pending: list[Action] = []
        self._lock = Lock()
        self._paused = False
        self._steps_requested = 0
        # Root body -> the base velocity SetVelocity holds it at.
        self._velocities: dict[int, Twist] = {}
        # Carried root body -> (carrier body, its position and orientation in the carrier's frame).
        self._carried: dict[int, tuple[int, np.ndarray, np.ndarray]] = {}
        self._roots = self._entity_roots()
        # Each entity's single-dof joints, and their names as the entity calls them.
        self._joints = {name: self._joints_of(name, body) for name, body in self._roots.items()}
        self.world = WorldWriter()
        self._announced = False
        # What the world was last told: each entity's pose and velocity, its joints' rows, and whether paused.
        self._sent: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._sent_joints: dict[str, np.ndarray] = {}
        self._sent_paused = False
        self._refusals: list[tuple[str, str]] = []
        self._wrench_until: dict[int, float] = {}
        self._files = _files_named_by(self._scene, None)
        for robot in self._robots.values():
            self._files.update(_files_named_by(robot.model, None))
        self._reset()

    def describe(self) -> dict[str, Any]:
        return {"scene": str(self._scene)}

    def assets(self) -> Mapping[str, Path | bytes]:
        return {"world.xml": self._scene.read_bytes(), **self._files}

    def scene_data(self) -> str:
        robot_roots = {self._roots[robot_id] for robot_id in self._robots}
        objects = {body for name, body in self._roots.items() if name not in self._robots}
        bodies, meshes = scene_bodies(self.model, self.data, robot_roots, objects)
        return json.dumps(
            {
                "format": "dimos.scene_data.v1",
                "id": f"{self._scene.stem}-{id(self)}",
                "frame": "world",
                "bodies": bodies,
                "meshes": meshes,
                "hfields": {},
                "textures": {},
            }
        )

    def apply(self, actions: Sequence[Action]) -> None:
        with self._lock:
            self._pending.extend(actions)
            for action in actions:
                if isinstance(action, JointCommand) and action.entity in self._acted:
                    self._acted[action.entity] += 1

    async def run(self, world_change: Publisher) -> None:
        model, data = self.model, self.data
        self._reset()
        started = time.monotonic()
        max_steps = max(1, int(_MAX_BURST_SECONDS / model.opt.timestep))
        while True:
            self._apply_pending()
            if self._paused:
                steps, self._steps_requested = self._steps_requested, 0
                started = time.monotonic() - data.time / self._config.speed
            else:
                steps = int(
                    ((time.monotonic() - started) * self._config.speed - data.time)
                    / model.opt.timestep
                )
                if steps > max_steps:
                    started += (steps - max_steps) * model.opt.timestep / self._config.speed
                    steps = max_steps
            for _ in range(steps):
                if await self._hold_for_controllers(world_change):
                    started = time.monotonic() - data.time / self._config.speed
                self._hold_velocities()
                self._hold_carried()
                mujoco.mj_step(model, data)
                self._expire_wrenches()
            if not world_change.busy:
                world_change.offer(self.frame())
            await asyncio.sleep(0.001)

    async def _hold_for_controllers(self, world_change: Publisher) -> bool:
        """At a controller's tick, send it the world and wait for its actions; True if the world waited."""
        now = self.data.time
        due = [
            robot.robot_id
            for robot, tick in zip(self._controllers, self._next_ticks, strict=True)
            if now >= tick - self.model.opt.timestep / 2
        ]
        if not due:
            return False
        for i, robot in enumerate(self._controllers):
            if robot.robot_id in due:
                self._next_ticks[i] += robot.control_period  # type: ignore[operator]
        with self._lock:
            # One that never acted is not there yet; the world does not wait for it.
            waiting = [robot_id for robot_id in due if self._acted.get(robot_id)]
            for robot_id in due:
                self._acted[robot_id] = 0
        # The tick's frame must reach its controllers, so it waits out a busy publisher rather than being dropped.
        while world_change.busy:
            await asyncio.sleep(0)
        world_change.offer(self.frame(tuple(due)))
        deadline = time.monotonic() + self._config.controller_timeout
        while time.monotonic() < deadline:
            with self._lock:
                if all(self._acted.get(robot_id) for robot_id in waiting):
                    break
            await asyncio.sleep(0)
        self._apply_pending()
        return bool(waiting)

    def _reset(self) -> None:
        self._next_ticks = [0.0] * len(self._controllers)
        self._velocities.clear()
        self._carried.clear()
        self._wrench_until.clear()
        mujoco.mj_resetData(self.model, self.data)
        self._hold_initial_pose()
        mujoco.mj_forward(self.model, self.data)

    def _initial_joints(self) -> dict[str, float]:
        """Every joint the world starts somewhere other than zero, and holds there until commanded."""
        initial = {}
        for robot in self._robots.values():
            held = keyframe(robot)[1] | {servo.joint: servo.initial for servo in robot.servos}
            initial |= {f"{robot.robot_id}/{name}": value for name, value in held.items()}
        return initial

    def _hold_initial_pose(self) -> None:
        for name, value in self._initial_joints().items():
            joint = self.model.joint(name)
            self.data.qpos[joint.qposadr[0]] = value
            self.data.ctrl[self.model.actuator_trnid[:, 0] == joint.id] = value

    def _apply_pending(self) -> None:
        with self._lock:
            pending, self._pending = self._pending, []
        for action in pending:
            try:
                self._apply(action)
            except (KeyError, ValueError) as error:
                # One bad action must not stop the world; its target hears why on the next frame.
                self._refusals.append(
                    (getattr(action, "entity", "world"), f"{type(action).__name__}: {error}")
                )
        if pending:
            mujoco.mj_forward(self.model, self.data)

    def _apply(self, action: Action) -> None:
        model, data = self.model, self.data
        match action:
            case JointCommand():
                self._command_joints(action)
            case SetJointPositions(entity, names, positions):
                for name, position in zip(names, positions, strict=True):
                    joint = model.joint(self._joint_name(entity, name))
                    data.qpos[joint.qposadr[0]] = position
                    data.qvel[joint.dofadr[0]] = 0.0
            case SetPose(entity, pose, twist):
                self._set_pose(self._roots[entity], pose, twist)
            case ApplyWrench(entity, force, torque, point, duration):
                index = self._roots[entity]
                # At point (in the body's frame), the force also turns the body about its origin.
                lever = data.xmat[index].reshape(3, 3) @ np.asarray(point, float)
                data.xfrc_applied[index] = [*force, *(np.asarray(torque) + np.cross(lever, force))]
                self._wrench_until[index] = data.time + duration
            case Pause():
                self._paused = True
            case Resume():
                self._paused = False
            case SetVelocity(entity, twist):
                self._velocities[self._roots[entity]] = twist
            case Interact(entity, action):
                self._interact(entity, action)
            case Attach(entity, to, body):
                carrier = (
                    self._roots[to] if body is None else model.body(self._body_name(to, body)).id
                )
                self._carry(self._roots[entity], carrier)
            case Detach(entity):
                self._carried.pop(self._roots[entity], None)
            case Spawn() | Remove():
                raise ValueError(
                    "the mujoco backend compiles its world once; it cannot add or remove objects"
                )
            case Step(steps):
                self._steps_requested += steps
            case Reset():
                self._reset()

    def _command_joints(self, command: JointCommand) -> None:
        if command.kp is not None or command.kd is not None:
            self._command_motors(command)
            return
        model = self.model
        given = {
            "position": command.position,
            "velocity": command.velocity,
            "effort": command.effort,
        }
        for index, name in enumerate(command.names):
            joint = model.joint(self._joint_name(command.entity, name))
            for actuator in np.flatnonzero(model.actuator_trnid[:, 0] == joint.id):
                kind = _actuator_kind(model, int(actuator))
                values = given[kind]
                if values is None:
                    raise ValueError(f"{name}'s actuator takes {kind}, which the command lacks")
                self.data.ctrl[actuator] = values[index]

    def _command_motors(self, command: JointCommand) -> None:
        """A motor command through the joint's affine actuator: its gains become kp and kd, its control the rest."""
        model, n = self.model, len(command.names)
        zeros = (0.0,) * n
        kp, kd = command.kp or zeros, command.kd or zeros
        position, velocity = command.position or zeros, command.velocity or zeros
        effort = command.effort or zeros
        for index, name in enumerate(command.names):
            joint = model.joint(self._joint_name(command.entity, name))
            for actuator in np.flatnonzero(model.actuator_trnid[:, 0] == joint.id):
                if model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_AFFINE:
                    raise ValueError(f"{name}'s actuator is not a servo, so it cannot take gains")
                p, d = kp[index], kd[index]
                # force = gain * ctrl + bias1 * q + bias2 * dq; solved for the motor law above.
                model.actuator_gainprm[actuator, 0] = p if p > 0 else 1.0
                model.actuator_biasprm[actuator, 1:3] = (-p, -d)
                feedforward = d * velocity[index] + effort[index]
                self.data.ctrl[actuator] = (
                    position[index] + feedforward / p if p > 0 else feedforward
                )

    def _interact(self, entity: str, action: str) -> None:
        """Sets the entity's joints as the scene's keyframe "<entity>/<action>" has them."""
        model, data = self.model, self.data
        key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, f"{entity}/{action}")
        if key < 0:
            raise ValueError(
                f"{entity} has no action {action!r}: the scene has no keyframe {entity}/{action}"
            )
        root = self._roots[entity]
        for joint in range(model.njnt):
            if (
                model.body_rootid[model.jnt_bodyid[joint]] != root
                or model.jnt_type[joint] not in _SINGLE_DOF
            ):
                continue
            value = model.key_qpos[key][model.jnt_qposadr[joint]]
            data.qpos[model.jnt_qposadr[joint]] = value
            data.qvel[model.jnt_dofadr[joint]] = 0.0
            data.ctrl[model.actuator_trnid[:, 0] == joint] = value

    def _carry(self, body: int, carrier: int) -> None:
        model, data = self.model, self.data
        joint = model.body_jntadr[body]
        if joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(
                f"{model.body(body).name!r} has no free joint, so it cannot be carried"
            )
        inverse = np.zeros(4)
        mujoco.mju_negQuat(inverse, data.xquat[carrier])
        offset, orientation = np.zeros(3), np.zeros(4)
        mujoco.mju_rotVecQuat(offset, data.xpos[body] - data.xpos[carrier], inverse)
        mujoco.mju_mulQuat(orientation, inverse, data.xquat[body])
        self._carried[body] = (carrier, offset, orientation)

    def _hold_carried(self) -> None:
        model, data = self.model, self.data
        for body, (carrier, offset, orientation) in self._carried.items():
            joint = model.body_jntadr[body]
            qpos, dof = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
            position, quat = np.zeros(3), np.zeros(4)
            mujoco.mju_rotVecQuat(position, offset, data.xquat[carrier])
            mujoco.mju_mulQuat(quat, data.xquat[carrier], orientation)
            data.qpos[qpos : qpos + 7] = [*(data.xpos[carrier] + position), *quat]
            data.qvel[dof : dof + 6] = 0.0

    def _hold_velocities(self) -> None:
        model, data = self.model, self.data
        for body, twist in self._velocities.items():
            joint = model.body_jntadr[body]
            if joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
                continue
            dof = model.jnt_dofadr[joint]
            w, x, y, z = data.xquat[body]
            yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            vx, vy = twist.linear.x, twist.linear.y
            # A free joint's linear velocity is world-frame; its angular one is the body's (z ~ yaw rate when upright).
            data.qvel[dof : dof + 2] = [
                vx * np.cos(yaw) - vy * np.sin(yaw),
                vx * np.sin(yaw) + vy * np.cos(yaw),
            ]
            data.qvel[dof + 5] = twist.angular.z

    def _set_pose(self, body: int, pose: Pose, twist: Twist | None) -> None:
        model, data = self.model, self.data
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat_wxyz = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        if model.body_mocapid[body] >= 0:
            data.mocap_pos[model.body_mocapid[body]] = position
            data.mocap_quat[model.body_mocapid[body]] = quat_wxyz
            return
        joint = model.body_jntadr[body]
        if joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f"body {model.body(body).name!r} has no free joint to move")
        qpos, dof = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
        data.qpos[qpos : qpos + 7] = [*position, *quat_wxyz]
        # A free joint's linear velocity is in the world frame and its angular one in the body's.
        velocity = np.zeros(6)
        if twist is not None:
            velocity[:3] = [twist.linear.x, twist.linear.y, twist.linear.z]
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, np.asarray(quat_wxyz, dtype=np.float64))
            angular = [twist.angular.x, twist.angular.y, twist.angular.z]
            velocity[3:] = rotation.reshape(3, 3).T @ angular
        data.qvel[dof : dof + 6] = velocity

    def _joint_name(self, entity: str, name: str) -> str:
        return f"{entity}/{name}" if entity in self._robots else name

    def _body_name(self, entity: str, name: str) -> str:
        return f"{entity}/{name}" if entity in self._robots else name

    def _entity_roots(self) -> dict[str, int]:
        """Each robot's root body, and each movable object: a world child with joints, outside any robot."""
        model = self.model
        roots = {
            robot_id: model.body(f"{robot_id}/{r.root_body}").id
            for robot_id, r in self._robots.items()
        }
        robot_roots = set(roots.values())
        jointed = set(model.body_rootid[model.jnt_bodyid])
        for body in range(1, model.nbody):
            if model.body_parentid[body] == 0 and body not in robot_roots and body in jointed:
                roots[model.body(body).name or f"body{body}"] = body
        return roots

    def frame(self, waiting_for: tuple[str, ...] = ()) -> WorldChange:
        """What changed since the last frame: entities that moved, joints, refusals, and the robots the world waits for."""
        world, data = self.world, self.data
        if not self._announced:
            self._announce()
        for name, body in self._roots.items():
            pose = np.array([*data.xpos[body], *_xyzw(data.xquat[body])])
            angular_linear = np.zeros(6)
            # The body origin's velocity (XBODY), world-aligned, not its center of mass's.
            mujoco.mj_objectVelocity(
                self.model, data, mujoco.mjtObj.mjOBJ_XBODY, body, angular_linear, 0
            )
            velocity = np.concatenate([angular_linear[3:], angular_linear[:3]])
            sent = self._sent.get(name)
            if sent is None or not (_close(sent[0], pose) and _close(sent[1], velocity)):
                world.update(name, pose_update(pose), velocity_update(velocity))
                self._sent[name] = (pose, velocity)
            joints, _ = self._joints[name]
            if joints:
                rows = np.column_stack(
                    [
                        data.qpos[self.model.jnt_qposadr[joints]],
                        data.qvel[self.model.jnt_dofadr[joints]],
                        data.qfrc_actuator[self.model.jnt_dofadr[joints]],
                    ]
                )
                sent_rows = self._sent_joints.get(name)
                if sent_rows is None or not _close(sent_rows, rows):
                    world.joints(name, rows)
                    self._sent_joints[name] = rows
        if self._paused != self._sent_paused:
            world.update("world", tag_update(PAUSED, self._paused))
            self._sent_paused = self._paused
        for target, reason in self._refusals:
            world.event(
                target if target in world.ids else "world", "events/refused", {"reason": reason}
            )
        self._refusals.clear()
        for robot_id in waiting_for:
            world.event(robot_id, "events/act")
        return world.end_frame(float(data.time))

    def _announce(self) -> None:
        """The first frame adds the world, each entity, and each entity's joints."""
        world = self.world
        world.add(
            "world",
            cbor_update(EPOCH, time.time() - self.data.time / self._config.speed),
            cbor_update("backend", "mujoco"),
        )
        for name in self._roots:
            robot = self._robots.get(name)
            updates = [tag_update(ROBOT if robot else OBJECT)]
            if robot is not None:
                # What to request on render_file_request to draw it on its own.
                updates.append(cbor_update("model", str(robot.model)))
            world.add(name, *updates)
            if self._joints[name][1]:
                world.add_joints(name, self._joints[name][1])
        self._announced = True

    def _joints_of(self, name: str, body: int) -> tuple[list[int], list[str]]:
        model = self.model
        joints = [
            j
            for j in range(model.njnt)
            if model.body_rootid[model.jnt_bodyid[j]] == body and model.jnt_type[j] in _SINGLE_DOF
        ]
        prefix = f"{name}/" if name in self._robots else ""
        return joints, [model.joint(j).name.removeprefix(prefix) for j in joints]

    def _expire_wrenches(self) -> None:
        for body, until in list(self._wrench_until.items()):
            if self.data.time >= until:
                self.data.xfrc_applied[body] = 0.0
                del self._wrench_until[body]


def _close(a: np.ndarray, b: np.ndarray) -> bool:
    return a.shape == b.shape and bool(np.allclose(a, b, rtol=0.0, atol=_MOVED))


def _actuator_kind(model: mujoco.MjModel, actuator: int) -> str:
    if model.actuator_biastype[actuator] == mujoco.mjtBias.mjBIAS_NONE:
        return "effort"
    return "position" if model.actuator_biasprm[actuator][1] != 0 else "velocity"


def _files_named_by(mjcf: Path, assets: Path | None) -> dict[str, Path]:
    """The model and every file it names (meshes, textures, ...), by full path; a robot's assets hold its own."""
    root = ET.parse(mjcf).getroot()
    compiler = root.find("compiler")
    directories = {
        kind: mjcf.parent / (compiler.get(f"{kind}dir") or compiler.get("assetdir") or "")
        if compiler is not None
        else mjcf.parent
        for kind in ("mesh", "texture")
    }
    files = {str(mjcf): mjcf}
    for element in root.iter():
        name = element.get("file")
        if name is None:
            continue
        directory = directories["texture" if element.tag in ("texture", "hfield") else "mesh"]
        path = assets / Path(name).name if assets is not None else directory / name
        if path.is_file():
            files[str(path)] = path
    return files
