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


"""The MuJoCo backend: one compiled world, stepped in real time, each frame a WorldObservations of what moved."""

import asyncio
import base64
from collections.abc import Mapping, Sequence
import json
from pathlib import Path
import re
from threading import Lock
import time
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET

from dimos_lcm.world_msgs import WorldObservations
import mujoco
import numpy as np

from dimos.sim2.actions import Action, CborKeyValueCommand, JointCommand
from dimos.sim2.backend import Publisher
from dimos.sim2.spec import Simulated, SimulatedRobot
from dimos.sim2.usd_layer import scene_layer, usd_layer
from dimos.sim2.world import compile_world
from dimos.sim2.world_observations import ACT, REFUSED, WorldWriter

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel
    from dimos.sim2.module import SimulationConfig

# Physics catches up to the wall clock in bursts, publishing after each, since macOS oversleeps short
# waits. Lag past one burst is dropped: a controller's command must not land a whole control period late.
_MAX_BURST_SECONDS = 0.01

# As ints, not as the enums themselves. A model array yields numpy scalars, and since MuJoCo 4
# these enums are pybind11 objects rather than IntEnum: np.int32(3) == mjJNT_HINGE is True while
# np.int32(3) in (mjJNT_HINGE, ...) is False, because `in` asks the comparison that does not
# answer. Every robot's joints were dropped here, so the world announced each robot and never
# one of its joints -- nothing to publish, nothing to pose, and nothing placed in a viewer.
_SINGLE_DOF = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
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
        self,
        config: "SimulationConfig",
        world: "UsdModel | Path | None",
        simulated: Mapping[str, Simulated],
        directory: Path,
    ) -> None:
        self._config = config
        # The world as a file, which is what MuJoCo opens and what a renderer asks for by name.
        self._scene_layer = scene_layer(world)
        self._robots = {d.robot_id: d for d in simulated.values() if isinstance(d, SimulatedRobot)}
        self.model = compile_world(self._scene_layer, simulated.values())
        # Robots whose controllers the world steps in lockstep with, and the actions each sent since its last tick.
        self._controllers = [r for r in self._robots.values() if r.control_period is not None]
        self._next_ticks = [0.0] * len(self._controllers)
        self._acted = {robot.robot_id: 0 for robot in self._controllers}
        self.data = mujoco.MjData(self.model)
        self._pending: list[Action] = []
        self._lock = Lock()
        self._roots = self._entity_roots()
        # Each entity's single-dof joints, and their names as the entity calls them.
        self._joints = {name: self._joints_of(name, body) for name, body in self._roots.items()}
        self.world = WorldWriter()
        self._announced = False
        # What the world was last told: each entity's pose and its joints' rows.
        self._sent: dict[str, np.ndarray] = {}
        self._sent_joints: dict[str, np.ndarray] = {}
        self._refusals: list[tuple[str, str]] = []
        self._files = _files_named_by(self._scene_layer, None)
        # A robot's layer as a file: the name a renderer asks for it by, and what the files it
        # names are relative to.
        self._layers = {id: usd_layer(r.model) for id, r in self._robots.items()}
        for layer in self._layers.values():
            self._files.update(_files_named_by(layer, None))
        self._reset()

    def describe(self) -> dict[str, Any]:
        return {"scene": str(self._scene_layer)}

    def assets(self) -> Mapping[str, Path | bytes]:
        # Under its own name as well as "world.xml": a renderer that has the description knows
        # the layer's path, and one that only knows it is talking to a world asks for the world.
        return {
            "world.xml": self._scene_layer,
            str(self._scene_layer): self._scene_layer,
            **self._files,
        }

    def scene_data(self) -> str:
        model = self.model
        robot_roots = {self._roots[robot_id] for robot_id in self._robots}
        bodies, meshes = [], {}
        for body in range(model.nbody):
            if model.body_rootid[body] in robot_roots:
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
                bodies.append(
                    {
                        "name": model.body(body).name or "world",
                        "label": model.body(body).name or "world",
                        "pos": self.data.xpos[body].tolist(),
                        "quat": _xyzw(self.data.xquat[body]),
                        "mocap": bool(model.body_mocapid[body] >= 0),
                        "geoms": geoms,
                        "lights": [],
                    }
                )
        return json.dumps(
            {
                "format": "dimos.scene_data.v1",
                "id": f"{self._scene_layer.stem}-{id(self)}",
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

    async def run(self, world_observations: Publisher) -> None:
        model, data = self.model, self.data
        self._reset()
        started = time.monotonic()
        max_steps = max(1, int(_MAX_BURST_SECONDS / model.opt.timestep))
        while True:
            self._apply_pending()
            steps = int(
                ((time.monotonic() - started) * self._config.speed - data.time) / model.opt.timestep
            )
            if steps > max_steps:
                started += (steps - max_steps) * model.opt.timestep / self._config.speed
                steps = max_steps
            for _ in range(steps):
                if await self._hold_for_controllers(world_observations):
                    started = time.monotonic() - data.time / self._config.speed
                mujoco.mj_step(model, data)
            if not world_observations.busy:
                world_observations.offer(self.frame())
            await asyncio.sleep(0.001)

    async def _hold_for_controllers(self, world_observations: Publisher) -> bool:
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
        while world_observations.busy:
            await asyncio.sleep(0)
        world_observations.offer(self.frame(tuple(due)))
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
        mujoco.mj_resetData(self.model, self.data)
        self._hold_initial_pose()
        mujoco.mj_forward(self.model, self.data)

    def _initial_joints(self) -> dict[str, float]:
        """Every joint the world starts somewhere other than zero, and holds there until commanded."""
        initial = {}
        for robot in self._robots.values():
            held = {servo.joint: servo.initial for servo in robot.servos}
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
        match action:
            case JointCommand():
                self._command_joints(action)
            case CborKeyValueCommand(entity, key, value):
                # No key moves anything in MuJoCo yet; the world holds it for whoever reads it.
                self.world.set(entity, key, value)

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

    def frame(self, waiting_for: tuple[str, ...] = ()) -> WorldObservations:
        """What changed since the last frame: entities and joints that moved, refusals, and the robots the world waits for."""
        world, data, model = self.world, self.data, self.model
        if not self._announced:
            self._announce()
        for name, body in self._roots.items():
            pose = np.array([*data.xpos[body], *_xyzw(data.xquat[body])])
            sent = self._sent.get(name)
            if sent is None or not _close(sent, pose):
                world.position(name, pose)
                self._sent[name] = pose
            joints, joint_names = self._joints[name]
            if joints:
                rows = np.column_stack(
                    [
                        data.qpos[model.jnt_qposadr[joints]],
                        data.qvel[model.jnt_dofadr[joints]],
                        data.qfrc_actuator[model.jnt_dofadr[joints]],
                    ]
                )
                sent_rows = self._sent_joints.get(name)
                if sent_rows is None or not _close(sent_rows, rows):
                    for joint_name, (q, dq, effort) in zip(joint_names, rows, strict=True):
                        world.joint(f"{name}/{joint_name}", q, dq, effort)
                    self._sent_joints[name] = rows
        for target, reason in self._refusals:
            world.set(target if target in world.ids else "world", REFUSED, {"reason": reason})
        self._refusals.clear()
        for robot_id in waiting_for:
            world.set(robot_id, ACT, True)
        return world.end_frame(float(data.time))

    def _announce(self) -> None:
        """The first frame adds the world, each entity, and each entity's joints."""
        world = self.world
        world.add("world", backend="mujoco")
        for name in self._roots:
            robot = self._robots.get(name)
            # What to request on render_file_request to draw a robot on its own.
            world.add(name, **({"model": str(self._layers[name])} if robot is not None else {}))
            for joint_name in self._joints[name][1]:
                world.add(f"{name}/{joint_name}")
        self._announced = True

    def _joints_of(self, name: str, body: int) -> tuple[list[int], list[str]]:
        model = self.model
        joints = [
            j
            for j in range(model.njnt)
            if model.body_rootid[model.jnt_bodyid[j]] == body
            and int(model.jnt_type[j]) in _SINGLE_DOF
        ]
        prefix = f"{name}/" if name in self._robots else ""
        return joints, [model.joint(j).name.removeprefix(prefix) for j in joints]


def _close(a: np.ndarray, b: np.ndarray) -> bool:
    return a.shape == b.shape and bool(np.allclose(a, b, rtol=0.0, atol=_MOVED))


def _actuator_kind(model: mujoco.MjModel, actuator: int) -> str:
    if model.actuator_biastype[actuator] == mujoco.mjtBias.mjBIAS_NONE:
        return "effort"
    return "position" if model.actuator_biasprm[actuator][1] != 0 else "velocity"


# USD suffixes a scene may arrive as, which are not XML and must not be parsed as such.
_USD_SUFFIXES = frozenset({".usd", ".usda", ".usdc", ".usdz"})
# A USD layer names an asset between @ signs: `prepend references = @meshes/desk.usdc@`.
_USD_ASSET = re.compile(r"@([^@\n]+)@")


def _files_named_by_usd(layer: Path) -> dict[str, Path]:
    """The same, for a scene that is a USD layer rather than an MJCF.

    A USD is not XML, and handing one to an XML parser gets "syntax error: line 1, column 0" --
    which reads like a corrupt model and is really a file in the wrong language. A layer names
    its assets between @ signs, relative to itself, so they are read the same way the XML
    branch reads `file=` attributes.

    Only the text form can be scanned like this; a binary .usdc scene names its assets inside
    the crate, so this finds the layer alone and whatever asks for a mesh gets told no.
    """
    files = {str(layer): layer}
    try:
        text = layer.read_text(errors="ignore")
    except OSError:
        return files
    for name in _USD_ASSET.findall(text):
        path = (layer.parent / name).resolve()
        if path.is_file():
            files[str(path)] = path
    return files


def _files_named_by(mjcf: Path, assets: Path | None) -> dict[str, Path]:
    """The model and every file it names (meshes, textures, ...), by full path; a robot's assets hold its own."""
    if mjcf.suffix.lower() in _USD_SUFFIXES:
        return _files_named_by_usd(mjcf)
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
