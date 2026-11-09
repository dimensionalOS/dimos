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


from collections.abc import Iterable, Mapping, Sequence
from pathlib import Path
import re
from threading import Lock
import time
from typing import TYPE_CHECKING, Any

from dimos_lcm.world_msgs import WorldObservations
import mujoco
import numpy as np

from dimos.sim2.actions import Action, CborKeyValueCommand, JointCommand
from dimos.sim2.mujoco.names import resolved, short_names
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.spec import Simulated, SimulatedRobot
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import ACT, REFUSED, WorldWriter

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel

_SINGLE_DOF = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
_MOVED = 1e-6


def _xyzw(wxyz: np.ndarray) -> list[float]:
    w, x, y, z = wxyz
    return [float(x), float(y), float(z), float(w)]


class MujocoBackend:
    def __init__(
        self,
        usd: "UsdModel",
        simulated: Iterable[Simulated] = (),
        speed: float = 1.0,
        max_burst_seconds: float = 0.01,
    ) -> None:
        simulated = list(simulated)
        self._speed = speed
        self._max_burst_seconds = max_burst_seconds
        self._scene_layer = usd_layer(usd)
        self._robots = {d.robot_id: d for d in simulated if isinstance(d, SimulatedRobot)}
        self.model = compile_world(self._scene_layer, simulated).compile()
        self._controllers = [r for r in self._robots.values() if r.control_period is not None]
        self._next_ticks = [0.0] * len(self._controllers)
        self._acted = {robot.robot_id: 0 for robot in self._controllers}
        self.data = mujoco.MjData(self.model)
        self._pending: list[Action] = []
        self._lock = Lock()
        self._roots = self._entity_roots()
        self._joints = {name: self._joints_of(name, body) for name, body in self._roots.items()}
        self.world = WorldWriter()
        self._announced = False
        self._sent: dict[str, np.ndarray] = {}
        self._sent_joints: dict[str, np.ndarray] = {}
        self._refusals: list[tuple[str, str]] = []
        self._started = time.monotonic()
        self._rebase = True
        self._waiting: tuple[str, ...] = ()
        self._files = _files_named_by_usd(self._scene_layer)
        self._layers = {id: usd_layer(r.model) for id, r in self._robots.items()}
        for layer in self._layers.values():
            self._files.update(_files_named_by_usd(layer))
        self._reset()

    def describe(self) -> dict[str, Any]:
        return {"backend": "mujoco", "scene": str(self._scene_layer)}

    def assets(self) -> Mapping[str, Path | bytes]:
        return {
            "world.xml": self._scene_layer,
            str(self._scene_layer): self._scene_layer,
            **self._files,
        }

    def step(self, actions: Sequence[Action] = ()) -> tuple[float, WorldObservations]:
        model, data = self.model, self.data
        if actions:
            self.apply(actions)
        self._apply_pending()
        if self._rebase:
            self._started = time.monotonic() - data.time / self._speed
            self._rebase = False
        max_steps = max(1, int(self._max_burst_seconds / model.opt.timestep))
        steps = int(
            ((time.monotonic() - self._started) * self._speed - data.time) / model.opt.timestep
        )
        if steps > max_steps:
            self._started += (steps - max_steps) * model.opt.timestep / self._speed
            steps = max_steps
        for _ in range(steps):
            due = self._due()
            if due:
                return float(data.time), self.frame(due)
            mujoco.mj_step(model, data)
        return float(data.time), self.frame()

    @property
    def waiting_for(self) -> tuple[str, ...]:
        return self._waiting

    def ready(self) -> bool:
        with self._lock:
            return all(self._acted.get(robot_id) for robot_id in self._waiting)

    def apply(self, actions: Sequence[Action]) -> None:
        with self._lock:
            self._pending.extend(actions)
            for action in actions:
                if isinstance(action, JointCommand) and action.entity in self._acted:
                    self._acted[action.entity] += 1

    def _due(self) -> tuple[str, ...]:
        now = self.data.time
        due = [
            robot.robot_id
            for robot, tick in zip(self._controllers, self._next_ticks, strict=True)
            if now >= tick - self.model.opt.timestep / 2
        ]
        if not due:
            self._waiting = ()
            return ()
        for i, robot in enumerate(self._controllers):
            if robot.robot_id in due:
                self._next_ticks[i] += robot.control_period  # type: ignore[operator]
        with self._lock:
            self._waiting = tuple(robot_id for robot_id in due if self._acted.get(robot_id))
            for robot_id in due:
                self._acted[robot_id] = 0
        self._rebase = bool(self._waiting)
        return tuple(due)

    def _reset(self) -> None:
        self._next_ticks = [0.0] * len(self._controllers)
        mujoco.mj_resetData(self.model, self.data)
        self._hold_initial_pose()
        mujoco.mj_forward(self.model, self.data)

    def _initial_joints(self) -> dict[str, float]:
        from dimos.robot.assets.usd import usd_drives

        initial = {}
        for robot in self._robots.values():
            for joint, (_kp, _kd, target) in usd_drives(robot.model.usd).items():
                initial[f"{robot.robot_id}/{joint}"] = target
        return initial

    def _hold_initial_pose(self) -> None:
        names = [self.model.joint(i).name for i in range(self.model.njnt)]
        for wanted, value in self._initial_joints().items():
            found = resolved(names, wanted)
            if found is None:
                continue
            joint = self.model.joint(found)
            self.data.qpos[joint.qposadr[0]] = value
            self.data.ctrl[self.model.actuator_trnid[:, 0] == joint.id] = value

    def _apply_pending(self) -> None:
        with self._lock:
            pending, self._pending = self._pending, []
        for action in pending:
            try:
                self._apply(action)
            except (KeyError, ValueError) as error:
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
                model.actuator_gainprm[actuator, 0] = p if p > 0 else 1.0
                model.actuator_biasprm[actuator, 1:3] = (-p, -d)
                feedforward = d * velocity[index] + effort[index]
                self.data.ctrl[actuator] = (
                    position[index] + feedforward / p if p > 0 else feedforward
                )

    def _joint_name(self, entity: str, name: str) -> str:
        return f"{entity}/{name}" if entity in self._robots else name

    def _entity_roots(self) -> dict[str, int]:
        model = self.model
        roots = {
            robot_id: model.body(f"{robot_id}/{r.root_body}").id
            for robot_id, r in self._robots.items()
        }
        robot_roots = set(roots.values())
        jointed = set(model.body_rootid[model.jnt_bodyid])
        objects = [
            body
            for body in range(1, model.nbody)
            if model.body_parentid[body] == 0 and body not in robot_roots and body in jointed
        ]
        names = [model.body(body).name or f"body{body}" for body in objects]
        short = short_names(names)
        for body, name in zip(objects, names, strict=True):
            roots[short[name]] = body
        return roots

    def frame(self, waiting_for: tuple[str, ...] = ()) -> WorldObservations:
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
        world = self.world
        world.add("world", backend="mujoco")
        for name in self._roots:
            robot = self._robots.get(name)
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


_USD_ASSET = re.compile(r"@([^@\n]+)@")


def _files_named_by_usd(layer: Path) -> dict[str, Path]:
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
