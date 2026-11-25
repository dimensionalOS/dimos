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


from collections.abc import Iterable, Sequence
from threading import Lock
import time
from typing import TYPE_CHECKING, Any

from dimos_lcm.world_msgs import WorldObservations
import mujoco
import numpy as np

from dimos.sim2.actions import Action, CborKeyValueCommand, JointCommand
from dimos.sim2.mujoco.drives import usd_with_mjc_servos
from dimos.sim2.mujoco.names import resolved, short_names
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.spec import ODOM_FRAME, Simulated, SimulatedRobot
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import REFUSED, WorldWriter
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel

logger = setup_logger()

_SINGLE_DOF = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
_MOVED = 1e-6


# Where to write the world before it is stepped, when something asks. Off unless set, because
# a dump of this scene is a 150 MB file.
def _xyzw(wxyz: np.ndarray) -> list[float]:
    w, x, y, z = wxyz
    return [float(x), float(y), float(z), float(w)]


# Seconds of world time between repeats of the entity table.
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
        self.usd = usd.usd
        # MuJoCo ignores UsdPhysics drives, so a robot authored with them is respelled as mjc servos.
        self._scene_layer = usd_layer(usd, transform=usd_with_mjc_servos)
        self._robots = {d.robot_id: d for d in simulated if isinstance(d, SimulatedRobot)}
        spec = compile_world(self._scene_layer, simulated)
        self.model = spec.compile()
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
        self._servo_lookup: dict[tuple[str, tuple[str, ...]], tuple[np.ndarray, np.ndarray]] = {}
        self._started = time.monotonic()
        self._rebase = True
        self._layers = {id: usd_layer(r.model) for id, r in self._robots.items()}
        self._reset()

    @property
    def timestep(self) -> float:
        """Wall seconds between steps at this speed."""
        return float(self.model.opt.timestep) / self._speed

    def describe(self) -> dict[str, Any]:
        return {"backend": "mujoco", "scene": str(self._scene_layer)}

    def step(
        self, actions: Sequence[Action] = (), hold: bool = False
    ) -> tuple[float, WorldObservations]:
        """Catch the world up to real time; held, only its clock runs and nothing in it moves."""
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
        if hold:
            data.time += steps * model.opt.timestep
        else:
            for _ in range(steps):
                mujoco.mj_step(model, data)
        return float(data.time), self.frame()

    def apply(self, actions: Sequence[Action]) -> None:
        with self._lock:
            self._pending.extend(actions)

    def restart_clock(self) -> None:
        """Start counting real time from now, so a wait before the first step costs no sim time."""
        self._rebase = True

    def _reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self._hold_initial_pose()
        mujoco.mj_forward(self.model, self.data)

    def _initial_joints(self) -> dict[str, float]:
        from dimos.sim2.mujoco.drives import usd_drives

        initial = {}
        for robot in self._robots.values():
            drives = usd_drives(robot.model.usd, robot.model._asset_root())
            for joint, (_kp, _kd, target) in drives.items():
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
        model = self.model
        actuators, of = self._servos(command.entity, command.names)
        zeros = np.zeros(len(command.names))

        def given(values: tuple[float, ...] | None) -> np.ndarray:
            every = zeros if values is None else np.asarray(values, dtype=np.float64)
            return np.asarray(every[of])

        kp, kd = given(command.kp), given(command.kd)
        feedforward = kd * given(command.velocity) + given(command.effort)
        gain = np.where(kp > 0, kp, 1.0)
        model.actuator_gainprm[actuators, 0] = gain
        model.actuator_biasprm[actuators, 1] = -kp
        model.actuator_biasprm[actuators, 2] = -kd
        self.data.ctrl[actuators] = np.where(
            kp > 0, given(command.position) + feedforward / gain, feedforward
        )

    def _servos(self, entity: str, names: tuple[str, ...]) -> tuple[np.ndarray, np.ndarray]:
        """The servo actuators driving these joints, and which of them each drives: found once per entity and joint list, as a policy sends the same one every tick."""
        if (entity, names) not in self._servo_lookup:
            model, actuators, of = self.model, [], []
            for index, name in enumerate(names):
                joint = model.joint(self._joint_name(entity, name))
                for actuator in np.flatnonzero(model.actuator_trnid[:, 0] == joint.id):
                    if model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_AFFINE:
                        raise ValueError(
                            f"{name}'s actuator is not a servo, so it cannot take gains"
                        )
                    actuators.append(int(actuator))
                    of.append(index)
            self._servo_lookup[(entity, names)] = (np.array(actuators, int), np.array(of, int))
        return self._servo_lookup[(entity, names)]

    def _joint_name(self, entity: str, name: str) -> str:
        return f"{entity}/{name}" if entity in self._robots else name

    def _entity_roots(self) -> dict[str, int]:
        model = self.model
        roots = {
            robot_id: model.body(f"{robot_id}/{r.root_body}").id
            for robot_id, r in self._robots.items()
        }
        robot_roots = set(roots.values())
        # The body a robot hangs off is not a thing in its own right. In a composed scene the
        # robot is grafted under a frame of the scene's -- `/office_555/main` -- which is where
        # `_free_if_unjointed` then puts the freejoint, so it is parented to the world and it is
        # jointed, and it would come through here as an object. Worse, it shortens to the same
        # name as the robot it holds, so it overwrote the robot's own entity: every consumer was
        # told the robot's pose was its mount's, 0.793 m below its pelvis, and drew it that far
        # into the floor. A body that is the root of a robot's tree is that robot's, not a
        # separate object.
        holders = {int(model.body_rootid[body]) for body in robot_roots}
        jointed = set(model.body_rootid[model.jnt_bodyid])
        objects = [
            body
            for body in range(1, model.nbody)
            if model.body_parentid[body] == 0
            and body not in robot_roots
            and body not in holders
            and body in jointed
        ]
        names = [model.body(body).name or f"body{body}" for body in objects]
        short = short_names(names)
        for body, name in zip(objects, names, strict=True):
            # And never let an object take a robot's name, whatever it is called.
            if short[name] not in roots:
                roots[short[name]] = body
        return roots

    def frame(self) -> WorldObservations:
        world, data, model = self.world, self.data, self.model
        if not self._announced:
            self._announce()
            self._announced = True
        for name, body in self._roots.items():
            pose = np.array([*data.xpos[body], *_xyzw(data.xquat[body])])
            sent = self._sent.get(name)
            if sent is None or not _close(sent, pose):
                world.position(name, pose.tolist())
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
        return world.end_frame(float(data.time))

    def _announce(self) -> None:
        """Name every entity once; a listener that missed it asks the world for ``snapshot()``."""
        world = self.world
        world.add("world", backend="mujoco")
        for name in self._roots:
            robot = self._robots.get(name)
            announced = {} if robot is None else {"model": str(self._layers[name])}
            if robot is not None and robot.odom_frame is not None:
                announced[ODOM_FRAME] = robot.odom_frame
            world.add(name, **announced)
            for joint_name in self._joints[name][1]:
                world.add(f"{name}/{joint_name}")

    def _joints_of(self, name: str, body: int) -> tuple[list[int], list[str]]:
        """The single-dof joints in this body's subtree.

        Its subtree and not its kinematic tree: in a composed scene the robot hangs off a frame
        of the scene's, and that frame is the root of the tree, so a robot whose entity is its
        own root body would own no joints at all -- nothing would report a joint state and no
        policy could ever arm.
        """
        model = self.model
        joints = [
            j
            for j in range(model.njnt)
            if self._descends(int(model.jnt_bodyid[j]), body)
            and int(model.jnt_type[j]) in _SINGLE_DOF
        ]
        prefix = f"{name}/" if name in self._robots else ""
        return joints, [model.joint(j).name.removeprefix(prefix) for j in joints]

    def _descends(self, body: int, ancestor: int) -> bool:
        parents = self.model.body_parentid
        while body > 0:
            if body == ancestor:
                return True
            body = int(parents[body])
        return body == ancestor


def _close(a: np.ndarray, b: np.ndarray) -> bool:
    # Not np.allclose: its overhead, per entity every step, was a sixth of the physics loop.
    return a.shape == b.shape and bool((np.abs(a - b) <= _MOVED).all())


def _actuator_kind(model: mujoco.MjModel, actuator: int) -> str:
    if model.actuator_biastype[actuator] == mujoco.mjtBias.mjBIAS_NONE:
        return "effort"
    return "position" if model.actuator_biasprm[actuator][1] != 0 else "velocity"
