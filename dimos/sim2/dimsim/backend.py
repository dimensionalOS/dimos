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

"""The DimSim backend's engine: DimSim's Rapier world (misc/DimSim/cli/sim2/world.ts), run by Deno and stepped over stdin."""

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path
import subprocess
from threading import Lock
import time
from typing import TYPE_CHECKING, Any

from dimos_lcm.world_msgs import WorldObservations
import numpy as np
from numpy.typing import NDArray

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.sim2.actions import Action, CborKeyValueCommand, JointCommand
from dimos.sim2.dimsim.scene import PURPOSES, is_guide, stage_bodies
from dimos.sim2.mujoco.names import resolve
from dimos.sim2.spec import ODOM_FRAME, ROBOT, SimulatedRobot
from dimos.sim2.world_observations import REFUSED, WorldWriter
from dimos.utils.deno import ensure_deno

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel

# The Deno world dimsim steps; it ships with the repo (misc/DimSim/cli), not with the dimos package.
WORLD_DIR = DIMOS_PROJECT_ROOT / "misc" / "DimSim" / "cli"
STILL = (0.0,) * 6


def world_dir() -> Path:
    if (WORLD_DIR / "sim2" / "world.ts").is_file():
        return WORLD_DIR
    raise FileNotFoundError(
        f"the dimsim world is not at {WORLD_DIR}: it ships with a dimos checkout (misc/DimSim/cli), "
        f"not with the dimos package, so run dimsim from a checkout"
    )


@dataclass(frozen=True)
class PlanarServos:
    """A robot's three servos as dimsim moves it: a slide along its mount's x, one along its y, and a turn about z."""

    names: tuple[str, str, str]
    # Its mount's frame (where the servos are zero): its world rotation about z, and the moving body's world position there.
    mount_yaw: float
    start: tuple[float, float, float]
    start_yaw: float
    # Whether the robot's root body is the one the servos move (else it is the still anchor they move off).
    root_moves: bool
    root_pose: tuple[float, ...]


class DimSimBackend:
    def __init__(
        self,
        usd: "UsdModel",
        simulated: Iterable[SimulatedRobot] = (),
        *,
        timestep: float,
        speed: float = 1.0,
        max_burst_steps: int = 5,
    ) -> None:
        from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr

        self._pxr = pxr = load_pxr()
        self.usd = usd.usd
        self._timestep = timestep
        self._speed = speed
        self._max_burst_steps = max_burst_steps
        self._stage = pxr.Usd.Stage.Open(layer_from(pxr, usd.usd, "a scene", usd.asset_root))
        self._robots = list(simulated)
        robot_ids = [robot.robot_id for robot in self._robots]
        bodies, meshes = stage_bodies(pxr, self._stage, exclude=robot_ids)
        self._time = 0.0
        self._pending: list[Action] = []
        self._lock = Lock()
        self._frame_lock = Lock()
        self.world = WorldWriter()
        self._known: set[str] = set()
        self._moved: dict[str, Any] = {}
        robots = {robot.robot_id: self._robot(robot) for robot in self._robots}
        self._servos = {robot_id: servos for robot_id, (servos, _) in robots.items()}
        # Each robot's servo velocities as last commanded, held until the next command.
        self._commanded = {robot_id: [0.0, 0.0, 0.0] for robot_id in robot_ids}
        # Each entity's pose and velocity as last sent, and each robot's servo rows.
        self._sent: dict[str, NDArray[np.float64]] = {}
        self._sent_joints: dict[str, NDArray[np.float64]] = {}
        self._refusals: list[tuple[str, str]] = []
        world = world_dir()
        self._world = subprocess.Popen(
            [
                ensure_deno(),
                "run",
                "--quiet",
                "--allow-read",
                "--allow-env",
                "--config",
                str(world / "deno.json"),
                str(world / "sim2" / "world.ts"),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        self._publish(
            self._call(
                {
                    "op": "init",
                    "timestep": timestep,
                    "bodies": bodies,
                    "meshes": meshes,
                    "robots": [box for _, box in robots.values()],
                }
            )
        )
        self._announce()
        self._started = time.monotonic()

    @property
    def timestep(self) -> float:
        """Wall seconds between steps at this speed."""
        return self._timestep / self._speed

    def snapshot(self) -> dict[str, Any]:
        with self._frame_lock:
            return self.world.snapshot()

    def step(self) -> tuple[float, WorldObservations]:
        """Catch the world up to real time (times ``speed``), at most ``max_burst_steps`` at once."""
        self._apply_pending()
        steps = int(
            ((time.monotonic() - self._started) * self._speed - self._time) / self._timestep
        )
        if steps > self._max_burst_steps:
            self._started += (steps - self._max_burst_steps) * self._timestep / self._speed
            steps = self._max_burst_steps
            # A world that fell behind stamps its frames on wall time again.
            self.world.rebase(self._time + steps * self._timestep)
        if steps < 1:
            return self._time, self.frame()
        return self._advance(steps)

    def restart_clock(self) -> None:
        """Count real time from now, so the wait before the first step costs no world time."""
        self._started = time.monotonic() - self._time / self._speed
        self.world.rebase(self._time)

    def advance(self, steps: int = 1) -> tuple[float, WorldObservations]:
        self._apply_pending()
        return self._advance(steps)

    def apply(self, actions: Sequence[Action]) -> None:
        with self._lock:
            self._pending.extend(actions)

    def close(self) -> None:
        if not hasattr(self, "_world"):
            return
        if self._world.poll() is None:
            self._world.kill()
            self._world.wait()
        for pipe in (self._world.stdin, self._world.stdout):
            if pipe is not None:
                pipe.close()

    def __del__(self) -> None:
        self.close()

    def _advance(self, steps: int) -> tuple[float, WorldObservations]:
        self._publish(self._call({"op": "step", "steps": steps}))
        return self._time, self.frame()

    def _call(self, request: dict[str, Any]) -> dict[str, Any]:
        assert self._world.stdin is not None and self._world.stdout is not None
        self._world.stdin.write(json.dumps(request) + "\n")
        self._world.stdin.flush()
        line = self._world.stdout.readline()
        if not line:
            raise RuntimeError(f"the dimsim world exited ({self._world.poll()})")
        reply: dict[str, Any] = json.loads(line)
        if "error" in reply:
            raise ValueError(reply["error"])
        return reply

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

    def _apply(self, action: Action) -> None:
        match action:
            case JointCommand():
                self._command_servos(action)
            case CborKeyValueCommand(entity, key, value):
                self.world.set(entity, key, value)

    def _command_servos(self, command: JointCommand) -> None:
        servos = self._servos.get(command.entity)
        if servos is None:
            raise KeyError(f"{command.entity!r} is no robot of this world")
        unknown = set(command.names) - set(servos.names)
        if unknown or command.velocity is None or command.position is not None:
            raise ValueError(
                f"the dimsim world moves a robot only by the velocity of its servos "
                f"{servos.names}, and this asks for {sorted(unknown) or 'a position'}"
            )
        if command.kp is not None and any(command.kp):
            raise ValueError("the dimsim world tracks a servo's velocity exactly, so it has no kp")
        held = self._commanded[command.entity]
        for name, value in zip(command.names, command.velocity, strict=True):
            held[servos.names.index(name)] = float(value)
        # The x and y servos slide along the mount's axes; the world takes world axes.
        vx, vy, wz = held
        cos, sin = math.cos(servos.mount_yaw), math.sin(servos.mount_yaw)
        self._call(
            {
                "op": "velocity",
                "entity": command.entity,
                "linear": [vx * cos - vy * sin, vx * sin + vy * cos, 0.0],
                "angular": [0.0, 0.0, wz],
            }
        )

    def _publish(self, state: dict[str, Any]) -> None:
        self._time = float(state.get("time", self._time))
        with self._lock:
            self._moved.update(state["entities"])

    def _announce(self) -> None:
        """Name every entity once; a listener that missed it asks the world for its ``snapshot()``."""
        world = self.world
        world.add("world", backend="dimsim")
        for robot in self._robots:
            announced = {} if robot.odom_frame is None else {ODOM_FRAME: robot.odom_frame}
            world.add(robot.robot_id, **{ROBOT: True}, **announced)
            for name in self._servos[robot.robot_id].names:
                world.add(f"{robot.robot_id}/{name}")
        self._known = set(world.ids)

    def frame(self) -> WorldObservations:
        with self._frame_lock:
            return self._frame()

    def _frame(self) -> WorldObservations:
        world = self.world
        with self._lock:
            moved, self._moved = self._moved, {}
        for name, entity in moved.items():
            servos = self._servos.get(name)
            if servos is not None:
                self._send_robot(name, servos, entity)
                continue
            if name not in self._known:
                world.add(name)
                self._known.add(name)
            self._send(
                name, [*entity["pos"], *entity["quat"]], entity["velocity"], entity["acceleration"]
            )
        for target, reason in self._refusals:
            world.set(target if target in world.ids else "world", REFUSED, {"reason": reason})
        self._refusals.clear()
        return world.end_frame(self._time)

    def _send_robot(self, robot_id: str, servos: PlanarServos, entity: dict[str, Any]) -> None:
        """A robot as the world has it: its root body, and its servos read off where its moving body has got to."""
        if servos.root_moves:
            pose = [*entity["pos"], *entity["quat"]]
            self._send(robot_id, pose, entity["velocity"], entity["acceleration"])
        else:
            self._send(robot_id, servos.root_pose, STILL, STILL)
        cos, sin = math.cos(servos.mount_yaw), math.sin(servos.mount_yaw)
        dx, dy = (entity["pos"][i] - servos.start[i] for i in range(2))
        vx, vy, _vz, _wx, _wy, wz = entity["velocity"]
        rows = np.array(
            [
                [dx * cos + dy * sin, vx * cos + vy * sin],
                [-dx * sin + dy * cos, -vx * sin + vy * cos],
                [entity["yaw"] - servos.start_yaw, wz],
            ]
        )
        sent = self._sent_joints.get(robot_id)
        if sent is not None and np.array_equal(sent, rows):
            return
        for name, (position, velocity) in zip(servos.names, rows, strict=True):
            self.world.joint(f"{robot_id}/{name}", float(position), float(velocity), 0.0)
        self._sent_joints[robot_id] = rows

    def _send(
        self,
        name: str,
        pose: Sequence[float],
        velocity: Sequence[float],
        acceleration: Sequence[float],
    ) -> None:
        # A body is sent when it moved or its velocity changed, so the last one sent of a body that stopped is still.
        state = np.array([*pose, *velocity], dtype=float)
        sent = self._sent.get(name)
        if sent is None or not np.array_equal(sent, state):
            self.world.body_state(name, pose, velocity, acceleration)
            self._sent[name] = state

    def _robot(self, robot: SimulatedRobot) -> tuple[PlanarServos, dict[str, Any]]:
        """The robot's servos, and the box the world moves for it: its colliders' bound, placed and turned (about z) as its moving body is."""
        pxr, stage = self._pxr, self._stage
        owned = f"/{robot.robot_id}/"
        prims = [p for p in stage.Traverse() if owned in f"{p.GetPath().pathString}/"]
        bodies = {p.GetPath().pathString: p for p in prims if p.HasAPI(pxr.UsdPhysics.RigidBodyAPI)}
        if not bodies:
            raise KeyError(
                f"{robot.robot_id}: no rigid body of it in the world; a robot is grafted into the "
                f"world by mounting it, so mount it onto a frame of the scene"
            )
        # Its frames are under its namespace on tf; its prims are named as its own layer names them.
        root = bodies[resolve(bodies, robot.root_body.rpartition("/")[2])]
        x, y, yaw, moving = _planar_joints(pxr, robot.robot_id, prims)
        at = pxr.Usd.TimeCode.Default()
        placed = pxr.UsdGeom.Xformable(moving).ComputeLocalToWorldTransform(at)
        mount = _joint_frame(pxr, x)
        inverse = placed.GetInverse()
        cache = pxr.UsdGeom.BBoxCache(at, PURPOSES)
        colliders = [p for p in prims if p.HasAPI(pxr.UsdPhysics.CollisionAPI)]
        shaped = colliders or [p for p in prims if not is_guide(pxr, p)]
        corners = []
        for prim in shaped:
            if not prim.IsA(pxr.UsdGeom.Gprim):
                continue
            aligned = cache.ComputeWorldBound(prim).Transform(inverse).ComputeAlignedRange()
            if not aligned.IsEmpty():
                corners += [np.array(aligned.GetMin()), np.array(aligned.GetMax())]
        if not corners:
            raise ValueError(f"{robot.robot_id}: it has no geometry to be the size of")
        low, high = np.min(corners, axis=0), np.max(corners, axis=0)
        start = tuple(float(v) for v in placed.ExtractTranslation())
        start_yaw = _yaw(pxr, placed)
        servos = PlanarServos(
            names=(x.GetName(), y.GetName(), yaw.GetName()),
            mount_yaw=_yaw(pxr, mount),
            start=start,  # type: ignore[arg-type]
            start_yaw=start_yaw,
            root_moves=root == moving,
            root_pose=_pose(pxr, root),
        )
        box = {
            "id": robot.robot_id,
            "pos": list(start),
            "yaw": start_yaw,
            "halfExtents": ((high - low) / 2).tolist(),
            "center": ((high + low) / 2).tolist(),
            "mass": _robot_mass(pxr, prims),
        }
        return servos, box


def _planar_joints(pxr: Any, robot_id: str, prims: Sequence[Any]) -> tuple[Any, Any, Any, Any]:
    """A robot's slide along x, its slide along y and its turn about z, and the body the turn moves."""
    found: dict[str, Any] = {}
    for prim in prims:
        if prim.IsA(pxr.UsdPhysics.PrismaticJoint):
            axis = str(pxr.UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get())
            found.setdefault(axis.lower(), prim)
        elif prim.IsA(pxr.UsdPhysics.RevoluteJoint):
            if str(pxr.UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()) == "Z":
                found.setdefault("yaw", prim)
    missing = [kind for kind in ("x", "y", "yaw") if kind not in found]
    if missing:
        raise ValueError(
            f"{robot_id}: the dimsim world moves a robot on three servos, a prismatic joint along "
            f"X, one along Y and a revolute about Z, and it lacks {missing}"
        )
    targets = pxr.UsdPhysics.Joint(found["yaw"]).GetBody1Rel().GetTargets()
    moving = found["yaw"].GetStage().GetPrimAtPath(targets[0]) if targets else None
    if moving is None or not moving.IsValid():
        raise ValueError(f"{robot_id}: its yaw joint {found['yaw'].GetPath()} turns no body")
    return found["x"], found["y"], found["yaw"], moving


def _joint_frame(pxr: Any, joint: Any) -> Any:
    """Where a joint's body0 holds it, in the world."""
    joined = pxr.UsdPhysics.Joint(joint)
    targets = joined.GetBody0Rel().GetTargets()
    at = pxr.Usd.TimeCode.Default()
    body = joint.GetStage().GetPrimAtPath(targets[0]) if targets else None
    world = (
        pxr.UsdGeom.Xformable(body).ComputeLocalToWorldTransform(at)
        if body is not None and body.IsValid()
        else pxr.Gf.Matrix4d(1.0)
    )
    position = joined.GetLocalPos0Attr().Get() or pxr.Gf.Vec3f(0, 0, 0)
    rotation = joined.GetLocalRot0Attr().Get() or pxr.Gf.Quatf(1, 0, 0, 0)
    local = pxr.Gf.Matrix4d().SetRotate(
        pxr.Gf.Rotation(pxr.Gf.Quatd(rotation))
    ) * pxr.Gf.Matrix4d().SetTranslate(pxr.Gf.Vec3d(position))
    return local * world


def _yaw(pxr: Any, placed: Any) -> float:
    forward = placed.TransformDir(pxr.Gf.Vec3d(1, 0, 0))
    return math.atan2(forward[1], forward[0])


def _pose(pxr: Any, prim: Any) -> tuple[float, ...]:
    placed = pxr.Gf.Transform(
        pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
    )
    quat = placed.GetRotation().GetQuat()
    imaginary = quat.GetImaginary()
    return (
        *(float(v) for v in placed.GetTranslation()),
        *(float(v) for v in imaginary),
        float(quat.GetReal()),
    )


def _robot_mass(pxr: Any, prims: Iterable[Any]) -> float:
    total = 0.0
    for prim in prims:
        attribute = pxr.UsdPhysics.MassAPI(prim).GetMassAttr()
        if attribute and attribute.HasAuthoredValue():
            total += float(attribute.Get() or 0.0)
    if total <= 0:
        raise ValueError(
            "a dimsim robot pushes what it walks into as hard as its mass, and this one has none in its USD"
        )
    return total
