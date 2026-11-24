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
from dataclasses import dataclass, field
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
from scipy.spatial.transform import Rotation

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.sim2.actions import Action, CborKeyValueCommand, JointCommand
from dimos.sim2.dimsim.dimsim_scene import INTERACTIVE, PICKABLE, STATE, TITLE
from dimos.sim2.dimsim.scene import PURPOSES, stage_bodies
from dimos.sim2.mujoco.names import resolve
from dimos.sim2.spec import Simulated, SimulatedRobot
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import REFUSED, WorldWriter
from dimos.utils.deno import ensure_deno

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel

# The Deno world dimsim steps; it ships with the repo (misc/DimSim/cli), not with the dimos package.
_WORLD = DIMOS_PROJECT_ROOT / "misc" / "DimSim" / "cli"
BASE_JOINTS = ("base_vx", "base_vy", "base_wz")
# CborKeyValueCommand(object, ACTION, <action id or label>) runs one of the ACTIONS its STATE has now.
ACTION = "action"
ACTIONS = "actions"
# CborKeyValueCommand(object, HELD_BY, <robot id>) picks a pickable object up; None puts it down where it is.
HELD_BY = "held_by"


def _world_dir() -> Path:
    if (_WORLD / "sim2" / "world.ts").is_file():
        return _WORLD
    raise FileNotFoundError(
        f"the dimsim world is not at {_WORLD}: it ships with a dimos checkout (misc/DimSim/cli), "
        f"not with the dimos package, so run dimsim from a checkout"
    )


@dataclass
class _Object:
    """A scene object DimSim lets you act on: its states and actions, and whether it can be picked up."""

    path: str
    title: str
    states: dict[str, str]
    actions: dict[str, dict[str, str]]
    state: str
    pickable: bool
    # Where it is now, from where its USD puts it (it moves only by being carried).
    moved: NDArray[np.float64] = field(default_factory=lambda: np.eye(4))
    held_by: str | None = None
    # Its pose in its carrier's frame, while it is held.
    grip: NDArray[np.float64] | None = None

    def available(self) -> dict[str, dict[str, str]]:
        return {id: action for id, action in self.actions.items() if action["from"] == self.state}

    def find(self, wanted: str) -> dict[str, str] | None:
        available = self.available()
        if wanted in available:
            return available[wanted]
        return next((a for a in available.values() if a["label"] == wanted), None)

    def cbor(self) -> dict[str, Any]:
        said: dict[str, Any] = {"title": self.title}
        if self.actions:
            said[STATE] = self.states[self.state]
            said[ACTIONS] = [action["label"] for action in self.available().values()]
        if self.pickable:
            said[HELD_BY] = self.held_by
        return said


class DimSimBackend:
    def __init__(
        self,
        usd: "UsdModel",
        simulated: Iterable[Simulated] = (),
        *,
        timestep: float,
        reach: float,
        speed: float = 1.0,
        max_burst_steps: int = 5,
    ) -> None:
        from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr

        self._pxr = pxr = load_pxr()
        self.usd = usd.usd
        self._timestep = timestep
        self._speed = speed
        self._max_burst_steps = max_burst_steps
        self._reach = reach
        self._scene_layer = usd_layer(usd)
        self._stage = pxr.Usd.Stage.Open(layer_from(pxr, usd.usd, "a scene", usd._asset_root()))
        self._robots = [d for d in simulated if isinstance(d, SimulatedRobot)]
        robot_ids = [robot.robot_id for robot in self._robots]
        bodies, meshes = stage_bodies(pxr, self._stage, exclude=robot_ids)
        self._objects = self._scene_objects()
        self._object_bodies = {
            name: [b["name"] for b in bodies if _under(b["name"], thing.path)]
            for name, thing in self._objects.items()
        }
        self._time = 0.0
        self._pending: list[Action] = []
        self._lock = Lock()
        self._frame_lock = Lock()
        self.world = WorldWriter()
        self._known: set[str] = set()
        self._moved: dict[str, Any] = {}
        self._poses: dict[str, NDArray[np.float64]] = {}
        self._base = {robot_id: [0.0, 0.0, 0.0] for robot_id in robot_ids}
        self._layers = {robot.robot_id: usd_layer(robot.model) for robot in self._robots}
        self._base_sent: set[str] = set()
        self._refusals: list[tuple[str, str]] = []
        world = _world_dir()
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
                    "robots": [self._robot(robot) for robot in self._robots],
                }
            )
        )
        self._announce()
        self._started = time.monotonic()

    @property
    def timestep(self) -> float:
        """Wall seconds between steps at this speed."""
        return self._timestep / self._speed

    def describe(self) -> dict[str, Any]:
        return {
            "backend": "dimsim",
            "scene": str(self._scene_layer),
            "objects": sorted(self._objects),
        }

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
        if steps < 1:
            return self._time, self.frame()
        return self._advance(steps)

    def restart_clock(self) -> None:
        """Count real time from now, so the wait before the first step costs no world time."""
        self._started = time.monotonic() - self._time / self._speed

    def advance(self, steps: int = 1) -> tuple[float, WorldObservations]:
        self._apply_pending()
        return self._advance(steps)

    def apply(self, actions: Sequence[Action]) -> None:
        with self._lock:
            self._pending.extend(actions)

    def close(self) -> None:
        if hasattr(self, "_world") and self._world.poll() is None:
            self._world.kill()
            self._world.wait()

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

    def _scene_objects(self) -> dict[str, _Object]:
        found = {}
        for prim in self._stage.Traverse():
            machine = dict(prim.GetCustomDataByKey(INTERACTIVE) or {})
            pickable = bool(prim.GetCustomDataByKey(PICKABLE))
            if not machine and not pickable:
                continue
            variants = prim.GetVariantSets()
            found[prim.GetName()] = _Object(
                path=prim.GetPath().pathString,
                title=str(prim.GetCustomDataByKey(TITLE) or prim.GetName()),
                states={str(k): str(v) for k, v in machine.get("states", {}).items()},
                actions={
                    str(id): {key: str(value) for key, value in action.items()}
                    for id, action in machine.get("actions", {}).items()
                },
                state=variants.GetVariantSet(STATE).GetVariantSelection()
                if variants.HasVariantSet(STATE)
                else "",
                pickable=pickable,
            )
        return found

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
            case JointCommand(entity, names, None, velocity, None, None, None) if (
                entity in self._base and velocity is not None and set(names) <= set(BASE_JOINTS)
            ):
                held = self._base[entity]
                for name, value in zip(names, velocity, strict=True):
                    held[BASE_JOINTS.index(name)] = value
                vx, vy, wz = held
                self._call(
                    {
                        "op": "velocity",
                        "entity": entity,
                        "linear": [vx, vy, 0.0],
                        "angular": [0.0, 0.0, wz],
                    }
                )
                self._base_sent.discard(entity)
            case CborKeyValueCommand(entity, key, wanted) if key == ACTION:
                self._act(self._object(entity), str(wanted))
            case CborKeyValueCommand(entity, key, carrier) if key == HELD_BY:
                self._hold(self._object(entity), None if carrier is None else str(carrier))
            case _:
                raise ValueError(
                    f"the dimsim world cannot do {action!r}: a robot has only its base joints"
                    f" {BASE_JOINTS}, by velocity, and an object only {ACTION!r} and {HELD_BY!r}"
                )

    def _object(self, entity: str) -> _Object:
        thing = self._objects.get(entity)
        if thing is None:
            raise KeyError(f"{entity!r} is no object to act on; those are {sorted(self._objects)}")
        return thing

    def _act(self, thing: _Object, wanted: str) -> None:
        """Run an object's action: its new state's variant replaces its geometry, as DimSim swaps its GLB."""
        action = thing.find(wanted)
        if action is None:
            raise ValueError(
                f"{thing.title} ({thing.states.get(thing.state, 'no state')}) has no action"
                f" {wanted!r} now; it has {[a['label'] for a in thing.available().values()]}"
            )
        self._remove(thing)
        prim = self._stage.GetPrimAtPath(thing.path)
        prim.GetVariantSets().GetVariantSet(STATE).SetVariantSelection(action["to"])
        thing.state = action["to"]
        if thing.held_by is None:
            self._spawn(thing)
        self._say(thing)

    def _hold(self, thing: _Object, carrier: str | None) -> None:
        """Pick it up (its geometry goes, as in DimSim's hand), or put it down where it is now."""
        if not thing.pickable:
            raise ValueError(f"{thing.title} cannot be picked up")
        if carrier is not None:
            if carrier not in self._base:
                raise KeyError(
                    f"no robot {carrier!r} to hold {thing.title}; the robots are {sorted(self._base)}"
                )
            if thing.held_by is not None:
                raise ValueError(f"{thing.title} is already held by {thing.held_by}")
            grip = np.linalg.inv(self._poses[carrier]) @ self._placed(thing)
            if np.linalg.norm(grip[:3, 3]) > self._reach:
                raise ValueError(f"{thing.title} is out of {carrier}'s reach ({self._reach} m)")
            self._remove(thing)
            thing.grip = grip
        elif thing.held_by is not None:
            thing.moved = self._carried(thing) @ np.linalg.inv(self._usd_pose(thing.path))
            thing.grip = None
            self._spawn(thing)
        thing.held_by = carrier
        self._say(thing)

    def _remove(self, thing: _Object) -> None:
        for name in self._object_bodies.pop(self._name(thing), []):
            self._call({"op": "remove", "entity": name})

    def _spawn(self, thing: _Object) -> None:
        bodies, meshes = stage_bodies(self._pxr, self._stage, under=thing.path)
        for body in bodies:
            placed = thing.moved @ _matrix(body["pos"], body["quat"])
            pose = _pose(placed)
            body["pos"], body["quat"] = pose[:3], pose[3:]
        self._call({"op": "spawn", "bodies": bodies, "meshes": meshes})
        self._object_bodies[self._name(thing)] = [body["name"] for body in bodies]

    def _say(self, thing: _Object) -> None:
        with self._frame_lock:
            for key, value in thing.cbor().items():
                self.world.set(self._name(thing), key, value)

    def _name(self, thing: _Object) -> str:
        return thing.path.rsplit("/", 1)[-1]

    def _placed(self, thing: _Object) -> NDArray[np.float64]:
        return thing.moved @ self._usd_pose(thing.path)

    def _carried(self, thing: _Object) -> NDArray[np.float64]:
        assert thing.held_by is not None and thing.grip is not None
        return self._poses[thing.held_by] @ thing.grip

    def _publish(self, state: dict[str, Any]) -> None:
        self._time = float(state.get("time", self._time))
        for name in state["entities"].keys() & self._base.keys():
            self._poses[name] = _matrix(
                state["entities"][name]["pos"], state["entities"][name]["quat"]
            )
        with self._lock:
            self._moved.update(state["entities"])

    def _announce(self) -> None:
        """Name every entity once; a listener that missed it asks the world for its ``snapshot()``."""
        world = self.world
        world.add("world", backend="dimsim")
        for robot_id in self._base:
            world.add(robot_id, model=str(self._layers[robot_id]))
            for name in BASE_JOINTS:
                world.add(f"{robot_id}/{name}")
        for name, thing in self._objects.items():
            world.add(name, **thing.cbor())
            world.position(name, _pose(self._placed(thing)))
        self._known = set(world.ids)

    def frame(self) -> WorldObservations:
        with self._frame_lock:
            return self._frame()

    def _frame(self) -> WorldObservations:
        world = self.world
        with self._lock:
            moved, self._moved = self._moved, {}
        for name, entity in moved.items():
            if name not in self._known:
                world.add(name)
                self._known.add(name)
            world.position(name, [*entity["pos"], *entity["quat"]])
        for name, thing in self._objects.items():
            if thing.held_by is not None and thing.held_by in moved:
                world.position(name, _pose(self._carried(thing)))
        for robot_id, held in self._base.items():
            if robot_id not in self._base_sent:
                for name, value in zip(BASE_JOINTS, held, strict=True):
                    world.joint(f"{robot_id}/{name}", 0.0, value, 0.0)
                self._base_sent.add(robot_id)
        for target, reason in self._refusals:
            world.set(target if target in world.ids else "world", REFUSED, {"reason": reason})
        self._refusals.clear()
        return world.end_frame(self._time)

    def _usd_pose(self, path: str) -> NDArray[np.float64]:
        """The prim's pose in the world as its USD places it, scale left out."""
        pxr = self._pxr
        placed = pxr.Gf.Transform(
            pxr.UsdGeom.Xformable(self._stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
                pxr.Usd.TimeCode.Default()
            )
        )
        quat = placed.GetRotation().GetQuat()
        return _matrix([*placed.GetTranslation()], [*quat.GetImaginary(), quat.GetReal()])

    def _robot(self, robot: SimulatedRobot) -> dict[str, Any]:
        """The robot's box, standing on the frame it is mounted on, turned as that frame is."""
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
        mount = root
        while mount.GetName() != robot.robot_id:
            mount = mount.GetParent()
        at = pxr.Usd.TimeCode.Default()
        placed = pxr.UsdGeom.Xformable(root).ComputeLocalToWorldTransform(at)
        inverse = placed.GetInverse()
        cache = pxr.UsdGeom.BBoxCache(at, PURPOSES)
        corners = []
        for prim in prims:
            if not prim.IsA(pxr.UsdGeom.Gprim):
                continue
            aligned = cache.ComputeWorldBound(prim).Transform(inverse).ComputeAlignedRange()
            if not aligned.IsEmpty():
                corners += [np.array(aligned.GetMin()), np.array(aligned.GetMax())]
        if not corners:
            raise ValueError(f"{robot.robot_id}: it has no geometry to be the size of")
        low, high = np.min(corners, axis=0), np.max(corners, axis=0)
        x, y, _ = placed.ExtractTranslation()
        floor = (
            pxr.UsdGeom.Xformable(mount).ComputeLocalToWorldTransform(at).ExtractTranslation()[2]
        )
        forward = placed.TransformDir(pxr.Gf.Vec3d(1, 0, 0))
        return {
            "id": robot.robot_id,
            "pos": [x, y, floor - float(low[2])],
            "yaw": math.atan2(forward[1], forward[0]),
            "halfExtents": ((high - low) / 2).tolist(),
            "center": ((high + low) / 2).tolist(),
            "mass": _robot_mass(pxr, prims),
        }


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


def _matrix(pos: Sequence[float], quat: Sequence[float]) -> NDArray[np.float64]:
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = Rotation.from_quat(quat).as_matrix(), pos
    return matrix


def _pose(matrix: NDArray[np.float64]) -> list[float]:
    return [*matrix[:3, 3].tolist(), *Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()]


def _under(name: str, path: str) -> bool:
    return name == path or name.startswith(f"{path}/")
