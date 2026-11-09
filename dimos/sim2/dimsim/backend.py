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
import json
from pathlib import Path
import subprocess
from threading import Lock
import time
from typing import TYPE_CHECKING, Any

from dimos_lcm.world_msgs import WorldObservations
import numpy as np

from dimos.sim2.actions import Action, JointCommand
from dimos.sim2.dimsim.scene import PURPOSES, dimsim_bodies
from dimos.sim2.mujoco.names import resolve
from dimos.sim2.spec import Simulated, SimulatedRobot
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import REFUSED, WorldWriter
from dimos.utils.deno import ensure_deno

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel

_WORLD = Path(__file__).parents[3] / "misc" / "DimSim" / "cli"
TIMESTEP = 0.02
BASE_JOINTS = ("base_vx", "base_vy", "base_wz")
_DEFAULT_ROBOT_MASS = 10.0


class DimSimBackend:
    def __init__(
        self,
        usd: "UsdModel",
        simulated: Iterable[Simulated] = (),
        speed: float = 1.0,
        max_burst_steps: int = 5,
    ) -> None:
        self._speed = speed
        self._max_burst_steps = max_burst_steps
        self._scene_layer = usd_layer(usd)
        self._robots = [d for d in simulated if isinstance(d, SimulatedRobot)]
        for robot in self._robots:
            if robot.control_period is not None:
                raise ValueError(
                    f"{robot.robot_id}: a dimsim robot is a kinematic base, with no controller for"
                    f" the world to step in lockstep with, so it has no control_period"
                )
        self._bodies, self._meshes = dimsim_bodies(
            usd, exclude=[robot.robot_id for robot in self._robots]
        )
        self._time = 0.0
        self._pending: list[Action] = []
        self._lock = Lock()
        self.world = WorldWriter()
        self._added: set[str] = set()
        self._moved: dict[str, Any] = {}
        self._base = {robot.robot_id: [0.0, 0.0, 0.0] for robot in self._robots}
        self._layers = {robot.robot_id: usd_layer(robot.model) for robot in self._robots}
        self._base_sent: set[str] = set()
        self._refusals: list[tuple[str, str]] = []
        self._world = subprocess.Popen(
            [
                ensure_deno(),
                "run",
                "--quiet",
                "--allow-read",
                "--allow-env",
                "--config",
                str(_WORLD / "deno.json"),
                str(_WORLD / "sim2" / "world.ts"),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        self._reset()
        self._started = time.monotonic()

    def describe(self) -> dict[str, Any]:
        return {"backend": "dimsim", "scene": str(self._scene_layer)}

    def step(self) -> tuple[float, WorldObservations]:
        self._apply_pending()
        steps = int(((time.monotonic() - self._started) * self._speed - self._time) / TIMESTEP)
        if steps > self._max_burst_steps:
            self._started += (steps - self._max_burst_steps) * TIMESTEP / self._speed
            steps = self._max_burst_steps
        if steps < 1:
            return self._time, self.frame()
        return self._advance(steps)

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

    def _reset(self) -> None:
        state = self._call(
            {
                "op": "init",
                "timestep": TIMESTEP,
                "bodies": self._bodies,
                "meshes": self._meshes,
                "robots": [self._robot(robot) for robot in self._robots],
            }
        )
        self._publish(state)

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
            case _:
                raise ValueError(
                    f"the dimsim world cannot do {action!r}: a robot has only its base joints"
                    f" {BASE_JOINTS}, by velocity"
                )

    def _publish(self, state: dict[str, Any]) -> None:
        self._time = float(state.get("time", self._time))
        with self._lock:
            self._moved.update(state["entities"])

    def frame(self) -> WorldObservations:
        world = self.world
        if not self._added:
            world.add("world", backend="dimsim")
            self._added.add("world")
            for robot_id in self._base:
                world.add(robot_id, model=str(self._layers[robot_id]))
                for name in BASE_JOINTS:
                    world.add(f"{robot_id}/{name}")
                self._added.add(robot_id)
        with self._lock:
            moved, self._moved = self._moved, {}
        for name, entity in moved.items():
            if name not in self._added:
                world.add(name)
                self._added.add(name)
            world.position(name, [*entity["pos"], *entity["quat"]])
        for robot_id, held in self._base.items():
            if robot_id not in self._base_sent:
                for name, value in zip(BASE_JOINTS, held, strict=True):
                    world.joint(f"{robot_id}/{name}", 0.0, value, 0.0)
                self._base_sent.add(robot_id)
        for target, reason in self._refusals:
            world.set(target if target in world.ids else "world", REFUSED, {"reason": reason})
        self._refusals.clear()
        return world.end_frame(self._time)

    def _robot(self, robot: SimulatedRobot) -> dict[str, Any]:
        from dimos.robot.assets.usd import layer_from, load_pxr

        pxr = load_pxr()
        stage = pxr.Usd.Stage.Open(
            layer_from(pxr, robot.model.usd, "a robot", robot.model._asset_root())
        )
        root = _root_prim(pxr, stage, robot.root_body)
        inverse = (
            pxr.UsdGeom.Xformable(root)
            .ComputeLocalToWorldTransform(pxr.Usd.TimeCode.Default())
            .GetInverse()
        )
        cache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), PURPOSES)
        corners = []
        for prim in stage.Traverse():
            if not prim.IsA(pxr.UsdGeom.Gprim):
                continue
            aligned = cache.ComputeWorldBound(prim).Transform(inverse).ComputeAlignedRange()
            if not aligned.IsEmpty():
                corners += [np.array(aligned.GetMin()), np.array(aligned.GetMax())]
        if not corners:
            raise ValueError(f"{robot.robot_id}: its layer has no geometry to be the size of")
        low, high = np.min(corners, axis=0), np.max(corners, axis=0)
        x, y, z = robot.xyz
        return {
            "id": robot.robot_id,
            "pos": [x, y, z - float(low[2])],
            "yaw": robot.rpy[2],
            "halfExtents": ((high - low) / 2).tolist(),
            "center": ((high + low) / 2).tolist(),
            "mass": _robot_mass(pxr, stage),
        }


def _root_prim(pxr: Any, stage: Any, root_body: str) -> Any:
    bodies = {
        prim.GetPath().pathString: prim
        for prim in stage.Traverse()
        if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI)
    }
    if not bodies:
        raise KeyError(f"no rigid body at all in this layer, so no {root_body!r} to stand it on")
    return bodies[resolve(bodies, root_body)]


def _robot_mass(pxr: Any, stage: Any) -> float:
    total = 0.0
    for prim in stage.Traverse():
        attribute = pxr.UsdPhysics.MassAPI(prim).GetMassAttr()
        if attribute and attribute.HasAuthoredValue():
            total += float(attribute.Get() or 0.0)
    return total if total > 0 else _DEFAULT_ROBOT_MASS
