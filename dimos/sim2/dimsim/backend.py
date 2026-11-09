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

"""The DimSim engine: DimSim's Rapier physics (misc/DimSim/cli/sim2/world.ts), stepped by sim2.

The engine, not the module. It is handed the world as USD and the knobs for running it, it steps,
and it says what changed -- it owns no streams and publishes nothing. `DimSimSim` in `sim.py` is
the module around it.

A robot here is DimSim's kind of robot: a kinematic box the size of its layer, moved by velocity
commands to its three base joints, sliding along static geometry and pushing dynamic bodies. It
has no articulation at all, so an action it cannot do it refuses rather than drops, and the joint
drives a robot's layer carries are ignored because there is nothing here to apply them to.
"""

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
# DimSim's own physics rate.
TIMESTEP = 0.02
# A dimsim robot's whole control surface: vx, vy and wz in its own heading frame, each held until
# another command replaces it.
BASE_JOINTS = ("base_vx", "base_vy", "base_wz")
# What a robot weighs where its layer never said. The character controller uses it to decide how
# hard walking into a crate pushes it, so a number that is wrong is better than no world.
_DEFAULT_ROBOT_MASS = 10.0


class DimSimBackend:
    def __init__(
        self,
        usd: "UsdModel",
        simulated: Iterable[Simulated] = (),
        speed: float = 1.0,
        max_burst_steps: int = 5,
    ) -> None:
        """A USD layer, what is in it, and how fast to run it. Nothing else.

        `max_burst_steps` is how much physics it will catch up on at once; lag past one burst is
        dropped, since a command landing a whole control period late is worse than a skip.
        """
        self._speed = speed
        self._max_burst_steps = max_burst_steps
        # The world as a file: nothing here opens it, but it is what a person looking at a world
        # that is not behaving goes and reads.
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
        # Entities the frames have added, and the moves Rapier reported since the last frame.
        self._added: set[str] = set()
        self._moved: dict[str, Any] = {}
        # Each robot's base joints' held velocities (vx, vy, wz), and whether the world has been
        # told them since.
        self._base = {robot.robot_id: [0.0, 0.0, 0.0] for robot in self._robots}
        # Each robot's own layer as a file. Announced on the robot's entity, which is what tells a
        # sensor reading the observations that this entity is a robot rather than a crate
        # (`sensor_world.robot_ids`) -- and so what gets the whole robot posed in its view instead
        # of left standing where its layer had it.
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
        # After the world exists, not before: starting Deno and loading a room into it is time the
        # world does not owe the clock, and counting it is a first step of nothing but catch-up.
        self._started = time.monotonic()

    def describe(self) -> dict[str, Any]:
        return {"backend": "dimsim", "scene": str(self._scene_layer)}

    def step(self) -> tuple[float, WorldObservations]:
        """Advance the world to the clock and say what moved: the sim time reached, and a frame.

        There is no lockstep here and so no `waiting_for`/`ready()`: a kinematic base has no servo
        controller for the world to hold a tick open for.
        """
        self._apply_pending()
        steps = int(((time.monotonic() - self._started) * self._speed - self._time) / TIMESTEP)
        if steps > self._max_burst_steps:
            self._started += (steps - self._max_burst_steps) * TIMESTEP / self._speed
            steps = self._max_burst_steps
        if steps < 1:
            return self._time, self.frame()
        return self._advance(steps)

    def advance(self, steps: int = 1) -> tuple[float, WorldObservations]:
        """Apply what is pending and step exactly this many times, off the clock (tests, offline use)."""
        self._apply_pending()
        return self._advance(steps)

    def apply(self, actions: Sequence[Action]) -> None:
        """Take actions now, from whatever thread they arrived on; they land on the next step."""
        with self._lock:
            self._pending.extend(actions)

    def close(self) -> None:
        # __init__ may have failed before starting the world.
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
                # One bad action must not stop the world; its target hears why on the next frame.
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
        """What Rapier moved since the last frame, entities it added, and refusals."""
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
        """A robot as DimSim moves one: a box around its layer's geometry, standing at its pose.

        Read off the robot's own layer rather than the composed scene, because that is where its
        geometry is still the robot's alone -- in the scene it has already been placed, and this
        box is what gets placed.
        """
        from dimos.robot.assets.usd import _layer_from, _pxr

        pxr = _pxr()
        stage = pxr.Usd.Stage.Open(
            _layer_from(pxr, robot.model.usd, "a robot", robot.model._asset_root())
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
        # Its pose is the root body's, and the root sits wherever its layer has it above the box's
        # bottom; `xyz` is where the robot stands, so the box's bottom goes there.
        return {
            "id": robot.robot_id,
            "pos": [x, y, z - float(low[2])],
            "yaw": robot.rpy[2],
            "halfExtents": ((high - low) / 2).tolist(),
            "center": ((high + low) / 2).tolist(),
            "mass": _robot_mass(pxr, stage),
        }


def _root_prim(pxr: Any, stage: Any, root_body: str) -> Any:
    """The robot's root link, by the name its description calls it. Raises KeyError naming the rest."""
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
