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
from dimos.sim2.actions import Action, JointCommand
from dimos.sim2.dimsim.scene import PURPOSES, stage_bodies
from dimos.sim2.mujoco.names import resolve
from dimos.sim2.spec import ODOM_FRAME, ROBOT, SimulatedRobot
from dimos.sim2.world_observations import REFUSED, WorldWriter
from dimos.utils.deno import ensure_deno

if TYPE_CHECKING:
    from dimos.robot.assets.model import UsdModel

# The Deno world dimsim steps; it ships with the repo (misc/DimSim/cli), not with the dimos package.
_WORLD = DIMOS_PROJECT_ROOT / "misc" / "DimSim" / "cli"
BASE_JOINTS = ("base_vx", "base_vy", "base_wz")


def _world_dir() -> Path:
    if (_WORLD / "sim2" / "world.ts").is_file():
        return _WORLD
    raise FileNotFoundError(
        f"the dimsim world is not at {_WORLD}: it ships with a dimos checkout (misc/DimSim/cli), "
        f"not with the dimos package, so run dimsim from a checkout"
    )


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
        self._base = {robot_id: [0.0, 0.0, 0.0] for robot_id in robot_ids}
        self._base_sent: set[str] = set()
        # Each entity's pose and velocity as last sent.
        self._sent: dict[str, NDArray[np.float64]] = {}
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

    def _announce(self) -> None:
        """Name every entity once; a listener that missed it asks the world for its ``snapshot()``."""
        world = self.world
        world.add("world", backend="dimsim")
        for robot in self._robots:
            announced = {} if robot.odom_frame is None else {ODOM_FRAME: robot.odom_frame}
            world.add(robot.robot_id, **{ROBOT: True}, **announced)
            for name in BASE_JOINTS:
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
            if name not in self._known:
                world.add(name)
                self._known.add(name)
            self._send(
                name, [*entity["pos"], *entity["quat"]], entity["velocity"], entity["acceleration"]
            )
        for robot_id, held in self._base.items():
            if robot_id not in self._base_sent:
                for name, value in zip(BASE_JOINTS, held, strict=True):
                    world.joint(f"{robot_id}/{name}", 0.0, value, 0.0)
                self._base_sent.add(robot_id)
        for target, reason in self._refusals:
            world.set(target if target in world.ids else "world", REFUSED, {"reason": reason})
        self._refusals.clear()
        return world.end_frame(self._time)

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

    def _robot(self, robot: SimulatedRobot) -> dict[str, Any]:
        """The robot's box, placed and turned (about z) as its USD places its root body."""
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
        forward = placed.TransformDir(pxr.Gf.Vec3d(1, 0, 0))
        return {
            "id": robot.robot_id,
            "pos": list(placed.ExtractTranslation()),
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
