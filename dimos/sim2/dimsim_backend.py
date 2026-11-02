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

"""The DimSim backend: DimSim's Rapier physics (misc/DimSim/cli/sim2/world.ts), stepped by sim2.

The scene is the same MJCF file the MuJoCo backend takes, read into dimos.scene_data.v1 bodies. A robot is
DimSim's kind of robot: a kinematic base the size of its model, moved by SetVelocity, sliding along static
geometry and pushing dynamic bodies. Each frame is a WorldChange of what Rapier moved (its sleeping bodies did not);
sensors draw the world from it, and actions it cannot do (joints) it refuses.
"""

import asyncio
from collections.abc import Mapping, Sequence
import json
from pathlib import Path
import subprocess
from threading import Lock
import time
from typing import TYPE_CHECKING, Any

from dimos_lcm.world_msgs import WorldChange
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.sim2.actions import (
    Action,
    Attach,
    Detach,
    Interact,
    Pause,
    Remove,
    Reset,
    Resume,
    SetPose,
    SetVelocity,
    Spawn,
    Step,
)
from dimos.sim2.backend import Publisher
from dimos.sim2.mujoco_backend import scene_bodies
from dimos.sim2.spec import Simulated, SimulatedRobot
from dimos.sim2.world import _assets, keyframe
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
from dimos.utils.deno import ensure_deno

if TYPE_CHECKING:
    from dimos.sim2.module import SimulationConfig

_WORLD = Path(__file__).parents[2] / "misc" / "DimSim" / "cli"
# DimSim's own physics rate.
TIMESTEP = 0.02
_MAX_BURST_STEPS = 5


class DimsimBackend:
    def __init__(
        self, config: "SimulationConfig", simulated: Mapping[str, Simulated], directory: Path
    ) -> None:
        self._config = config
        self._scene = config.scene or Path(__file__).with_name("logistics.xml")
        # Sensors are drawn from world_change by their own views, not by the physics.
        self._robots = [d for d in simulated.values() if isinstance(d, SimulatedRobot)]
        for robot in self._robots:
            if robot.servos or robot.control_period is not None:
                raise ValueError(
                    f"{robot.robot_id}: a dimsim robot is a kinematic base, without servos"
                )
        model = mujoco.MjModel.from_xml_path(str(self._scene))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        movers = {
            int(model.body_rootid[model.jnt_bodyid[j]])
            for j in range(model.njnt)
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
        }
        self._bodies, self._meshes = scene_bodies(model, data, set(), movers)
        self._spawned: dict[str, dict[str, Any]] = {}
        self._version = 0
        self._latest: dict[str, Any] = {}
        self._pending: list[Action] = []
        self._lock = Lock()
        self._paused = False
        self._steps_requested = 0
        self.world = WorldWriter()
        self._sent_paused = False
        # Entities the frames have added, and the moves Rapier reported since the last frame.
        self._added: set[str] = set()
        self._moved: dict[str, Any] = {}
        self._removed: list[str] = []
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

    def describe(self) -> dict[str, Any]:
        return {"scene": str(self._scene)}

    def assets(self) -> Mapping[str, Path | bytes]:
        return {"world.xml": self._scene.read_bytes()}

    def scene_data(self) -> str:
        with self._lock:
            spawned = list(self._spawned.values())
            version = self._version
        # Movers where they are now, for a viewer that starts drawing after they moved.
        now = self._latest.get("entities", {})
        bodies = [
            {**body, "pos": now[body["name"]]["pos"], "quat": now[body["name"]]["quat"]}
            if body["name"] in now
            else body
            for body in self._bodies + spawned
        ]
        return json.dumps(
            {
                "format": "dimos.scene_data.v1",
                # A new id per change, so a viewer that drew the old world draws this one.
                "id": f"{self._scene.stem}-dimsim-{id(self)}-{version}",
                "frame": "world",
                "bodies": bodies,
                "meshes": self._meshes,
                "hfields": {},
                "textures": {},
            }
        )

    def apply(self, actions: Sequence[Action]) -> None:
        with self._lock:
            for action in actions:
                match action:
                    case Spawn():
                        self._spawned[action.entity] = _spawned_body(action)
                        self._version += 1
                    case Remove(entity) if entity in self._spawned:
                        del self._spawned[entity]
                        self._version += 1
            self._pending.extend(actions)

    async def run(self, world_change: Publisher) -> None:
        started = time.monotonic()
        sim_time = 0.0
        while True:
            await asyncio.to_thread(self._apply_pending)
            if self._paused:
                steps, self._steps_requested = self._steps_requested, 0
                started = time.monotonic() - sim_time / self._config.speed
            else:
                steps = int(
                    ((time.monotonic() - started) * self._config.speed - sim_time) / TIMESTEP
                )
                if steps > _MAX_BURST_STEPS:
                    started += (steps - _MAX_BURST_STEPS) * TIMESTEP / self._config.speed
                    steps = _MAX_BURST_STEPS
            if steps:
                state = await asyncio.to_thread(self._call, {"op": "step", "steps": steps})
                sim_time = state["time"]
                self._publish(state)
                if not world_change.busy:
                    world_change.offer(self.frame())
            await asyncio.sleep(0.002)

    def step(self, steps: int = 1) -> WorldChange:
        """Apply what is pending and advance, without the real-time loop (tests, offline use)."""
        self._apply_pending()
        self._publish(self._call({"op": "step", "steps": steps}))
        return self.frame()

    def close(self) -> None:
        # __init__ may have failed before starting the world.
        if hasattr(self, "_world") and self._world.poll() is None:
            self._world.kill()
            self._world.wait()

    def __del__(self) -> None:
        self.close()

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
        with self._lock:
            spawned = list(self._spawned.values())
        state = self._call(
            {
                "op": "init",
                "timestep": TIMESTEP,
                "bodies": self._bodies + spawned,
                "meshes": self._meshes,
                "robots": [_robot(robot) for robot in self._robots],
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
            case SetVelocity(entity, twist):
                self._call(
                    {
                        "op": "velocity",
                        "entity": entity,
                        "linear": _xyz(twist.linear),
                        "angular": _xyz(twist.angular),
                    }
                )
            case SetPose(entity, pose, _):
                p, q = pose.position, pose.orientation
                self._call(
                    {
                        "op": "pose",
                        "entity": entity,
                        "pos": [p.x, p.y, p.z],
                        "quat": [q.x, q.y, q.z, q.w],
                    }
                )
            case Spawn():
                self._call({"op": "spawn", "body": _spawned_body(action), "mass": action.mass})
            case Remove(entity):
                self._call({"op": "remove", "entity": entity})
                with self._lock:
                    self._moved.pop(entity, None)
                    self._removed.append(entity)
            case Attach(entity, to, None):
                self._call({"op": "attach", "entity": entity, "to": to})
            case Detach(entity):
                self._call({"op": "detach", "entity": entity})
            case Interact(entity, action):
                raise ValueError(
                    f"the dimsim world gives {entity} no states, so it has no {action!r}"
                )
            case Pause():
                self._paused = True
            case Resume():
                self._paused = False
            case Step(steps):
                self._steps_requested += steps
            case Reset():
                self._reset()
            case _:
                raise ValueError(
                    f"the dimsim world cannot do {type(action).__name__}: it has no joints, wrenches or carrying by a body"
                )

    def _publish(self, state: dict[str, Any]) -> None:
        self._latest = state
        with self._lock:
            self._moved.update(state["entities"])

    def frame(self) -> WorldChange:
        """What Rapier moved since the last frame, entities added and removed, and refusals."""
        world = self.world
        if not self._added:
            world.add(
                "world",
                cbor_update(
                    EPOCH, time.time() - self._latest.get("time", 0.0) / self._config.speed
                ),
                cbor_update("backend", "dimsim"),
            )
            self._added.add("world")
        robot_ids = {robot.robot_id for robot in self._robots}
        with self._lock:
            moved, self._moved = self._moved, {}
            removed, self._removed = self._removed, []
        for name in removed:
            if name in self._added:
                world.remove(name)
                self._added.discard(name)
        for name, entity in moved.items():
            linear, angular = np.array(entity["linear"]), np.array(entity["angular"])
            if name in robot_ids:
                # A robot's velocity is the one commanded, in its own frame; the world's are world-aligned.
                to_world = Rotation.from_quat(entity["quat"]).as_matrix()
                linear, angular = to_world @ linear, to_world @ angular
            motion = (
                pose_update([*entity["pos"], *entity["quat"]]),
                velocity_update([*linear, *angular]),
            )
            if name in self._added:
                world.update(name, *motion)
            else:
                world.add(name, tag_update(ROBOT if name in robot_ids else OBJECT), *motion)
                self._added.add(name)
        if self._paused != self._sent_paused:
            world.update("world", tag_update(PAUSED, self._paused))
            self._sent_paused = self._paused
        for target, reason in self._refusals:
            world.event(
                target if target in world.ids else "world", "events/refused", {"reason": reason}
            )
        self._refusals.clear()
        return world.end_frame(float(self._latest.get("time", 0.0)))


def _robot(robot: SimulatedRobot) -> dict[str, Any]:
    """A robot as DimSim moves one: a box around its model's geoms, resting on the floor, at its pose."""
    spec = mujoco.MjSpec.from_file(str(robot.model), assets=_assets(robot))
    for key in list(spec.keys):
        spec.delete(key)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    corners = []
    for g in range(model.ngeom):
        if model.geom_group[g] >= 3 and not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        rotation = data.geom_xmat[g].reshape(3, 3)
        center = data.geom_xpos[g] + rotation @ model.geom_aabb[g][:3]
        reach = np.abs(rotation) @ model.geom_aabb[g][3:]
        corners += [center - reach, center + reach]
    low, high = np.min(corners, axis=0), np.max(corners, axis=0)
    root = data.xpos[model.body(robot.root_body).id]
    height, _ = keyframe(robot)
    x, y, z = robot.xyz
    # The robot's pose is its root body's, which sits wherever its model has it above the box's bottom.
    return {
        "id": robot.robot_id,
        "pos": [x, y, max(z + height, root[2] - low[2])],
        "yaw": robot.rpy[2],
        "halfExtents": ((high - low) / 2).tolist(),
        "center": ((high + low) / 2 - root).tolist(),
        "mass": float(model.body_subtreemass[0]),
    }


def _spawned_body(spawn: Spawn) -> dict[str, Any]:
    p, q = spawn.pose.position, spawn.pose.orientation
    return {
        "name": spawn.entity,
        "label": spawn.entity,
        "pos": [p.x, p.y, p.z],
        "quat": [q.x, q.y, q.z, q.w],
        "mocap": spawn.mass > 0,
        "mass": spawn.mass,
        "geoms": [
            {
                "name": spawn.entity,
                "type": spawn.shape,
                "size": list(spawn.size),
                "pos": [0.0, 0.0, 0.0],
                "quat": [0.0, 0.0, 0.0, 1.0],
                "rgba": list(spawn.rgba),
            }
        ],
        "lights": [],
    }


def _xyz(vector: Any) -> list[float]:
    return [vector.x, vector.y, vector.z]
