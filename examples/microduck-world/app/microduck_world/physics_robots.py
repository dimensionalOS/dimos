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

"""Robot bodies and shared gait execution, owned by the physics thread."""

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.pollen.microduck.gait import CONTROL_DT
from dimos.robot.pollen.microduck.policies import FALL_GRAVITY_Z, PolicyBank, PolicyScheduler
from dimos.robot.pollen.microduck.sim_module import (
    POLICY_DECIMATION,
    MicroduckSimModuleConfig,
    shape_twist,
)
from microduck_world.robot_io import (
    ROBOT_IDS,
    VISITOR_IDS,
    RobotCommand,
    RobotState,
)
from microduck_world.roster import ROSTER
from numpy.typing import NDArray


@dataclass
class RobotBody:
    id: str
    bank: PolicyBank
    scheduler: PolicyScheduler
    actuators: list[int]
    joint_qpos: list[int]
    joint_qvel: list[int]
    geoms: list[int]
    collision: NDArray[np.int32]
    origin: tuple[float, float] = (0.0, 0.0)
    active: bool = False
    generation: str = ""
    fallen_since: float | None = None
    respawns: int = 0


class WorldRobots:
    """Only step() mutates physics; subscribers update bounded mailboxes."""

    def __init__(
        self,
        model: mujoco.MjModel,
        bank: PolicyBank,
        settings: dict[str, Any],
        config: MicroduckSimModuleConfig,
    ) -> None:
        self.model, self.config = model, config
        self.roster = settings.get("robots", ROSTER)
        self.clearance = float(settings["clearance"])
        self.robots: dict[str, RobotBody] = {}
        self._lock = threading.Lock()
        self._wanted: dict[str, str] = {}
        self._commands: dict[str, tuple[RobotCommand, float]] = {}
        self._policies: dict[str, RobotCommand] = {}
        self._respawns: dict[str, RobotCommand] = {}
        self._lease_at = 0.0
        self._step = 0
        for id in ROBOT_IDS:
            prefix = "" if id == "duck1" else id + "_"
            scoped = bank.for_robot(model, prefix)
            joints = [model.joint(prefix + name) for name in bank.joint_names]
            actuators = [int(np.flatnonzero(model.actuator_trnid[:, 0] == j.id)[0]) for j in joints]
            root = model.body(prefix + "trunk_base").id
            geoms = []
            for i in range(model.ngeom):
                ancestor = int(model.geom_bodyid[i])
                while ancestor and ancestor != root:
                    ancestor = int(model.body_parentid[ancestor])
                if ancestor == root:
                    geoms.append(i)
            self.robots[id] = RobotBody(
                id,
                scoped,
                PolicyScheduler(scoped.availability, scoped.variant),
                actuators,
                [int(j.qposadr[0]) for j in joints],
                [int(j.dofadr[0]) for j in joints],
                geoms,
                np.array([model.geom_contype[geoms], model.geom_conaffinity[geoms]]),
            )

    def leases(self, occupants: dict[str, str]) -> None:
        with self._lock:
            self._wanted = {id: gen for id, gen in occupants.items() if id in VISITOR_IDS}
            self._lease_at = time.monotonic()

    def command(self, id: str, command: RobotCommand) -> None:
        if id not in ROBOT_IDS:
            return
        with self._lock:
            if command.kind == "twist":
                if not isinstance(command.value, tuple) or len(command.value) != 3:
                    return
                if not all(math.isfinite(v) for v in command.value):
                    return
                self._commands[id] = (command, time.monotonic())
            elif command.kind == "policy":
                self._policies[id] = command
            elif command.kind == "respawn":
                self._respawns[id] = command

    def _place(self, robot: RobotBody, data: mujoco.MjData, xy: tuple[float, float]) -> None:
        adr = robot.bank.root_qpos_adr
        data.qpos[adr : adr + 2] = xy
        robot.bank.initial_qpos(data)
        robot.bank.reset()
        data.ctrl[robot.actuators] = robot.bank.default_pose

    def _spawn(self, robot: RobotBody, data: mujoco.MjData, *, reset_origin: bool = True) -> bool:
        player = self.roster[robot.id]
        candidates = [
            player,
            *(p for id, p in self.roster.items() if id != robot.id and p["team"] == player["team"]),
        ]
        for candidate in candidates:
            x, y = candidate["spawn"]
            if all(
                other is robot
                or not other.active
                or math.hypot(
                    data.qpos[other.bank.root_qpos_adr] - x,
                    data.qpos[other.bank.root_qpos_adr + 1] - y,
                )
                >= self.clearance
                for other in self.robots.values()
            ) and self._clear_spawn(data, x, y):
                self._place(robot, data, (x, y))
                yaw = float(candidate["yaw"])
                adr = robot.bank.root_qpos_adr
                data.qpos[adr + 3 : adr + 7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
                if reset_origin:
                    # All supplied places use this frame. Knowledge is still private.
                    robot.origin = (0.0, 0.0)
                self.model.geom_contype[robot.geoms] = robot.collision[0]
                self.model.geom_conaffinity[robot.geoms] = robot.collision[1]
                robot.active = True
                return True
        return False

    def _clear_spawn(self, data: mujoco.MjData, x: float, y: float) -> bool:
        """Reject fixed obstacles and free balls in the footprint above the floor."""
        radius = 0.18
        robot_geoms = {g for r in self.robots.values() for g in r.geoms}
        for i in range(self.model.ngeom):
            if i in robot_geoms or not (
                self.model.geom_contype[i] or self.model.geom_conaffinity[i]
            ):
                continue
            position = data.geom_xpos[i]
            if self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX:
                rotation = data.geom_xmat[i].reshape(3, 3)
                extent = np.abs(rotation) @ self.model.geom_size[i]
                if position[2] + extent[2] <= 0.035 or position[2] - extent[2] > 0.35:
                    continue
                dx = max(abs(x - position[0]) - extent[0], 0)
                dy = max(abs(y - position[1]) - extent[1], 0)
                if math.hypot(dx, dy) < radius:
                    return False
            elif self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_SPHERE:
                size = float(self.model.geom_size[i, 0])
                if position[2] + size > 0.035 and position[2] - size < 0.35:
                    if math.hypot(x - position[0], y - position[1]) < radius + size:
                        return False
        return True

    def step(self, data: mujoco.MjData) -> None:
        now = time.monotonic()
        with self._lock:
            wanted = dict(self._wanted) if now - self._lease_at < 3 else {}
            commands = dict(self._commands)
            policies, self._policies = self._policies, {}
            respawns, self._respawns = self._respawns, {}
        for index, robot in enumerate(self.robots.values()):
            generation = wanted.get(robot.id, "")
            if generation != robot.generation or self._step == 0:
                robot.active = False
                robot.generation = generation
                robot.fallen_since = None
                robot.respawns = 0
                robot.scheduler = PolicyScheduler(robot.bank.availability, robot.bank.variant)
                robot.bank.reset()
                self.model.geom_contype[robot.geoms] = 0
                self.model.geom_conaffinity[robot.geoms] = 0
            if generation and not robot.active:
                self._spawn(robot, data)
            if not robot.active:
                self._place(robot, data, (10 + index, 10))
                data.qpos[robot.bank.root_qpos_adr + 2] = -5
                continue
            respawn = respawns.get(robot.id)
            if respawn and respawn.generation == generation:
                if self._spawn(robot, data, reset_origin=False):
                    robot.scheduler = PolicyScheduler(robot.bank.availability, robot.bank.variant)
                    robot.fallen_since = None
                    robot.respawns += 1
                    # Old queued commands must not restart motion after a reset.
                    commands.pop(robot.id, None)
                    policies.pop(robot.id, None)
                    with self._lock:
                        self._commands.pop(robot.id, None)
                        self._policies.pop(robot.id, None)
                else:
                    # Keep an explicit reset pending until its own team has space.
                    with self._lock:
                        self._respawns[robot.id] = respawn
                        self._commands.pop(robot.id, None)
                    commands.pop(robot.id, None)
            request = policies.get(robot.id)
            if request and request.generation == generation and isinstance(request.value, str):
                try:
                    payload = json.loads(request.value)
                    if isinstance(payload, dict):
                        robot.scheduler.request(payload.get("policy"), payload.get("action", ""))
                except (ValueError, TypeError):
                    pass
            if self._step % POLICY_DECIMATION:
                continue
            twist = (0.0, 0.0, 0.0)
            current = commands.get(robot.id)
            if (
                current
                and current[0].generation == generation
                and now - current[1] < self.config.cmd_timeout
            ):
                value = current[0].value
                if isinstance(value, tuple):
                    twist = value
            robot.scheduler.set_twist(*shape_twist(self.config, *twist))
            if not robot.scheduler.suspend_fall_detector:
                gravity = float(robot.bank.projected_gravity(data)[2])
                if gravity > FALL_GRAVITY_Z:
                    if robot.fallen_since is None:
                        robot.fallen_since = now
                    robot.scheduler.notify_fall(
                        now - robot.fallen_since > self.config.auto_stand_after
                    )
                else:
                    robot.fallen_since = None
                    robot.scheduler.notify_fall(False)
            name, command = robot.scheduler.tick(CONTROL_DT)
            data.ctrl[robot.actuators] = robot.bank.step(name, command, data)
        self._step += 1

    def states(self, data: mujoco.MjData) -> dict[str, RobotState]:
        result = {}
        now = time.time()
        for robot in self.robots.values():
            if not robot.active:
                continue
            adr = robot.bank.root_qpos_adr
            x, y, z, w, qx, qy, qz = data.qpos[adr : adr + 7]
            result[robot.id] = RobotState(
                robot.generation,
                PoseStamped(
                    position=(x - robot.origin[0], y - robot.origin[1], z),
                    orientation=(qx, qy, qz, w),
                    frame_id="world",
                    ts=now,
                ),
                JointState(
                    name=list(robot.bank.joint_names),
                    position=data.qpos[robot.joint_qpos].tolist(),
                    velocity=data.qvel[robot.joint_qvel].tolist(),
                    ts=now,
                ),
                json.dumps(
                    {**robot.scheduler.snapshot(), "respawns": robot.respawns},
                    separators=(",", ":"),
                ),
            )
        return result

    def snapshot(self) -> list[dict[str, Any]]:
        return [
            {"id": r.id, "active": r.active, "generation": r.generation}
            for r in self.robots.values()
        ]

    def sensor_assignments(self) -> dict[str, tuple[str, tuple[float, float]]]:
        return {r.id: (r.generation, r.origin) for r in self.robots.values() if r.active}
