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

"""Shared packing goal geometry and physical evidence for native deployment."""

from dataclasses import dataclass
import math
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.grasping_sim import BOTTLE_HALF_HEIGHT, BOTTLE_RADIUS
from dimos.robot.galaxea.r1pro.grasping_task import HOME_TCP, TaskResult, score_task
from dimos.robot.galaxea.r1pro.packing import OccupiedFootprint, empty_slots
from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES, PACKING_JOINTS


@dataclass(frozen=True)
class PackingResult(TaskResult):
    upright: bool


def score_packing(
    data: mujoco.MjData,
    index: int,
    *,
    peak_lift: float,
    bilateral_grasp: bool,
    touching_pads: set[int],
) -> PackingResult:
    """Require upright placement as well as the shared grasp/release evidence."""
    result = score_task(
        data,
        peak_lift=peak_lift,
        bilateral_grasp=bilateral_grasp,
        touching_pads=touching_pads,
        bottle_name=PACKING_BODIES[index],
        joint_name=PACKING_JOINTS[index],
        require_open_gripper=False,
    )
    tray_axis = data.body("task_bin").xmat.reshape(3, 3)[:, 2]
    bottle_axis = data.body(PACKING_BODIES[index]).xmat.reshape(3, 3)[:, 2]
    upright = bool(tray_axis @ bottle_axis >= math.cos(math.radians(15)))
    return PackingResult(
        **(result.to_dict() | {"success": result.success and upright, "upright": upright})
    )


def open_gripper_at_home(data: mujoco.MjData) -> bool:
    """The next pick starts only after ACT returns with an open, clear gripper."""
    return bool(
        np.linalg.norm(data.site("right_tcp").xpos - HOME_TCP) < 0.015
        and data.joint("r1pro/right_gripper").qpos[0] > 0.04
    )


def plan_bottle_goal(data: mujoco.MjData, index: int) -> NDArray[np.float32] | None:
    """Read simulator geometry and choose an empty slot without modifying state."""
    if not 0 <= index < len(PACKING_BODIES):
        raise ValueError("Unknown bottle index")
    container = data.body("task_bin")
    rotation = container.xmat.reshape(3, 3)
    occupied = []
    for other, name in enumerate(PACKING_BODIES):
        if other == index:
            continue
        bottle = data.body(name)
        pos = rotation.T @ (bottle.xpos - container.xpos)
        axis = rotation.T @ bottle.xmat.reshape(3, 3)[:, 2]
        # A tilted bottle occupies more tabletop space than its upright radius.
        radius = BOTTLE_RADIUS + BOTTLE_HALF_HEIGHT * float(np.linalg.norm(axis[:2]))
        if abs(pos[0]) < 0.105 + radius and abs(pos[1]) < 0.105 + radius:
            occupied.append(OccupiedFootprint(float(pos[0]), float(pos[1]), radius))
    slots = empty_slots(BOTTLE_RADIUS, tuple(occupied))
    if not slots:
        return None
    source = data.body(PACKING_BODIES[index]).xpos
    target = container.xpos + rotation @ np.array((*slots[0], 0.015 + BOTTLE_HALF_HEIGHT))
    return np.asarray((*source, *target, BOTTLE_RADIUS, BOTTLE_HALF_HEIGHT), dtype=np.float32)


class PackingMonitor:
    """Retain lift and bilateral-contact evidence for every bottle on physics steps."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model, self.data = model, data
        self.bids = [model.body(name).id for name in PACKING_BODIES]
        self.geom_to_bottle = {
            int(geom): index
            for index, body in enumerate(self.bids)
            for geom in np.flatnonzero(model.geom_bodyid == body)
        }
        self.pads = {model.geom(f"right_finger_pad{i}").id for i in (1, 2)}
        self.initial = data.xpos[self.bids, 2].copy()
        self.peak = np.zeros(5)
        self.grasped = np.zeros(5, dtype=bool)
        self.touching: list[set[int]] = [set() for _ in self.bids]

    def observe(self) -> None:
        self.touching = [set() for _ in self.bids]
        for contact in self.data.contact:
            if contact.dist > 0:
                continue
            first, second = map(int, contact.geom)
            for bottle_geom, pad in ((first, second), (second, first)):
                if pad in self.pads and bottle_geom in self.geom_to_bottle:
                    self.touching[self.geom_to_bottle[bottle_geom]].add(pad)
        lift = self.data.xpos[self.bids, 2] - self.initial
        self.peak = np.maximum(self.peak, lift)
        for index, pads in enumerate(self.touching):
            self.grasped[index] |= lift[index] > 0.04 and pads == self.pads

    def bottle_state(self, index: int) -> dict[str, Any]:
        result = score_packing(
            self.data,
            index,
            peak_lift=float(self.peak[index]),
            bilateral_grasp=bool(self.grasped[index]),
            touching_pads=self.touching[index],
        )
        return {
            "bottle": index + 1,
            **result.to_dict(),
            "pick_complete": result.success and open_gripper_at_home(self.data),
        }

    def report(self) -> dict[str, Any]:
        rows = [self.bottle_state(index) for index in range(5)]
        return {
            "success": all(row["success"] for row in rows),
            "bottles": rows,
            "packed": sum(row["success"] for row in rows),
            "sim_time": float(self.data.time),
        }
