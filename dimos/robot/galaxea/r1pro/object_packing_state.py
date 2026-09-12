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

"""Shared read-only object goals and physical evidence for offline and native ACT."""

from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.grasping_task import HOME_TCP, TaskResult
from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.object_packing import OBJECT_GOAL_FEATURES
from dimos.robot.galaxea.r1pro.object_packing_scene import (
    MAX_OBJECTS,
    OBJECT_SLOT_BOUNDS,
    OBJECT_TRAY_HALF_SIZE,
    SHAPES,
    ObjectLayout,
    PackingObject,
)
from dimos.robot.galaxea.r1pro.packing import OccupiedFootprint, empty_slots


def object_extent(obj: PackingObject, rotation: NDArray[Any]) -> NDArray[np.float64]:
    """Conservative half extents in a reference frame, including tilted objects."""
    if obj.shape == "box":
        return np.asarray(np.abs(rotation) @ obj.half_size, dtype=np.float64)
    axis = rotation[:, 2]
    return np.asarray(
        obj.half_size[2] * np.abs(axis) + obj.radius * np.sqrt(np.maximum(0, 1 - axis**2)),
        dtype=np.float64,
    )


class ObjectPackingState:
    """Observe supplied MuJoCo state; never command, step or reset the robot."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        layout: ObjectLayout,
        home: NDArray[Any],
    ) -> None:
        self.model, self.data, self.layout = model, data, layout
        self.home = np.asarray(home, dtype=np.float64).copy()
        self.qids = np.array([model.joint(n).qposadr[0] for n in R1PRO_PICK_PLACE_JOINTS])
        self.pad_ids = {model.geom(f"right_finger_pad{i}").id for i in (1, 2)}
        self.guard = PlanarTransport(
            model, data, cargo_bodies=tuple(o.name for o in layout.objects)
        )
        self.selected = 0
        self.target = np.zeros(3)
        self.bottle_id = model.body(layout.objects[0].name).id
        self.bottle_geoms = set(map(int, np.flatnonzero(model.geom_bodyid == self.bottle_id)))
        self.initial_height = float(data.body(self.bottle_id).xpos[2])
        self.peak_lift = 0.0
        self.bilateral_grasp = False

    def observe(self) -> None:
        lift = float(self.data.body(self.bottle_id).xpos[2]) - self.initial_height
        self.peak_lift = max(self.peak_lift, lift)
        if lift > 0.04:
            touching = set()
            for contact in self.data.contact:
                geoms = set(map(int, contact.geom))
                if contact.dist <= 0 and geoms & self.bottle_geoms:
                    touching.update(geoms & self.pad_ids)
            self.bilateral_grasp |= touching == self.pad_ids

    def geometry(self, index: int) -> dict[str, Any]:
        obj = self.layout.objects[index]
        body = self.data.body(obj.name)
        tray = self.data.body("task_bin")
        rotation = tray.xmat.reshape(3, 3)
        relative = rotation.T @ (body.xpos - tray.xpos)
        extent = object_extent(obj, rotation.T @ body.xmat.reshape(3, 3))
        geoms = set(map(int, np.flatnonzero(self.model.geom_bodyid == body.id)))
        touching = set()
        supported = False
        supports = set()
        force = np.zeros(6)
        for i, contact in enumerate(self.data.contact):
            first, second = map(int, contact.geom)
            if contact.dist > 0 or not geoms.intersection((first, second)):
                continue
            mujoco.mj_contactForce(self.model, self.data, i, force)
            if force[0] < 0.02:
                continue
            other = second if first in geoms else first
            if other in self.pad_ids:
                touching.add(other)
            normal = contact.frame[:3] * (1 if second in geoms else -1)
            supported |= self.model.geom_bodyid[other] == tray.id and normal[2] > 0.7
            if normal[2] > 0.7 and int(
                self.model.geom_bodyid[other]
            ) not in self.guard.robot_bodies - self.guard.cargo_ids - {self.guard.tray_id}:
                supports.add(self.model.geom(other).name or f"geom:{other}")
        inside = bool(
            np.all(np.abs(relative[:2]) + extent[:2] < OBJECT_TRAY_HALF_SIZE)
            and abs(relative[2] - extent[2] - 0.015) < 0.008
        )
        return dict(
            object=obj.name,
            shape=obj.shape,
            position=body.xpos.tolist(),
            inside=inside,
            supported=bool(supported),
            support_geoms=sorted(supports),
            upright=bool(body.xmat[8] > np.cos(np.deg2rad(15))),
            released=not touching,
            grasped=touching == self.pad_ids,
            settled=bool(np.linalg.norm(self.data.joint(obj.joint).qvel) < 0.03),
        )

    def inventory(self) -> list[dict[str, Any]]:
        return [self.geometry(i) for i in range(len(self.layout.objects))]

    def placement_target(self, index: int, *, occupied: bool = True) -> NDArray[np.float64] | None:
        """Compute a tray goal without changing the selected object or grasp evidence."""
        tray = self.data.body("task_bin")
        rotation = tray.xmat.reshape(3, 3)
        footprints = []
        for j, other in enumerate(self.layout.objects):
            if j == index or not occupied:
                continue
            body = self.data.body(other.name)
            pos = rotation.T @ (body.xpos - tray.xpos)
            extent = object_extent(other, rotation.T @ body.xmat.reshape(3, 3))
            radius = max(
                other.radius,
                float(np.linalg.norm(extent[:2]))
                if not self.geometry(j)["upright"]
                else other.radius,
            )
            if np.all(np.abs(pos[:2]) < np.asarray(OBJECT_TRAY_HALF_SIZE) + radius):
                footprints.append(OccupiedFootprint(float(pos[0]), float(pos[1]), radius))
        obj = self.layout.objects[index]
        slots = empty_slots(obj.radius, tuple(footprints), inner_half_size=OBJECT_SLOT_BOUNDS)
        if not slots:
            return None
        return np.asarray(tray.xpos + rotation @ np.array((*slots[0], 0.015 + obj.half_size[2])))

    def select_object(self, index: int, *, grasp_only: bool = False) -> bool:
        if not 0 <= index < len(self.layout.objects):
            raise ValueError("Unknown object index")
        if self.geometry(index)["inside"]:
            raise ValueError("Selected object is already in the tray")
        # The existing checkpoint still takes a tray-goal input during grasping.
        # For pick-only this is context, never a placement reservation or action.
        # A full tray must not prevent picking; place recomputes real free space.
        target = self.placement_target(index)
        if target is None and grasp_only:
            target = self.placement_target(index, occupied=False)
        if target is None:
            return False
        obj = self.layout.objects[index]
        self.selected = index
        self.bottle_id = self.model.body(obj.name).id
        self.bottle_geoms = set(map(int, np.flatnonzero(self.model.geom_bodyid == self.bottle_id)))
        self.target = target
        self.initial_height = float(self.data.body(obj.name).xpos[2])
        self.peak_lift, self.bilateral_grasp = 0.0, False
        return True

    def holding(self) -> bool:
        """Require current two-pad contact and measured lift, not historical grasp success."""
        row = self.geometry(self.selected)
        return bool(
            row["grasped"]
            and row["upright"]
            and not row["support_geoms"]
            and row["position"][2] - self.initial_height >= 0.10
        )

    def goal(self) -> NDArray[np.float32]:
        base = self.data.body("base_link")
        rotation = base.xmat.reshape(3, 3)
        tcp = self.data.site("right_tcp").xpos
        obj = self.layout.objects[self.selected]
        body = self.data.body(obj.name)
        values = [
            *(rotation.T @ (body.xpos - tcp)),
            *(rotation.T @ (self.target - tcp)),
            *(rotation.T @ body.xmat.reshape(3, 3)).ravel(),
            *obj.half_size,
            *(float(obj.shape == s) for s in SHAPES),
            *(np.asarray(HOME_TCP) - rotation.T @ (tcp - base.xpos)),
        ]
        neighbors = []
        for i, other in enumerate(self.layout.objects):
            if i != self.selected:
                b = self.data.body(other.name)
                position = rotation.T @ (b.xpos - tcp)
                extent = object_extent(other, rotation.T @ b.xmat.reshape(3, 3))
                neighbors.append([1.0, *map(float, position), *map(float, extent)])
        neighbors.sort(key=lambda row: tuple(row[1:4]))
        neighbors.extend([[0.0] * 7 for _ in range(MAX_OBJECTS - 1 - len(neighbors))])
        values.extend(value for row in neighbors for value in row)
        goal = np.asarray(values, dtype=np.float32)
        if goal.shape != (len(OBJECT_GOAL_FEATURES),) or not np.isfinite(goal).all():
            raise RuntimeError("Invalid selected-object observation")
        return goal

    def result(self) -> TaskResult:
        state = self.geometry(self.selected)
        success = (
            state["inside"]
            and state["supported"]
            and state["upright"]
            and state["released"]
            and state["settled"]
            and self.peak_lift > 0.06
            and self.bilateral_grasp
        )
        return TaskResult(
            bool(success),
            self.peak_lift,
            self.bilateral_grasp,
            state["inside"],
            state["released"],
            state["settled"],
            state["position"],
        )

    def pick_complete(self) -> bool:
        return bool(
            self.result().success and np.max(np.abs(self.data.qpos[self.qids] - self.home)) < 0.015
        )

    def validate(self, initial: list[dict[str, Any]]) -> None:
        for i, (before, after) in enumerate(zip(initial, self.inventory(), strict=True)):
            if i == self.selected:
                continue
            if (
                not after["upright"]
                or not after["released"]
                or (before["inside"] and not after["inside"])
                or np.linalg.norm(np.asarray(after["position"]) - before["position"]) > 0.015
            ):
                raise RuntimeError(f"Disturbed unrequested {after['object']}")
        collisions = self.guard.collisions(self.data, ignore_cargo=True)
        if collisions:
            raise RuntimeError(f"Robot collided with environment: {collisions}")
