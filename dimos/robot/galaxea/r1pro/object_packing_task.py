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

"""Physical selected-object task with a demonstration-only SDK teacher."""

from collections.abc import Iterator
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.galaxea.r1pro.grasping_task import HOME_TCP, GraspingTask, TaskResult
from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.robot.galaxea.r1pro.home_kinematics import HomeKinematics
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_FPS as FPS
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


class ObjectPackingTask(GraspingTask):
    def __init__(self, scene: Path, layout: ObjectLayout, *, images: bool = True) -> None:
        self.layout = layout
        self.selected = 0
        self.target = np.zeros(3)
        self.evidence: dict[int, tuple[float, bool]] = {}
        super().__init__(scene, images=images, object_body=layout.objects[0].name)
        self.home[4:11] = (0.6, 0.1, 0.0, -1.9, 0.0, 0.0, 0.0)
        self.reset(layout.seed)
        self._kinematics: HomeKinematics | None = None
        self.guard = PlanarTransport(
            self.model, self.data, cargo_bodies=tuple(o.name for o in layout.objects)
        )

    def reset(self, seed: int, jitter: float = 0.0) -> None:
        """Restore this compiled layout; geometry randomization happens at generation."""
        if jitter != 0:
            raise ValueError("Generate a new layout instead of perturbing compiled geometry")
        mujoco.mj_resetData(self.model, self.data)  # type: ignore[attr-defined]
        self.data.qpos[self.qids] = self.home
        self.data.ctrl[self.aids] = self.home
        for side in ("left", "right"):
            self.data.joint(f"{side}_gripper_follower").qpos[:] = 0.05
        mujoco.mj_forward(self.model, self.data)
        for _ in range(round(0.6 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.evidence = {}
        self.peak_lift, self.bilateral_grasp = 0.0, False
        self.initial_height = float(self.data.body(self.bottle_id).xpos[2])
        self.target[:] = 0

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
            upright=bool(body.xmat[8] > np.cos(np.deg2rad(15))),
            released=not touching,
            settled=bool(np.linalg.norm(self.data.joint(obj.joint).qvel) < 0.03),
        )

    def inventory(self) -> list[dict[str, Any]]:
        return [self.geometry(i) for i in range(len(self.layout.objects))]

    def select_object(self, index: int) -> bool:
        if not 0 <= index < len(self.layout.objects):
            raise ValueError("Unknown object index")
        if self.geometry(index)["inside"]:
            raise ValueError("Selected object is already in the tray")
        tray = self.data.body("task_bin")
        rotation = tray.xmat.reshape(3, 3)
        occupied = []
        for j, other in enumerate(self.layout.objects):
            if j == index:
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
                occupied.append(OccupiedFootprint(float(pos[0]), float(pos[1]), radius))
        obj = self.layout.objects[index]
        slots = empty_slots(obj.radius, tuple(occupied), inner_half_size=OBJECT_SLOT_BOUNDS)
        if not slots:
            return False
        self.selected = index
        self.bottle_id = self.model.body(obj.name).id
        self.bottle_geoms = set(map(int, np.flatnonzero(self.model.geom_bodyid == self.bottle_id)))
        self.target = tray.xpos + rotation @ np.array((*slots[0], 0.015 + obj.half_size[2]))
        self.initial_height = float(self.data.body(obj.name).xpos[2])
        self.peak_lift, self.bilateral_grasp = 0.0, False
        return True

    def observation(self, *, render_images: bool = True) -> dict[str, NDArray[Any]]:
        obs = super().observation(render_images=render_images)
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
        obs["observation.environment_state"] = goal
        return obs

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
        return (
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

    def teacher_actions(self) -> Iterator[tuple[str, NDArray[np.float32]]]:
        """SDK-generated training actions; never called by learned evaluation."""
        if self._kinematics is None:
            self._kinematics = HomeKinematics(self.model, self.data, lock_lower_torso=True)
        kinematics = self._kinematics
        source = self.data.body(self.bottle_id).xpos.copy()
        obj = self.layout.objects[self.selected]
        grasp = source + np.array([0, 0, min(0.03, obj.half_size[2] * 0.45)])
        clearance = max(o.position[2] + o.half_size[2] for o in self.layout.objects) + 0.16
        probe = mujoco.MjData(self.model)
        probe.qpos[:] = self.data.qpos
        mujoco.mj_forward(self.model, probe)

        def move(
            phase: str, target: NDArray[Any], opening: float, seconds: float
        ) -> Iterator[tuple[str, NDArray[np.float32]]]:
            start = self.data.site("right_tcp").xpos.copy()
            start_opening = float(self.data.ctrl[self.aids[-1]])
            for frame in range(round(seconds * FPS)):
                t = (frame + 1) / (seconds * FPS)
                u = t * t * (3 - 2 * t)
                xyz = start + (target - start) * u
                goal = kinematics.solve(
                    probe,
                    {"right": xyz},
                    torso_yaw_only=phase not in ("lift", "clear_sources", "lower_clear"),
                    position_tolerance=0.001
                    if phase in ("approach", "grasp", "place", "seek_support", "release")
                    else 0.003,
                    orientation_tolerance=0.005,
                )
                if np.max(np.abs(goal[:18] - self.data.ctrl[self.aids[:18]])) > 0.14:
                    raise RuntimeError("Teacher IK produced a discontinuous joint command")
                goal[-1] = start_opening + (opening - start_opening) * u
                probe.qpos[self.qids] = goal
                mujoco.mj_forward(self.model, probe)
                yield phase, goal.astype(np.float32)
            for _ in range(FPS // 2):
                yield phase, goal.astype(np.float32)

        above = np.r_[source[:2], clearance]
        yield from move("above", np.r_[source[:2], 0.92], 0.05, 1.5)
        yield from move("approach", grasp, 0.05, 1.8)
        yield from move("grasp", grasp, 0.0, 0.8)
        yield from move("lift", above, 0.0, 1.5)
        if not self.bilateral_grasp:
            raise RuntimeError("Teacher did not establish a bilateral physical grasp")
        # Measure the actual carried offset after lift, including any slip. This
        # informs only the demonstrator; ACT receives current scene observations.
        offset = self.data.site("right_tcp").xpos - self.data.body(self.bottle_id).xpos
        destination = self.target + offset
        staging = np.array([0.33, -0.24, clearance])
        yield from move("clear_sources", staging, 0.0, 1.8)
        staging[2] = 0.94
        yield from move("lower_clear", staging, 0.0, 1.5)
        start_torso = self.data.qpos[self.qids[:4]].copy()
        for frame in range(3 * FPS):
            t = (frame + 1) / (3 * FPS)
            probe.qpos[self.qids[:4]] = start_torso + (self.home[:4] - start_torso) * t * t * (
                3 - 2 * t
            )
            mujoco.mj_forward(self.model, probe)
            goal = kinematics.solve(
                probe,
                {"right": staging},
                torso_yaw_only=True,
                position_tolerance=0.003,
                orientation_tolerance=0.005,
            )
            if np.max(np.abs(goal[:18] - self.data.ctrl[self.aids[:18]])) > 0.14:
                raise RuntimeError("Teacher torso return produced a discontinuous joint command")
            goal[-1] = 0.0
            probe.qpos[self.qids] = goal
            yield "restore_torso", goal.astype(np.float32)
        for _ in range(FPS):
            yield "restore_torso", goal.astype(np.float32)
        # Refresh the held offset after the torso settles.
        offset = self.data.site("right_tcp").xpos - self.data.body(self.bottle_id).xpos
        destination = self.target + offset
        yield from move("transfer", np.r_[destination[:2], 0.94], 0.0, 1.8)
        yield from move("place", destination, 0.0, 1.8)
        for _ in range(8):
            if self.geometry(self.selected)["supported"]:
                break
            destination = destination - np.array([0, 0, 0.003])
            yield from move("seek_support", destination, 0.0, 0.25)
        if not self.geometry(self.selected)["supported"]:
            raise RuntimeError("Teacher found no tray support before release")
        yield from move("release", destination, 0.05, 0.8)
        yield from move("retreat", np.r_[destination[:2], 0.94], 0.05, 1.5)
        yield from move("home", np.asarray(HOME_TCP), 0.05, 1.5)
        initial = self.data.ctrl[self.aids].copy()
        for frame in range(2 * FPS):
            t = (frame + 1) / (2 * FPS)
            yield (
                "home_posture",
                (initial + (self.home - initial) * t * t * (3 - 2 * t)).astype(np.float32),
            )
        for _ in range(2 * FPS):
            yield "settle", self.home.astype(np.float32)
