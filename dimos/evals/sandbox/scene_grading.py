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

"""Grading a Go2 run through a scene from the simulator's ground truth.

Everything here reads the log GroundTruthLog writes (true base pose, contacts, where the people
were) and the scene geometry itself; nothing depends on what the agent built or reports.

Map coverage asks what the robot's Mid-360 could have seen: dim-go2-sim's lidar model is
cast from the true poses against the scene, and compared with what it sees from every
reachable spot on the floor.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from itertools import pairwise
import json
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from dim_go2_sim.lidar import Raycaster

from dimos.constants import CACHE_DIR

# Map cells, metres. Fine enough to tell aisles apart, coarse enough to forgive odometry.
CELL = 0.25
# Obstacle returns between these heights are what a 2D navigation map is made of.
MAP_Z = (0.1, 2.0)
# Pattern points cast per pose: one Mid-360 scan's worth.
POINTS_PER_POSE = 20_000
# Free-floor survey: grid spacing, sensor height, clearance from obstacles, headings.
SURVEY_STEP = 0.5
SENSOR_Z = 0.45
CLEARANCE = 0.35
SURVEY_YAWS = (0.0, np.pi / 2, np.pi, 3 * np.pi / 2)
# Contacts closer together than this are one collision.
COLLISION_GAP_S = 2.0
# ...and so are contacts until the robot has moved this far from where the last one began
# (a dog parked with a leg on a sofa chatters in and out of contact for minutes).
COLLISION_MOVE_M = 0.3
# The kind of contact dim-go2-sim reports for touching a person.
PERSON = "person"
# Bump when the survey changes, so cached references are rebuilt.
SURVEY_VERSION = 2


@dataclass(frozen=True)
class GroundTruth:
    pose_t: NDArray[np.float64]  # (N,) seconds
    poses: NDArray[np.float64]  # (N, 7) x y z qx qy qz qw, base_link in the world
    contact_t: NDArray[np.float64]  # (M,) times the contact set changed
    contacts: list[list[tuple[str, str]]]  # (part, kind) pairs from that time on
    people_t: NDArray[np.float64]  # (K,) seconds
    people: NDArray[np.float64]  # (K, P, 3) x y yaw of each person

    @classmethod
    def load(cls, path: Path) -> GroundTruth:
        pose_t, poses, contact_t, contacts, people_t, people = [], [], [], [], [], []
        lines = path.read_text().splitlines()
        for i, line in enumerate(lines):
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                if i == len(lines) - 1:  # a robot still running may be mid-line
                    break
                raise
            if "pose" in record:
                pose_t.append(record["t"])
                poses.append(record["pose"])
            elif "people" in record:
                people_t.append(record["t"])
                people.append(record["people"])
            else:
                contact_t.append(record["t"])
                contacts.append([tuple(c) for c in record["contacts"]])
        count = len(people[0]) if people else 0
        return cls(
            np.asarray(pose_t, np.float64),
            np.asarray(poses, np.float64).reshape(-1, 7),
            np.asarray(contact_t, np.float64),
            contacts,
            np.asarray(people_t, np.float64),
            np.asarray(people, np.float64).reshape(len(people), count, 3),
        )

    def between(self, start: float, end: float = np.inf) -> NDArray[np.float64]:
        """Poses with start <= t < end."""
        keep = (self.pose_t >= start) & (self.pose_t < end)
        return self.poses[keep]

    def collisions(
        self, start: float = -np.inf, end: float = np.inf, *, people: bool = False
    ) -> list[float]:
        """Times the robot hit anything but floor with anything but its feet; with *people*,
        times it touched a person (with any part) instead.

        A bump chatters in and out of contact; touches within COLLISION_GAP_S of the
        last one, or before the robot has moved COLLISION_MOVE_M from where the last collision
        began, are the same collision.
        """

        def hit(part: str, kind: str) -> bool:
            if people:
                return kind == PERSON
            return kind != PERSON and (part != "foot" or kind != "floor")

        events: list[float] = []
        touching, last_touch, last_at = False, -np.inf, None
        for t, contacts in zip(self.contact_t, self.contacts, strict=True):
            now = any(hit(part, kind) for part, kind in contacts)
            at = self._position_at(t)
            moved = last_at is None or float(np.linalg.norm(at - last_at)) >= COLLISION_MOVE_M
            if (
                now
                and not touching
                and start <= t < end
                and t - last_touch > COLLISION_GAP_S
                and moved
            ):
                events.append(float(t))
                last_at = at
            if now or touching:
                last_touch = float(t)
            touching = now
        return events

    def _position_at(self, t: float) -> NDArray[np.float64]:
        """The robot's x, y at the last pose logged at or before *t* (the first, before any; the
        origin with none)."""
        if not len(self.pose_t):
            return np.zeros(2)
        return self.poses[max(int(np.searchsorted(self.pose_t, t, side="right")) - 1, 0), :2]

    def closest_person(self, start: float = -np.inf) -> float:
        """The nearest the robot's body centre came to a person's, metres, from *start* on."""
        keep = self.people_t >= start
        if not keep.any() or not len(self.pose_t) or not self.people.shape[1]:
            return float("inf")
        # The robot's pose logged nearest after each people record.
        at = np.clip(np.searchsorted(self.pose_t, self.people_t[keep]), 0, len(self.poses) - 1)
        offsets = self.people[keep, :, :2] - self.poses[at, None, :2]
        return float(np.linalg.norm(offsets, axis=2).min())


@dataclass(frozen=True)
class Zone:
    """A circle (center, radius) or an axis-aligned box (x0, y0, x1, y1) on the floor."""

    name: str
    center: tuple[float, float] | None = None
    radius: float = 0.0
    box: tuple[float, float, float, float] | None = None

    def contains(self, xy: NDArray[np.float64]) -> NDArray[np.bool_]:
        if self.center is not None:
            inside: NDArray[np.bool_] = np.hypot(*(xy - np.asarray(self.center)).T) <= self.radius
            return inside
        assert self.box is not None
        x0, y0, x1, y1 = self.box
        return (xy[:, 0] >= x0) & (xy[:, 0] <= x1) & (xy[:, 1] >= y0) & (xy[:, 1] <= y1)


def visits(poses: NDArray[np.float64], zones: Sequence[Zone]) -> list[str]:
    """The zones the robot entered, in order, with repeats of the same zone collapsed."""
    sequence: list[str] = []
    xy = poses[:, :2]
    inside = np.stack([zone.contains(xy) for zone in zones], axis=1)
    for row in inside:
        hits = np.flatnonzero(row)
        if len(hits) and (not sequence or sequence[-1] != zones[hits[0]].name):
            sequence.append(zones[hits[0]].name)
    return sequence


def round_trips(sequence: Iterable[str], a: str, b: str) -> int:
    """Completed a -> b -> a (or b -> a -> b) trips: every two moves between the zones."""
    moves = sum(1 for x, y in pairwise(sequence) if {x, y} == {a, b})
    return moves // 2


class SceneCoverage:
    """What the Go2's Mid-360, as dim-go2-sim casts it, can see of a scene (any dim-go2-sim
    scene.package), from given poses or from everywhere."""

    def __init__(self, package: str, start: tuple[float, float, float]) -> None:
        from dim_go2_sim import LidarConfig
        from dim_go2_sim.lidar import mount_rotation
        from dim_go2_sim.mid360 import Mid360

        x, y, _ = start  # the robot's start, where the reachable floor is flood-filled from
        self.package = package
        self.start = (x, y)
        self.raycaster = scene_raycaster(package)
        lidar = LidarConfig()
        self._mount = (np.array(lidar.xyz), mount_rotation(lidar.pitch_deg))
        self._max_range = lidar.max_range
        device = Mid360(point_rate_hz=lidar.point_rate_hz)
        self._directions = device.scan(0.0, POINTS_PER_POSE / lidar.point_rate_hz).directions

    def seen_from(self, poses: NDArray[np.float64]) -> set[tuple[int, int]]:
        """Map cells the lidar returns obstacles in, from base poses (x y z qx qy qz qw)."""
        mount_xyz, mount_r = self._mount
        cells: set[tuple[int, int]] = set()
        for pose in poses:
            rotation = Rotation.from_quat(pose[3:]).as_matrix()
            origin = pose[:3] + rotation @ mount_xyz
            directions = self._directions @ (rotation @ mount_r).T
            ranges, _ = self.raycaster.cast(origin, directions, self._max_range)
            hit = ranges > 0
            points = origin + directions[hit] * ranges[hit, None]
            band = points[(points[:, 2] >= MAP_Z[0]) & (points[:, 2] <= MAP_Z[1])]
            cells.update(map(tuple, np.floor(band[:, :2] / CELL).astype(int).tolist()))
        return cells

    def reference(self) -> set[tuple[int, int]]:
        """Cells seen from every reachable free spot on the floor: the complete map."""
        x, y = self.start  # the reachable floor is flood-filled from here
        name = f"coverage-{self.package.replace('/', '_')}-{x:.2f}_{y:.2f}-v{SURVEY_VERSION}.npy"
        cached = CACHE_DIR / "evals" / name
        if cached.is_file():
            return set(map(tuple, np.load(cached).tolist()))
        cells = self.seen_from(self._survey_poses())
        cached.parent.mkdir(parents=True, exist_ok=True)
        np.save(cached, np.array(sorted(cells), dtype=np.int64).reshape(-1, 2))
        return cells

    def _survey_poses(self) -> NDArray[np.float64]:
        """Base poses on a grid over the floor reachable from the start, a few headings each."""
        free = self._free_cells()
        poses = []
        for i, j in free:
            for yaw in SURVEY_YAWS:
                q = Rotation.from_euler("z", yaw).as_quat()
                base_z = SENSOR_Z - self._mount[0][2]
                poses.append([i * SURVEY_STEP, j * SURVEY_STEP, base_z, *q])
        return np.asarray(poses)

    def _free_cells(self) -> set[tuple[int, int]]:
        """Grid points with floor under them and room to stand, flood-filled from the start."""
        down = np.array([[0.0, 0.0, -1.0]])
        around = np.array(
            [[np.cos(a), np.sin(a), 0.0] for a in np.linspace(0, 2 * np.pi, 16, endpoint=False)]
        )

        def free(i: int, j: int) -> bool:
            origin = np.array([i * SURVEY_STEP, j * SURVEY_STEP, SENSOR_Z])
            floor, _ = self.raycaster.cast(origin, down, 1.0)
            if not 0 < floor[0] <= SENSOR_Z + 0.05:
                return False
            near, _ = self.raycaster.cast(origin - [0, 0, 0.3], around, CLEARANCE)
            return bool(np.all(near < 0))

        start = (round(self.start[0] / SURVEY_STEP), round(self.start[1] / SURVEY_STEP))
        seen, frontier, cells = {start}, [start], set()
        while frontier:
            i, j = frontier.pop()
            if not free(i, j):
                continue
            cells.add((i, j))
            for step in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (i + step[0], j + step[1])
                if nxt not in seen:
                    seen.add(nxt)
                    frontier.append(nxt)
        return cells


def scene_raycaster(package: str) -> Raycaster:
    """Casts against dim-go2-sim's scene *package* alone, as its lidar does: a cooked package's
    geoms, or a splat scene's surfaces."""
    from dim_go2_sim import PhysicsConfig, SceneConfig
    from dim_go2_sim.lidar import Raycaster
    from dim_go2_sim.module import resolve_scene
    from dim_go2_sim.world import PREFIX, load_world
    import mujoco

    scene = resolve_scene(SceneConfig(package=package))
    model = load_world(scene.mjcf, PhysicsConfig())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    robot = {b for b in range(model.nbody) if model.body(b).name.startswith(PREFIX)}
    return Raycaster(model, data, robot, None, 0, scene.lidar_mesh)
