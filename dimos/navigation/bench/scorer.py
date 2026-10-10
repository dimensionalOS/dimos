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

"""Scores one episode from its recording, by rules fixed in advance."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, TypeVar

import numpy as np
from numpy.typing import NDArray

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sim_msgs.Contacts import Contact, Contacts
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.bench.suite import Point, Rules

Outcome = Literal[
    "success",
    "reached_with_collision",
    "wrong_place",
    "wrong_floor",
    "fall",
    "stuck",
    "stalled",
    "no_plan",
    "timeout",
]

COLLIDING_KINDS = ("wall", "clutter", "ceiling")
T = TypeVar("T")
GOAL_ECHO_M = 0.05


@dataclass(frozen=True)
class Poses:
    t: NDArray[np.float64]
    xyz: NDArray[np.float64]
    rpy: NDArray[np.float64]


@dataclass(frozen=True)
class Commands:
    t: NDArray[np.float64]
    v: NDArray[np.float64]


@dataclass(frozen=True)
class PathSample:
    t: float
    points: NDArray[np.float64]


@dataclass(frozen=True)
class ContactSample:
    t: float
    contacts: list[Contact]


@dataclass(frozen=True)
class Recording:
    """The streams the scorer reads. A stream that was not recorded is None."""

    end: float
    pose: Poses | None = None
    contacts: list[ContactSample] | None = None
    goals: list[tuple[float, Point]] | None = None
    planner_paths: list[PathSample] | None = None
    commands: Commands | None = None
    arrivals: list[float] | None = None

    @classmethod
    def from_store(cls, path: Path) -> Recording:
        store = SqliteStore(path=str(path))
        store.start()
        try:
            names = set(store.list_streams())

            def stamped(name: str, kind: type[T]) -> list[Observation[T]] | None:
                return store.stream(name, kind).to_list() if name in names else None

            end = max(
                (float(store.stream(n).last().ts) for n in names if store.stream(n).exists()),
                default=0.0,
            )
            pose: list[Observation[PoseStamped]] | list[Observation[Odometry]] | None = stamped(
                "ground_truth", PoseStamped
            ) or stamped("odometry", Odometry)
            goals = stamped("goal", PointStamped)
            paths = stamped("planner_path", PathMsg)
            commands = stamped("cmd_vel", Twist)
            arrivals = stamped("goal_reached", Bool)
            contacts = stamped("contacts", Contacts)
            return cls(
                end=end,
                pose=_poses(pose) if pose else None,
                contacts=None
                if contacts is None
                else [ContactSample(float(o.ts), o.data.contacts) for o in contacts],
                goals=None
                if goals is None
                else [(float(o.ts), (o.data.x, o.data.y, o.data.z)) for o in goals],
                planner_paths=None
                if paths is None
                else [
                    PathSample(
                        float(o.ts),
                        np.array([tuple(q.position) for q in o.data.poses]).reshape(-1, 3),
                    )
                    for o in paths
                ],
                commands=None
                if commands is None
                else Commands(
                    np.array([float(o.ts) for o in commands]),
                    np.array(
                        [(o.data.linear.x, o.data.linear.y, o.data.angular.z) for o in commands]
                    ).reshape(-1, 3),
                ),
                arrivals=None
                if arrivals is None
                else [float(o.ts) for o in arrivals if o.data.data],
            )
        finally:
            store.stop()


def _poses(observations: list[Observation[PoseStamped]] | list[Observation[Odometry]]) -> Poses:
    t = np.array([float(o.ts) for o in observations])
    xyz = np.array([tuple(o.data.position) for o in observations]).reshape(-1, 3)
    rpy = np.array([tuple(o.data.orientation.to_euler()) for o in observations]).reshape(-1, 3)
    return Poses(t, xyz, rpy)


@dataclass(frozen=True)
class Score:
    """One episode's outcome and soft metrics. None means not measured."""

    outcome: Outcome
    window: tuple[float, float]
    arrived_s: float | None
    traveled_m: float | None
    spl: float | None
    final_error_xy: float | None
    final_error_z: float | None
    reroutes: int | None
    reroute_rate: float | None
    path_change_p95: float | None
    path_change_max: float | None
    empty_path_s: float | None
    yaw_reversals: int | None
    collisions: int | None
    collision_s: float | None
    trunk_floor_s: float | None
    fell: bool | None
    missing: list[str] = field(default_factory=list)


def score(
    recording: Recording, rules: Rules, goal: Point, route_length: float | None = None
) -> Score:
    """Score the episode in the recording that drives to the goal."""
    if recording.pose is None:
        raise ValueError("a recording needs ground_truth or odometry to be scored")
    missing = [
        name
        for name, stream in (
            ("contacts", recording.contacts),
            ("goal", recording.goals),
            ("planner_path", recording.planner_paths),
            ("cmd_vel", recording.commands),
            ("goal_reached", recording.arrivals),
        )
        if stream is None
    ]
    t0 = _episode_start(recording, goal)
    timeout = rules.timeout_s(route_length)
    deadline = recording.end if timeout is None else min(recording.end, t0 + timeout)
    arrival = _arrival(recording, t0, deadline)
    t1 = arrival if arrival is not None else deadline
    pose = _slice(recording.pose, t0, t1)
    collisions = _collisions(recording.contacts, t0, t1)
    fell = _fell(pose, recording.contacts, rules, t0, t1)
    final = _at(recording.pose, t1)
    error_xy = float(np.hypot(*(final[:2] - goal[:2])))
    error_z = float(abs(final[2] - rules.stand_height_m - goal[2]))
    paths = _path_metrics(recording.planner_paths, rules, t0, t1)
    outcome = _outcome(
        recording, rules, t0, t1, arrival, error_xy, error_z, collisions, fell, paths
    )
    traveled = float(np.linalg.norm(np.diff(pose.xyz[:, :2], axis=0), axis=1).sum())
    spl = None
    if route_length is not None:
        spl = (route_length / max(route_length, traveled)) if outcome == "success" else 0.0
    return Score(
        outcome=outcome,
        window=(t0, t1),
        arrived_s=arrival - t0 if arrival is not None else None,
        traveled_m=traveled,
        spl=spl,
        final_error_xy=error_xy,
        final_error_z=error_z,
        reroutes=paths.reroutes,
        reroute_rate=paths.reroutes / (t1 - t0) if paths.reroutes is not None and t1 > t0 else None,
        path_change_p95=paths.change_p95,
        path_change_max=paths.change_max,
        empty_path_s=paths.empty_s,
        yaw_reversals=_yaw_reversals(recording.commands, rules, t0, t1),
        collisions=collisions.count if collisions else None,
        collision_s=collisions.seconds if collisions else None,
        trunk_floor_s=collisions.trunk_floor_s if collisions else None,
        fell=fell,
        missing=missing,
    )


def _episode_start(recording: Recording, goal: Point) -> float:
    """The first echo of the goal on the goal topic, or the recording's start without one."""
    if recording.goals:
        echoes = [
            t for t, g in recording.goals if np.linalg.norm(np.subtract(g, goal)) <= GOAL_ECHO_M
        ]
        if echoes:
            return min(echoes)
    assert recording.pose is not None
    return float(recording.pose.t[0])


def _arrival(recording: Recording, t0: float, deadline: float) -> float | None:
    """The arrival signal: the follower's goal_reached, the first one after the goal."""
    if recording.arrivals is None:
        return None
    after = [t for t in recording.arrivals if t0 <= t <= deadline]
    return min(after) if after else None


def _slice(pose: Poses, t0: float, t1: float) -> Poses:
    keep = (pose.t >= t0) & (pose.t <= t1)
    if not keep.any():
        keep = np.zeros_like(keep)
        keep[np.argmin(np.abs(pose.t - t0))] = True
    return Poses(pose.t[keep], pose.xyz[keep], pose.rpy[keep])


def _at(pose: Poses, t: float) -> NDArray[np.float64]:
    index = int(np.searchsorted(pose.t, t, side="right")) - 1 if t >= pose.t[0] else 0
    point: NDArray[np.float64] = pose.xyz[index]
    return point


@dataclass(frozen=True)
class _Collisions:
    count: int
    seconds: float
    trunk_floor_s: float


def _collisions(samples: list[ContactSample] | None, t0: float, t1: float) -> _Collisions | None:
    """Collision episodes and time, from the piecewise-constant contact state over the window."""
    if samples is None:
        return None
    before = [s for s in samples if s.t <= t0]
    state = before[-1].contacts if before else []
    colliding, on_trunk = _colliding(state), Contact("trunk", "floor") in state
    count, seconds, trunk_floor = int(colliding), 0.0, 0.0
    t_prev = t0
    for sample in (s for s in samples if t0 < s.t <= t1):
        seconds += (sample.t - t_prev) * colliding
        trunk_floor += (sample.t - t_prev) * on_trunk
        t_prev = sample.t
        now = _colliding(sample.contacts)
        count += int(now and not colliding)
        colliding, on_trunk = now, Contact("trunk", "floor") in sample.contacts
    seconds += (t1 - t_prev) * colliding
    trunk_floor += (t1 - t_prev) * on_trunk
    return _Collisions(count, seconds, trunk_floor)


def _colliding(contacts: list[Contact]) -> bool:
    return any(c.kind in COLLIDING_KINDS for c in contacts)


def _fell(
    pose: Poses, samples: list[ContactSample] | None, rules: Rules, t0: float, t1: float
) -> bool:
    tipped = bool(np.any(np.abs(pose.rpy[:, :2]) > rules.fall_rad))
    on_trunk = samples is not None and any(
        t0 <= s.t <= t1 and Contact("trunk", "floor") in s.contacts for s in samples
    )
    return tipped or on_trunk


@dataclass(frozen=True)
class _Paths:
    any_plan: bool | None
    reroutes: int | None
    change_p95: float | None
    change_max: float | None
    empty_s: float | None


def _path_metrics(samples: list[PathSample] | None, rules: Rules, t0: float, t1: float) -> _Paths:
    if samples is None:
        return _Paths(None, None, None, None, None)
    before = [s for s in samples if s.t <= t0]
    inside = [s for s in samples if t0 < s.t <= t1]
    current = before[-1] if before else PathSample(t0, np.zeros((0, 3)))
    changes: list[float] = []
    empty_s = 0.0
    t_prev = t0
    for sample in [*inside, PathSample(t1, current.points)]:
        if len(current.points) == 0:
            empty_s += sample.t - t_prev
        t_prev = sample.t
        if sample.t < t1 and len(sample.points) and len(current.points):
            changes.append(_path_change(current.points, sample.points))
        if sample.t < t1:
            current = sample
    any_plan = any(len(s.points) for s in [*before[-1:], *inside])
    reroutes = sum(c >= rules.reroute_m for c in changes)
    return _Paths(
        any_plan,
        reroutes,
        float(np.quantile(changes, 0.95)) if changes else 0.0,
        max(changes) if changes else 0.0,
        empty_s,
    )


def _path_change(old: NDArray[np.float64], new: NDArray[np.float64]) -> float:
    """How far the new path strays from the old one: the largest distance to the old path's segments."""
    a, b = old[:-1, :2], old[1:, :2]
    ab = b - a
    spans = (ab**2).sum(axis=1) > 0
    if not spans.any():
        return float(np.linalg.norm(new[:, :2] - old[0, :2], axis=1).max())
    a, ab = a[spans], ab[spans]
    ap = new[:, None, :2] - a[None]
    along = np.clip((ap * ab[None]).sum(axis=2) / (ab**2).sum(axis=1)[None], 0.0, 1.0)
    nearest = a[None] + along[:, :, None] * ab[None]
    d = np.linalg.norm(new[:, None, :2] - nearest, axis=2)
    return float(d.min(axis=1).max())


def _yaw_reversals(commands: Commands | None, rules: Rules, t0: float, t1: float) -> int | None:
    if commands is None:
        return None
    keep = (commands.t >= t0) & (commands.t <= t1)
    w = commands.v[keep, 2]
    w = w[np.abs(w) >= rules.yaw_reversal_rad_s]
    return int(np.sum(np.sign(w[1:]) != np.sign(w[:-1]))) if len(w) > 1 else 0


def _outcome(
    recording: Recording,
    rules: Rules,
    t0: float,
    t1: float,
    arrival: float | None,
    error_xy: float,
    error_z: float,
    collisions: _Collisions | None,
    fell: bool,
    paths: _Paths,
) -> Outcome:
    if fell:
        return "fall"
    if arrival is not None:
        if error_z > rules.goal_z_m:
            return "wrong_floor"
        if error_xy > rules.goal_xy_m:
            return "wrong_place"
        return "reached_with_collision" if collisions and collisions.count else "success"
    if paths.any_plan is False:
        return "no_plan"
    assert recording.pose is not None
    if stuck(recording.pose, recording.commands, rules, t1):
        return "stuck"
    if stalled(recording.commands, rules, t0, t1):
        return "stalled"
    return "timeout"


def moving(commands: Commands, rules: Rules) -> NDArray[np.bool_]:
    speed = np.hypot(commands.v[:, 0], commands.v[:, 1])
    moving: NDArray[np.bool_] = (speed >= rules.moving_cmd) | (
        np.abs(commands.v[:, 2]) >= rules.moving_cmd
    )
    return moving


def stuck(pose: Poses, commands: Commands | None, rules: Rules, t1: float) -> bool:
    """Commanding motion through the final stretch while the body went nowhere."""
    if commands is None:
        return False
    keep = (commands.t >= t1 - rules.stuck_s) & (commands.t <= t1)
    if not keep.any() or moving(commands, rules)[keep].mean() < 0.8:
        return False
    span = (pose.t >= t1 - rules.stuck_s) & (pose.t <= t1)
    if not span.any():
        return False
    start = pose.xyz[span][0, :2]
    return bool(
        np.linalg.norm(pose.xyz[span][:, :2] - start, axis=1).max() < rules.stuck_progress_m
    )


def stalled(commands: Commands | None, rules: Rules, t0: float, t1: float) -> bool:
    """No motion commanded through the final stretch."""
    if commands is None:
        return False
    drove = moving(commands, rules) & (commands.t >= t0) & (commands.t <= t1)
    last = commands.t[drove].max() if drove.any() else t0
    return bool(t1 - last >= rules.stalled_s)
