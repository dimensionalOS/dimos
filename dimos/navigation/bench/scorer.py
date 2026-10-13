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

from collections import Counter, defaultdict
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Literal, TypeVar, cast, get_args

import numpy as np
from numpy.typing import NDArray

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sim_msgs.Contacts import Contact, Contacts
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.bench.ground_truth import GO2
from dimos.navigation.bench.suite import Point, Rules
from dimos.simulation.scenes.procedural import Scene

Outcome = Literal[
    "success",
    "reached_with_collision",
    "unclaimed",
    "wrong_place",
    "wrong_floor",
    "fall",
    "stuck",
    "stalled",
    "no_plan",
    "timeout",
]
DEFINITIONS: dict[Outcome, str] = {
    "success": "The stack reported arrival with the body within {goal_xy_m:g} m of the goal and "
    "{goal_z_m:g} m of its height, and the body touched no wall or clutter.",
    "reached_with_collision": "As success, but the body touched a wall or clutter on the way.",
    "unclaimed": "The body ended within {goal_xy_m:g} m of the goal and {goal_z_m:g} m of its "
    "height, and the stack never reported arrival.",
    "wrong_place": "The stack reported arrival more than {goal_xy_m:g} m from the goal.",
    "wrong_floor": "The stack reported arrival more than {goal_z_m:g} m above or below the goal.",
    "fall": "The body tilted more than {fall_rad:g} rad in roll or pitch, or its trunk lay on the "
    "floor for {fall_trunk_floor_s:g} s or more. Checked before every other outcome.",
    "stuck": "No arrival. Motion was commanded for at least {stuck_moving_fraction:.0%} of the "
    "last {stuck_s:g} s and the body moved less than {stuck_progress_m:g} m.",
    "stalled": "No arrival, and no motion was commanded for the last {stalled_s:g} s.",
    "no_plan": "No arrival, and the global planner published no path.",
    "timeout": "No arrival, and the time budget ran out: {timeout_base_s:g} s plus "
    "{timeout_s_per_m:g} s per meter of route.",
}

Signature = Literal[
    "planner_empty",
    "local_refused",
    "follower_zeroed",
    "commanded_no_progress",
    "body_fell",
    "out_of_time",
    "arrival_mismatch",
    "pushing",
]
COLLIDING_KINDS = ("wall", "clutter")
TRUNK_ON_FLOOR = Contact("trunk", "floor")
T = TypeVar("T")
GOAL_ECHO_M = 0.05
TIMEOUT_SLACK_S = 0.2
PUSHING_FRACTION = 0.5
CONTACT_SPEED_S = 0.2
ARRIVED = ("success", "reached_with_collision", "wrong_place", "wrong_floor")
GAVE_UP = ("stuck", "stalled", "timeout")


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
    local_paths: list[PathSample] | None = None
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
            pose = stamped("ground_truth", PoseStamped)
            goals = stamped("goal", PointStamped)
            paths = stamped("planner_path", PathMsg)
            local = stamped("path", PathMsg)
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
                planner_paths=_path_samples(paths),
                local_paths=_path_samples(local),
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


def _poses(observations: list[Observation[PoseStamped]]) -> Poses:
    t = np.array([float(o.ts) for o in observations])
    xyz = np.array([tuple(o.data.position) for o in observations]).reshape(-1, 3)
    rpy = np.array([tuple(o.data.orientation.to_euler()) for o in observations]).reshape(-1, 3)
    return Poses(t, xyz, rpy)


@dataclass(frozen=True)
class Score:
    """One episode's outcome and soft metrics. None means not measured."""

    outcome: Outcome
    signature: Signature | None
    signatures: list[Signature]
    window: tuple[float, float]
    arrived_s: float | None
    traveled_m: float | None
    spl: float | None
    final_xy: tuple[float, float]
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
    reached: bool
    claimed: bool
    mismatch: bool = False
    feature_m: float | None = None
    contacts: dict[str, int] = field(default_factory=dict)
    first_contact: str | None = None
    first_contact_s: float | None = None
    first_contact_speed: float | None = None
    margin_m: float | None = None
    margin_s: float | None = None
    final_margin_m: float | None = None
    pushing_s: float | None = None
    pushing_on: str | None = None
    missing: list[str] = field(default_factory=list)


def score(
    recording: Recording,
    rules: Rules,
    goal: Point,
    route_length: float | None = None,
    feature: tuple[float, float] | None = None,
    verdict: str | None = None,
    scene: Scene | None = None,
) -> Score:
    """Score the episode in the recording that drives to the goal.

    With a feature, also the closest the body's center came to it.
    With the driver's verdict, a recording that shows no outcome takes it.
    With the scene, also the body's margin to a wall or clutter, at its smallest and at the end.
    """
    if recording.pose is None:
        raise ValueError("a recording needs ground_truth to be scored")
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
    fell = _fell(pose, collisions, rules)
    final = _at(recording.pose, t1)
    error_xy = float(np.hypot(*(final[:2] - goal[:2])))
    error_z = float(abs(final[2] - rules.stand_height_m - goal[2]))
    paths = _path_metrics(recording.planner_paths, rules, t0, t1)
    shown = _outcome(recording, rules, t0, t1, arrival, error_xy, error_z, collisions, fell, paths)
    if shown is None and timeout is not None and t1 - t0 >= timeout - TIMEOUT_SLACK_S:
        shown = "timeout"
    outcome, mismatch = _settle(shown, verdict)
    traveled = float(np.linalg.norm(np.diff(pose.xyz[:, :2], axis=0), axis=1).sum())
    touched = collisions.first if collisions else None
    spl = None
    if route_length is not None:
        spl = (route_length / max(route_length, traveled)) if outcome == "success" else 0.0
    margins = _margins(pose, scene) if scene is not None else None
    closest = int(np.argmin(margins)) if margins is not None else None
    final_contacts = _collisions(recording.contacts, t1 - rules.stuck_s, t1)
    pushing_s = final_contacts.seconds if final_contacts else None
    pushed = _held_longest(recording.contacts or [], t1 - rules.stuck_s, t1)
    signatures = _signatures(
        _Episode(outcome, recording.local_paths, t0, t1, pushing_s=pushing_s, stuck_s=rules.stuck_s)
    )
    return Score(
        outcome=outcome,
        signature=signatures[0] if signatures else None,
        signatures=signatures,
        window=(t0, t1),
        arrived_s=arrival - t0 if arrival is not None else None,
        traveled_m=traveled,
        spl=spl,
        final_xy=(float(final[0]), float(final[1])),
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
        reached=not fell and _at_goal(rules, error_xy, error_z),
        claimed=arrival is not None,
        mismatch=mismatch,
        feature_m=None
        if feature is None
        else float(np.linalg.norm(pose.xyz[:, :2] - feature, axis=1).min()),
        contacts={_name(c): n for c, n in sorted(collisions.started.items())} if collisions else {},
        first_contact=_name(touched[1]) if touched else None,
        first_contact_s=touched[0] - t0 if touched else None,
        first_contact_speed=_speed(recording.pose, touched[0]) if touched else None,
        margin_m=float(margins[closest]) if margins is not None else None,
        margin_s=float(pose.t[closest] - t0) if closest is not None else None,
        final_margin_m=float(margins[-1]) if margins is not None else None,
        pushing_s=pushing_s,
        pushing_on=_name(pushed) if pushed else None,
        missing=missing,
    )


def definitions(rules: Rules) -> dict[Outcome, str]:
    """What each outcome means, with the limits these rules set."""
    return {outcome: text.format(**asdict(rules)) for outcome, text in DEFINITIONS.items()}


def _path_samples(observations: list[Observation[PathMsg]] | None) -> list[PathSample] | None:
    if observations is None:
        return None
    return [
        PathSample(float(o.ts), np.array([tuple(q.position) for q in o.data.poses]).reshape(-1, 3))
        for o in observations
    ]


@dataclass(frozen=True)
class _Episode:
    """What a signature check sees of a scored episode."""

    outcome: Outcome
    local_paths: list[PathSample] | None
    t0: float
    t1: float
    pushing_s: float | None
    stuck_s: float


def _last_local_path(episode: _Episode) -> int | None:
    """The number of poses in the last local path of the episode, or None without one."""
    inside = [p for p in episode.local_paths or [] if episode.t0 <= p.t <= episode.t1]
    return len(inside[-1].points) if inside else None


def _local_refused(episode: _Episode) -> bool:
    poses = _last_local_path(episode)
    return episode.outcome == "stalled" and poses is not None and poses <= 1


def _follower_zeroed(episode: _Episode) -> bool:
    poses = _last_local_path(episode)
    return episode.outcome == "stalled" and poses is not None and poses > 1


def _pushing(episode: _Episode) -> bool:
    held = episode.pushing_s
    return (
        episode.outcome == "stuck"
        and held is not None
        and held >= PUSHING_FRACTION * episode.stuck_s
    )


CHECKS: tuple[tuple[Signature, Callable[[_Episode], bool]], ...] = (
    ("body_fell", lambda e: e.outcome == "fall"),
    ("arrival_mismatch", lambda e: e.outcome in ("unclaimed", "wrong_place", "wrong_floor")),
    ("planner_empty", lambda e: e.outcome == "no_plan"),
    ("commanded_no_progress", lambda e: e.outcome == "stuck"),
    ("out_of_time", lambda e: e.outcome == "timeout"),
    ("local_refused", _local_refused),
    ("follower_zeroed", _follower_zeroed),
    ("pushing", _pushing),
)


def _signatures(episode: _Episode) -> list[Signature]:
    """Every check that applies to a failed episode, the primary cause first."""
    if episode.outcome in ("success", "reached_with_collision"):
        return []
    return [name for name, applies in CHECKS if applies(episode)]


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
    started: Counter[Contact]
    first: tuple[float, Contact] | None


def _collisions(samples: list[ContactSample] | None, t0: float, t1: float) -> _Collisions | None:
    """Collision episodes and time, from the piecewise-constant contact state over the window."""
    if samples is None:
        return None
    before = [s for s in samples if s.t <= t0]
    state = before[-1].contacts if before else []
    held, on_trunk = _colliding(state), TRUNK_ON_FLOOR in state
    count, seconds, trunk_floor = int(bool(held)), 0.0, 0.0
    started = Counter(held)
    first = (t0, min(held)) if held else None
    t_prev = t0
    for sample in (s for s in samples if t0 < s.t <= t1):
        seconds += (sample.t - t_prev) * bool(held)
        trunk_floor += (sample.t - t_prev) * on_trunk
        t_prev = sample.t
        now = _colliding(sample.contacts)
        count += int(bool(now) and not held)
        started.update(now - held)
        if first is None and now:
            first = (sample.t, min(now))
        held, on_trunk = now, TRUNK_ON_FLOOR in sample.contacts
    seconds += (t1 - t_prev) * bool(held)
    trunk_floor += (t1 - t_prev) * on_trunk
    return _Collisions(count, seconds, trunk_floor, started, first)


def _held_longest(samples: list[ContactSample], t0: float, t1: float) -> Contact | None:
    """The colliding contact held for the most time over the window."""
    seconds: defaultdict[Contact, float] = defaultdict(float)
    before = [s for s in samples if s.t <= t0]
    held, t_prev = _colliding(before[-1].contacts if before else []), t0
    for sample in [*(s for s in samples if t0 < s.t <= t1), ContactSample(t1, [])]:
        for contact in held:
            seconds[contact] += sample.t - t_prev
        held, t_prev = _colliding(sample.contacts), sample.t
    return max(sorted(seconds), key=seconds.__getitem__, default=None)


def _colliding(contacts: list[Contact]) -> set[Contact]:
    return {c for c in contacts if c.kind in COLLIDING_KINDS}


def _name(contact: Contact) -> str:
    return f"{contact.part} {contact.kind}"


def _speed(pose: Poses, t: float) -> float | None:
    """The body's ground speed over the moment before t."""
    span = _slice(pose, t - CONTACT_SPEED_S, t)
    if len(span.t) < 2 or span.t[-1] == span.t[0]:
        return None
    steps = np.linalg.norm(np.diff(span.xyz[:, :2], axis=0), axis=1)
    return float(steps.sum() / (span.t[-1] - span.t[0]))


def _margins(pose: Poses, scene: Scene) -> NDArray[np.float64] | None:
    """Per pose, the horizontal gap from the body to the nearest wall or clutter box at its height."""
    boxes = [box for box in scene.boxes if box.kind in COLLIDING_KINDS]
    if not boxes:
        return None
    center, half = np.array([b.center for b in boxes]), np.array([b.half for b in boxes])
    outside = np.maximum(np.abs(pose.xyz[:, None, :2] - center[:, :2]) - half[:, :2], 0.0)
    foot = pose.xyz[:, None, 2] - GO2.stand
    level = (center[:, 2] + half[:, 2] > foot) & (center[:, 2] - half[:, 2] < foot + GO2.height)
    gap = np.where(level, np.hypot(outside[..., 0], outside[..., 1]), np.inf).min(axis=1)
    if not np.isfinite(gap).all():
        return None
    margins: NDArray[np.float64] = gap - GO2.radius
    return margins


def _fell(pose: Poses, collisions: _Collisions | None, rules: Rules) -> bool:
    tipped = bool(np.any(np.abs(pose.rpy[:, :2]) > rules.fall_rad))
    on_trunk = collisions is not None and collisions.trunk_floor_s >= rules.fall_trunk_floor_s
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
) -> Outcome | None:
    """What the recording shows, or None when no rule applies."""
    if fell:
        return "fall"
    if arrival is not None:
        if error_z > rules.goal_z_m:
            return "wrong_floor"
        if error_xy > rules.goal_xy_m:
            return "wrong_place"
        return "reached_with_collision" if collisions and collisions.count else "success"
    if _at_goal(rules, error_xy, error_z):
        return "unclaimed"
    if paths.any_plan is False:
        return "no_plan"
    assert recording.pose is not None
    if stuck(recording.pose, recording.commands, rules, t1):
        return "stuck"
    if stalled(recording.commands, rules, t0, t1):
        return "stalled"
    return None


def _settle(shown: Outcome | None, verdict: str | None) -> tuple[Outcome, bool]:
    """The outcome, and whether the driver's verdict disagrees with what the recording shows."""
    if verdict is None:
        return shown or "timeout", False
    if shown is None:
        if verdict not in get_args(Outcome):
            raise ValueError(f"the driver's verdict {verdict} is not in the recording")
        return cast("Outcome", verdict), True
    if verdict == "arrived":
        return shown, shown not in ARRIVED
    if shown in ("unclaimed", "no_plan"):
        return shown, verdict not in GAVE_UP
    return shown, verdict != shown


def _at_goal(rules: Rules, error_xy: float, error_z: float) -> bool:
    return error_xy <= rules.goal_xy_m and error_z <= rules.goal_z_m


def moving(commands: Commands, rules: Rules) -> NDArray[np.bool_]:
    speed = np.hypot(commands.v[:, 0], commands.v[:, 1])
    commanded: NDArray[np.bool_] = (speed >= rules.moving_cmd) | (
        np.abs(commands.v[:, 2]) >= rules.moving_cmd
    )
    return commanded


def stuck(pose: Poses, commands: Commands | None, rules: Rules, t1: float) -> bool:
    """Commanding motion through the final stretch while the body went nowhere."""
    if commands is None:
        return False
    keep = (commands.t >= t1 - rules.stuck_s) & (commands.t <= t1)
    if not keep.any() or moving(commands, rules)[keep].mean() < rules.stuck_moving_fraction:
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
