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

"""Deterministic "populated environment" scenario for DimSim e2e tests.

Loads a fixed config (``fixtures/human_tasks.json``) describing a pool of
predefined everyday tasks (take out garbage, sleep, eat, ...) at named
locations, plus a few human actors. From a single integer ``seed`` it builds a
schedule in which each human repeatedly performs randomly-drawn tasks at
randomly-drawn intervals — "randomly generated" yet identical on every run.

``HumanActivityDriver`` then plays that schedule against a live DimSim scene:
it spawns one NPC per human and, as wall-clock elapses, walks each NPC to its
current task's location. The config, loader, and scheduler are pure and
importable without a running simulator (see the determinism unit test).
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.e2e_tests.dim_sim_client import DimSimClient

logger = setup_logger()


@dataclass(frozen=True)
class Location:
    """A named place in robot/map-frame meters.

    ``furniture`` optionally anchors the location to a named object in the live
    DimSim scene (e.g. ``"bed-mattress"``): at spawn the driver resolves the
    object's real world position and uses it instead of the (x, y) fallback, so
    activities happen at the actual furniture. ``standoff`` is added after
    resolution so actors stand next to the object rather than inside its mesh.
    """

    name: str
    x: float
    y: float
    furniture: str | None = None
    standoff: tuple[float, float] = (0.0, 0.0)

    @classmethod
    def from_dict(cls, name: str, raw: dict[str, Any]) -> Location:
        so = raw.get("standoff", (0.0, 0.0))
        return cls(
            name=str(name),
            x=float(raw["x"]),
            y=float(raw["y"]),
            furniture=str(raw["furniture"]) if raw.get("furniture") else None,
            standoff=(float(so[0]), float(so[1])),
        )


def _parse_color(value: Any) -> int:
    """Accept a hex string ("#37474F" / "0x37474f") or an int; return a 24-bit int."""
    if isinstance(value, int):
        return value
    text = str(value).strip().lstrip("#")
    if text.lower().startswith("0x"):
        text = text[2:]
    return int(text, 16)


@dataclass(frozen=True)
class PropSpec:
    """A colored primitive standing in for the object an activity needs.

    A hermetic stand-in (box / sphere / cylinder) so the scene concretely
    contains a target for every task regardless of the base map. ``size`` follows
    the DimSim ``add_object`` convention: (w, h, d) for a box, (radiusTop,
    radiusBottom, height) for a cylinder, (radius,) for a sphere.
    """

    kind: str
    geometry: str
    size: tuple[float, ...]
    color: int

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> PropSpec:
        geometry = str(raw.get("geometry", "box"))
        size = tuple(float(v) for v in raw["size"])
        if geometry in ("box", "cylinder") and len(size) < 3:
            raise ValueError(f"prop {raw.get('kind')!r}: {geometry} needs a 3-tuple size")
        return cls(
            kind=str(raw["kind"]),
            geometry=geometry,
            size=size,
            color=_parse_color(raw.get("color", "#888888")),
        )

    @property
    def rest_height(self) -> float:
        """Three.js up-offset that seats the prop's base on the floor."""
        if self.geometry == "sphere":
            return self.size[0]
        # box: size[1] is height; cylinder: size[2] is height
        return (self.size[1] if self.geometry == "box" else self.size[2]) / 2.0


@dataclass(frozen=True)
class TaskSpec:
    """One predefined task: what it is, where it happens, how it looks."""

    id: str
    location: str
    animation: str = "idle"
    dwell_s: float = 6.0
    prop: PropSpec | None = None

    @classmethod
    def from_dict(cls, raw: dict[str, Any], *, dwell_default_s: float) -> TaskSpec:
        prop = raw.get("prop")
        return cls(
            id=str(raw["id"]),
            location=str(raw["location"]),
            animation=str(raw.get("animation", "idle")),
            dwell_s=float(raw.get("dwell_s", dwell_default_s)),
            prop=PropSpec.from_dict(prop) if prop else None,
        )


@dataclass(frozen=True)
class HumanSpec:
    """An actor who performs tasks, starting at its ``home`` location."""

    name: str
    home: str

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> HumanSpec:
        return cls(name=str(raw["name"]), home=str(raw["home"]))


@dataclass(frozen=True)
class ScheduledEvent:
    """A resolved timeline entry: ``human`` starts ``task`` at ``at_s`` seconds."""

    at_s: float
    human: str
    task: TaskSpec


@dataclass(frozen=True)
class HumanTaskScenario:
    """The whole fixed scenario, loaded from JSON.

    Mirrors the JSON-backed frozen-dataclass ``from_json`` idiom used across the
    codebase (e.g. ``dimos/experimental/scene_cooking/sidecar.py``).
    """

    seed: int
    duration_s: float
    interval_s_range: tuple[float, float]
    model_url: str
    locations: dict[str, Location]
    tasks: tuple[TaskSpec, ...]
    humans: tuple[HumanSpec, ...]
    dwell_default_s: float = 6.0

    @classmethod
    def from_json(cls, path: str | Path) -> HumanTaskScenario:
        p = Path(path).expanduser().resolve()
        raw = json.loads(p.read_text())
        dwell_default_s = float(raw.get("dwell_default_s", 6.0))
        lo, hi = raw.get("interval_s_range", [4.0, 12.0])
        locations = {
            name: Location.from_dict(name, loc) for name, loc in raw["locations"].items()
        }
        tasks = tuple(
            TaskSpec.from_dict(t, dwell_default_s=dwell_default_s) for t in raw["tasks"]
        )
        humans = tuple(HumanSpec.from_dict(h) for h in raw["humans"])
        if not tasks:
            raise ValueError(f"{p}: scenario defines no tasks")
        if not humans:
            raise ValueError(f"{p}: scenario defines no humans")
        unknown = {t.location for t in tasks} | {h.home for h in humans}
        unknown -= set(locations)
        if unknown:
            raise ValueError(f"{p}: tasks/humans reference unknown locations {sorted(unknown)}")
        return cls(
            seed=int(raw.get("seed", 0)),
            duration_s=float(raw.get("duration_s", 90.0)),
            interval_s_range=(float(lo), float(hi)),
            model_url=str(raw.get("model_url", "/agent-model/robot.glb")),
            locations=locations,
            tasks=tasks,
            humans=humans,
            dwell_default_s=dwell_default_s,
        )

    def location_of(self, task: TaskSpec) -> Location:
        return self.locations[task.location]

    def build_schedule(self) -> tuple[ScheduledEvent, ...]:
        """Deterministically expand the scenario into a sorted event timeline.

        Every human walks its own clock: draw a random task and a random
        gap in ``interval_s_range``, advance, repeat until ``duration_s``.
        Seeded by ``self.seed``, so the "random" schedule is identical on every
        call and every run. Humans are drawn from a per-human child generator so
        adding/removing a human doesn't reshuffle the others.
        """
        root = np.random.default_rng(self.seed)
        lo, hi = self.interval_s_range
        events: list[ScheduledEvent] = []
        for human in self.humans:
            # a stable child stream per human (order-independent)
            rng = np.random.default_rng(root.integers(0, 2**63 - 1))
            t = float(rng.uniform(lo, hi))
            while t < self.duration_s:
                task = self.tasks[int(rng.integers(0, len(self.tasks)))]
                events.append(ScheduledEvent(at_s=round(t, 3), human=human.name, task=task))
                t += float(rng.uniform(lo, hi))
        events.sort(key=lambda e: (e.at_s, e.human))
        return tuple(events)


class HumanActivityDriver:
    """Plays a scenario's schedule against a live DimSim scene.

    Spawns one NPC per human at its home location, then as ``elapsed_s``
    advances, walks each NPC toward the location of its most-recently-started
    task. Movement is incremental (a fixed step per ``step`` call) so the actors
    visibly travel between rooms rather than teleporting.
    """

    _speed_mps = 1.4  # human walking speed

    def __init__(self, dim_sim: DimSimClient, scenario: HumanTaskScenario) -> None:
        self._dim_sim = dim_sim
        self._scenario = scenario
        self._schedule = scenario.build_schedule()
        self._next_idx = 0
        # per-human current position and goal (robot frame)
        self._pos: dict[str, np.ndarray] = {}
        self._goal: dict[str, np.ndarray] = {}
        self._current_task: dict[str, TaskSpec] = {}
        self._prop_names: list[str] = []
        self._spawned = False
        self._last_step_s = -1.0
        # location name -> resolved robot-frame (x, y); filled at spawn from the
        # live scene's furniture anchors (config coords are the fallback)
        self._resolved: dict[str, tuple[float, float]] = {}
        self._fallback_humans: set[str] = set()  # humans whose GLB failed to load

    @property
    def dim_sim(self) -> DimSimClient:
        return self._dim_sim

    @property
    def prop_names(self) -> tuple[str, ...]:
        return tuple(self._prop_names)

    @property
    def resolved_locations(self) -> dict[str, tuple[float, float]]:
        return dict(self._resolved)

    @property
    def fallback_humans(self) -> set[str]:
        """Humans spawned as fallback cylinders because the person GLB failed."""
        return set(self._fallback_humans)

    def position_of(self, location_name: str) -> tuple[float, float]:
        """Resolved robot-frame position of a location (config fallback if unresolved)."""
        if location_name in self._resolved:
            return self._resolved[location_name]
        loc = self._scenario.locations[location_name]
        return (loc.x, loc.y)

    def resolve_locations(self) -> None:
        """Snap each furniture-anchored location to the real object's live position."""
        for name, loc in self._scenario.locations.items():
            xy = (loc.x, loc.y)
            if loc.furniture:
                try:
                    fx, fy = self._dim_sim.get_object_position(loc.furniture)
                    xy = (fx + loc.standoff[0], fy + loc.standoff[1])
                    logger.info(
                        f"location '{name}' anchored to '{loc.furniture}' at "
                        f"({xy[0]:.2f}, {xy[1]:.2f})"
                    )
                except Exception as e:
                    logger.warning(
                        f"location '{name}': anchor '{loc.furniture}' not found ({e}) — "
                        f"using config fallback ({loc.x}, {loc.y})"
                    )
            self._resolved[name] = xy

    @property
    def schedule(self) -> tuple[ScheduledEvent, ...]:
        return self._schedule

    def current_task(self, human: str) -> TaskSpec | None:
        return self._current_task.get(human)

    def human_names(self) -> list[str]:
        return [h.name for h in self._scenario.humans]

    def live_position(self, name: str) -> tuple[float, float] | None:
        """Current robot-frame (x, y) of a spawned human, or None."""
        p = self._pos.get(name)
        return (float(p[0]), float(p[1])) if p is not None else None

    def spawn_props(self) -> None:
        """Place one prop per task at its location, so each activity has a target.

        Props sharing a room are fanned out on a small deterministic ring so they
        don't overlap (e.g. the kitchen gets both a sink and a stove).
        """
        for i, task in enumerate(self._scenario.tasks):
            if task.prop is None:
                continue
            lx, ly = self.position_of(task.location)
            angle = 2.0 * np.pi * i / max(len(self._scenario.tasks), 1)
            px = lx + 0.6 * float(np.cos(angle))
            py = ly + 0.6 * float(np.sin(angle))
            name = f"prop_{task.id}"
            self._dim_sim.add_prop(
                name,
                px,
                py,
                geometry=task.prop.geometry,
                size=task.prop.size,
                color=task.prop.color,
                up=task.prop.rest_height,
            )
            self._prop_names.append(name)

    # distinct colors so the actors are tellable apart in the recording
    _HUMAN_COLORS = (0xE0A070, 0x4FA3D1, 0x8BC34A, 0xD16BA5, 0xF2C14E, 0x9C7BE0)

    def spawn(self) -> None:
        """Resolve real furniture anchors, then spawn props and human actors."""
        self.resolve_locations()
        self.spawn_props()
        for i, human in enumerate(self._scenario.humans):
            hx, hy = self.position_of(human.home)
            self._pos[human.name] = np.array([hx, hy], dtype=float)
            self._goal[human.name] = np.array([hx, hy], dtype=float)
            result = self._dim_sim.add_human(
                human.name,
                hx,
                hy,
                model_url=self._scenario.model_url,
                color=self._HUMAN_COLORS[i % len(self._HUMAN_COLORS)],
            )
            if result.get("fallback"):
                self._fallback_humans.add(human.name)
                logger.warning(f"{human.name}: person model failed to load — cylinder fallback")
        # No lidar-snapshot refresh: the actors are visual-only (camera sees
        # them; they are deliberately absent from the physics/lidar world).
        self._spawned = True

    def step(self, elapsed_s: float, dt_s: float) -> None:
        """Apply any events due by ``elapsed_s`` and advance each NPC.

        Movement uses the *real* elapsed delta between calls (each call blocks
        on synchronous scene execs, so the loop can run well under its nominal
        rate — with a fixed ``dt_s`` the humans would walk slower than
        ``_speed_mps``). ``dt_s`` remains an upper-bound hint for the first call.
        """
        if not self._spawned:
            return
        real_dt = min(elapsed_s - self._last_step_s, 1.0) if self._last_step_s >= 0 else dt_s
        self._last_step_s = elapsed_s
        dt_s = max(real_dt, 0.0)
        # activate any events whose time has arrived
        while (
            self._next_idx < len(self._schedule)
            and self._schedule[self._next_idx].at_s <= elapsed_s
        ):
            event = self._schedule[self._next_idx]
            self._next_idx += 1
            gx, gy = self.position_of(event.task.location)
            self._goal[event.human] = np.array([gx, gy], dtype=float)
            self._current_task[event.human] = event.task

        # walk every NPC toward its goal
        for name, pos in self._pos.items():
            goal = self._goal[name]
            delta = goal - pos
            dist = float(np.linalg.norm(delta))
            if dist < 1e-3:
                continue
            move = min(self._speed_mps * dt_s, dist)
            pos += delta / dist * move
            self._dim_sim.move_human(name, float(pos[0]), float(pos[1]))
        # NOTE: no physics-snapshot refresh here — the actors are visual-only
        # (collider=False), so resending the 26 MB Rapier snapshot would be pure
        # overhead that stalls this loop (and starves the walking speed).

    def teardown(self) -> None:
        for human in self._scenario.humans:
            try:
                self._dim_sim.remove_human(human.name)
            except Exception:  # best-effort cleanup
                pass
        for name in self._prop_names:
            try:
                self._dim_sim.remove_prop(name)
            except Exception:  # best-effort cleanup
                pass


def _rgb(color: int) -> tuple[int, int, int]:
    return ((color >> 16) & 255, (color >> 8) & 255, color & 255)


def write_activity_rerun(
    path: str | Path,
    scenario: HumanTaskScenario,
    dt_s: float = 0.2,
    tail_s: float = 15.0,
    robot_trajectory: list[tuple[float, float, float]] | None = None,
    locations_override: dict[str, tuple[float, float]] | None = None,
) -> Path:
    """Write a ground-truth activity recording you can scrub in dimos-viewer.

    Unlike the robot-observed ``.rrd`` (the robot's egocentric camera + lidar),
    this is a fixed-world top-down/3D view of the *actors*: each human is a
    moving colored sphere labeled with its current task, over the labeled task
    rooms and their props. The human motion is pure and deterministic (recomputes
    the same walk the driver drives in-sim), so no simulator is needed.

    Pass ``robot_trajectory`` (``[(elapsed_s, x, y), ...]``, e.g.
    ``ActivityPatrol.trajectory``) to overlay the robot's *real* patrol path as a
    moving marker + trail — so you can watch the robot drive among the humans it
    is observing, all in one fixed world.

    Open with: ``dimos-viewer <path>`` (or ``rerun <path>``).
    """
    import rerun as rr

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rr.init("dimos_human_activity")
    rr.save(str(path))

    resolved = dict(locations_override or {})

    def _xy(location_name: str) -> tuple[float, float]:
        if location_name in resolved:
            return resolved[location_name]
        loc = scenario.locations[location_name]
        return (loc.x, loc.y)

    for name, loc in scenario.locations.items():
        lx, ly = _xy(name)
        label = f"{name} ({loc.furniture})" if loc.furniture else name
        rr.log(
            f"rooms/{name}",
            rr.Points3D([[lx, ly, 0.0]], labels=[label],
                        colors=[(90, 90, 90)], radii=[0.12]),
            static=True,
        )
    for task in scenario.tasks:
        if task.prop is None:
            continue
        lx, ly = _xy(task.location)
        p = task.prop
        if p.geometry == "cylinder":
            half = [p.size[0], p.size[2] / 2.0, p.size[0]]
        elif p.geometry == "sphere":
            half = [p.size[0]] * 3
        else:
            half = [p.size[0] / 2.0, p.size[1] / 2.0, p.size[2] / 2.0]
        rr.log(
            f"props/{task.id}",
            rr.Boxes3D(centers=[[lx, ly, p.rest_height]], half_sizes=[half],
                       labels=[p.kind], colors=[_rgb(p.color)]),
            static=True,
        )

    # deterministic constant-speed walk (mirrors HumanActivityDriver.step)
    schedule = scenario.build_schedule()
    humans = scenario.humans
    colors = HumanActivityDriver._HUMAN_COLORS
    pos = {h.name: np.array(_xy(h.home), float) for h in humans}
    goal = {h.name: pos[h.name].copy() for h in humans}
    current = {h.name: "idle" for h in humans}
    hcolor = {h.name: _rgb(colors[i % len(colors)]) for i, h in enumerate(humans)}
    speed = HumanActivityDriver._speed_mps

    idx = 0
    t = 0.0
    total = scenario.duration_s + tail_s
    while t <= total:
        while idx < len(schedule) and schedule[idx].at_s <= t:
            event = schedule[idx]
            idx += 1
            goal[event.human] = np.array(_xy(event.task.location), float)
            current[event.human] = event.task.id
        rr.set_time("elapsed", duration=t)
        for h in humans:
            p_ = pos[h.name]
            delta = goal[h.name] - p_
            dist = float(np.linalg.norm(delta))
            if dist > 1e-3:
                p_ += delta / dist * min(speed * dt_s, dist)
            rr.log(
                f"humans/{h.name}",
                rr.Points3D([[p_[0], p_[1], 0.85]], radii=[0.25], colors=[hcolor[h.name]],
                            labels=[f"{h.name}: {current[h.name]}"]),
            )
        t += dt_s

    # the robot's real patrol path (moving marker + growing trail), or a static
    # spawn marker if no trajectory was captured
    if robot_trajectory:
        trail: list[list[float]] = []
        for elapsed, rx, ry in robot_trajectory:
            trail.append([rx, ry, 0.15])
            rr.set_time("elapsed", duration=elapsed)
            rr.log(
                "robot",
                rr.Points3D([[rx, ry, 0.2]], labels=["robot"], colors=[(240, 240, 240)],
                            radii=[0.22]),
            )
            rr.log("robot/trail", rr.LineStrips3D([trail], colors=[(200, 200, 200)]))
    else:
        rr.log(
            "robot",
            rr.Points3D([[3.0, 2.0, 0.2]], labels=["robot (stationary)"],
                        colors=[(240, 240, 240)], radii=[0.22]),
            static=True,
        )

    logger.info(f"Wrote activity ground-truth recording: {path}")
    return path
