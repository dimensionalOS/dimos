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

if TYPE_CHECKING:
    from dimos.e2e_tests.dim_sim_client import DimSimClient


@dataclass(frozen=True)
class Location:
    """A named place in robot/map-frame meters."""

    name: str
    x: float
    y: float

    @classmethod
    def from_dict(cls, name: str, raw: dict[str, Any]) -> Location:
        return cls(name=str(name), x=float(raw["x"]), y=float(raw["y"]))


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
        self._lidar_refresh_s = 3.0  # keep server-side lidar roughly in sync
        self._last_lidar_refresh = 0.0

    @property
    def dim_sim(self) -> DimSimClient:
        return self._dim_sim

    @property
    def prop_names(self) -> tuple[str, ...]:
        return tuple(self._prop_names)

    @property
    def schedule(self) -> tuple[ScheduledEvent, ...]:
        return self._schedule

    def current_task(self, human: str) -> TaskSpec | None:
        return self._current_task.get(human)

    def spawn_props(self) -> None:
        """Place one prop per task at its location, so each activity has a target.

        Props sharing a room are fanned out on a small deterministic ring so they
        don't overlap (e.g. the kitchen gets both a sink and a stove).
        """
        for i, task in enumerate(self._scenario.tasks):
            if task.prop is None:
                continue
            loc = self._scenario.location_of(task)
            angle = 2.0 * np.pi * i / max(len(self._scenario.tasks), 1)
            px = loc.x + 0.6 * float(np.cos(angle))
            py = loc.y + 0.6 * float(np.sin(angle))
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

    def spawn(self) -> None:
        """Add the task props, then one NPC per human at its home location."""
        self.spawn_props()
        for human in self._scenario.humans:
            home = self._scenario.locations[human.home]
            self._pos[human.name] = np.array([home.x, home.y], dtype=float)
            self._goal[human.name] = np.array([home.x, home.y], dtype=float)
            self._dim_sim.add_human(
                human.name,
                home.x,
                home.y,
                model_url=self._scenario.model_url,
                animation="idle",
            )
        # so a lidar-based map/perception sees the new geometry too (the camera
        # renders NPCs live regardless)
        try:
            self._dim_sim.refresh_lidar()
        except Exception:  # best-effort — camera perception is unaffected
            pass
        self._spawned = True

    def step(self, elapsed_s: float, dt_s: float) -> None:
        """Apply any events due by ``elapsed_s`` and advance each NPC by ``dt_s``."""
        if not self._spawned:
            return
        # activate any events whose time has arrived
        while (
            self._next_idx < len(self._schedule)
            and self._schedule[self._next_idx].at_s <= elapsed_s
        ):
            event = self._schedule[self._next_idx]
            self._next_idx += 1
            loc = self._scenario.location_of(event.task)
            self._goal[event.human] = np.array([loc.x, loc.y], dtype=float)
            self._current_task[event.human] = event.task

        # walk every NPC toward its goal
        moved = False
        for name, pos in self._pos.items():
            goal = self._goal[name]
            delta = goal - pos
            dist = float(np.linalg.norm(delta))
            if dist < 1e-3:
                continue
            move = min(self._speed_mps * dt_s, dist)
            pos += delta / dist * move
            self._dim_sim.move_human(name, float(pos[0]), float(pos[1]))
            moved = True

        # periodically resync the lidar snapshot so it tracks the moving humans
        if moved and elapsed_s - self._last_lidar_refresh >= self._lidar_refresh_s:
            self._last_lidar_refresh = elapsed_s
            try:
                self._dim_sim.refresh_lidar()
            except Exception:  # best-effort
                pass

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
