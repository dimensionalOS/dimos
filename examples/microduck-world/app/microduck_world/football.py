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

"""Physical footballs, directional goal detection, and the room's scoreboard."""

import math
import time
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np
from dimos.robot.pollen.microduck.places import BALL_BODY
from microduck_world.ball_physics import BALL_RADIUS, configure_contacts
from numpy.typing import NDArray

TEAMS = ("blue", "coral")
BALL_NAMES = (BALL_BODY, "football_ball_1", "football_ball_2", "football_ball_3")
DIGITS = ("abcdef", "bc", "abdeg", "abcdg", "bcfg", "acdfg", "acdefg", "abc", "abcdefg", "abcdfg")
OFF_COLOR = (0.018, 0.025, 0.024)
GROUND_TOLERANCE = 0.003  # MuJoCo soft-contact penetration at the floor.


def add_footballs(spec: mujoco.MjSpec) -> None:
    """Attach free bodies after the robots so the host remains the first joint."""
    for index, name in enumerate(BALL_NAMES[1:], 1):
        spawn = spec.site(f"football_spawn_{index}")
        body = spec.worldbody.add_body(name=name, pos=list(spawn.pos))
        body.add_freejoint(name=name + "_freejoint")
        body.add_geom(
            name=name + "_geom",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[BALL_RADIUS, 0, 0],
            rgba=[0.94, 0.95, 0.90, 1],
            group=0,
        )
        # Thin painted panels follow the same rigid ball; no extra collision or mass.
        for patch, direction in enumerate(
            ((0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0))
        ):
            rotation = np.empty(4)
            mujoco.mju_quatZ2Vec(rotation, np.asarray(direction, dtype=float))
            body.add_geom(
                name=f"{name}_patch_{patch}",
                type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                pos=(np.asarray(direction) * (BALL_RADIUS * (1 - 0.0008 / 0.035))).tolist(),
                quat=rotation.tolist(),
                size=(np.array([0.009, 0.009, 0.001]) * BALL_RADIUS / 0.035).tolist(),
                mass=0,
                contype=0,
                conaffinity=0,
                rgba=[0.035, 0.075, 0.085, 1],
                group=1,
            )
    # This includes the original benchmark ball created by the framework.
    configure_contacts(spec, BALL_NAMES)


class Scoreboard:
    """Lamp lookup shared by the physics publisher and private native renderers."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.ids = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i): i
            for i in range(model.ngeom)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith("score_")
        }
        self.on_colors = {i: model.geom_rgba[i, :3].copy() for i in self.ids.values()}

    def lamps(self, scores: list[int]) -> list[int]:
        lit: list[int] = []
        for team, score in zip(TEAMS, scores, strict=True):
            for digit, character in enumerate(f"{min(score, 999):03}"):
                lit.extend(
                    self.ids[f"score_{team}_{digit}_{segment}"]
                    for segment in DIGITS[int(character)]
                )
            if score > 999:
                lit.extend(self.ids[f"score_{team}_overflow_{axis}"] for axis in (0, 1))
        return lit

    def apply(self, model: mujoco.MjModel, lit: list[int]) -> None:
        enabled = set(lit)
        for i, color in self.on_colors.items():
            model.geom_rgba[i, :3] = color if i in enabled else OFF_COLOR


@dataclass
class Goal:
    position: NDArray[np.float64]
    rotation: NDArray[np.float64]
    half_width: float
    height: float
    scoring_team: int

    def local(self, point: NDArray[np.float64]) -> NDArray[np.float64]:
        return (point - self.position) @ self.rotation

    def clear_passage(
        self, start: NDArray[np.float64], end: NDArray[np.float64], radius: float
    ) -> bool:
        """Check the swept sphere's intersection with the rectangular goal plane.

        Between physics samples the centre is interpolated linearly. Extrema of
        y/z +/- sqrt(r*r - x*x) test the complete circular cross-section, including
        fast diagonal shots and partial crossings, without requiring extra steps.
        """
        delta = end - start
        if abs(delta[0]) < 1e-12:
            points = (start, end)
        else:
            lo = max(-radius, min(start[0], end[0]))
            hi = min(radius, max(start[0], end[0]))
            if lo > hi:
                return True
            xs = [lo, hi]
            for slope in delta[1:] / delta[0]:
                extreme = radius * slope / math.hypot(1, slope)
                xs.extend(x for x in (extreme, -extreme) if lo < x < hi)
            points = tuple(start + delta * ((x - start[0]) / delta[0]) for x in xs)
        for point in points:
            section = math.sqrt(max(0, radius * radius - point[0] * point[0]))
            if (
                abs(point[1]) + section > self.half_width
                or point[2] - section < -GROUND_TOLERANCE
                or point[2] + section > self.height
            ):
                return False
        return True


@dataclass
class Crossing:
    previous: NDArray[np.float64]
    armed: bool
    entering: bool = False


class FootballMatch:
    """Physics-thread-only counter. Scoring never moves a ball or applies a force."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.scoreboard = Scoreboard(model)
        self.scores = [0, 0]
        self.lit = self.scoreboard.lamps(self.scores)
        self.goals = []
        for index, team in enumerate(TEAMS):
            site = model.site("football_goal_" + team)
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, site.quat)
            self.goals.append(
                Goal(
                    site.pos.copy(),
                    rotation.reshape(3, 3),
                    float(site.size[0]),
                    float(site.size[1]),
                    1 - index,
                )
            )
        self.balls = {
            name: (
                int(model.joint(name + "_freejoint").qposadr[0]),
                float(model.geom(name + "_geom").size[0]),
            )
            for name in BALL_NAMES
        }
        self._crossings: dict[tuple[str, int], Crossing] = {}
        self.last_goal: dict[str, Any] | None = None
        self.ledger = None
        self._last_touch = {}
        self.drops = {name: 0 for name in self.balls}

    def drop_ball(self, data, ball):
        if ball not in self.balls:
            return False
        center = np.array([0.0, 4.3, 2.0])
        radius = self.balls[ball][1]
        for other, (adr, other_radius) in self.balls.items():
            if (
                other != ball
                and np.linalg.norm(data.qpos[adr : adr + 3] - center) < radius + other_radius + 0.02
            ):
                return False
        adr = self.balls[ball][0]
        dof = int(data.model.joint(ball + "_freejoint").dofadr[0])
        data.qpos[adr : adr + 7] = [*center, 1, 0, 0, 0]
        data.qvel[dof : dof + 6] = 0
        data.qacc_warmstart[dof : dof + 6] = 0
        data.xfrc_applied[data.model.body(ball).id] = 0
        self._last_touch.pop(ball, None)
        for key in list(self._crossings):
            if key[0] == ball:
                del self._crossings[key]
        self.drops[ball] += 1
        return True

    def touches(self, data, robots, identities):
        owners = {geom: r for r in robots.values() if r.active for geom in r.geoms}
        ball_geoms = {data.model.geom(name + "_geom").id: name for name in self.balls}
        touched = {}
        for contact in data.contact[: data.ncon]:
            if contact.dist > 0.003:
                continue
            a, b = map(int, contact.geom)
            ball, robot = (
                (ball_geoms.get(a), owners.get(b))
                if a in ball_geoms
                else (ball_geoms.get(b), owners.get(a))
            )
            if ball is None or robot is None:
                continue
            touched.setdefault(ball, {})[robot.id] = robot
        from microduck_world.roster import ROSTER

        for ball, contacts in touched.items():
            self._last_touch[ball] = None
            if len(contacts) != 1:
                continue
            robot = next(iter(contacts.values()))
            identity = identities.get(robot.id)
            if identity and identity["generation"] == robot.generation:
                self._last_touch[ball] = {**identity, "team": ROSTER[robot.id]["team"]}

    def update(self, data: mujoco.MjData) -> None:
        for ball, (adr, radius) in self.balls.items():
            for index, goal in enumerate(self.goals):
                point = goal.local(data.qpos[adr : adr + 3])
                key = (ball, index)
                state = self._crossings.get(key)
                if state is None:
                    self._crossings[key] = Crossing(point.copy(), bool(point[0] <= -radius))
                    continue
                previous = state.previous
                dx = point[0] - previous[0]
                if state.armed and dx > 0 and previous[0] <= -radius < point[0]:
                    state.entering = True
                if state.entering and not goal.clear_passage(previous, point, radius):
                    state.entering = False
                if state.armed and dx > 0:
                    if previous[0] <= radius < point[0]:
                        if state.entering:
                            toucher = self._last_touch.pop(ball, None)
                            team = "blue" if goal.scoring_team == 0 else "red"
                            scorer = toucher if toucher and toucher["team"] == team else None
                            if scorer and self.ledger:
                                self.ledger.record(scorer, time.time())
                            self.scores[goal.scoring_team] += 1
                            self.lit = self.scoreboard.lamps(self.scores)
                            self.last_goal = {
                                "ball": ball,
                                "scorer": scorer["handle"] if scorer else None,
                                "team": TEAMS[goal.scoring_team],
                                "time": float(data.time),
                            }
                        state.armed = False
                        state.entering = False
                if point[0] <= -radius:
                    state.armed = True
                    state.entering = False
                state.previous = point.copy()

    def snapshot(self) -> dict[str, Any]:
        return {
            "drops": dict(self.drops),
            "scores": list(self.scores),
            "lit": list(self.lit),
            "lastGoal": self.last_goal,
            "scorers": list(self.ledger.rows) if self.ledger else [],
        }
