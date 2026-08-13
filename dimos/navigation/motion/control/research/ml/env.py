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

"""The RL environment: one episode of the referee, taped.

There is no simulator here and there must not be one. The plant is the matched
MuJoCo go2 under the real walking policy (:mod:`~...motion.simulation`), and
the loop around it is the referee's own :func:`run_episode` -- the same
mechanism chain, the same 29 Hz zero-order-hold pose, the same 5 Hz replan, the
same judge. A learned law that scores here scores against the shipped laws by
construction, which is the entire reason the referee is off-limits to research.

So the environment is an INVERSION, not a rewrite. ``run_episode`` calls
``controller.update()`` inside its loop; :class:`Agent` is a controller that
answers with the net and writes down what it saw and did on the way past. The
episode runs to completion, and only then is the tape priced -- terminal judge
score plus per-tick shaping recovered from the trace the judge itself reads.
Nothing in ``referee/`` changes, so no lab's ``referee.lock`` moves.

Rewards are shaping, not the objective. The objective is
``judge.score_episode``, and that is what the training loop reports.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np
import torch

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.motion.control.controller import ControllerConfig, TrajectoryController
from dimos.navigation.motion.control.referee.episode import EpisodeConfig, run_episode
from dimos.navigation.motion.control.referee.judge import executed_clearance, score_episode
from dimos.navigation.motion.control.research.ml import obs, policy
from dimos.navigation.motion.control.tracks import TRACKS
from dimos.navigation.motion.scenarios import GenRules, Scenario, generated

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Path import Path
    from dimos.navigation.motion.simulation.policy import FreePolicy

# Seed hygiene, the planner lab's rule pointed at worlds instead of samples.
# The referee's generated battery starts at scenarios.GEN_SEED = 0 and the
# held-out OOD battery at battery.OOD_SEED = 7700; training may touch neither,
# and the curated 16 are never generated at all.
VAL_SEED0 = 15_000
TRAIN_SEED0 = 20_000

# Reward weights.
# Sized to keep the judge's lexicographic ordering INSIDE the shaping: over a
# typical 6 s / 300-tick episode, closing a 4 m route earns ~40, while riding
# the clearance floor the whole way costs ~120. A policy cannot buy its way out
# of a precision violation with pace, which is the property the judge has and
# a naive dense reward loses.
W_PROGRESS = 10.0  # per metre closed toward the goal
W_BELOW = 20.0  # per second fully below the precision floor
W_TIME = 1.0  # per second alive -- see below
W_SMOOTH = 0.05  # per unit^2 of normalised action change
R_GATE = -100.0  # collision or fall: the judge's gate, as a terminal
TERMINAL_SCALE = 1.0  # the judge's own total, added at the end

# Everything above is written in judge points, which is what makes it readable
# and what makes the weights arguable against the score. The critic then has to
# regress returns of O(100), and a freshly initialised value head that outputs
# ~0 produces an MSE two orders of magnitude larger than any policy term --
# gradients that swamp the update even under norm clipping. Scaling once, here,
# keeps the ratios exactly as reasoned about and hands the optimiser O(1)
# targets.
REWARD_SCALE = 0.01

# W_TIME is not a pace term, it closes a hole. The judge pays
# ``10 * precision`` for a world the robot never entered, so a body that stands
# still for the full 40 s banks 10.1 with a clean precision pillar and risks
# none of the gate -- a local optimum that a from-scratch policy finds
# immediately and never leaves. Charging for the clock makes a timeout (~-30)
# strictly worse than an arrival (~+147) without touching the ordering that
# matters: a full-episode precision violation still costs more (~120) than any
# pace difference can earn.


@dataclass
class Tape:
    """What the agent saw, did and thought, one row per control tick."""

    t: list[float] = field(default_factory=list)
    feat: list[np.ndarray] = field(default_factory=list)
    action: list[np.ndarray] = field(default_factory=list)
    logp: list[float] = field(default_factory=list)
    value: list[float] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.t)


class Agent:
    """The net behind the controller protocol; optionally taping as it goes.

    Stateful in exactly one thing, the previous normalised action, which is
    both an input to the next observation and what the smoothness term is
    priced against. ``reset()`` clears it, so a used instance is
    indistinguishable from a fresh one -- the discipline ``laws/hinted.py``
    holds itself to.
    """

    config: ControllerConfig

    def __init__(
        self,
        net: policy.ActorCritic,
        config: ControllerConfig | None = None,
        deterministic: bool = False,
        tape: Tape | None = None,
    ) -> None:
        self.config = config or ControllerConfig()
        self._net = net
        self._deterministic = deterministic
        self.tape = tape
        self._vel = obs.Velocity()
        self.reset()

    def reset(self) -> None:
        self._prev = np.zeros(policy.ACT_DIM)
        self._prev2 = np.zeros(policy.ACT_DIM)
        self._vel.reset()

    def _shift(self, action: np.ndarray) -> None:
        self._prev2, self._prev = self._prev, action

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: np.ndarray | None = None
    ) -> Twist:
        xy_yaw = (float(pose.position.x), float(pose.position.y), float(pose.yaw))
        # fed on every tick, including the held ones: the estimator's whole job
        # is knowing which ticks carried new information
        vel = self._vel.update(xy_yaw, t)

        if len(path) < 2:
            # veto stub or empty path: the planner is saying stop, and there is
            # no observation to build. Same answer every law gives.
            self._shift(np.zeros(policy.ACT_DIM))
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        feat = obs.build(xy_yaw, path, t, clearance, self._prev, self._prev2, vel)
        with torch.inference_mode():
            action, logp, value = self._net.act(
                torch.from_numpy(feat).unsqueeze(0), deterministic=self._deterministic
            )
        a = action[0].numpy().astype(float)

        if self.tape is not None:
            self.tape.t.append(t)
            self.tape.feat.append(feat)
            self.tape.action.append(a.copy())
            self.tape.logp.append(float(logp[0]))
            self.tape.value.append(float(value[0]))

        self._shift(np.clip(a, -1.0, 1.0))
        vx, vy, wz = policy.to_twist(a, self.config.max_speed, self.config.max_yaw_rate)
        return Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))


class TeacherAgent:
    """A shipped law, taped through the net's own eyes.

    Wraps any :class:`TrajectoryController` and records the observation the net
    WOULD have seen alongside the action the law actually took, normalised into
    the net's action space. That pairing is the behaviour-cloning dataset, and
    because it comes from the law running inside a real episode it is on the
    state distribution the law itself induces -- the states a fresh net has to
    survive before it can improve on any of them.
    """

    config: ControllerConfig

    def __init__(
        self,
        law: TrajectoryController,
        config: ControllerConfig | None = None,
        tape: Tape | None = None,
    ) -> None:
        self.config = config or ControllerConfig()
        self._law = law
        self.tape = tape
        self._vel = obs.Velocity()
        self.reset()

    def reset(self) -> None:
        self._prev = np.zeros(policy.ACT_DIM)
        self._prev2 = np.zeros(policy.ACT_DIM)
        self._vel.reset()
        self._law.reset()

    def update(
        self, pose: PoseStamped, path: Path, t: float, clearance: np.ndarray | None = None
    ) -> Twist:
        xy_yaw = (float(pose.position.x), float(pose.position.y), float(pose.yaw))
        vel = self._vel.update(xy_yaw, t)
        twist = self._law.update(pose, path, t, clearance)
        action = np.array(
            [
                twist.linear.x / max(self.config.max_speed, 1e-9),
                twist.linear.y / max(self.config.max_speed, 1e-9),
                twist.angular.z / max(self.config.max_yaw_rate, 1e-9),
            ]
        )
        action = np.clip(action, -1.0, 1.0)

        if self.tape is not None and len(path) >= 2:
            feat = obs.build(xy_yaw, path, t, clearance, self._prev, self._prev2, vel)
            self.tape.t.append(t)
            self.tape.feat.append(feat)
            self.tape.action.append(action.copy())
            self.tape.logp.append(0.0)
            self.tape.value.append(0.0)

        self._shift(action)
        return twist

    def _shift(self, action: np.ndarray) -> None:
        self._prev2, self._prev = self._prev, action


@dataclass
class Rollout:
    tape: Tape
    reward: np.ndarray  # (n,) per taped tick
    row: dict[str, Any]  # judge.score_episode -- the real objective

    @property
    def total(self) -> float:
        return float(self.row["total"])


def shape_rewards(result: Any, row: dict[str, Any], tape: Tape) -> np.ndarray:
    """Price the tape from the executed trace.

    The tape is a subset of the episode's ticks -- ``update()`` is not called
    during the policy's settle window -- so rows are matched back to the trace
    by their timestamp rather than by position.
    """
    n = len(tape)
    reward = np.zeros(n)
    if n == 0:
        return reward

    idx = np.clip(np.searchsorted(result.t, np.asarray(tape.t)), 0, len(result.t) - 1)
    goal = np.asarray(result.scenario.goal, dtype=float)
    dist = np.linalg.norm(result.pos[:, :2] - goal, axis=1)
    clear = executed_clearance(result)
    # A body that falls hard enough can take the integrator with it, and NaN
    # trunk positions would poison the whole batch through the advantage
    # normalisation. The judge has already called that episode dead; the
    # shaping only has to not spread it.
    finite = np.isfinite(dist)
    dist = np.where(finite, dist, dist[finite][-1] if finite.any() else 0.0)
    floor = result.scenario.emb.precision
    dt = float(np.median(np.diff(result.t))) if len(result.t) > 1 else 0.02

    prev_i = idx[0]
    prev_a = np.zeros(policy.ACT_DIM)
    for k in range(n):
        i = idx[k]
        # progress: distance closed since the previous taped tick
        reward[k] += W_PROGRESS * float(dist[prev_i] - dist[i])
        # precision: how far into the floor the body is, in clearance space
        room = float(clear[i])
        if np.isfinite(room) and room < floor:
            reward[k] -= W_BELOW * dt * min(1.0, (floor - room) / max(floor, 1e-6))
        a = np.clip(tape.action[k], -1.0, 1.0)
        reward[k] -= W_SMOOTH * float(np.sum((a - prev_a) ** 2))
        reward[k] -= W_TIME * dt
        prev_i, prev_a = i, a

    # Terminal: the judge, and the judge's gate. Nothing else decides the sign
    # of an episode.
    reward[-1] += R_GATE if row["dq"] else TERMINAL_SCALE * float(row["total"])
    return reward * REWARD_SCALE


def rollout(
    sc: Scenario,
    net: policy.ActorCritic,
    walk: FreePolicy,
    cfg: EpisodeConfig,
    config: ControllerConfig | None = None,
    deterministic: bool = False,
    rng: np.random.Generator | None = None,
    dr: Any = None,
) -> Rollout:
    """One episode of the referee under the net; tape priced on the way out."""
    tape = Tape()
    agent = Agent(net, config=config, deterministic=deterministic, tape=tape)
    result = run_episode(sc, agent, walk, cfg, dr=dr, rng=rng)
    row = score_episode(result)
    return Rollout(tape=tape, reward=shape_rewards(result, row, tape), row=row)


def episode_config(track: str = "hinted", replan_hz: float = 5.0) -> EpisodeConfig:
    """The referee's episode config for a track -- the CLI's own construction."""
    return EpisodeConfig(replan_hz=replan_hz, annotate_clearance=TRACKS[track].annotate_clearance)


def worlds(count: int, seed0: int, rules: GenRules | None = None) -> list[Scenario]:
    """Generated worlds from a seed base disjoint from every eval battery."""
    return generated(count, seed=seed0, rules=rules)


def train_worlds(count: int) -> list[Scenario]:
    return worlds(count, TRAIN_SEED0)


def val_worlds(count: int) -> list[Scenario]:
    """Held out from training, in distribution. The OOD battery stays unseen."""
    return worlds(count, VAL_SEED0)
