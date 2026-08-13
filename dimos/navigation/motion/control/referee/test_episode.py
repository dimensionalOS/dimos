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

"""End-to-end episode smokes: real planner, real policy, matched physics.

Each case is a full closed-loop MuJoCo rollout (~1 s wall per 10 s sim), so
this file stays small; the battery lives behind the CLI.
"""

import numpy as np
import pytest

from dimos.navigation.motion.control.laws.seed import PursuitController
from dimos.navigation.motion.control.referee.episode import EpisodeConfig, run_episode
from dimos.navigation.motion.control.referee.judge import score_episode
from dimos.navigation.motion.scenarios import SCENARIOS
from dimos.navigation.motion.simulation.policy import FreePolicy
from dimos.utils.data import get_data


@pytest.fixture(scope="module")
def policy() -> FreePolicy:
    return FreePolicy.load(get_data("ml-trajectory-research/freewalk_mcf.bin"))


def _scenario(name: str):  # type: ignore[no-untyped-def]
    return next(s for s in SCENARIOS if s.name == name)


def test_corridor_reaches_goal_plan_once(policy: FreePolicy) -> None:
    result = run_episode(
        _scenario("corridor"), PursuitController(), policy, EpisodeConfig(replan_hz=0.0)
    )
    assert len(result.plan_t) == len(result.plans) == 1
    assert result.outcome == "goal"
    assert result.time_to_goal is not None and result.time_to_goal < 25.0
    assert not result.contact.any()
    row = score_episode(result)
    assert not row["dq"]
    assert row["total"] > 80.0


def test_boxed_in_refuses(policy: FreePolicy) -> None:
    result = run_episode(_scenario("boxed_in"), PursuitController(), policy)
    assert result.outcome == "refused"
    row = score_episode(result)
    assert row["arrived"] == 1.0 and row["total"] > 100.0


def test_dr_draws_are_seeded_and_in_range() -> None:
    from dimos.navigation.motion.control.referee.episode import DomainRandomization, EpisodeConfig

    dr = DomainRandomization(physics={"damping": (0.1, 0.4)})
    base = EpisodeConfig()
    a = dr.draw(base, np.random.default_rng(7))
    b = dr.draw(base, np.random.default_rng(7))
    assert a.command_delay == b.command_delay and a.physics == b.physics  # reproducible
    assert dr.command_delay[0] <= a.command_delay <= dr.command_delay[1]
    assert dr.actuator_tau[0] <= a.actuator_tau <= dr.actuator_tau[1]
    assert 0.1 <= a.physics["damping"] <= 0.4
    # the base config is never mutated by a draw
    from dimos.navigation.motion.simulation.evaluate import FITTED_COMMAND_DELAY, FITTED_PHYSICS

    assert base.command_delay == FITTED_COMMAND_DELAY
    assert base.physics == FITTED_PHYSICS


def test_commands_respect_hardware_slew(policy: FreePolicy) -> None:
    result = run_episode(_scenario("empty"), PursuitController(), policy)
    steps = np.abs(np.diff(result.used_cmd, axis=0))
    from dimos.navigation.motion.simulation.walk import COMMAND_SLEW

    assert (steps <= COMMAND_SLEW[None, :] + 1e-9).all()


def test_corridor_replan_battery_serial(policy: FreePolicy) -> None:
    """Reality mode (5 Hz replanning) through the battery path, one world."""
    from dimos.navigation.motion.control.referee.battery import group_summaries, run_battery

    rows = run_battery(
        [(_scenario("corridor"), "curated")],
        policy_path="ml-trajectory-research/freewalk_mcf.bin",
        controller_name="seed",
        cfg=EpisodeConfig(),
        jobs=1,
    )
    assert len(rows) == 1
    assert rows[0]["outcome"] == "goal"
    assert rows[0]["plan_tight"] > 0.1  # corridor plans keep real margins
    assert group_summaries(rows) == {"curated": rows[0]["total"]}
