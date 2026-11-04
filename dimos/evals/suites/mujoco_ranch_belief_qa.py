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

"""Answer-only existence, state and location questions in the MuJoCo Go2's NJ ranch scan
(belief_apartment_qa's counterpart).

    dimos evals run dimos.evals.suites.mujoco_ranch_belief_qa --agent dimos.evals.agents.pi

Answers come from dim-go2-sim's ranch labels and the scan as it was captured (a scan is frozen:
the fridge is shut, the TVs dark), awaiting human validation.
"""

from collections.abc import Callable
from typing import TypeVar

from dimos.evals.constants import RAW_README
from dimos.evals.environments.go2_sim import Go2SimEnvironment
from dimos.evals.scorers import choice, exact, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite

T = TypeVar("T")

INSTRUCTION = (
    "You are answering questions about a live simulated house. "
    "You control the robot, and its sensor recording grows as it observes "
    "the environment. Initial observations do not cover the whole house. "
    "Move around to gather the evidence needed to answer the question. "
    "For whole-house counts or absence claims, inspect all relevant rooms. "
    "Use observations rather than assumptions about a typical house. "
    "When you have enough evidence, return the answer in the requested format."
)


def _parsed(parser: Callable[[str], T], score: Callable[[T], float]) -> Callable[[Outcome], float]:
    """Keep answer parsing separate from scoring; unparseable answers earn zero."""

    def grade(o: Outcome) -> float:
        try:
            value = parser(o.trajectory.final_answer)
        except ValueError:
            return 0.0
        return score(value)

    return grade


_LETTER = choice("ABCD", case_sensitive=True)


def _case(
    case_id: str, question: str, grade: Callable[[Outcome], float], *, tags: frozenset[str]
) -> EvalCase:
    return EvalCase(
        id=case_id,
        inputs=INSTRUCTION + "\n\n" + question,
        environment=Go2SimEnvironment(
            blueprint=["unitree-go2", "raw-robot-bridge"],
            raw_bridge=True,  # agents without dimOS get the robot as plain topics
            raw_guide=RAW_README,
        ),
        grade=grade,
        timeout_s=1200.0,
        tags=tags,
    )


SUITE: Suite = [
    _case(
        "mujoco_ranch_belief_plant_in_bedroom",
        "Is there a potted plant on the floor of the bedroom with the largest bed? "
        "Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    _case(
        "mujoco_ranch_belief_dog_bed_present",
        "Is there a dog bed in the house? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    _case(
        "mujoco_ranch_belief_dollhouse_present",
        "Is there a dollhouse in the house? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    _case(
        "mujoco_ranch_belief_fridge_closed",
        "Is the refrigerator closed? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean", "kitchen"}),
    ),
    _case(
        "mujoco_ranch_belief_tv_over_fireplace",
        "Is there a television mounted over a fireplace? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    _case(
        "mujoco_ranch_belief_keyboard_room",
        "Which room contains the piano keyboard? A: Kitchen; B: Bedroom; C: Office; "
        "D: Living room. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
]
