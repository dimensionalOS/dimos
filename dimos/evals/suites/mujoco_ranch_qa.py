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

"""Answer-only QA in the MuJoCo Go2's NJ ranch scan (dimsim_apartment_qa's counterpart).

    dimos evals run dimos.evals.suites.mujoco_ranch_qa --agent dimos.evals.agents.pi

The answers come from dim-go2-sim's ranch labels (data/supersplat/3f89bbd3_labels.json: each
object's box is the room cleanup's box for it, placed by hand on the scan) and await human
validation; "draft-reference" marks the ones read off the scan less surely. Each case starts a
fresh simulation at the living-room spawn.
"""

from collections.abc import Callable
from typing import TypeVar

from dimos.evals.constants import RAW_README
from dimos.evals.environments.go2_sim import Go2SimEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, yes_no
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
        "mujoco_ranch_fridge_location",
        "Which room contains the refrigerator? A: Bedroom; B: Kitchen; C: Bathroom; "
        "D: Living room. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    _case(
        "mujoco_ranch_wood_stove_location",
        "Which room has the wood stove? A: Kitchen; B: Family room; C: Living room; "
        "D: Office. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("C", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    _case(
        "mujoco_ranch_bathtub_exists",
        "Does the house contain a bathtub? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    _case(
        "mujoco_ranch_bed_count",
        "How many beds for people are in the house (not counting pet beds or toy beds)? "
        "Return only the count.",
        _parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    _case(
        "mujoco_ranch_dining_chair_count",
        "How many chairs are around the dining table? Return only the count.",
        _parsed(first_number, lambda value: exact(8, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    _case(
        "mujoco_ranch_bar_stool_count",
        "How many bar stools are at the kitchen counter? Return only the count.",
        _parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count", "draft-reference"}),
    ),
    _case(
        "mujoco_ranch_fridge_height",
        "What is the approximate height of the refrigerator, in meters? Return only the number.",
        # its box is 1.86 m: the scan's scale (0.92 m a unit) is measured from its fittings
        _parsed(first_number, lambda value: numeric(1.86, value, tolerance=0.15, band=0.4)),
        tags=frozenset({"dimensions", "numeric", "draft-reference"}),
    ),
    _case(
        "mujoco_ranch_closest_to_dining_table",
        "Which object is closest to the dining table by straight-line distance? "
        "A: Refrigerator; B: Wood stove; C: The bed with the palm beside it. Return only A, B, or C.",
        # centre to centre: fridge 5.0 m, wood stove 5.9 m, that bed 12.7 m
        _parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"spatial-ordering", "single-choice"}),
    ),
]
