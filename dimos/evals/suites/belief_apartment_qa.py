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

"""Answer-only QA cases for the DimSim apartment scene.

    DIMOS_TRANSPORT=lcm VIEWER=none DIMSIM_HEADLESS=true \\
        dimos evals run dimos.evals.suites.belief_apartment_qa \\
        --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TypeVar

from dimos.evals.constants import RAW_README
from dimos.evals.environments.dimsim import DimSimEnvironment
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
_SAME_DIFF = choice(["same", "different"])


def _environment() -> DimSimEnvironment:
    return DimSimEnvironment(
        blueprint=["unitree-go2", "mcp-server", "unitree-skill-container", "raw-robot-bridge"],
        disable=("wavefront-frontier-explorer", "patrolling-module"),
        scene="apartment",
        raw_bridge=True,
        raw_guide=RAW_README,
    )


def _case(
    case_id: str,
    question: str,
    grade: Callable[[Outcome], float],
    *,
    tags: frozenset[str],
    timeout_s: float = 1200.0,
) -> EvalCase:
    return EvalCase(
        id=case_id,
        inputs=INSTRUCTION + "\n\n" + question,
        environment=_environment(),
        grade=grade,
        timeout_s=timeout_s,
        tags=tags,
    )


SUITE: Suite = [
    _case(
        "belief_q051_microwave_in_kitchen",
        "Is there a microwave in the kitchen? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean", "kitchen"}),
    ),
    _case(
        "belief_q073_kettle_near_stove",
        "Is there an electric kettle near the gas range? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean", "kitchen"}),
    ),
    _case(
        "belief_q128_wardrobe_present",
        "Does the house contain a large wardrobe? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    _case(
        "belief_q133_desk_has_laptop",
        "Does every work desk in the house have a laptop on it? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"spatial-relation", "boolean"}),
    ),
    _case(
        "belief_q059_refrigerator_closed",
        "Is the refrigerator closed? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean"}),
    ),
    _case(
        "belief_q169_gas_range_off",
        "Is the gas range off? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean", "kitchen"}),
    ),
    _case(
        "belief_q110_no_open_cabinets",
        "Are any refrigerator or cabinet doors currently open? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("no", value)),
        tags=frozenset({"object-state", "boolean"}),
    ),
    _case(
        "belief_q118_television_off",
        "Is the television powered on? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("no", value)),
        tags=frozenset({"object-state", "boolean"}),
    ),
    _case(
        "belief_q108_floor_lamp_on",
        "Is at least one floor lamp currently on? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean"}),
    ),
    _case(
        "belief_q086_kitchen_idle",
        "Are both the refrigerator closed and the gas range off? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean", "kitchen"}),
    ),
    _case(
        "belief_q114_kitchen_passable",
        "Can a robot of radius 0.25 m pass from the kitchen doorway to the sink "
        "with the refrigerator in its current state? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"clearance", "boolean", "kitchen"}),
    ),
    _case(
        "belief_q066_watering_can_present",
        "Is there a watering can in the house or yard? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    _case(
        "belief_q076_laundry_hamper_in_bathroom",
        "Is there a laundry hamper in the bathroom? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean", "bathroom"}),
    ),
    _case(
        "belief_q049_laptop_room",
        "Which room contains the laptop? A: Kitchen; B: Bathroom; C: Living room; "
        "D: Bedroom. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    _case(
        "belief_q063_laptop_room_now",
        "Where is the laptop right now? A: Kitchen; B: Bathroom; C: Living room; "
        "D: Bedroom. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    _case(
        "belief_q054_television_area",
        "Which area contains the television? A: Kitchen; B: Bedroom; C: Bathroom; "
        "D: Living room. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    _case(
        "belief_q070_wine_glass_room",
        "Which room contains the wine glass? A: Kitchen; B: Bedroom; C: Bathroom; "
        "D: Living room. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    _case(
        "belief_q062_dining_table_room",
        "Which room contains the dining table? A: Kitchen; B: Bedroom; C: Bathroom; "
        "D: Living room. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    _case(
        "belief_q050_farthest_from_spawn",
        "Which object is farthest from the robot spawn in a straight horizontal line? "
        "A: Refrigerator; B: Bathtub; C: Dining table; D: Television. "
        "Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"spatial-ordering", "single-choice"}),
    ),
    _case(
        "belief_q053_closest_to_sofa",
        "Which object is closest to the sofa by straight-line distance? "
        "A: Refrigerator; B: Dining table; C: Work desk. Return only A, B, or C.",
        _parsed(choice("ABC", case_sensitive=True), lambda value: exact("B", value)),
        tags=frozenset({"spatial-ordering", "single-choice"}),
    ),
    _case(
        "belief_q068_teddy_room",
        "Which room is the teddy bear in? A: Kitchen; B: Bedroom; C: Bathroom; "
        "D: Living room. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    _case(
        "belief_q182_gas_range_idle",
        "Is the gas range idle or actively heating a burner? "
        "A: Idle; B: Active. Return only A or B.",
        _parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"object-state", "single-choice", "kitchen"}),
    ),
    _case(
        "belief_q081_chair_vs_bedside_count",
        "Are the number of dining chairs and the number of bedside tables the same "
        "or different? Return only same or different.",
        _parsed(_SAME_DIFF, lambda value: exact("different", value)),
        tags=frozenset({"object-count", "comparison"}),
    ),
    _case(
        "belief_q071_wine_glass_count",
        "How many wine glasses are in the house? Return only the count.",
        _parsed(first_number, lambda value: exact(1, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    _case(
        "belief_q074_chairs_near_table",
        "How many dining chairs are within 2 meters of the dining table center? "
        "Return only the count.",
        _parsed(first_number, lambda value: exact(4, value)),
        tags=frozenset({"object-count", "count", "spatial-relation"}),
    ),
    _case(
        "belief_q079_base_cabinet_count",
        "How many kitchen base cabinets are in the house? Return only the count.",
        _parsed(first_number, lambda value: exact(3, value)),
        tags=frozenset({"object-count", "count", "kitchen"}),
    ),
    _case(
        "belief_q117_narrowest_doorway",
        "What is the largest robot radius that can fit through all the doorways "
        "in 2D, in meters? Return only the number.",
        _parsed(first_number, lambda value: numeric(0.5, value, tolerance=0.025, band=0.1)),
        tags=frozenset({"clearance", "numeric"}),
    ),
    _case(
        "belief_q123_doorways_below_threshold",
        "Is the clear width of every doorway at least 0.8 m "
        "(equivalently: can a robot of radius 0.4 m fit through all doorways)? "
        "Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"clearance", "boolean"}),
    ),
    _case(
        "belief_q111_sub_meter_doorway",
        "Is there any doorway whose clear width is below 0.8 m? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("no", value)),
        tags=frozenset({"clearance", "boolean"}),
    ),
    _case(
        "belief_q058_watering_can_yard",
        "Search the living room and outdoor yard area for a watering can. "
        "Is it present? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean", "search"}),
    ),
    _case(
        "belief_q067_find_teddy",
        "Search the house for a teddy bear. Did you find one? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean", "search"}),
    ),
    _case(
        "belief_q234_teddy_room_after_search",
        "Search until you locate the teddy bear. Which room is it in? "
        "A: Kitchen; B: Bedroom; C: Bathroom; D: Living room. Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice", "search"}),
    ),
    _case(
        "belief_q227_smartphone_room",
        "Search Kitchen, Living room, Bedroom, then Bathroom for a smartphone. "
        "Which room contains it? A: Kitchen; B: Bedroom; C: Bathroom; D: Living room. "
        "Return only A, B, C, or D.",
        _parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice", "search"}),
    ),
    _case(
        "belief_q064_hamper_nearby_stop",
        "Is there a laundry hamper you could stop beside inside the bathroom? "
        "Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean", "bathroom"}),
    ),
    _case(
        "belief_q183_free_space_exists",
        "Does the house contain open floor space large enough for the robot to stand "
        "without intersecting furniture? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"clearance", "boolean"}),
    ),
    _case(
        "belief_q112_low_object_present",
        "Are there small movable objects resting low near the floor or furniture "
        "surfaces (for example a watering can or stuffed toy)? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    _case(
        "belief_q065_left_out_small_object",
        "After inspecting the rooms, is there at least one non-furniture small object "
        "left out on a bed, desk, or table (for example a smartphone or book)? "
        "Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    _case(
        "belief_q119_notable_lamp_on",
        "Is any floor lamp left on while the television is off? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean"}),
    ),
    _case(
        "belief_q120_fridge_closed_tv_off",
        "At the end of your inspection, is the refrigerator closed and the television "
        "off? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean"}),
    ),
    _case(
        "belief_q163_bathroom_no_tv",
        "Does the bathroom contain a television? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("no", value)),
        tags=frozenset({"existence", "boolean", "bathroom"}),
    ),
    _case(
        "belief_q224_left_out_and_lamp",
        "Is there both a floor lamp that is on and at least one small object left out "
        "on furniture? Return only yes or no.",
        _parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"object-state", "boolean"}),
    ),
]
