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

"""Draft answer-only QA for the static DimSim apartment.

    DIMOS_TRANSPORT=lcm dimos evals run dimos.evals.suites.dimsim_apartment_qa \
        --agent dimos.evals.agents.pi

References derive from the apartment manifest and GLBs at c5b78bd9a23344db91a59aabdd7f7db614998ab8
and await human validation. Room area uses inside wall faces; distance uses
horizontal asset-center proxies. Tolerances are draft, not measurement precision.
Each case starts a fresh simulation with the authored spawn and static states.
The runner/agent adapter owns tool access; prompts do not prescribe a navigation API.

Pending references (not scored): maximum passable radius, refrigerator-open
passability change, and closest object by collision-free travel distance.
"""

from collections.abc import Callable
import math
from typing import TypeVar

from dimos.evals.environments.sim import Sim
from dimos.evals.scorers import exact, first_number, numeric, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite

T = TypeVar("T")


def _parsed(parser: Callable[[str], T], score: Callable[[T], float]) -> Callable[[Outcome], float]:
    """Keep answer parsing separate from scoring; unparseable answers earn zero."""

    def grade(o: Outcome) -> float:
        try:
            value = parser(o.trajectory.final_answer)
        except ValueError:
            return 0.0
        return score(value)

    return grade


def _environment() -> Sim:
    return Sim(
        blueprint=["unitree-go2", "mcp-server", "unitree-skill-container"],
        disable=("wavefront-frontier-explorer", "patrolling-module"),
        simulator="dimsim",
        scene="apartment",
    )


def _table_dimensions(o: Outcome) -> float:
    """Accept swapped dimensions, but require both finite positive measurements."""
    try:
        length, width = sorted(
            (float(v.strip()) for v in o.trajectory.final_answer.split(",")), reverse=True
        )
        if not all(math.isfinite(v) and v > 0 for v in (length, width)):
            return 0.0
    except ValueError:
        return 0.0
    return 0.5 * numeric(2.2, length, tolerance=0.1, band=0.4) + 0.5 * numeric(
        1.1, width, tolerance=0.05, band=0.25
    )


SUITE: Suite = [
    EvalCase(
        id="dimsim_apartment_refrigerator_location",
        inputs="Which room contains the refrigerator? A: Bedroom; B: Kitchen; C: Bathroom; D: Living room. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "object-location", "single-choice"}),
    ),
    EvalCase(
        id="dimsim_apartment_work_desk_location",
        inputs="Which room contains the work desk? A: Kitchen; B: Bathroom; C: Living room; D: Bedroom. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("D", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "object-location", "single-choice"}),
    ),
    EvalCase(
        id="dimsim_apartment_bathtub_exists",
        inputs="Does the house contain a bathtub? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "existence", "boolean"}),
    ),
    EvalCase(
        id="dimsim_apartment_washing_machine_exists",
        inputs="Does the house contain a washing machine? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "existence", "boolean", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_dining_chair_count",
        inputs="How many dining chairs are in the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(4, value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "object-count", "count"}),
    ),
    EvalCase(
        id="dimsim_apartment_bedside_table_count",
        inputs="How many bedside tables are in the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(2, value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "object-count", "count"}),
    ),
    EvalCase(
        id="dimsim_apartment_wall_cabinet_count",
        inputs="How many wall-mounted kitchen cabinets are in the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(3, value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "object-count", "count"}),
    ),
    EvalCase(
        id="dimsim_apartment_every_desk_has_laptop",
        inputs="Does every work desk in the house have a laptop on it? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        timeout_s=1200.0,
        tags=frozenset(
            {"dimsim", "apartment", "qa", "spatial-relation", "boolean", "draft-reference"}
        ),
    ),
    EvalCase(
        id="dimsim_apartment_room_count",
        inputs="How many rooms are in the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(4, value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "rooms", "count"}),
    ),
    EvalCase(
        id="dimsim_apartment_largest_room_area",
        inputs="What is the approximate area of the largest room, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(37.8, value, tolerance=2.5, band=8.0)),
        timeout_s=1200.0,
        tags=frozenset(
            {"dimsim", "apartment", "qa", "rooms", "area", "numeric", "draft-reference"}
        ),
    ),
    EvalCase(
        id="dimsim_apartment_refrigerator_height",
        inputs="What is the approximate height of the refrigerator, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(1.8, value, tolerance=0.1, band=0.4)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "dimensions", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_dining_table_dimensions",
        inputs="What are the approximate length and width of the dining table, in meters? Return only two positive numbers separated by a comma: length, width.",
        environment=_environment(),
        grade=_table_dimensions,
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "dimensions", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_bed_footprint_area",
        inputs="What is the approximate footprint area of the bed frame, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(3.52, value, tolerance=0.25, band=1.0)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "dimensions", "area", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_house_perimeter",
        inputs="What is the approximate perimeter of the house, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(44.0, value, tolerance=1.0, band=5.0)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "geometry", "perimeter", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_yard_door_count",
        inputs="How many doors connect the house to the yard? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(2, value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "doors", "count", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_refrigerator_tv_distance",
        inputs="What is the approximate straight-line distance between the refrigerator and television, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(8.37, value, tolerance=0.3, band=1.5)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "distance", "numeric", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_refrigerator_state",
        inputs="Is the refrigerator open or closed? A: Closed; B: Open. Return only A or B.",
        environment=_environment(),
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "object-state", "single-choice"}),
    ),
    EvalCase(
        id="dimsim_apartment_kitchen_bathroom_crossings",
        inputs="What is the minimum number of doorway crossings between the kitchen and bathroom without leaving the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(3, value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "connectivity", "count", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_wall_separated_objects",
        inputs="Which pair of objects is on opposite sides of the same wall? A: Sofa and television; B: Work desk and bed; C: Refrigerator and work desk; D: Bathtub and toilet. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset(
            {"dimsim", "apartment", "qa", "spatial-relation", "single-choice", "draft-reference"}
        ),
    ),
    EvalCase(
        id="dimsim_apartment_closest_to_sofa",
        inputs="Which object is closest to the sofa by straight-line distance? A: Refrigerator; B: Dining table; C: Work desk. Return only A, B, or C.",
        environment=_environment(),
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset(
            {"dimsim", "apartment", "qa", "spatial-ordering", "single-choice", "draft-reference"}
        ),
    ),
    EvalCase(
        id="dimsim_apartment_bedside_table_coverage",
        inputs="Is observing only the living room and kitchen sufficient to determine how many bedside tables are in the house? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        timeout_s=1200.0,
        tags=frozenset({"dimsim", "apartment", "qa", "coverage", "boolean", "draft-reference"}),
    ),
]
