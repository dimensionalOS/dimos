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

Doorway radius uses the agreed 2D width-only convention (minimum width / 2).
Refrigerator-open passability is user-validated. Closest-by-route uses offline
projected geometry: radius 0.25 m, body slab scene Y=0.12..0.9 m, and reachable
regions within 0.75 m of each object's horizontal bounding box. The dining table
wins at both 0.05 m and 0.025 m grid resolution; distances are approximate.
"""

from collections.abc import Callable
from typing import TypeVar

from dimos.evals.environments.sim import Sim
from dimos.evals.scorers import exact, first_number, numeric, rank_order, ranking, yes_no
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


SUITE: Suite = [
    EvalCase(
        id="dimsim_apartment_refrigerator_location",
        inputs="Which room contains the refrigerator? A: Bedroom; B: Kitchen; C: Bathroom; D: Living room. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id="dimsim_apartment_work_desk_location",
        inputs="Which room contains the work desk? A: Kitchen; B: Bathroom; C: Living room; D: Bedroom. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("D", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id="dimsim_apartment_bathtub_exists",
        inputs="Does the house contain a bathtub? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        timeout_s=1200.0,
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id="dimsim_apartment_washing_machine_exists",
        inputs="Does the house contain a washing machine? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        timeout_s=1200.0,
        tags=frozenset({"existence", "boolean", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_dining_chair_count",
        inputs="How many dining chairs are in the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(4, value)),
        timeout_s=1200.0,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id="dimsim_apartment_bedside_table_count",
        inputs="How many bedside tables are in the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(2, value)),
        timeout_s=1200.0,
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id="dimsim_apartment_every_desk_has_laptop",
        inputs="Does every work desk in the house have a laptop on it? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        timeout_s=1200.0,
        tags=frozenset({"spatial-relation", "boolean", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_room_count",
        inputs="How many rooms are in the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(4, value)),
        timeout_s=1200.0,
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id="dimsim_apartment_largest_room_area",
        inputs="What is the approximate area of the largest room, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(37.8, value, tolerance=2.5, band=8.0)),
        timeout_s=1200.0,
        tags=frozenset({"rooms", "area", "numeric", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_refrigerator_height",
        inputs="What is the approximate height of the refrigerator, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(1.8, value, tolerance=0.1, band=0.4)),
        timeout_s=1200.0,
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_dining_table_diagonal",
        inputs="What is the approximate diagonal length of the rectangular dining table, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(2.46, value, tolerance=0.1, band=0.4)),
        timeout_s=1200.0,
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_bed_footprint_area",
        inputs="What is the approximate footprint area of the bed frame, in square meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(3.52, value, tolerance=0.25, band=1.0)),
        timeout_s=1200.0,
        tags=frozenset({"dimensions", "area", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_house_perimeter",
        inputs="What is the approximate perimeter of the house, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(44.0, value, tolerance=1.0, band=5.0)),
        timeout_s=1200.0,
        tags=frozenset({"geometry", "perimeter", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_yard_door_count",
        inputs="How many doors connect the house to the yard? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(2, value)),
        timeout_s=1200.0,
        tags=frozenset({"doors", "count", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_doorway_radius",
        inputs="What is the largest robot radius that can fit through all the doorways in 2D, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(0.5, value, tolerance=0.025, band=0.1)),
        timeout_s=1200.0,
        tags=frozenset({"clearance", "numeric"}),
    ),
    EvalCase(
        id="dimsim_apartment_refrigerator_tv_distance",
        inputs="What is the approximate straight-line distance between the refrigerator and television, in meters? Return only the number.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: numeric(8.37, value, tolerance=0.3, band=1.5)),
        timeout_s=1200.0,
        tags=frozenset({"distance", "numeric", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_refrigerator_state",
        inputs="Is the refrigerator open or closed? A: Closed; B: Open. Return only A or B.",
        environment=_environment(),
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset({"object-state", "single-choice"}),
    ),
    EvalCase(
        id="dimsim_apartment_kitchen_bathroom_crossings",
        inputs="What is the minimum number of doorway crossings between the kitchen and bathroom without leaving the house? Return only the count.",
        environment=_environment(),
        grade=_parsed(first_number, lambda value: exact(3, value)),
        timeout_s=1200.0,
        tags=frozenset({"connectivity", "count", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_refrigerator_open_passability",
        inputs="Would opening the refrigerator change whether a robot of radius 0.25 m can pass from the kitchen doorway to the sink? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        timeout_s=1200.0,
        tags=frozenset({"counterfactual", "clearance", "boolean"}),
    ),
    EvalCase(
        id="dimsim_apartment_wall_separated_objects",
        inputs="Which pair of objects is on opposite sides of the same wall? A: Sofa and television; B: Work desk and bed; C: Refrigerator and work desk; D: Bathtub and toilet. Return only A, B, C, or D.",
        environment=_environment(),
        grade=lambda o: exact("C", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset({"spatial-relation", "single-choice", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_closest_to_sofa",
        inputs="Which object is closest to the sofa by straight-line distance? A: Refrigerator; B: Dining table; C: Work desk. Return only A, B, or C.",
        environment=_environment(),
        grade=lambda o: exact("B", o.trajectory.final_answer.strip().upper()),
        timeout_s=1200.0,
        tags=frozenset({"spatial-ordering", "single-choice", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_object_order_by_path",
        inputs="What is the order of these objects from nearest to farthest from the sofa by collision-free travel distance for a robot of radius 0.25 m? A: Refrigerator; B: Dining table; C: Work desk. Return all three letters once, in order, optionally separated by commas. Do not include an explanation.",
        environment=_environment(),
        grade=_parsed(ranking, lambda value: rank_order("BCA", value)),
        timeout_s=1200.0,
        tags=frozenset({"spatial-ordering", "ranking", "draft-reference"}),
    ),
    EvalCase(
        id="dimsim_apartment_bedside_table_coverage",
        inputs="Is observing only the living room and kitchen sufficient to determine how many bedside tables are in the house? Return only yes or no.",
        environment=_environment(),
        grade=_parsed(yes_no, lambda value: exact("no", value)),
        timeout_s=1200.0,
        tags=frozenset({"coverage", "boolean", "draft-reference"}),
    ),
]
