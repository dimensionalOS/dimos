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

"""HM3D 00337-CFVBbU9Rsyb: real-home scan, 3 levels, ~186 m² (~2,000 sqft) navigable.
Utility/storage, kitchen/living, and loft bedrooms under a pitched wooden roof, joined by stairs."""

from collections.abc import Callable
import os
from typing import TypeVar

from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import choice, exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.utils.data import get_data_dir

T = TypeVar("T")

SCENE_KEY = "hm3d_CFVBbU9Rsyb"
SCENE_NAME = "HM3D scene 1"

INSTRUCTION = (
    "You are answering questions about a live simulated home. You control the robot, "
    "and its sensor recording grows as it observes the environment. Initial observations "
    "do not cover the whole home. Move around to gather the evidence needed to answer "
    "the question. Inspect relevant interior rooms for counts and absence claims. "
    "Indoor areas, including an attached garage, are in scope. Exterior openings may "
    "be observed from indoors; do not leave the home. Use observations rather than "
    "assumptions about a typical home. Return the answer in the requested format."
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


def _environment() -> HabitatEnvironment:
    return HabitatEnvironment(
        scene_dataset_config=os.environ.get(
            "HM3D_DATASET_CONFIG",
            str(get_data_dir("hm3d-0.2/hm3d/example/hm3d_example_basis.scene_dataset_config.json")),
        ),
        scene_id="00337-CFVBbU9Rsyb",
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
    )


SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_sofa_color",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What color is the sofa beneath the framed trousers display in the middle-level living area? A) Blue; B) Green; C) Red; D) White. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("C", value)),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_kitchen_cabinet_color",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What color are the lower kitchen cabinets beside the dining table with rectangular placemats? A) Blue-gray; B) Red; C) Black; D) Yellow. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("A", value)),
        tags=frozenset({"visual-attribute", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_washing_machine_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a washing machine in the scanned environment? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_washing_machine_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which type of room contains the washing machine beneath the long worktop? A) Bedroom; B) Utility room; C) Living room; D) Bathroom. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_utility_high_chair_count",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many child high chairs stand in front of the utility-room worktop? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_fire_extinguisher_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a wall-mounted fire extinguisher beside a stair landing? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_red_sofa_cushion_count",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many dark throw cushions are on the red sofa below the framed trousers display? Return only the count.",
        grade=_parsed(first_number, lambda value: exact(2, value)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_below_trousers_display",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is directly below the framed trousers display in the red-sofa living area? A) Bed; B) Dining table; C) Washing machine; D) Sofa. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("D", value)),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bed_below_skylight_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is there a bed beneath a skylight in a room with a sloped wooden ceiling? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bunk_beds_exists",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Are there bunk beds in the upper-level sleeping area? Return only yes or no.",
        grade=_parsed(yes_no, lambda value: exact("yes", value)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_blue_armchair_bedroom_wardrobe_state",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Is the wardrobe in the bedroom with the blue armchair and balcony doors open or closed? A) Open; B) Closed. Return only the letter.",
        grade=_parsed(_LETTER, lambda value: exact("B", value)),
        tags=frozenset({"object-state", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_location_elevation_order",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Order these locations from lowest to highest floor elevation. A) Upper-level bunk-bed area; B) Utility room with the washing machine; C) Living area with the red sofa below the framed trousers display. Return all three letters once in order, optionally separated by commas.",
        grade=_parsed(ranking, lambda value: rank_order("BCA", value)),
        tags=frozenset({"elevation", "ranking"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_bunk_utility_height_difference",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Approximately how much higher is the floor of the bunk-bed area than the floor of the utility room, in meters? Return only the number.",
        grade=_parsed(first_number, lambda value: numeric(5.60, value, tolerance=0.20, band=1.0)),
        tags=frozenset({"elevation", "numeric"}),
    ),
]
