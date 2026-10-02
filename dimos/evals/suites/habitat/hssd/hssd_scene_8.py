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

from functools import partial

from dimos.evals.scorers import exact, first_number, numeric, yes_no
from dimos.evals.suites.lib.habitat_qa import (
    HSSD_DATASET,
    INSTRUCTION,
    environment,
    parsed,
)
from dimos.evals.types import EvalCase, Suite

SCENE_KEY = "hssd_107734110_175999914"
SCENE_NAME = "HSSD scene 8"

_environment = partial(environment, "107734110_175999914", "HSSD_DATASET_CONFIG", HSSD_DATASET)
SUITE: Suite = [
    EvalCase(
        id=f"{SCENE_KEY}_bedrooms",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "How many bedrooms are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(1, v)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_closets",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many separate closet spaces are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(2, v)),
        tags=frozenset({"rooms", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_piano_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the piano? A) Bedroom; B) Office; C) Kitchen; D) Living room. Return only A, B, C, or D.",
        grade=lambda o: exact("D", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_computer_location",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "Which room contains the desktop computer? A) Office; B) Living room; C) Kitchen; D) Bedroom. Return only A, B, C, or D.",
        grade=lambda o: exact("A", o.trajectory.final_answer.strip().upper()),
        tags=frozenset({"object-location", "single-choice"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_sofa_office",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION + "\n\n" + "Is there a sofa in the office? Return only yes or no.",
        grade=parsed(yes_no, lambda v: exact("yes", v)),
        tags=frozenset({"existence", "boolean"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_televisions",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How many televisions are in the home? Return only the count.",
        grade=parsed(first_number, lambda v: exact(3, v)),
        tags=frozenset({"object-count", "count"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_living_area",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate living-room area, in square meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(51.53, v, tolerance=3, band=10)),
        tags=frozenset({"area", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_piano_width",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the approximate width of the digital piano, in meters? Return only the number.",
        grade=parsed(first_number, lambda v: numeric(1.33, v, tolerance=0.08, band=0.3)),
        tags=frozenset({"dimensions", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_office_doorway_radius",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "What is the largest circular robot radius that fits through the office doorway in 2D, based on the structural opening width, in meters? Return only the number.",
        # Stage slice Y=1 m at X=-4.79/-4.85/-4.90: Z gap [-3.997643,-3.017643].
        # Width .98 m / 2 is structural clearance only, not a furnished-route guarantee.
        grade=parsed(first_number, lambda v: numeric(0.49, v, tolerance=0.025, band=0.10)),
        tags=frozenset({"clearance", "numeric"}),
    ),
    EvalCase(
        id=f"{SCENE_KEY}_piano_computer_distance",
        environment=_environment(),
        timeout_s=1200,
        inputs=INSTRUCTION
        + "\n\n"
        + "How far is the piano from the office computer in a horizontal straight line, in meters? Return only the number.",
        # Horizontal visual-AABB centers including node transforms/instance scale: 11.635427 m.
        grade=parsed(first_number, lambda v: numeric(11.64, v, tolerance=0.5, band=2)),
        tags=frozenset({"distance", "numeric"}),
    ),
]
