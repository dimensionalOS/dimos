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

"""Launch and scoring helpers for user-reviewed Habitat suites."""

from collections.abc import Callable
from functools import partial
import os
from typing import TypeVar

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import exact, first_number, numeric, rank_order, ranking, yes_no
from dimos.evals.types import Outcome

T = TypeVar("T")
INSTRUCTION = (
    "You are answering questions about a live simulated home. You control the robot, "
    "and its sensor recording grows as it observes the environment. Initial observations "
    "do not cover the whole home. Move around to gather the evidence needed to answer "
    "the question. Inspect relevant interior rooms for counts and absence claims. "
    "Indoor areas, including an attached garage, are in scope. Exterior openings may "
    "be observed from indoors; do not leave the home. Use observations rather than "
    "assumptions about a typical home. Return the answer in the requested format."
)

HSSD_DATASET = str(
    DIMOS_PROJECT_ROOT / "target/habitat/data/hssd-hab/hssd-hab.scene_dataset_config.json"
)
DATA_ROOT = DIMOS_PROJECT_ROOT / "target/habitat/data/versioned_data"
HM3D_ROOT = DATA_ROOT / "hm3d-0.2/hm3d/example"
HM3D_DATASET = str(HM3D_ROOT / "hm3d_example_basis.scene_dataset_config.json")
HM3D_ANNOTATED_DATASET = str(HM3D_ROOT / "hm3d_annotated_example_basis.scene_dataset_config.json")
REPLICACAD_DATASET = str(DATA_ROOT / "replica_cad_dataset/replicaCAD.scene_dataset_config.json")
TEST_APARTMENT = str(DATA_ROOT / "habitat_test_scenes/apartment_1.glb")


def parsed(parser: Callable[[str], T], score: Callable[[T], float]) -> Callable[[Outcome], float]:
    """Score malformed final answers as zero rather than an evaluation error."""

    def grade(outcome: Outcome) -> float:
        try:
            value = parser(outcome.trajectory.final_answer)
        except ValueError:
            return 0.0
        return score(value)

    return grade


def count(expected: int) -> Callable[[Outcome], float]:
    return parsed(first_number, partial(exact, expected))


def boolean(expected: str) -> Callable[[Outcome], float]:
    return parsed(yes_no, partial(exact, expected))


def choice(expected: str) -> Callable[[Outcome], float]:
    return lambda outcome: exact(expected, outcome.trajectory.final_answer.strip().upper())


def measurement(reference: float, tolerance: float, band: float) -> Callable[[Outcome], float]:
    return parsed(first_number, partial(numeric, reference, tolerance=tolerance, band=band))


def order(expected: str) -> Callable[[Outcome], float]:
    return parsed(ranking, partial(rank_order, expected))


def environment(
    scene_id: str,
    dataset_env: str,
    default_dataset: str,
    *,
    scene_env: str | None = None,
) -> HabitatEnvironment:
    """Create a fresh Habitat episode with the common navigation/observation stack."""
    return HabitatEnvironment(
        scene_dataset_config=os.environ.get(dataset_env, default_dataset),
        scene_id=os.environ.get(scene_env, scene_id) if scene_env else scene_id,
        seed=0,
        blueprint=["habitat-nav", "mcp-server", "observe-skill"],
    )
