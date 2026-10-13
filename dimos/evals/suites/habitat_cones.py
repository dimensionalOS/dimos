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

"""Find every traffic cone in a house, live in Habitat and from a recorded tour of it.

The house is HSSD scene 106366134_174226362; the live cases need the dataset at
target/habitat/data/hssd-hab/.

dimos evals run dimos.evals.suites.habitat_cones --agent dimos.evals.agents.pi
"""

from collections.abc import Callable, Sequence
import json
import math
from typing import TYPE_CHECKING

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.evals.environments.dataset import Dataset
from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.simulation.habitat.server import HabitatProp, MotionType
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from dimos.simulation.habitat.connection import HabitatConnectionConfig

# x, y in the world frame, metres.
_CONES = ((5.75, 10.8), (8.125, 2.125), (4.55, 4.4), (1.375, 11.0), (-1.2, 5.25))
# Distractors to provoke false-positives
_ORANGE_BOX = (3.5, 0.5)
_ORANGE_BALL = (0.625, 8.25)

_TASK = (
    "Find all traffic cones inside the main house, excluding the yard, pool and detached "
    "annex. Return only a JSON list of [x, y] positions in the world frame, "
    "in metres, one per cone. Return [] if there are no cones."
)


class ConeHabitatEnvironment(HabitatEnvironment):
    """Resolve the shared prop bundle when configuring a live episode."""

    def connection_config(self) -> "HabitatConnectionConfig":
        config = super().connection_config()
        assets = get_data("cone_eval_assets")
        config.props = tuple(
            prop._replace(glb_path=str(assets / prop.glb_path)) for prop in config.props
        )
        return config


def _prop(model: str, xy: tuple[float, float]) -> HabitatProp:
    return HabitatProp(model, (*xy, 0.001), MotionType.STATIC)


def _found(cones: Sequence[tuple[float, float]]) -> Callable[[Outcome], float]:
    """Share of cones reported, with every wrong or repeated position counting against it."""

    def grade(o: Outcome) -> float:
        reply = o.trajectory.final_answer
        try:
            listed_positions = json.loads(reply[reply.find("[") : reply.rfind("]") + 1])
            reported_positions = [(float(x), float(y)) for x, y in listed_positions]
        except (TypeError, ValueError):
            return 0.0
        if not all(math.isfinite(v) for position in reported_positions for v in position):
            return 0.0
        if not cones or not reported_positions:
            return float(len(cones) == len(reported_positions))
        # A cone is found by the report nearest to it: full credit within 1 m, none past 2 m.
        credit = sum(
            ramp(max(0.0, min(math.dist(cone, p) for p in reported_positions) - 1.0), band=1.0)
            for cone in cones
        )
        return credit / max(len(reported_positions), len(cones))

    return grade


def _live(name: str, cones: Sequence[tuple[float, float]]) -> EvalCase:
    return EvalCase(
        id=f"habitat_cones_{name}_live",
        inputs=f"{_TASK} Drive the robot through the house to look for them.",
        environment=ConeHabitatEnvironment(
            blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
            scene_dataset_config=str(
                DIMOS_PROJECT_ROOT
                / "target/habitat/data/hssd-hab/hssd-hab.scene_dataset_config.json"
            ),
            scene_id="106366134_174226362",
            start_position_ros_override=(1.625, 1.125, 0.05),
            props=(
                *(_prop("traffic_cone.glb", cone) for cone in cones),
                _prop("orange_box.glb", _ORANGE_BOX),
                _prop("orange_ball.glb", _ORANGE_BALL),
            ),
        ),
        grade=_found(cones),
        timeout_s=1200.0,
        tags=frozenset({"habitat", "perception", "navigation", "live"}),
    )


def _replay(name: str, cones: Sequence[tuple[float, float]]) -> EvalCase:
    return EvalCase(
        id=f"habitat_cones_{name}_replay",
        inputs=f"{_TASK} The recording is a robot's tour of the house.",
        environment=Dataset(f"habitat_cones_{name}"),
        grade=_found(cones),
        timeout_s=1200.0,
        tags=frozenset({"habitat", "perception", "memory", "replay"}),
    )


SUITE: Suite = [
    _live("5", _CONES),
    _live("0", ()),
    _replay("5", _CONES),
    _replay("0", ()),
]
