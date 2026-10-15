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

"""Check 20-foot extinguisher coverage, live in Habitat and from a recorded house tour.

The house is HSSD scene 106366134_174226362; the live cases need the dataset at
target/habitat/data/hssd-hab/.

dimos evals run dimos.evals.suites.habitat_extinguishers --agent dimos.evals.agents.pi
"""

from collections.abc import Callable, Sequence
import json
import math
from pathlib import Path
import struct
from typing import TYPE_CHECKING

from dimos.constants import CACHE_DIR, DIMOS_PROJECT_ROOT
from dimos.evals.environments.dataset import Dataset
from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite
from dimos.simulation.habitat.server import HabitatProp, MotionType

if TYPE_CHECKING:
    from dimos.simulation.habitat.connection import HabitatConnectionConfig

# HSSD's wall cabinet with an extinguisher inside; the house has one, by the kitchen.
_CABINET = "a50a4618218d207822655304b02711b63bd4e2f4"
# x, y in the world frame in metres, and the way the cabinet faces in degrees.
_CABINETS = (
    (3.876, 8.424, -90.5),
    (5.540, 2.371, -20.8),
    (-0.180, 4.125, 0.0),
    (2.875, 6.949, 90.0),
)

_TASK = (
    "Is every part of the main house's floor within a 20-foot walk of a fire extinguisher? "
    "Exclude the yard, pool and detached annex. Return only JSON: "
    '{"covered": true or false, "extinguishers": [[x, y], ...]}, with one position in the '
    "world frame, in metres, for each extinguisher."
)


class ExtinguisherHabitatEnvironment(HabitatEnvironment):
    """Hang the cabinets from the dataset's own model when configuring a live episode."""

    def connection_config(self) -> "HabitatConnectionConfig":
        config = super().connection_config()
        hssd = Path(config.scene_dataset_config).parent
        cabinet = _without_glass(hssd / "objects" / _CABINET[0] / f"{_CABINET}.glb")
        config.props = tuple(prop._replace(glb_path=str(cabinet)) for prop in config.props)
        return config


def _without_glass(cabinet: Path) -> Path:
    """A copy of the cabinet without its glass door, which Habitat draws opaque."""
    copy = CACHE_DIR / "habitat_extinguishers" / cabinet.name
    if copy.exists():
        return copy
    glb = cabinet.read_bytes()
    json_length = struct.unpack_from("<I", glb, 12)[0]
    model = json.loads(glb[20 : 20 + json_length])
    for node in model["nodes"]:
        if "mesh" in node and model["meshes"][node["mesh"]]["name"] == "Glass_Door_2017":
            del node["mesh"]
    text = json.dumps(model, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    rest = glb[20 + json_length :]
    header = struct.pack("<4sIII4s", b"glTF", 2, 20 + len(text) + len(rest), len(text), b"JSON")
    copy.parent.mkdir(parents=True, exist_ok=True)
    copy.write_bytes(header + text + rest)
    return copy


def _prop(x: float, y: float, facing_deg: float) -> HabitatProp:
    # The model lies on its back: stand it against the wall, centred 0.75 m up, facing facing_deg.
    half = math.radians(facing_deg) / 2
    c, s = math.cos(half) * math.sqrt(0.5), math.sin(half) * math.sqrt(0.5)
    return HabitatProp(f"{_CABINET}.glb", (x, y, 0.75), MotionType.STATIC, (c, s, c, -s))


def _answered(
    covered: bool, cabinets: Sequence[tuple[float, float, float]]
) -> Callable[[Outcome], float]:
    """Share of extinguishers located, counted only when the yes-or-no answer is right."""

    def grade(o: Outcome) -> float:
        reply = o.trajectory.final_answer
        try:
            answer = json.loads(reply[reply.find("{") : reply.rfind("}") + 1])
            verdict = answer["covered"]
            reported = [(float(x), float(y)) for x, y in answer["extinguishers"]]
        except (TypeError, ValueError, KeyError):
            return 0.0
        if verdict is not covered or not reported:
            return 0.0
        if not all(math.isfinite(v) for position in reported for v in position):
            return 0.0
        # An extinguisher is found by the report nearest to it: full credit within 1 m,
        # none past 2 m.
        credit = sum(
            ramp(max(0.0, min(math.dist((x, y), p) for p in reported) - 1.0), band=1.0)
            for x, y, _ in cabinets
        )
        return credit / max(len(reported), len(cabinets))

    return grade


def _live(covered: bool, cabinets: Sequence[tuple[float, float, float]]) -> EvalCase:
    return EvalCase(
        id=f"habitat_extinguishers_{len(cabinets)}_live",
        inputs=f"{_TASK} Drive the robot through the house to look for them.",
        environment=ExtinguisherHabitatEnvironment(
            blueprint=["habitat-nav", "mcp-server", "observe-skill", "point-nav-skill-container"],
            scene_dataset_config=str(
                DIMOS_PROJECT_ROOT
                / "target/habitat/data/hssd-hab/hssd-hab.scene_dataset_config.json"
            ),
            scene_id="106366134_174226362",
            start_position_ros_override=(1.625, 1.125, 0.05),
            removed_objects=(_CABINET,),
            props=tuple(_prop(*cabinet) for cabinet in cabinets),
        ),
        grade=_answered(covered, cabinets),
        timeout_s=1200.0,
        tags=frozenset({"habitat", "perception", "navigation", "live"}),
    )


def _replay(covered: bool, cabinets: Sequence[tuple[float, float, float]]) -> EvalCase:
    return EvalCase(
        id=f"habitat_extinguishers_{len(cabinets)}_replay",
        inputs=f"{_TASK} The recording is a robot's tour of the house.",
        environment=Dataset(f"habitat_extinguishers_{len(cabinets)}"),
        grade=_answered(covered, cabinets),
        timeout_s=1200.0,
        tags=frozenset({"habitat", "perception", "memory", "replay"}),
    )


SUITE: Suite = [
    _live(False, _CABINETS[:2]),
    _live(True, _CABINETS),
    _replay(False, _CABINETS[:2]),
    _replay(True, _CABINETS),
]
