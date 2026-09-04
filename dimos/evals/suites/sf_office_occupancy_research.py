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

"""Occupancy-grid autoresearch cases for the latest SF office run."""

from pathlib import Path

from dimos.evals.environments.dataset import Dataset
from dimos.evals.suites.sf_office_occupancy_grading import grader
from dimos.evals.types import EvalCase, Suite

_DATASET = str(Path.home() / "Documents/worktrees/agentencode/recording_go2.db")
_TIMEOUT_S = 120.0

_EVIDENCE_CONTRACT = (
    "Open the recording listed in the system prompt and use only its global_costmap "
    "OccupancyGrid observations. You may use SqliteStore only to open the recording and "
    "iterate that stream. The only permitted operation on each obs.data message is "
    "obs.data.agent_encode(). Treat the returned text and image blocks as the sole spatial "
    "evidence. Do not inspect raw message attributes or database payloads, access another "
    "stream, inspect evaluation files or labels, or use repository mapping, navigation, "
    "perception, or evaluation utilities for analysis. You may decode returned PNGs and "
    "perform generic calculations using only information in the returned blocks. "
)

_MAP_RULES = (
    "Coordinates are world-frame meters. Only known-free space is traversable; unknown space "
    "is not traversable. A doorway-sized opening is 0.6 to 1.2 meters wide. Headings are "
    "degrees counterclockwise from world +X. Robot paths and clearances must account for the "
    "stated circular robot radius. Times are seconds from the first grid in the selected "
    "sequence. "
)


def _costmaps() -> Dataset:
    return Dataset(_DATASET, select=(lambda store: store.streams.global_costmap,))


def _case(id: str, question: str, *tags: str) -> EvalCase:
    return EvalCase(
        id=id,
        inputs=_EVIDENCE_CONTRACT + _MAP_RULES + question,
        environment=_costmaps(),
        grade=grader(id),
        timeout_s=_TIMEOUT_S,
        tags=frozenset({"sf-office", "occupancy", "research", *tags}),
    )


SUITE: Suite = [
    _case(
        "sf_office_occupancy_room_count",
        "Using the latest grid, how many distinct rooms are enclosed by the mapped wall "
        'structure? Return only JSON: {"room_count": integer}.',
        "rooms",
        "static",
    ),
    _case(
        "sf_office_occupancy_movement_square",
        "Between 190 and 215 seconds in the grid sequence, find the axis-aligned 1 meter by 1 "
        "meter square with the most free-to-occupied and occupied-to-free transitions. Ignore "
        'unknown-to-known changes. Return only JSON: {"center_m": [x, y]}.',
        "motion",
        "temporal",
    ),
    _case(
        "sf_office_occupancy_three_point_loop",
        "Using the latest grid, find the shortest collision-free closed loop for a robot of "
        "radius 0.25 meters that starts at (0.00, -4.20), visits (-7.00, -10.30) and "
        "(3.30, -1.70) in either order, and returns to (0.00, -4.20). Consecutive waypoints "
        'define straight path segments. Return only JSON: {"waypoints_m": [[x, y], ...], '
        '"length_m": number}.',
        "routing",
        "static",
    ),
    _case(
        "sf_office_occupancy_max_robot_radius",
        "Using the latest grid, what is the largest circular robot radius that can travel from "
        '(0.00, -4.20) to (-7.00, -10.30)? Return only JSON: {"max_radius_m": number}.',
        "clearance",
        "routing",
        "static",
    ),
    _case(
        "sf_office_occupancy_blocked_opening_reachability",
        "Using the latest grid, suppose the complete opening nearest (-1.50, -7.05) becomes "
        "occupied. Can a robot of radius 0.25 meters still travel from (0.00, -4.20) to "
        '(-7.00, -10.30)? Return only JSON: {"reachable": boolean}.',
        "counterfactual",
        "routing",
        "static",
    ),
    _case(
        "sf_office_occupancy_doorways",
        "Using the latest grid, locate all likely doorway-sized openings in the mapped wall "
        "structure. Each segment must span an opening from one wall edge to the opposite edge, "
        "without duplicates. Return only JSON: "
        '{"doorways_m": [[[x1, y1], [x2, y2]], ...]}.',
        "doors",
        "static",
    ),
    _case(
        "sf_office_occupancy_largest_free_circle",
        "Using the latest grid, find the largest circle contained entirely in known-free space. "
        'Return only JSON: {"center_m": [x, y], "radius_m": number}.',
        "clearance",
        "static",
    ),
    _case(
        "sf_office_occupancy_possible_person_motion",
        "Between 440 and 464 seconds in the grid sequence, identify time intervals with the "
        "strongest evidence that a person or another moving object crossed the mapped space. "
        "Do not treat unknown-to-known exploration as movement. Return only JSON: "
        '{"intervals_s": [[start, end], ...]}.',
        "motion",
        "temporal",
    ),
    _case(
        "sf_office_occupancy_hide_location",
        "Using the latest grid, choose the deepest reachable place to hide. It must be in "
        "known-free space and reachable from (0.00, -4.20) by a robot of radius 0.25 meters; "
        'prefer a location concealed by mapped obstacles. Return only JSON: '
        '{"position_m": [x, y]}.',
        "spatial-reasoning",
        "static",
    ),
    _case(
        "sf_office_occupancy_independent_routes",
        "Using the latest grid, are there two independent collision-free routes for a robot of "
        "radius 0.20 meters between (-1.00, -5.00) and (3.30, -1.70), such that blocking one "
        'route at a single location does not block the other? Return only JSON: '
        '{"two_independent_routes": boolean}.',
        "topology",
        "routing",
        "static",
    ),
    _case(
        "sf_office_occupancy_doorway_bottleneck",
        "Using the latest grid, which doorway-sized opening is the most important bottleneck for "
        "reaching the rest of the mapped area from (-7.00, -10.30)? Return its wall-edge "
        'segment, or null if none exists. Return only JSON: '
        '{"opening_m": [[x1, y1], [x2, y2]] | null}.',
        "doors",
        "topology",
        "static",
    ),
    _case(
        "sf_office_occupancy_constant_twist_collision",
        "Using the latest grid, a circular robot of radius 0.25 meters starts at (0.00, -4.20) "
        "facing 20 degrees and moves for at most 8 seconds with constant forward velocity 0.50 "
        "meters per second and angular velocity 15 degrees per second. Find its first collision "
        "with occupied or unknown space. Return only JSON: "
        '{"collision": boolean, "time_s": number | null, '
        '"position_m": [x, y] | null}.',
        "collision",
        "motion-model",
        "static",
    ),
    _case(
        "sf_office_occupancy_first_reachable_time",
        "Between 380 and 410 seconds in the grid sequence, when did (0.00, -4.20) first become "
        "reachable from (-7.00, -10.30) for a robot of radius 0.20 meters using only known-free "
        'space? Return null if this never occurred. Return only JSON: '
        '{"time_s": number | null}.',
        "routing",
        "temporal",
    ),
    _case(
        "sf_office_occupancy_known_free_area",
        "Using the latest grid, what is the total known-free floor area? Return only JSON: "
        '{"known_free_area_m2": number}.',
        "area",
        "static",
    ),
    _case(
        "sf_office_occupancy_forward_clearance",
        "Using the latest grid, a circular robot of radius 0.30 meters is at (0.00, -4.20) "
        "facing 90 degrees. Is the full one-meter corridor directly ahead clear enough to move "
        'forward without collision? Return only JSON: {"clear": boolean}.',
        "clearance",
        "static",
    ),
]
