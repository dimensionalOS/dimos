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

"""Exploratory occupancy-grid questions for the SF office Go2 recording."""

from pathlib import Path

from dimos.evals.environments.occupancy_dataset import OccupancyDataset
from dimos.evals.types import EvalCase, Suite

_DATASET = str(
    # Path.home() / "Documents/go2_recordings/2026-08-27_sf_office_8mins_moshi/go2_SF_office_8mins_moshi.db"
    Path.home()
    / "Documents/go2_recordings/2026-07-18_sf_office_survey1/sf_office_go2_20260718_survey1.db"
)

# _DATASET = "go2_short"
_EVIDENCE_INSTRUCTIONS = (
    "Open the recording file listed in the system prompt and use only its global_costmap "
    "OccupancyGrid observations. Call obs.data.agent_encode() on every observation. Use only "
    "the returned text and image blocks as spatial evidence; do not inspect the raw grid array "
    "or other message fields. Decode each returned data:image/png;base64 image_url to a PNG "
    "file and inspect it with the read tool. "
)

SUITE: Suite = [
    EvalCase(
        id="sf_office_room_count",
        inputs=_EVIDENCE_INSTRUCTIONS
        + (
            "Infer all distinct rooms from the enclosing wall structure, not from furniture "
            "or isolated obstacles. Return one simple boundary polygon per room, kept inside "
            "that room's enclosing walls and excluding corridors and exterior unknown space. "
            "Polygons must not self-intersect or overlap; adjacent rooms may share boundary "
            "edges. Return only a JSON list in world-frame meters: [[[x, y], ...], ...]."
        ),
        environment=OccupancyDataset(_DATASET, emit_every=0),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "occupancy", "rooms", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_door_locations",
        inputs=_EVIDENCE_INSTRUCTIONS
        + (
            "Identify each distinct door or door-sized opening in the wall structure. Each "
            "segment must span the free opening from one wall edge to the opposite wall edge; "
            "do not mark gaps caused by clutter or incomplete observations and do not return "
            "duplicates. Return only a JSON list in world-frame meters: "
            "[[[x1, y1], [x2, y2]], ...]."
        ),
        environment=OccupancyDataset(_DATASET, emit_every=0),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "occupancy", "doors", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_movement_hotspots",
        inputs=_EVIDENCE_INSTRUCTIONS
        + (
            "Identify areas whose changes across snapshots provide the strongest evidence of "
            "moving objects, rather than newly observed static map structure. Return one "
            "simple boundary polygon per distinct hotspot; merge overlapping hotspot regions "
            "and do not return overlapping polygons. Return only a JSON list in world-frame "
            "meters: [[[x, y], ...], ...]."
        ),
        # Roughly one cumulative grid every 20 seconds at this recording's lidar rate.
        environment=OccupancyDataset(_DATASET, emit_every=150),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "occupancy", "motion", "temporal", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_closed_doors",
        inputs=_EVIDENCE_INSTRUCTIONS
        + (
            "Identify doors that the temporal snapshots show as closed, not merely occluded or "
            "unknown. Each segment must span the closed doorway from one wall edge to the "
            "opposite wall edge, with no duplicates. Return only a JSON list in world-frame "
            "meters: [[[x1, y1], [x2, y2]], ...]. Return [] if none are supported by the maps."
        ),
        environment=OccupancyDataset(_DATASET, emit_every=150),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "occupancy", "doors", "temporal", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_door_cover_route",
        inputs=_EVIDENCE_INSTRUCTIONS
        + (
            "Find the shortest traversable route that visits every visible door opening. "
            "Consecutive waypoints define straight path segments: add enough waypoints that "
            "every segment remains entirely in observed white free space and never crosses or "
            "enters black occupied or gray unknown cells. Route through doorway openings, "
            "never through walls. Return only the ordered JSON waypoints in world-frame "
            "meters: [[x, y], ...]."
        ),
        environment=OccupancyDataset(_DATASET, emit_every=150),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "occupancy", "doors", "routing", "temporal", "exploratory"}),
    ),
    EvalCase(
        id="sf_office_hide_and_seek",
        inputs=_EVIDENCE_INSTRUCTIONS
        + (
            "Choose the best place to hide in this space. The point must be inside observed "
            "white free space, not inside or beyond black occupied cells or gray unknown space. "
            "Prefer a location concealed by mapped obstacles while still reachable through "
            "free space. Return only its world-frame location as JSON: [x, y]."
        ),
        environment=OccupancyDataset(_DATASET, emit_every=0),
        grade=lambda _: 0.0,
        timeout_s=120.0,
        tags=frozenset({"sf-office", "occupancy", "spatial-reasoning", "exploratory"}),
    ),
]
