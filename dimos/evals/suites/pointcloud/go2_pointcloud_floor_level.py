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


"""Floor-level VQA over the go2 stairs replay.

Is the floor all at one level, or is part of it raised or sunken? An
open-ended answer — a set of places and, for each, how far off the main floor
it sits — scored by :func:`~dimos.evals.scorers.matched_set` at a 1.2 m radius
with a 0.15 m band on the offset, so finding the right place at the wrong
height earns partial credit.

This suite shares its six frames with
:mod:`dimos.evals.suites.pointcloud.go2_pointcloud_stairs`, which asks a different
question of them: not "is anything at a different level" but "are there
steps". The gap between the two scores is the point — it separates an encoding
that carries elevation from one that carries structure. Keep them separate so
they can be run and reported independently.

The labels are **hand-authored** against the recording. Three frames see the
raised area, three do not. ``go2_teleop_20260819`` is deliberately absent: its
map contains a ~15 m^2 false depression from the robot being carried before
the recording started, so it cannot be used for anything about floor height.

Regenerate the context windows (needs the recording; keeps the labels)::

    uv run python -m dimos.evals.suites.pointcloud.go2_pointcloud_floor_level
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals.suites.pointcloud.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_floor_level.json"

# Hand-authored rows, tagged holdout: for testing an encoding, never for tuning one.
SUITE: Suite = generate.cases(
    json.loads(_JSON.read_text()), tags=frozenset({"pointcloud", "holdout"})
)

DATASET = "go2_stairs_20260819"
RADIUS = 1.2  # match tolerance, meters
VALUE_BAND = 0.15  # band on the height offset

QUESTION = (
    "You are shown the point cloud the robot mapped (world frame: +x is east, +y is "
    "north, coordinates in meters). Is the floor all at one level, or is some part of it "
    "raised or sunken relative to the rest? If some part is at a different level, give the "
    "world coordinate of the middle of each such area and how far it sits above or below "
    "the main floor, in meters, positive for above. Answer with the single word level, or "
    "with one line per area as x,y,dz — nothing else."
)

# Hand-authored, verified frame by frame against the recording. Transcribed, not
# derived. Same frames as go2_pointcloud_stairs, different question.
_LABELS: tuple[tuple[float, list[list[float]]], ...] = (
    (75.0, [[9.0, 4.1, 0.25]]),
    (95.0, [[9.0, 4.1, 0.25]]),
    (130.0, [[9.0, 4.1, 0.25]]),
    (3.0, []),
    (23.0, []),
    (25.0, []),
)


def rows() -> list[generate.Row]:
    """The hand-authored labels, with each frame's context window resolved.

    Lidar alone — the question is about the map, not about where the robot is
    standing, so no odom is shown.
    """
    with generate._dataset(DATASET) as store:
        out: list[generate.Row] = []
        for t, answer in _LABELS:
            _, context = generate._frame_at(store, t)
            out.append(
                {
                    "id": f"{DATASET}_floorlevel_t{t:g}",
                    "family": "floorlevel",
                    "type": "coords",
                    "q": QUESTION,
                    "a": answer,
                    "radius": RADIUS,
                    "value_band": VALUE_BAND,
                    "context": context,
                    "dataset": DATASET,
                }
            )
        return out


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
