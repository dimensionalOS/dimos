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


"""Stair-spotting VQA over the go2 stairs replay.

Are there stairs or steps in view, and if so where, and how far do they rise?
An open-ended answer scored by :func:`~dimos.evals.scorers.matched_set` at a
1.2 m radius with a 0.15 m band on the rise.

Deliberately the same six frames as
:mod:`dimos.evals.suites.pointcloud.dataset.go2_pointcloud_floor_level`, asked differently. That
suite asks whether any part of the floor sits at a different level; this one
asks whether there are *steps*. An encoding can carry elevation — a patch of
floor is higher over there — without carrying the structure that makes it a
staircase, and the gap between the two suites' scores is exactly that
difference. Keep them separate so they can be run and reported independently.

The labels are **hand-authored** against the recording. Three frames see the
steps, three do not.

Regenerate the context windows (needs the recording; keeps the labels)::

    uv run python -m dimos.evals.suites.pointcloud.dataset.go2_pointcloud_stairs
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals.suites.pointcloud.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_stairs.json"

# Hand-authored rows, tagged holdout: for testing an encoding, never for tuning one.
SUITE: Suite = generate.cases(
    json.loads(_JSON.read_text()), tags=frozenset({"pointcloud", "holdout"})
)

DATASET = "go2_stairs_20260819"
RADIUS = 1.2  # match tolerance, meters
VALUE_BAND = 0.15  # band on the rise

QUESTION = (
    "You are shown the point cloud the robot mapped (world frame: +x is east, +y is "
    "north, coordinates in meters). Are there any stairs or steps in what you can see? If "
    "there are, give the world coordinate of the middle of each one and how far it rises "
    "from bottom to top in meters. Answer with the single word none, or with one line per "
    "staircase as x,y,rise — nothing else."
)

# Hand-authored, verified frame by frame against the recording. Transcribed, not
# derived. Same frames as go2_pointcloud_floor_level, different question — and so
# a different answer where the two overlap: the rise from bottom to top is not
# the raised area's offset from the main floor.
_LABELS: tuple[tuple[float, list[list[float]]], ...] = (
    (75.0, [[9.0, 4.1, 0.45]]),
    (95.0, [[9.0, 4.1, 0.45]]),
    (130.0, [[9.0, 4.1, 0.45]]),
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
                    "id": f"{DATASET}_stairs_t{t:g}",
                    "family": "stairs",
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
