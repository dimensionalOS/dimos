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


"""Doorway-spotting VQA over the go2 agentic replay.

Open-ended, unlike the multiple-choice pointcloud families: the model is asked
whether it can see a doorway and, if so, where — so a reply is a set of points,
not a word from a list. Scored by
:func:`~dimos.evals.scorers.matched_set` at a 0.8 m radius, which charges for
both a missed doorway and an invented one. There is nothing to guess: a blind
model has no way to name a coordinate, which is what makes the blind ablation
the honest floor here.

The labels are **hand-authored** against the recording, one frame at a time.
Three frames see the doorway, three do not.

``_LABELS`` carries two different coordinates for what is physically the same
doorway. That is not a typo. The map relocalises between t ~ 1116 s and
t ~ 1440 s — a 0.65 m x / 0.27 m y shift — and the gap was measured
independently in each frame's own coordinates. Labels are per-frame; never
substitute one for the other.

Regenerate the context windows (needs the recording; keeps the labels)::

    uv run python -m dimos.evals.suites.pointcloud.go2_pointcloud_doorway
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_doorway.json"

# Hand-authored rows, tagged holdout: for testing an encoding, never for tuning one.
SUITE: Suite = generate.cases(
    json.loads(_JSON.read_text()), tags=frozenset({"pointcloud", "holdout"})
)

DATASET = "go2_agentic_20260819"
RADIUS = 0.8  # match tolerance, meters — above the ~0.15 m odom/lidar disagreement

QUESTION = (
    "You are the robot; your current pose is the odom observation shown (world frame: "
    "+x is east, +y is north, coordinates in meters). Using only the mapped point cloud, "
    "are there any doorways in what you can see? If there are, give the world coordinate "
    "of the middle of each one. Answer with the single word none, or with one coordinate "
    "per doorway as x,y — one per line, nothing else."
)

# Hand-authored, verified frame by frame against the recording. Transcribed, not
# derived. The two coordinates are the same doorway either side of the
# relocalisation — see the module docstring.
_LABELS: tuple[tuple[float, list[list[float]]], ...] = (
    (400.0, [[-2.05, -2.47]]),
    (900.0, [[-2.05, -2.47]]),
    (1500.0, [[-2.70, -2.22]]),
    (1200.0, []),
    (1409.0, []),
    (1395.0, []),
)


def rows() -> list[generate.Row]:
    """The hand-authored labels, with each frame's context window resolved.

    Only the window is read from the recording: ``_frame_at`` picks the last
    cloud at or before ``t`` and returns a +/- 0.05 s window around it, relative
    to ``lidar.first().ts``. Relative windows are also what keeps the 47
    glitched-timestamp frames (stamped at the stream minimum, ~11.9 s before
    ``first().ts``) out of the selection.
    """
    with generate._dataset(DATASET) as store:
        out: list[generate.Row] = []
        for t, answer in _LABELS:
            _, context = generate._frame_at(store, t)
            out.append(
                {
                    "id": f"{DATASET}_doorway_t{t:g}",
                    "family": "doorway",
                    "type": "coords",
                    "q": QUESTION,
                    "a": answer,
                    "radius": RADIUS,
                    "context": [
                        *context,
                        ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]],
                    ],
                    "dataset": DATASET,
                }
            )
        return out


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
