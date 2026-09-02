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


"""Room-count VQA over fused go2 maps.

The first suite whose context selector *fuses*: instead of showing the model
``context_budget`` snapshots of a rolling local map, the window is thinned to
every 6th frame and accumulated into one voxel map (``emit_every=0`` yields a
single observation, at the end). The question is about the shape of a whole
mapped stretch, which no single sweep carries.

Counting rooms is deliberately structural — two areas count as separate only
when a doorway-width opening is the only way between them — so the answer turns
on whether the encoding preserves the pinch between them at all.

The labels are **hand-authored** against the recordings. Scored with
``within(1.0)`` on the parsed number, so off-by-one earns nothing but is not
punished past zero.

No window may span t ~ 1200-1440 s in the agentic recording: that is the
relocalisation, and fusing across it smears the one doorway into two.
``go2_teleop_20260819`` appears here and nowhere else — its map has a ~15 m^2
false depression from the robot being carried before the recording started, so
it must never be used for anything about floor height.

Regenerate (needs both recordings; keeps the labels)::

    uv run python -m dimos.evals.suites.pointcloud.go2_pointcloud_rooms
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals.suites.pointcloud.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_rooms.json"

# Hand-authored rows, tagged holdout: for testing an encoding, never for tuning one.
SUITE: Suite = generate.cases(
    json.loads(_JSON.read_text()), tags=frozenset({"pointcloud", "holdout"})
)

BAND = 1.0  # within() band on the count — one room out scores zero
FUSE = {"downsample": 6, "voxel_size": 0.05, "device": "CPU:0"}

QUESTION = (
    "You are shown the point cloud the robot mapped over a stretch of driving (world "
    "frame: +x is east, +y is north, coordinates in meters). How many separate rooms does "
    "this map cover? Count two areas as separate rooms only when the only way between them "
    "is a doorway-width opening. Answer with a single number."
)

# Hand-authored, verified against the recordings. Transcribed, not derived.
_LABELS: tuple[tuple[str, float, float, int], ...] = (
    ("go2_agentic_20260819", 300.0, 1150.0, 2),
    ("go2_agentic_20260819", 330.0, 1200.0, 2),
    ("go2_agentic_20260819", 350.0, 450.0, 2),
    ("go2_agentic_20260819", 1150.0, 1430.0, 1),
    ("go2_teleop_20260819", 0.0, 60.0, 1),
    ("go2_teleop_20260819", 0.0, 150.0, 1),
)


def rows() -> list[generate.Row]:
    """The hand-authored labels as rows. Nothing is read from the recordings —
    a fused window is named by its endpoints, so there is no frame to resolve."""
    return [
        {
            "id": f"{dataset}_rooms_{w0:g}_{w1:g}",
            "family": "rooms",
            "type": "numeric",
            "q": QUESTION,
            "a": count,
            "band": BAND,
            "context": [["lidar", [w0, w1], FUSE]],
            "dataset": dataset,
        }
        for dataset, w0, w1, count in _LABELS
    ]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
