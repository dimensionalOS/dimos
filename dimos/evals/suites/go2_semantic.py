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


"""Semantic object-map VQA over the go2_bigoffice exploration.

The surface under test is the encoding an agent receives for a semantic
detection stream: :func:`dimos.memory.objects.detections3d_stream` materializes
the frozen detections as ``ImageDetections3DPC`` observations (the
``tool_localize.py`` convention), and the runner encodes each one.

Context parity — load-bearing, keep both sides in step:

1. Ground truth in :mod:`dimos.evals.generate_semantic` comes from the FULL
   detection span, so the objects context is the full stream: every frame, no
   curation. ``_BUDGET`` sits above the frame count so the runner's evenly
   spaced subsampler never drops one.
2. Time-conditioned families (nearest / egoside / robotdist) condition on the
   robot pose at the question timestamp, so each of those rows carries an odom
   window ending exactly there — "your current pose is the last odom
   observation shown". Odom may be subsampled; the last observation is always
   kept.

Regenerate rows (frozen detections in, rows out)::

    python -m dimos.evals.suites.go2_semantic

The detections fixture itself is frozen: yolov8m at conf 0.4 over every 5th
color frame of ``go2_bigoffice``, each detection grounded at the robot's odom
pose at that timestamp (~1-2 m error, which the row bands account for).
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals import generate, generate_semantic
from dimos.evals.types import Suite
from dimos.memory.objects import DETECTIONS_STREAM, detections3d_stream

_JSON = Path(__file__).parent / "go2_semantic_vqa.json"
_DETECTIONS = json.loads((Path(__file__).parent / "go2_semantic_detections.json").read_text())
_BUDGET = 256  # >= detection frame count: full stream, no subsampling

SUITE: Suite = generate.cases(
    json.loads(_JSON.read_text()),
    tags=frozenset({"semantic"}),
    streams={DETECTIONS_STREAM: lambda _store: detections3d_stream(_DETECTIONS)},
    context_budget=_BUDGET,
)


def rows() -> list[generate.Row]:
    """The generator call behind the committed JSON."""
    return generate_semantic.rows(_DETECTIONS)


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
