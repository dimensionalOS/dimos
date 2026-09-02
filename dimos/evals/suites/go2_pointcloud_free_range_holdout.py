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


"""Hand-labelled ``free_range`` frames, tagged ``holdout``.

Real frames a live session got wrong, labelled by hand against the lidar.
The question and scoring are ``free_range``'s; the frame is resolved from the
recording, the answer is transcribed here.

The 2026-08-20 19:04:35Z frame of ``go2_agentic_memory_20260820``: from
(1.44, 7.68) the southern bearings run to the window edge and the nearest
return above the floor is due north. ``south`` and ``southeast`` both reach
the cap — the clearing is the whole southern arc.

Regenerate the context window (needs the recording; keeps the label)::

    uv run python -m dimos.evals.suites.go2_pointcloud_free_range_holdout
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from dimos.evals.suites import go2_pointcloud_free_range as fr
from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_free_range_holdout.json"

SUITE: Suite = generate.cases(
    json.loads(_JSON.read_text()), tags=frozenset({"pointcloud", "holdout"})
)

# (dataset, time from start of recording, query point, labelled bearing)
_LABELS: tuple[tuple[str, float, tuple[float, float], str], ...] = (
    ("go2_agentic_memory_20260820", 68.73, (1.44, 7.68), "southeast"),
)


def rows() -> list[generate.Row]:
    """The hand-labelled frames, with each frame's context window resolved and
    the question re-rendered from its own cap so it reads like a train row."""
    out: list[generate.Row] = []
    for dataset, t, point, bearing in _LABELS:
        with generate._dataset(dataset) as store:
            pts, context = generate._cloud_at(store, t)
            _, cap = fr.runs(pts, np.array(point))
            out.append(
                {
                    "id": f"{dataset}_freerange_t{t:g}_labelled",
                    "family": "free_range",
                    "type": "mcq",
                    "q": fr._question(np.array(point), cap),
                    "a": bearing,
                    "choices": list(generate.COMPASS),
                    "context": [
                        *context,
                        ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]],
                    ],
                    "dataset": dataset,
                }
            )
    return out


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
