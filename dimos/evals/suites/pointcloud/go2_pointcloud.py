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


"""Pointcloud geometry VQA over the go2 replays.

Rows (``go2_pointcloud.json``) are pure data emitted by :func:`rows` —
ground truth computed analytically from full-resolution clouds plus odom,
quizzing whatever lossy encoding the agent receives for a ``PointCloud2``.

``go2_short`` maps its whole room in the first frame (an accumulated map);
``go2_bigoffice`` is a 292 s exploration whose lidar is a rolling ~6 m local
window, which is why the two datasets take different stream families.

Free-space families live in their own suites, one per question class:
:mod:`dimos.evals.suites.pointcloud.go2_pointcloud_clearance` and
:mod:`dimos.evals.suites.pointcloud.go2_pointcloud_route`.

Regenerate (needs both recordings)::

    uv run python -m dimos.evals.suites.pointcloud.go2_pointcloud
"""

from __future__ import annotations

import json
from pathlib import Path

from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))

_SHORT_TS = [5.0, 20.0, 40.0, 58.0]
_SHORT_WINDOWS = [(0.0, 20.0), (15.0, 45.0), (30.0, 59.0), (0.0, 59.0)]
_BIGOFFICE_TS = [10.0, 80.0, 150.0, 220.0, 285.0]
_BIGOFFICE_WINDOWS = [(0.0, 60.0), (60.0, 150.0), (150.0, 240.0), (200.0, 291.0), (0.0, 291.0)]


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON.

    go2_short gets no shift/extent-change families — it maps the room
    in frame 1, so those truths degenerate to ~0 and a blind "0" wins.
    """
    short, big = "go2_short", "go2_bigoffice"
    return [
        *generate.extent_rows(short, _SHORT_TS),
        *generate.zspan_rows(short, _SHORT_TS),
        *generate.nearest_obstacle_rows(short, _SHORT_TS),
        *generate.area_trend_rows(short, _SHORT_WINDOWS),
        *generate.coverage_direction_rows(short, _SHORT_WINDOWS),
        *generate.nearest_obstacle_rows(big, _BIGOFFICE_TS),
        *generate.footprint_rows(big, _BIGOFFICE_TS),
        *generate.map_shift_rows(big, _BIGOFFICE_WINDOWS),
        *generate.map_direction_rows(big, _BIGOFFICE_WINDOWS),
    ]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
