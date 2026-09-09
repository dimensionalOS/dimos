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

"""cloud.py: the height raster reads the way the legend says."""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import cloud


def scene() -> np.ndarray:
    rng = np.random.default_rng(0)
    floor = np.column_stack(
        [rng.uniform(0, 4, 4000), rng.uniform(0, 4, 4000), rng.normal(0.0, 0.02, 4000)]
    )
    wall = np.column_stack([np.full(600, 4.1), rng.uniform(0, 4, 600), rng.uniform(0, 2.5, 600)])
    chair = np.column_stack(
        [rng.uniform(1.0, 1.4, 200), rng.uniform(1.0, 1.4, 200), rng.uniform(0.0, 0.6, 200)]
    )
    return np.vstack([floor, wall, chair]).astype(np.float32)


def test_raster_floor_wall_chair_and_nothing() -> None:
    enc = cloud.encode(scene())
    rows = enc["raster"]["rows"]
    assert enc["raster"]["cell_m"] == 0.25
    grid = {float(r.split()[0]): r.split()[1] for r in rows}
    floor_row = grid[3.0]
    assert floor_row[:2] in ("55", "45", "56")  # z=0 is level 5, +-1 across flat ground
    wall_col = int((4.1 - enc["raster"]["origin_xy_m"][0]) / 0.25)
    assert floor_row[2 * wall_col + 1] == "U"  # 2.5 m clamps to the top glyph
    chair = grid[1.0]
    chair_col = int((1.2 - enc["raster"]["origin_xy_m"][0]) / 0.25)
    assert chair[2 * chair_col + 1] == "B"  # 0.6 m -> level 11
    assert "@@" not in floor_row


def test_boxes_only_list_body_height_returns() -> None:
    enc = cloud.encode(scene())
    boxes = enc["boxes"]["xmin:xmax@ymin:ymax"]
    assert "4.10" in boxes  # the wall
    assert "1." in boxes  # the chair
    assert enc["num_points"] == 4800


def test_robot_marker_lands_on_its_cell() -> None:
    enc = cloud.encode(scene())
    cloud.mark_robot(enc, 2.1, 2.1)
    row = next(r for r in enc["raster"]["rows"] if r.startswith("2.00 "))
    body = row.split()[1]
    col = int((2.1 - enc["raster"]["origin_xy_m"][0]) / 0.25)
    assert body[2 * col : 2 * col + 2] == "@@"
