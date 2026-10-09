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

from __future__ import annotations

from collections import deque
import math

import numpy as np
import pytest

from dimos.simulation.scenes.procedural import (
    CEILING_HEIGHT,
    SLAB_THICKNESS,
    START_CLEARANCE,
    Scene,
    generate,
    office,
)


def test_office_is_deterministic_and_pinned() -> None:
    assert office(3).digest() == office(3).digest() == "ab569178c96c7815"
    assert office(3).digest() != office(4).digest()


def test_office_geometry_is_enclosed_and_on_one_floor() -> None:
    scene = generate("office", 1)
    z0 = scene.params["z0"]
    assert {box.kind for box in scene.boxes} == {"floor", "wall", "ceiling", "clutter"}
    lo, hi = scene.bounds()
    assert lo[2] == pytest.approx(z0 - SLAB_THICKNESS)
    assert hi[2] > z0 + CEILING_HEIGHT
    assert scene.start[2] == z0


def test_office_keeps_clutter_and_tables_clear_of_the_start() -> None:
    for seed in range(1, 101):
        scene = office(seed)
        sx, sy, _ = scene.start
        for box in scene.boxes:
            if box.kind != "clutter":
                continue
            dx = max(box.center[0] - box.half[0] - sx, 0.0, sx - box.center[0] - box.half[0])
            dy = max(box.center[1] - box.half[1] - sy, 0.0, sy - box.center[1] - box.half[1])
            assert math.hypot(dx, dy) >= START_CLEARANCE - 1e-9, seed


def _reachable_fraction(scene: Scene, cell: float = 0.1, inflate: float = 0.17) -> float:
    """Free floor reachable from the start on a grid at body height, grown by the body's half width."""
    width, length, z0 = scene.params["width"], scene.params["length"], scene.params["z0"]
    nx, ny = int(width / cell), int(length / cell)
    xs, ys = (np.arange(nx) + 0.5) * cell, (np.arange(ny) + 0.5) * cell
    blocked = np.zeros((nx, ny), dtype=bool)
    for box in scene.boxes:
        low, high = box.center[2] - box.half[2], box.center[2] + box.half[2]
        if box.kind in ("floor", "ceiling") or high < z0 + 0.02 or low > z0 + 0.45:
            continue
        ix = np.abs(xs - box.center[0]) < box.half[0] + inflate
        iy = np.abs(ys - box.center[1]) < box.half[1] + inflate
        blocked[np.ix_(ix, iy)] = True
    start = (int(scene.start[0] / cell), int(scene.start[1] / cell))
    seen = np.zeros_like(blocked)
    seen[start] = True
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        for nx_, ny_ in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx_ < nx and 0 <= ny_ < ny and not blocked[nx_, ny_] and not seen[nx_, ny_]:
                seen[nx_, ny_] = True
                queue.append((nx_, ny_))
    return float(seen.sum() / (~blocked).sum())


def test_office_rooms_are_reachable_from_the_start() -> None:
    for seed in range(1, 61):
        assert _reachable_fraction(office(seed)) > 0.95, seed


def test_office_records_its_parameters() -> None:
    params = office(2).params
    assert {"width", "length", "z0", "rooms", "clutter", "tables"} <= params.keys()
    assert params["rooms"] in (4, 6)


def test_degenerate_boxes_are_dropped() -> None:
    scene = Scene("empty")
    scene.add((0.0, 0.0, 0.0), (1.0, 0.0, 1.0), "wall")
    assert scene.boxes == []


def test_generate_dispatches_by_family() -> None:
    assert generate("office", 3).digest() == office(3).digest()
