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
    BLOCK_SIZE,
    CORRIDOR_LENGTH,
    CORRIDOR_WIDTH,
    DOOR_HEIGHT,
    DOOR_WIDTH,
    OVERHANG_LEG,
    OVERHANG_SIZE,
    SLAB_THICKNESS,
    START_CLEARANCE,
    TABLE_HEIGHT,
    TABLE_LEG,
    TABLE_SIZE,
    TABLE_TOP_THICKNESS,
    WALL_HEIGHT,
    WALL_THICKNESS,
    Scene,
    block,
    corridor,
    corridor_turn,
    doorway,
    generate,
    office,
    overhang,
    room,
    table,
)


def test_office_is_deterministic_and_pinned() -> None:
    assert office(3).digest() == office(3).digest() == "02d2bcbade80b2a6"
    assert office(3).digest() != office(4).digest()


def test_office_geometry_is_enclosed_and_on_one_floor() -> None:
    scene = generate("office", 1)
    z0 = scene.params["z0"]
    assert {box.kind for box in scene.boxes} == {"floor", "wall", "clutter"}
    lo, hi = scene.bounds()
    assert lo[2] == pytest.approx(z0 - SLAB_THICKNESS)
    assert hi[2] == pytest.approx(z0 + WALL_HEIGHT)
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
        if box.kind == "floor" or high < z0 + 0.02 or low > z0 + 0.45:
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


def _door_widths(scene: Scene) -> list[float]:
    """The width of every doorway, read off its lintel."""
    lintel = scene.params["z0"] + DOOR_HEIGHT
    return [
        2 * max(box.half[0], box.half[1])
        for box in scene.boxes
        if box.kind == "wall" and abs(box.center[2] - box.half[2] - lintel) < 1e-9
    ]


def test_office_door_width_is_drawn_unless_given() -> None:
    drawn = _door_widths(office(3))
    assert len(set(drawn)) > 1
    assert all(DOOR_WIDTH[0] <= w <= DOOR_WIDTH[1] for w in drawn)
    narrow = office(3, door_width=0.6)
    assert _door_widths(narrow) == pytest.approx([0.6] * len(drawn))
    assert narrow.params["width"] == office(3).params["width"]


def test_door_clutter_adds_one_box_past_every_doorway() -> None:
    plain, cluttered = office(3), office(3, door_clutter=True)
    assert len(cluttered.boxes) == len(plain.boxes) + len(plain.doors)
    assert cluttered.boxes[: len(plain.boxes)] == plain.boxes
    assert len(plain.doors) == 4


def test_degenerate_boxes_are_dropped() -> None:
    scene = Scene("empty")
    scene.add((0.0, 0.0, 0.0), (1.0, 0.0, 1.0), "wall")
    assert scene.boxes == []


def test_generate_dispatches_by_family() -> None:
    assert generate("office", 3).digest() == office(3).digest()
    assert generate("room", 3, width=6.5).digest() == room(3, width=6.5).digest()


def test_room_is_deterministic_and_pinned() -> None:
    assert room(3).digest() == room(3).digest() == "5833274ec7fd2415"
    assert room(3).digest() != room(4).digest()


def test_room_is_empty_and_enclosed() -> None:
    scene = room(1)
    width, length, z0 = scene.params["width"], scene.params["length"], scene.params["z0"]
    assert sorted(box.kind for box in scene.boxes) == ["floor", *["wall"] * 4]
    assert scene.doors == [] and scene.start == (1.0, 1.0, z0)
    assert _reachable_fraction(scene) == 1.0
    lo, hi = scene.bounds()
    assert lo[:2] == pytest.approx([-WALL_THICKNESS] * 2)
    assert hi[:2] == pytest.approx([width + WALL_THICKNESS, length + WALL_THICKNESS])


def test_room_size_is_drawn_unless_given() -> None:
    assert 6 <= room(2).params["width"] <= 8 and 5 <= room(2).params["length"] <= 7
    sized = room(2, width=6.5, length=5.5)
    assert (sized.params["width"], sized.params["length"]) == (6.5, 5.5)
    assert sized.params["z0"] == room(2).params["z0"]


def test_doorway_is_deterministic_and_pinned() -> None:
    assert doorway(3).digest() == doorway(3).digest() == "45dc20e43b3572f6"
    assert doorway(3).digest() != doorway(4).digest()


def test_doorway_joins_two_empty_rooms_through_one_door() -> None:
    for seed in range(1, 21):
        scene = doorway(seed)
        (door,) = scene.doors
        length = scene.params["length"]
        assert {box.kind for box in scene.boxes} == {"floor", "wall"}
        assert door.axis == 0 and DOOR_WIDTH[0] <= door.width <= DOOR_WIDTH[1]
        assert door.start >= 1.0 and door.start + door.width <= length - 1.0
        assert 1.0 + START_CLEARANCE < door.at < scene.params["width"] - 3.0
        assert _reachable_fraction(scene) == 1.0, seed


def test_doorway_width_is_drawn_unless_given() -> None:
    drawn, narrow = doorway(3), doorway(3, door_width=0.5)
    assert _door_widths(narrow) == pytest.approx([0.5])
    assert narrow.doors[0].at == drawn.doors[0].at
    assert narrow.params == drawn.params


def test_table_is_deterministic_and_pinned() -> None:
    assert table(3).digest() == table(3).digest() == "9618ac5b4763012c"
    assert table(3).digest() != table(4).digest()


def test_table_stands_in_open_floor_on_four_legs_of_the_given_width() -> None:
    for seed in range(1, 21):
        scene = table(seed, leg_width=0.02)
        z0, tx, ty = (scene.params[k] for k in ("z0", "table_x", "table_y"))
        top, *legs = [box for box in scene.boxes if box.kind == "clutter"]
        assert top.center[2] + top.half[2] == pytest.approx(z0 + TABLE_HEIGHT)
        assert (2 * top.half[0], 2 * top.half[1]) == pytest.approx(TABLE_SIZE)
        assert len(legs) == 4 and len({leg.center[:2] for leg in legs}) == 4
        for leg in legs:
            assert leg.half[:2] == pytest.approx((0.01, 0.01))
            assert leg.center[2] - leg.half[2] == pytest.approx(z0)
            assert leg.center[2] + leg.half[2] == pytest.approx(top.center[2] - top.half[2])
            assert tx <= leg.center[0] <= tx + TABLE_SIZE[0]
            assert ty <= leg.center[1] <= ty + TABLE_SIZE[1]
        assert tx > 1.0 + START_CLEARANCE and tx + TABLE_SIZE[0] < scene.params["width"] - 3.0
        assert ty > 1.5 and ty + TABLE_SIZE[1] < scene.params["length"] - 1.5


def test_table_leg_width_defaults_and_can_be_removed() -> None:
    drawn = [box for box in table(3).boxes if box.kind == "clutter"]
    assert all(box.half[0] == pytest.approx(TABLE_LEG / 2) for box in drawn[1:])
    assert drawn[0].half[2] == pytest.approx(TABLE_TOP_THICKNESS / 2)
    bare = table(3, leg_width=0.0)
    assert len([box for box in bare.boxes if box.kind == "clutter"]) == 1
    assert bare.params == table(3).params


def test_block_is_deterministic_and_pinned() -> None:
    assert block(3).digest() == block(3).digest() == "375c1d95815ff1af"
    assert block(3).digest() != block(4).digest()


def test_block_stands_in_open_floor_at_the_given_height() -> None:
    for seed in range(1, 21):
        scene = block(seed, block_height=0.12)
        z0, x, y = (scene.params[k] for k in ("z0", "block_x", "block_y"))
        (box,) = [b for b in scene.boxes if b.kind == "clutter"]
        assert box.center[:2] == pytest.approx((x, y))
        assert box.half == pytest.approx((BLOCK_SIZE / 2, BLOCK_SIZE / 2, 0.06))
        assert box.center[2] - box.half[2] == pytest.approx(z0)
        assert x > 1.0 + START_CLEARANCE + 1.0 and x < scene.params["width"] - 4.0
        assert 2.0 < y < scene.params["length"] - 2.0


def test_block_height_is_drawn_unless_given_and_zero_removes_it() -> None:
    (drawn,) = [b for b in block(3).boxes if b.kind == "clutter"]
    assert 0.1 <= 2 * drawn.half[2] <= 0.4
    bare = block(3, block_height=0.0)
    assert not [b for b in bare.boxes if b.kind == "clutter"]
    assert bare.params == block(3).params


def test_overhang_is_deterministic_and_pinned() -> None:
    assert overhang(3).digest() == overhang(3).digest() == "452ccec8edfe8ec0"
    assert overhang(3).digest() != overhang(4).digest()


def test_overhang_is_a_slab_on_posts_with_its_underside_at_the_clearance() -> None:
    for seed in range(1, 21):
        scene = overhang(seed, clearance=0.5)
        z0, x, y = (scene.params[k] for k in ("z0", "slab_x", "slab_y"))
        slab, *posts = [b for b in scene.boxes if b.kind == "clutter"]
        assert slab.center[:2] == pytest.approx((x, y))
        assert (2 * slab.half[0], 2 * slab.half[1]) == pytest.approx(OVERHANG_SIZE)
        assert slab.center[2] - slab.half[2] == pytest.approx(z0 + 0.5)
        assert len(posts) == 4
        for post in posts:
            assert post.half[:2] == pytest.approx((OVERHANG_LEG / 2, OVERHANG_LEG / 2))
            assert post.center[2] + post.half[2] == pytest.approx(z0 + 0.5)
        assert (
            2.0 < y - OVERHANG_SIZE[1] / 2
            and y + OVERHANG_SIZE[1] / 2 < scene.params["length"] - 2.0
        )


def test_overhang_clearance_is_drawn_unless_given_and_zero_removes_it() -> None:
    slab = next(b for b in overhang(3).boxes if b.kind == "clutter")
    assert 0.3 <= slab.center[2] - slab.half[2] - overhang(3).params["z0"] <= 0.9
    bare = overhang(3, clearance=0.0)
    assert not [b for b in bare.boxes if b.kind == "clutter"]
    assert bare.params == overhang(3).params


def _gap(scene: Scene, x: float, y: float, axis: int) -> tuple[float, float]:
    """The nearest wall faces on both sides of a point along an axis."""
    at = (x, y)
    below, above = [], []
    for box in scene.boxes:
        if box.kind != "wall" or abs(at[1 - axis] - box.center[1 - axis]) >= box.half[1 - axis]:
            continue
        low, high = box.center[axis] - box.half[axis], box.center[axis] + box.half[axis]
        if high <= at[axis]:
            below.append(high)
        else:
            above.append(low)
    return max(below), min(above)


def test_corridor_is_deterministic_and_pinned() -> None:
    assert corridor(3).digest() == corridor(3).digest() == "f70db16e317a35aa"
    assert corridor(3).digest() != corridor(4).digest()
    assert corridor_turn(3).digest() == corridor_turn(3).digest() == "0fc849a623765780"
    assert corridor_turn(3).digest() != corridor_turn(4).digest()


def test_corridor_joins_two_empty_rooms_between_walls_the_given_width_apart() -> None:
    for seed in range(1, 21):
        scene = corridor(seed, corridor_width=0.6)
        width, length = scene.params["width"], scene.params["length"]
        x, y, room_x = (scene.params[k] for k in ("corridor_x", "corridor_y", "room_x"))
        assert sorted(box.kind for box in scene.boxes) == ["floor", *["wall"] * 6]
        assert CORRIDOR_LENGTH[0] <= room_x - x <= CORRIDOR_LENGTH[1]
        assert 1.0 <= y <= length - 1.0 and scene.params["room_y"] == 0.0
        assert x >= 1.0 + START_CLEARANCE and width - room_x >= 4.0
        for along in (x + 0.01, (x + room_x) / 2, room_x - 0.01):
            assert _gap(scene, along, y, 1) == pytest.approx((y - 0.3, y + 0.3))
        assert _gap(scene, x - 0.01, y, 1) == pytest.approx((0.0, length))
        assert _gap(scene, room_x + 0.01, y, 1) == pytest.approx((0.0, length))
        assert _gap(scene, (x + room_x) / 2, y, 0) == pytest.approx((0.0, width))
        assert _reachable_fraction(scene) == 1.0, seed
        lo, hi = scene.bounds()
        assert lo[:2] == pytest.approx([-WALL_THICKNESS] * 2)
        assert hi[:2] == pytest.approx([width + WALL_THICKNESS, length + WALL_THICKNESS])


def test_corridor_turn_bends_once_between_walls_the_given_width_apart() -> None:
    for seed in range(1, 21):
        scene = corridor_turn(seed, corridor_width=0.6)
        width, length = scene.params["width"], scene.params["length"]
        x, y, turn = (scene.params[k] for k in ("corridor_x", "corridor_y", "turn_x"))
        room_x, room_y = scene.params["room_x"], scene.params["room_y"]
        assert sorted(box.kind for box in scene.boxes) == ["floor", *["wall"] * 10]
        first, second = turn - x, room_y - y
        assert CORRIDOR_LENGTH[0] <= first + second <= CORRIDOR_LENGTH[1]
        assert 0.45 <= first / (first + second) <= 0.55
        assert x + 0.5 < room_x == pytest.approx(turn - 1.0)
        assert width - room_x >= 4.0 and length - room_y >= 4.0
        assert _gap(scene, (x + turn - 0.3) / 2, y, 1) == pytest.approx((y - 0.3, y + 0.3))
        assert _gap(scene, turn, (y + 0.3 + room_y) / 2, 0) == pytest.approx(
            (turn - 0.3, turn + 0.3)
        )
        assert _gap(scene, turn, y, 0) == pytest.approx((0.0, turn + 0.3))
        assert _gap(scene, turn, y, 1) == pytest.approx((y - 0.3, length))
        assert _gap(scene, room_x + 0.01, room_y + 0.01, 0) == pytest.approx((room_x, width))
        assert _gap(scene, x - 0.01, y, 1) == pytest.approx((0.0, length))
        assert _reachable_fraction(scene) == 1.0, seed


def test_corridor_width_is_drawn_unless_given() -> None:
    for family in (corridor, corridor_turn):
        drawn, narrow = family(3), family(3, corridor_width=0.5)
        x, y = drawn.params["corridor_x"] + 0.01, drawn.params["corridor_y"]
        low, high = _gap(drawn, x, y, 1)
        assert CORRIDOR_WIDTH[0] <= high - low <= CORRIDOR_WIDTH[1]
        assert (low + high) / 2 == pytest.approx(y)
        assert _gap(narrow, x, y, 1) == pytest.approx((y - 0.25, y + 0.25))
        assert narrow.params == drawn.params
