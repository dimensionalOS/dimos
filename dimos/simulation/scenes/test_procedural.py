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
import itertools
import math

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.simulation.scenes.procedural import (
    BLOCK_SIZE,
    CORRIDOR_LENGTH,
    CORRIDOR_WIDTH,
    DOOR_HEIGHT,
    DOOR_WIDTH,
    HIDDEN_CLUTTER_ASTRIDE,
    HIDDEN_CORRIDOR_WIDTH,
    HIDDEN_ROOM_DEPTH,
    OVERHANG_LEG,
    OVERHANG_SIZE,
    SCATTER_AREA,
    SCATTER_COUNT,
    SCATTER_HEIGHT,
    SCATTER_SIDE,
    SLAB_THICKNESS,
    SMALL_OBJECT_ASPECT,
    SMALL_OBJECT_HEIGHT,
    SMALL_OBJECT_LENGTH,
    START_CLEARANCE,
    TABLE_HEIGHT,
    TABLE_LEG,
    TABLE_SIZE,
    TABLE_TOP_THICKNESS,
    WALL_HEIGHT,
    WALL_THICKNESS,
    WINDING_LENGTH,
    WINDING_SEGMENT,
    WINDING_SPAN,
    WINDING_TURN,
    Box,
    Scene,
    _winding,
    block,
    corridor,
    corridor_turn,
    doorway,
    generate,
    hidden_clutter,
    hidden_object,
    office,
    overhang,
    room,
    scattered,
    small_object,
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


def _reached(
    scene: Scene, cell: float = 0.1, inflate: float = 0.17
) -> tuple[NDArray[np.bool_], NDArray[np.bool_]]:
    """The cells reached from the start, and the cells blocked, on a grid at body height.

    Boxes are grown by the body's half width.
    """
    width, length, z0 = scene.params["width"], scene.params["length"], scene.params["z0"]
    nx, ny = int(width / cell), int(length / cell)
    xs, ys = (np.arange(nx) + 0.5) * cell, (np.arange(ny) + 0.5) * cell
    centers = np.stack(np.meshgrid(xs, ys, indexing="ij"), axis=-1)
    blocked = np.zeros((nx, ny), dtype=bool)
    for box in scene.boxes:
        low, high = box.center[2] - box.half[2], box.center[2] + box.half[2]
        if box.kind == "floor" or high < z0 + 0.02 or low > z0 + 0.45:
            continue
        blocked |= box.gap(centers) < inflate
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
    return seen, blocked


def _reachable_fraction(scene: Scene, cell: float = 0.1, inflate: float = 0.17) -> float:
    """Free floor reachable from the start on a grid at body height, grown by the body's half width."""
    seen, blocked = _reached(scene, cell, inflate)
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


def test_small_object_is_deterministic_and_pinned() -> None:
    assert small_object(3).digest() == small_object(3).digest() == "7f6131007ec27606"
    assert small_object(3).digest() != small_object(4).digest()


def test_small_object_lies_low_in_open_floor_at_a_drawn_angle() -> None:
    yaws = set()
    for seed in range(1, 21):
        scene = small_object(seed, object_length=0.28)
        z0, x, y = (scene.params[k] for k in ("z0", "object_x", "object_y"))
        (box,) = [b for b in scene.boxes if b.kind == "clutter"]
        assert box.center == pytest.approx((x, y, z0 + SMALL_OBJECT_HEIGHT / 2))
        assert box.half == pytest.approx(
            (0.14, 0.14 * SMALL_OBJECT_ASPECT, SMALL_OBJECT_HEIGHT / 2)
        )
        assert -math.pi / 2 <= box.yaw <= math.pi / 2
        assert x > 1.0 + START_CLEARANCE + 1.0 and x < scene.params["width"] - 4.0
        assert 2.0 < y < scene.params["length"] - 2.0
        yaws.add(round(box.yaw, 1))
    assert len(yaws) > 10


def test_small_object_length_is_drawn_unless_given_and_zero_removes_it() -> None:
    (drawn,) = [b for b in small_object(3).boxes if b.kind == "clutter"]
    assert SMALL_OBJECT_LENGTH[0] <= 2 * drawn.half[0] <= SMALL_OBJECT_LENGTH[1]
    bare = small_object(3, object_length=0.0)
    assert not [b for b in bare.boxes if b.kind == "clutter"]
    assert bare.params == small_object(3).params


def test_hidden_object_is_deterministic_and_pinned() -> None:
    assert hidden_object(3).digest() == hidden_object(3).digest() == "d536559f053535e1"
    assert hidden_object(3).digest() != hidden_object(4).digest()
    assert hidden_object(3, object_length=0.0).params == hidden_object(3).params


def test_hidden_object_lies_in_the_corridor_out_of_sight_of_the_first_room() -> None:
    for seed in range(1, 21):
        scene = hidden_object(seed, object_length=0.5)
        params = scene.params
        (box,) = [b for b in scene.boxes if b.kind == "clutter"]
        spot = np.array([params["object_x"], params["object_y"]])
        assert box.center[:2] == pytest.approx(spot) and 2 * box.half[0] == pytest.approx(0.5)
        walls = [b for b in scene.boxes if b.kind == "wall"]
        assert min(wall.gap(spot) for wall in walls) >= HIDDEN_CORRIDOR_WIDTH / 2 - 1e-6
        assert _reached(scene, cell=0.05, inflate=0.16)[0][_far_room(scene, cell=0.05)], seed
        for x in np.arange(0.5, params["corridor_x"], 0.5):
            for y in np.arange(0.5, params["length"], 0.5):
                line = np.linspace((x, y), spot, 200)
                assert min(float(wall.gap(line).min()) for wall in walls) == 0.0, (seed, x, y)


def test_hidden_clutter_is_deterministic_and_pinned() -> None:
    assert hidden_clutter(3).digest() == hidden_clutter(3).digest() == "86f98684b5d2afd0"
    assert hidden_clutter(3).digest() != hidden_clutter(4).digest()


def test_hidden_clutter_lies_across_the_way_to_the_goal_out_of_sight_of_the_first_room() -> None:
    for seed in range(1, 11):
        full = hidden_clutter(seed, clutter_count=6)
        params = full.params
        boxes = [box for box in full.boxes if box.kind == "clutter"]
        walls = [box for box in full.boxes if box.kind == "wall"]
        assert len(boxes) == 6 and boxes == full.boxes[-6:]
        for count in (0, 3):
            fewer = hidden_clutter(seed, clutter_count=count)
            assert fewer.params == params
            assert fewer.boxes == full.boxes[: len(full.boxes) - 6 + count]
        assert HIDDEN_ROOM_DEPTH[0] <= params["room_depth"] <= HIDDEN_ROOM_DEPTH[1]
        c, s = math.cos(params["room_yaw"]), math.sin(params["room_yaw"])
        ahead = WALL_THICKNESS + 0.5
        door = np.array([params["room_x"] + ahead * c, params["room_y"] + ahead * s])
        goal = np.array([params["goal_x"], params["goal_y"]])
        way = (goal - door) / np.linalg.norm(goal - door)
        assert np.linalg.norm(goal - door) >= HIDDEN_ROOM_DEPTH[0] - 2.0
        for box in boxes:
            assert box.yaw == params["room_yaw"]
            offset = np.array(box.center[:2]) - door
            assert abs(way[0] * offset[1] - way[1] * offset[0]) <= HIDDEN_CLUTTER_ASTRIDE + 1e-9
            assert 0.0 < float(offset @ way) < np.linalg.norm(goal - door)
            assert box.gap(goal) >= START_CLEARANCE - 1e-9
        assert _reached(full, cell=0.05, inflate=0.16)[0][
            int(goal[0] / 0.05), int(goal[1] / 0.05)
        ], seed
        for x in np.arange(0.5, params["corridor_x"], 1.0):
            for y in np.arange(0.5, params["length"], 1.0):
                for box in boxes:
                    line = np.linspace((x, y), box.center[:2], 200)
                    assert min(float(wall.gap(line).min()) for wall in walls) == 0.0, (seed, x, y)


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
    assert corridor_turn(3).digest() == corridor_turn(3).digest() == "f25fbdad34ae379a"
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


def _far_room(scene: Scene, cell: float = 0.1) -> tuple[int, int]:
    """The grid cell at the middle of a winding corridor's far room."""
    params = scene.params
    ahead = WALL_THICKNESS + params["room_depth"] / 2
    across = (params["room_left"] - params["room_right"]) / 2
    c, s = math.cos(params["room_yaw"]), math.sin(params["room_yaw"])
    x, y = params["room_x"] + ahead * c - across * s, params["room_y"] + ahead * s + across * c
    return int(x / cell), int(y / cell)


def test_corridor_turn_winds_a_drawn_number_of_turns_as_the_only_way_between_the_rooms() -> None:
    turns = set()
    for seed in range(1, 21):
        scene = corridor_turn(seed, corridor_width=0.6)
        params = scene.params
        x, y, z0 = params["corridor_x"], params["corridor_y"], params["z0"]
        turns.add(params["turns"])
        assert WINDING_LENGTH[0] <= params["corridor_length"] <= WINDING_LENGTH[1] + 1e-9
        assert 1 <= params["turns"] <= params["corridor_length"] / WINDING_SEGMENT[0] - 1
        assert params["room_depth"] >= 4.0 and params["room_left"] + params["room_right"] >= 4.0
        assert min(params["room_left"], params["room_right"]) >= 1.0
        assert params["length"] <= WINDING_SPAN + 1e-9
        lo, hi = scene.bounds()
        assert lo[:2] == pytest.approx([-WALL_THICKNESS] * 2)
        assert hi[:2] == pytest.approx(
            [params["width"] + WALL_THICKNESS, params["length"] + WALL_THICKNESS]
        )
        room = _far_room(scene)
        assert _reached(scene, inflate=0.2)[0][room], seed
        assert not _reached(scene, inflate=0.33)[0][room], seed
        assert corridor_turn(seed, corridor_width=1.2).params == params
        assert _reached(corridor_turn(seed, corridor_width=1.2), inflate=0.33)[0][room], seed
        scene.add((x, y - 0.3, z0), (x + 0.1, y + 0.3, z0 + 1.0), "wall")
        assert not _reached(scene, inflate=0.2)[0][room], seed
    assert len(turns) >= 3


def test_winding_corridors_turn_both_ways_by_drawn_angles_in_runs_no_shorter_than_the_least() -> (
    None
):
    turns = []
    for seed in range(1, 21):
        points, heading = _winding(np.random.default_rng(seed))
        steps = [(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in itertools.pairwise(points)]
        assert min(math.hypot(*step) for step in steps) >= WINDING_SEGMENT[0] - 1e-9
        headings = [math.atan2(dy, dx) for dx, dy in steps]
        assert headings[0] == 0.0
        bends = [math.remainder(b - a, math.tau) for a, b in itertools.pairwise(headings)]
        assert math.remainder(sum(bends) - heading, math.tau) == pytest.approx(0.0, abs=1e-9)
        turns += bends
    assert all(WINDING_TURN[0] - 1e-9 <= abs(turn) <= WINDING_TURN[1] + 1e-9 for turn in turns)
    assert min(turns) < 0 < max(turns)
    assert max(map(abs, turns)) - min(map(abs, turns)) > math.radians(30)


def test_corridor_width_is_drawn_unless_given() -> None:
    drawn, narrow = corridor(3), corridor(3, corridor_width=0.5)
    x, y = drawn.params["corridor_x"] + 0.01, drawn.params["corridor_y"]
    low, high = _gap(drawn, x, y, 1)
    assert CORRIDOR_WIDTH[0] <= high - low <= CORRIDOR_WIDTH[1]
    assert (low + high) / 2 == pytest.approx(y)
    assert _gap(narrow, x, y, 1) == pytest.approx((y - 0.25, y + 0.25))
    assert narrow.params == drawn.params
    assert corridor_turn(3).params == corridor_turn(3, corridor_width=0.5).params


def test_a_turned_box_measures_gaps_and_extents_in_its_own_frame() -> None:
    box = Box((1.0, 2.0, 0.5), (2.0, 0.5, 0.5), "wall", math.pi / 2)
    assert box.reach() == pytest.approx((0.5, 2.0, 0.5))
    points = np.array([[1.0, 2.0], [1.0, 4.5], [2.5, 2.0], [1.4, 3.9]])
    assert box.gap(points) == pytest.approx([0.0, 0.5, 1.0, 0.0])
    scene = Scene("turned", boxes=[box])
    lo, hi = scene.bounds()
    assert lo == pytest.approx([0.5, 0.0, 0.0]) and hi == pytest.approx([1.5, 4.0, 1.0])
    assert scene.digest() != Scene("flat", boxes=[Box(box.center, box.half, "wall")]).digest()


def test_scattered_is_deterministic_and_pinned() -> None:
    assert scattered(3).digest() == scattered(3).digest() == "69770d2ca9c94a86"
    assert scattered(3).digest() != scattered(4).digest()


def test_scattered_stands_the_given_count_of_boxes_clear_of_the_start_and_goal() -> None:
    for seed in range(1, 21):
        scene = scattered(seed, clutter_count=80)
        width, length, z0 = (scene.params[k] for k in ("width", "length", "z0"))
        goal = (scene.params["goal_x"], scene.params["goal_y"])
        boxes = [box for box in scene.boxes if box.kind == "clutter"]
        assert sorted(box.kind for box in scene.boxes[:5]) == ["floor", *["wall"] * 4]
        assert len(boxes) == 80 and boxes == scene.boxes[5:]
        assert 8 <= width <= 10 and width * length == pytest.approx(SCATTER_AREA)
        assert 1.5 <= width - goal[0] <= 2.5 and 1.5 <= goal[1] <= length - 1.5
        assert len({box.half for box in boxes}) == 80
        for box in boxes:
            assert all(SCATTER_SIDE[0] <= 2 * h <= SCATTER_SIDE[1] for h in box.half[:2])
            assert SCATTER_HEIGHT[0] <= 2 * box.half[2] <= SCATTER_HEIGHT[1]
            assert box.center[2] - box.half[2] == pytest.approx(z0)
            assert box.half[0] <= box.center[0] <= width - box.half[0]
            assert box.half[1] <= box.center[1] <= length - box.half[1]
            for x, y in (scene.start[:2], goal):
                apart = max(
                    abs(box.center[0] - x) - box.half[0], abs(box.center[1] - y) - box.half[1]
                )
                assert apart >= START_CLEARANCE - 1e-9
        for i, a in enumerate(boxes):
            for b in boxes[:i]:
                apart = max(
                    abs(a.center[0] - b.center[0]) - a.half[0] - b.half[0],
                    abs(a.center[1] - b.center[1]) - a.half[1] - b.half[1],
                )
                assert apart >= -1e-9


def test_scattered_count_only_adds_boxes_and_zero_leaves_the_room_empty() -> None:
    full = scattered(3, clutter_count=80)
    for count in (0, 10, 40):
        fewer = scattered(3, clutter_count=count)
        assert fewer.boxes == full.boxes[: 5 + count] and fewer.params == full.params
    assert len(scattered(3).boxes) == 5 + SCATTER_COUNT
    assert generate("scattered", 3, clutter_count=40.0).boxes == scattered(3).boxes[:45]
