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

from collections.abc import Iterator
import math

import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import spec_annotation_compliance


def _cloud(
    points: np.ndarray, *, frame_id: str = "world", timestamp: float | None = 1.0
) -> PointCloud2:
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id=frame_id, timestamp=timestamp)


@pytest.fixture
def module() -> Iterator[HeuristicGraspModule]:
    instance = HeuristicGraspModule()
    yield instance
    instance.stop()


def test_heuristic_grasp_implements_grasp_provider_spec(module: HeuristicGraspModule) -> None:
    assert spec_annotation_compliance(module, GraspGenSpec)


def test_heuristic_grasp_proposes_centered_top_down_pose(module: HeuristicGraspModule) -> None:
    proposals = module.propose_grasps(
        _cloud(
            np.asarray(
                [
                    [-0.10, -0.02, 0.10],
                    [-0.10, 0.02, 0.10],
                    [0.10, -0.02, 0.20],
                    [0.10, 0.02, 0.20],
                ]
            )
        )
    )

    assert len(proposals.candidates) == 1
    assert proposals.header.frame_id == "world"
    assert proposals.header.timestamp == pytest.approx(1.0)
    pose = proposals.candidates[0].pose
    assert pose.position.x == pytest.approx(0.0)
    assert pose.position.y == pytest.approx(0.0)
    assert pose.position.z == pytest.approx(0.15)
    approach = pose.orientation.rotate_vector(Vector3(0.0, 0.0, 1.0))
    assert approach.z == pytest.approx(-1.0)
    assert proposals.candidates[0].score == pytest.approx(1.0)


def test_heuristic_grasp_aligns_jaw_axis_with_narrow_axis(module: HeuristicGraspModule) -> None:
    proposals = module.propose_grasps(
        _cloud(
            np.asarray(
                [
                    [-0.02, -0.10, 0.10],
                    [0.02, -0.10, 0.10],
                    [-0.02, 0.10, 0.20],
                    [0.02, 0.10, 0.20],
                ]
            )
        )
    )

    jaw_axis = proposals.candidates[0].pose.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0))
    assert abs(jaw_axis.x) == pytest.approx(1.0)
    assert jaw_axis.y == pytest.approx(0.0, abs=1e-6)
    assert jaw_axis.z == pytest.approx(0.0, abs=1e-6)


def _block(yaw: float, size: tuple[float, float] = (0.04, 0.04)) -> np.ndarray:
    """A box's top and side surfaces, turned ``yaw`` about z, centred at (0.5, 0.1)."""
    rng = np.random.default_rng(0)
    local = rng.uniform(-0.5, 0.5, (400, 3)) * np.asarray([*size, 0.04])
    turn = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    xy = local[:, :2] @ turn.T + np.asarray([0.5, 0.1])
    return np.column_stack([xy, local[:, 2] + 0.83])


@pytest.mark.parametrize("yaw_deg", [0.0, 17.0, 30.0, 45.0, -38.0])
def test_square_block_jaws_close_across_its_faces_at_any_yaw(
    module: HeuristicGraspModule, yaw_deg: float
) -> None:
    proposals = module.propose_grasps(_cloud(_block(math.radians(yaw_deg))))

    assert len(proposals.candidates) == 1  # the two sides are the same width
    pose = proposals.candidates[0].pose
    assert (pose.position.x, pose.position.y) == pytest.approx((0.5, 0.1), abs=2e-3)
    jaw = pose.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0))
    jaw_deg = math.degrees(math.atan2(jaw.y, jaw.x))
    # Square: either pair of faces; the jaw axis lies along a face normal, modulo 90 degrees.
    assert (jaw_deg - yaw_deg + 45.0) % 90.0 - 45.0 == pytest.approx(0.0, abs=2.0)


def test_narrow_block_gets_both_sides_when_the_jaws_span_the_long_one(
    module: HeuristicGraspModule,
) -> None:
    proposals = module.propose_grasps(_cloud(_block(math.radians(20.0), size=(0.07, 0.03))))

    first, second = proposals.candidates
    assert first.score > second.score
    jaws = [c.pose.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0)) for c in (first, second)]
    # First across the 3 cm side, i.e. along the box's local y axis at 20 + 90 degrees.
    first_deg = math.degrees(math.atan2(jaws[0].y, jaws[0].x))
    assert (first_deg - 110.0 + 90.0) % 180.0 - 90.0 == pytest.approx(0.0, abs=2.0)
    assert abs(jaws[0].x * jaws[1].x + jaws[0].y * jaws[1].y) == pytest.approx(0.0, abs=1e-6)


def test_a_cloud_of_only_the_top_is_grasped_below_it(module: HeuristicGraspModule) -> None:
    top = _block(0.0)
    top[:, 2] = 0.85

    pose = module.propose_grasps(_cloud(top)).candidates[0].pose

    assert pose.position.z == pytest.approx(0.83)


def test_too_long_a_side_is_not_offered(module: HeuristicGraspModule) -> None:
    proposals = module.propose_grasps(_cloud(_block(0.0, size=(0.20, 0.04))))

    assert len(proposals.candidates) == 1


def _bin(yaw: float, size: tuple[float, float] = (0.30, 0.10), wall: float = 0.005) -> np.ndarray:
    """An open bin's floor, wall faces and rims as a camera above sees them, 6 cm deep, turned
    ``yaw`` about z, centred at (0.56, 0.0) with its floor at z 0.80."""
    rng = np.random.default_rng(1)
    length, width = size
    floor = np.column_stack([rng.uniform(-0.5, 0.5, (600, 2)) * [length, width], np.full(600, 0.0)])
    walls = []
    for axis, half, other in ((0, length / 2, width / 2), (1, width / 2, length / 2)):
        for sign in (1.0, -1.0):
            n = 300
            local = np.zeros((n, 3))
            local[:, axis] = sign * (half - rng.uniform(0.0, wall, n))
            local[:, 1 - axis] = rng.uniform(-other, other, n)
            local[:, 2] = rng.uniform(0.0, 0.06, n)
            local[: n // 4, 2] = 0.06  # the rim's top face, seen from above
            walls.append(local)
    local = np.vstack([floor, *walls])
    turn = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    xy = local[:, :2] @ turn.T + np.asarray([0.56, 0.0])
    return np.column_stack([xy, local[:, 2] + 0.80])


@pytest.mark.parametrize("yaw_deg", [0.0, 25.0, -60.0])
def test_an_open_bin_wider_than_the_jaws_is_grasped_on_its_walls(
    module: HeuristicGraspModule, yaw_deg: float
) -> None:
    yaw = math.radians(yaw_deg)
    proposals = module.propose_grasps(_cloud(_bin(yaw)))

    assert len(proposals.candidates) == 4
    long_walls, short_walls = proposals.candidates[:2], proposals.candidates[2:]
    assert all(c.score > s.score for c in long_walls for s in short_walls)
    side = np.asarray(
        [-math.sin(yaw), math.cos(yaw)]
    )  # the bin's short side, across the long walls
    for c in long_walls:
        p = c.pose.position
        offset = np.asarray([p.x - 0.56, p.y])
        # On a long wall (about 5 cm from the middle), at its middle, 3 cm above the floor.
        assert abs(offset @ side) == pytest.approx(0.0475, abs=4e-3)
        assert abs(offset @ np.asarray([math.cos(yaw), math.sin(yaw)])) < 0.02
        assert p.z == pytest.approx(0.83, abs=3e-3)
        jaw = c.pose.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0))
        assert abs(jaw.x * side[0] + jaw.y * side[1]) == pytest.approx(1.0, abs=1e-3)
        assert c.pose.orientation.rotate_vector(Vector3(0.0, 0.0, 1.0)).z == pytest.approx(-1.0)


def test_unseen_walls_are_placed_just_beyond_the_floor(module: HeuristicGraspModule) -> None:
    # What localize gave in the sim: the bin's floor and one end wall's inner face, no side walls.
    cloud = _bin(0.0, size=(0.29, 0.086))
    keep = (cloud[:, 2] < 0.801) | (cloud[:, 0] > 0.56 + 0.14)
    proposals = module.propose_grasps(_cloud(cloud[keep]))

    long_walls = [c for c in proposals.candidates if c.score == pytest.approx(0.9)]
    assert len(long_walls) == 2
    for c in long_walls:
        p = c.pose.position
        assert abs(p.y) == pytest.approx(0.043 + 0.004, abs=3e-3)
        assert p.z == pytest.approx(0.83, abs=3e-3)


def test_a_solid_object_wider_than_the_jaws_gets_no_grasp(module: HeuristicGraspModule) -> None:
    proposals = module.propose_grasps(_cloud(_block(0.3, size=(0.12, 0.12))))

    assert proposals.candidates == []


@pytest.mark.parametrize(
    "points, frame_id, timestamp, error",
    [
        (
            np.asarray([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]),
            "world",
            1.0,
            "at least three",
        ),
        (np.asarray([[0.0, 0.0, math.nan]] * 3), "world", 1.0, "finite"),
        (np.zeros((3, 3)), "", 1.0, "frame_id"),
        (np.zeros((3, 3)), "world", None, "timestamp"),
        (np.zeros((3, 3)), "world", math.nan, "finite timestamp"),
        (np.zeros((3, 3)), "world", math.inf, "finite timestamp"),
    ],
)
def test_heuristic_grasp_rejects_invalid_pointclouds(
    module: HeuristicGraspModule,
    points: np.ndarray,
    frame_id: str,
    timestamp: float | None,
    error: str,
) -> None:
    with pytest.raises(ValueError, match=error):
        module.propose_grasps(_cloud(points, frame_id=frame_id, timestamp=timestamp))


# Held-out shapes: none of these was used to set the heuristic's constants.


def _container(
    outline: str, size: tuple[float, float], depth: float, *, walls_seen: bool = True
) -> np.ndarray:
    """An open container on a table at z 0.80, centred at (0.5, 0.0), as a camera above sees
    it: its floor, and (``walls_seen``) its inner walls and rim. ``outline`` is "rect" or
    "round" (``size`` then gives the diameter twice)."""
    rng = np.random.default_rng(7)
    a, b = size[0] / 2, size[1] / 2
    if outline == "round":
        r = np.sqrt(rng.uniform(0, 1, 800)) * a
        t = rng.uniform(0, 2 * np.pi, 800)
        floor = np.column_stack([r * np.cos(t), r * np.sin(t), np.zeros(800)])
        t = rng.uniform(0, 2 * np.pi, 600)
        wall = np.column_stack([a * np.cos(t), a * np.sin(t), rng.uniform(0, depth, 600)])
    else:
        floor = np.column_stack([rng.uniform(-a, a, 800), rng.uniform(-b, b, 800), np.zeros(800)])
        side = rng.integers(0, 4, 600)
        u = rng.uniform(-1, 1, 600)
        wx = np.where(side < 2, np.where(side == 0, a, -a), u * a)
        wy = np.where(side < 2, u * b, np.where(side == 2, b, -b))
        wall = np.column_stack([wx, wy, rng.uniform(0, depth, 600)])
    wall[: len(wall) // 4, 2] = depth  # the rim's top
    points = np.vstack([floor, wall]) if walls_seen else floor
    return points + np.asarray([0.5, 0.0, 0.80])


def _on_rim(c, outline: str, size: tuple[float, float]) -> bool:
    x, y = c.pose.position.x - 0.5, c.pose.position.y
    if outline == "round":
        return abs(math.hypot(x, y) - size[0] / 2) < 0.008
    return min(abs(abs(x) - size[0] / 2), abs(abs(y) - size[1] / 2)) < 0.008


@pytest.mark.parametrize(
    "outline, size, depth",
    [
        ("round", (0.12, 0.12), 0.05),
        ("round", (0.07, 0.07), 0.08),
        ("rect", (0.16, 0.10), 0.07),
    ],
    ids=["bowl", "cup", "box"],
)
def test_held_out_open_containers_are_held_on_their_rim_below_its_top(
    module: HeuristicGraspModule, outline: str, size: tuple[float, float], depth: float
) -> None:
    proposals = module.propose_grasps(_cloud(_container(outline, size, depth)))

    assert proposals.candidates
    for c in proposals.candidates:
        assert _on_rim(c, outline, size), (c.pose.position, c.note)
        assert 0.80 + 0.01 <= c.pose.position.z <= 0.80 + depth - 0.004
        assert "points seen" in c.note


def test_a_tray_shallower_than_the_open_test_gets_no_grasp(module: HeuristicGraspModule) -> None:
    # A known limit: an open container is one whose floor is seen at least _OPEN_DROP (3 cm)
    # below its top, so a 2.5 cm tray reads as a solid too wide for the jaws.
    for walls_seen in (True, False):
        cloud = _container("rect", (0.20, 0.15), 0.025, walls_seen=walls_seen)
        assert module.propose_grasps(_cloud(cloud)).candidates == []


def test_held_out_small_solids_are_grasped_at_their_centre(module: HeuristicGraspModule) -> None:
    for size in ((0.05, 0.03), (0.06, 0.06), (0.08, 0.02)):
        c = module.propose_grasps(_cloud(_block(0.4, size=size))).candidates[0]
        assert (c.pose.position.x, c.pose.position.y) == pytest.approx((0.5, 0.1), abs=3e-3)
        assert c.note.startswith("footprint centre")
