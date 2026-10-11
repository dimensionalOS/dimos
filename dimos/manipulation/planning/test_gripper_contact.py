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

"""Grasp contact checks on the xArm Gripper and on a made-up three-finger hand.

Every scene stands on a table at z = 0, world frame, Z up, metres. Grasps come
from above: the tool tip's z axis points down.
"""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path
import time

import numpy as np
from numpy.typing import NDArray
import pytest
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from dimos.manipulation.grasping.grasp_gen_x.module import GripperBox, gripper_body_points
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspConfig, heuristic_grasps
from dimos.manipulation.planning.gripper_contact import (
    GripperContactModel,
    approach_offsets,
    depth_shifts,
)
from dimos.manipulation.planning.spec.gripper import ContactPatch, GraspContact, GripperSpec
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.assets.model import RobotModel
from dimos.utils.transform_utils import pose_to_matrix

CONTACT_MARGIN = 0.005
OBSTACLE_MARGIN = 0.003
MIN_GAP_POINTS = 10
CLEAR = GraspContact()
EMPTY_GAP = GraspContact(empty_gap=True)


def _top_down(x: float, y: float, z: float, yaw: float = 0.0) -> NDArray[np.float64]:
    """A tool tip pose at (x, y, z) pointing straight down, turned ``yaw`` radians about Z."""
    c, s = np.cos(yaw), np.sin(yaw)
    pose = np.eye(4)
    # Half a turn about X (tool z down), then the yaw about world Z.
    pose[:3, :3] = np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])
    pose[:3, 3] = (x, y, z)
    return pose


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> NDArray:
    """Points 2 mm apart on the surface of an axis-aligned box."""
    half = np.asarray(size) / 2.0
    faces = []
    for axis in range(3):
        u, v = (a for a in range(3) if a != axis)
        grid_u = np.arange(-half[u], half[u] + 1e-9, 0.002)
        grid_v = np.arange(-half[v], half[v] + 1e-9, 0.002)
        mesh_u, mesh_v = np.meshgrid(grid_u, grid_v)
        for side in (-1.0, 1.0):
            face = np.zeros((mesh_u.size, 3))
            face[:, u], face[:, v], face[:, axis] = (
                mesh_u.ravel(),
                mesh_v.ravel(),
                side * half[axis],
            )
            faces.append(face)
    return np.concatenate(faces) + np.asarray(center)


def _cylinder(radius: float, height: float) -> NDArray:
    """Points about 2 mm apart on an upright cylinder's side and top, standing on the table."""
    angles = np.linspace(0.0, 2 * np.pi, int(2 * np.pi * radius / 0.002), endpoint=False)
    heights = np.arange(0.0, height + 1e-9, 0.002)
    a, h = np.meshgrid(angles, heights)
    side = np.stack([radius * np.cos(a.ravel()), radius * np.sin(a.ravel()), h.ravel()], axis=1)
    r, a = np.meshgrid(np.arange(0.0, radius, 0.002), angles)
    top = np.stack([(r * np.cos(a)).ravel(), (r * np.sin(a)).ravel(), np.full(r.size, height)], 1)
    return np.concatenate([side, top])


def _table(*, hole: tuple[float, float, float, float] | None = None) -> NDArray:
    """Table points 1 cm apart, as the camera cloud has them, minus the points under ``hole``
    (x_low, x_high, y_low, y_high), where an object stands."""
    x, y = np.meshgrid(np.arange(-0.3, 0.3, 0.01), np.arange(-0.3, 0.3, 0.01))
    points = np.stack([x.ravel(), y.ravel(), np.zeros(x.size)], axis=1)
    if hole is not None:
        x0, x1, y0, y1 = hole
        under = (
            (points[:, 0] > x0) & (points[:, 0] < x1) & (points[:, 1] > y0) & (points[:, 1] < y1)
        )
        points = points[~under]
    return points


def _check(
    model: GripperContactModel,
    grasp: NDArray,
    target: NDArray,
    *,
    min_gap_points: int = 0,
    **obstacles: NDArray,
) -> GraspContact:
    return model.check(
        grasp[None], target, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN, min_gap_points
    )[0]


# The xArm Gripper, from the xArm description the planner loads.


@pytest.fixture(scope="module")
def xarm() -> GripperContactModel:
    from dimos.robot.manipulators.xarm.config import make_xarm7_sim_robot_config

    config = make_xarm7_sim_robot_config()
    assert config.gripper is not None
    return GripperContactModel(config.model.load(), config.gripper, "link_tcp")


# A 4 x 4 x 6 cm box standing at the origin.
BOX = _box((0.0, 0.0, 0.03), (0.04, 0.04, 0.06))
BOX_TABLE = _table(hole=(-0.02, 0.02, -0.02, 0.02))
# Open xArm fingertips sit this far (metres) behind the tool tip, along the approach.
XARM_TIP_BEHIND_TCP = 0.0099
# The left finger's centre, along the tool tip's y axis, metres.
XARM_LEFT_FINGER_Y = 0.06


@pytest.mark.self_hosted
def test_xarm_keeps_a_grasp_with_the_pads_on_the_box_sides(xarm: GripperContactModel) -> None:
    assert _check(xarm, _top_down(0.0, 0.0, 0.03), BOX, table=BOX_TABLE) == CLEAR


@pytest.mark.self_hosted
def test_xarm_rejects_a_fingertip_resting_on_top_of_the_box(xarm: GripperContactModel) -> None:
    # The left finger straight over the box, its tip on the box's top face. The
    # tool tip's y axis points along world -y when it points down.
    resting = _top_down(0.0, XARM_LEFT_FINGER_Y, 0.06 - XARM_TIP_BEHIND_TCP)

    assert _check(xarm, resting, BOX, table=BOX_TABLE) == GraspContact(("target",))


@pytest.mark.self_hosted
def test_xarm_rejects_pads_pressing_into_a_box_wider_than_the_open_jaws(
    xarm: GripperContactModel,
) -> None:
    # The open pads stand 8.89 cm apart; this box is 9.2 cm across them.
    wide = _box((0.0, 0.0, 0.03), (0.04, 0.092, 0.06))
    table = _table(hole=(-0.02, 0.02, -0.046, 0.046))

    assert _check(xarm, _top_down(0.0, 0.0, 0.03), wide, table=table) == GraspContact(("target",))


@pytest.mark.self_hosted
def test_xarm_closing_sweep_catches_fingertips_swinging_into_the_table(
    xarm: GripperContactModel,
) -> None:
    # A narrow block: the pads meet it only near the end of the close, after the
    # fingertips have swung forward about 1.3 cm. Open, they clear the table by 5 mm.
    block = _box((0.0, 0.0, 0.0175), (0.02, 0.02, 0.035))
    table = _table(hole=(-0.01, 0.01, -0.01, 0.01))
    grasp = _top_down(0.0, 0.0, 0.005 - XARM_TIP_BEHIND_TCP)

    assert _check(xarm, grasp, block, table=table) == GraspContact(("table",), while_closing=True)


@pytest.mark.self_hosted
def test_xarm_rejects_the_table_and_a_neighbour(xarm: GripperContactModel) -> None:
    # Too low on a 3 cm box: the open fingertips are already 2 mm into the table.
    short_box = _box((0.0, 0.0, 0.015), (0.04, 0.04, 0.03))
    low = _top_down(0.0, 0.0, -0.002 - XARM_TIP_BEHIND_TCP)
    # A neighbour 3.5 cm from the box, right where the left finger comes down.
    neighbour = _box((0.0, -0.07, 0.03), (0.04, 0.03, 0.06))

    assert _check(xarm, low, short_box, table=BOX_TABLE) == GraspContact(("table",))
    assert _check(xarm, _top_down(0.0, 0.0, 0.03), BOX, table=BOX_TABLE, objects=neighbour) == (
        GraspContact(("objects",))
    )


@pytest.mark.self_hosted
def test_xarm_checks_a_hundred_grasps_at_once(xarm: GripperContactModel) -> None:
    heights = np.linspace(0.0, 0.12, 100)
    grasps = np.stack([_top_down(0.0, 0.0, z, yaw=z * 20.0) for z in heights])

    results = xarm.check(grasps, BOX, {"table": BOX_TABLE}, CONTACT_MARGIN, OBSTACLE_MARGIN)

    assert len(results) == 100
    # Low grasps press the knuckles onto the box top; high ones close above it.
    assert "target" in results[0].touches
    assert results[-1] == CLEAR


@pytest.mark.self_hosted
def test_xarm_approach_offsets_match_the_finger_mesh(xarm: GripperContactModel) -> None:
    from dimos.robot.manipulators.xarm.config import (
        XARM_APPROACH_OFFSETS,
        make_xarm7_sim_robot_config,
    )

    config = make_xarm7_sim_robot_config()
    assert config.gripper is not None
    offsets = approach_offsets(config.model.load(), config.gripper, "link_tcp")
    # Measured on the left finger mesh, in its frame (see xarm_gripper_spec):
    # the pad face's centre and the fingertip's end.
    face = np.array([0.0, -0.026, 0.041, 1.0])
    fingertip = np.array([0.0, 0.0, 0.061, 1.0])
    closed = xarm.poses - 1
    pad_behind = -(xarm.tip_from_link(closed, "left_finger") @ face)[2]
    reach = max((xarm.tip_from_link(s, "left_finger") @ fingertip)[2] for s in range(xarm.poses))

    assert pad_behind == pytest.approx(0.0175, abs=0.0005)
    assert reach == pytest.approx(0.003, abs=0.0005)
    assert offsets.pad_behind_tcp == pytest.approx(pad_behind, abs=0.001)
    assert offsets.fingertip_reach == pytest.approx(reach, abs=0.001)
    assert np.allclose(offsets, XARM_APPROACH_OFFSETS, atol=0.0001)


# The gap the xArm pads close across.


@pytest.mark.self_hosted
def test_xarm_gap_check_keeps_a_grasp_centred_on_the_box(xarm: GripperContactModel) -> None:
    grasp = _top_down(0.0, 0.0, 0.03)

    assert _check(xarm, grasp, BOX, table=BOX_TABLE, min_gap_points=MIN_GAP_POINTS) == CLEAR


@pytest.mark.self_hosted
def test_xarm_gap_check_rejects_pads_closing_beside_the_box(xarm: GripperContactModel) -> None:
    # 5 cm along the pad faces: the pads (4.4 cm wide) close just past the box's side.
    beside = _top_down(0.05, 0.0, 0.03)

    assert _check(xarm, beside, BOX, table=BOX_TABLE, min_gap_points=MIN_GAP_POINTS) == EMPTY_GAP


@pytest.mark.self_hosted
def test_xarm_gap_check_rejects_pads_closing_above_a_short_box(xarm: GripperContactModel) -> None:
    short_box = _box((0.0, 0.0, 0.015), (0.04, 0.04, 0.03))
    # The pads reach at most 8.5 mm past the tool tip while closing: 6.5 mm above the box.
    above = _top_down(0.0, 0.0, 0.045)

    assert (
        _check(xarm, above, short_box, table=BOX_TABLE, min_gap_points=MIN_GAP_POINTS) == EMPTY_GAP
    )


@pytest.mark.self_hosted
@pytest.mark.parametrize("yaw", [0.0, np.pi / 2.0], ids=["closing across view", "along view"])
def test_xarm_gap_check_keeps_a_grasp_on_the_front_of_a_can(
    xarm: GripperContactModel, yaw: float
) -> None:
    # A camera at +x sees only the near half of a 6 cm can's side.
    can = _cylinder(0.03, 0.033)
    side = can[np.isclose(np.hypot(can[:, 0], can[:, 1]), 0.03)]
    front = side[side[:, 0] > 0.0]
    table = _table(hole=(-0.03, 0.03, -0.03, 0.03))
    grasp = _top_down(0.0, 0.0, 0.02, yaw=yaw)

    assert _check(xarm, grasp, front, table=table, min_gap_points=MIN_GAP_POINTS) == CLEAR


@pytest.mark.self_hosted
def test_xarm_gap_check_only_adds_its_own_reason(xarm: GripperContactModel) -> None:
    heights = np.linspace(0.0, 0.12, 25)
    grasps = np.stack(
        [_top_down(x, 0.0, z) for x in (0.0, 0.03, 0.05) for z in heights], dtype=np.float64
    )

    def checked(min_gap_points: int) -> list[GraspContact]:
        return xarm.check(
            grasps, BOX, {"table": BOX_TABLE}, CONTACT_MARGIN, OBSTACLE_MARGIN, min_gap_points
        )

    off = checked(0)
    on = checked(MIN_GAP_POINTS)

    assert off == xarm.check(grasps, BOX, {"table": BOX_TABLE}, CONTACT_MARGIN, OBSTACLE_MARGIN)
    assert not any(result.empty_gap for result in off)
    assert any(result.empty_gap for result in on)
    assert [dataclasses.replace(result, empty_gap=False) for result in on] == off


@pytest.mark.self_hosted
def test_xarm_gap_holds_the_space_the_pads_sweep_while_closing(
    xarm: GripperContactModel,
) -> None:
    # Points in the tool tip's frame. Open, the pads end 4 mm short of the tool
    # tip; closing, they swing 1.2 cm forward and meet on its axis.
    points = np.array(
        [
            [0.0, 0.0, -0.03],  # between the open pads
            [0.0, 0.0, 0.005],  # reached only as the pads swing forward
            [0.0, 0.0, 0.015],  # past the closed pads
            [0.0, 0.06, -0.03],  # outside the open pads
            [0.03, 0.0, -0.03],  # beside the pads, along their faces
        ]
    )

    assert xarm.in_gap(points).tolist() == [True, True, False, False, False]


# Every grasp GraspGenX samples goes through this check.


def _graspgenx_scene_filter_keeps(grasps: NDArray, scene: NDArray) -> list[bool]:
    """Which grasps GraspGenX's own scene check keeps: the open xArm gripper as
    boxes (XARM_GRIPPER_BODY), clear when every point on them is at least 1 cm
    (the pick's scene_collision_threshold) from every scene point."""
    from dimos.robot.manipulators.xarm.grasp_geometry import (
        XARM_GRASP_FRAME_TO_TCP,
        XARM_GRIPPER_BODY,
    )

    body = gripper_body_points([GripperBox.model_validate(box) for box in XARM_GRIPPER_BODY])
    # GraspGenX's grasp frame from a tool tip pose.
    frames = grasps @ np.linalg.inv(np.asarray(XARM_GRASP_FRAME_TO_TCP))
    tree = cKDTree(scene)
    return [
        bool(tree.query(body @ frame[:3, :3].T + frame[:3, 3])[0].min() >= 0.01) for frame in frames
    ]


@pytest.mark.self_hosted
def test_xarm_keeps_grasps_on_a_flat_box_that_graspgenx_scene_check_drops(
    xarm: GripperContactModel,
) -> None:
    # 12 cm long, 8 cm across the jaws, 1.5 cm tall. The open fingertips stop
    # 5 to 10 mm above the table: inside GraspGenX's 1 cm, clear of the 3 mm margin.
    flat = _box((0.0, 0.0, 0.0075), (0.12, 0.08, 0.015))
    table = _table(hole=(-0.06, 0.06, -0.04, 0.04))
    grasps = np.stack(
        [
            _top_down(x, 0.0, fingertips - XARM_TIP_BEHIND_TCP)
            for x in (-0.03, 0.0, 0.03)
            for fingertips in (0.005, 0.0075, 0.0095)
        ]
    )

    verdicts = xarm.check(grasps, flat, {"table": table}, CONTACT_MARGIN, OBSTACLE_MARGIN)

    assert _graspgenx_scene_filter_keeps(grasps, table) == [False] * len(grasps)
    assert verdicts == [CLEAR] * len(grasps)


@pytest.mark.self_hosted
def test_xarm_still_rejects_fingers_going_into_the_table_beside_a_flat_box(
    xarm: GripperContactModel,
) -> None:
    flat = _box((0.0, 0.0, 0.0075), (0.12, 0.08, 0.015))
    table = _table(hole=(-0.06, 0.06, -0.04, 0.04))
    # The open fingertips 2 mm below the table top.
    into = _top_down(0.0, 0.0, -0.002 - XARM_TIP_BEHIND_TCP)

    assert _check(xarm, into, flat, table=table) == GraspContact(("table",))


def _grasps_around(center: NDArray, count: int, seed: int = 0) -> NDArray:
    """Tool tip poses spread around ``center`` like GraspGenX's samples: approaching
    from anywhere above the table, at any turn about the approach, the tool tip
    within 3 cm of the centre."""
    rng = np.random.default_rng(seed)
    poses = np.tile(np.eye(4), (count, 1, 1))
    # Tool z (the approach) tilted up to 90 degrees from straight down.
    tilts = Rotation.from_euler(
        "zyz",
        np.stack(
            [
                rng.uniform(0.0, 2 * np.pi, count),
                rng.uniform(np.pi / 2, np.pi, count),
                rng.uniform(0.0, 2 * np.pi, count),
            ],
            axis=1,
        ),
    )
    poses[:, :3, :3] = tilts.as_matrix()
    poses[:, :3, 3] = center + rng.uniform(-0.03, 0.03, (count, 3))
    return poses


@pytest.mark.self_hosted
def test_xarm_checks_five_hundred_grasps_in_about_a_second(xarm: GripperContactModel) -> None:
    # A mug-sized cylinder with points 5 mm apart, as the camera's object cloud
    # has them, a table 1 cm apart and two neighbours.
    mug = _thin(_cylinder(0.04, 0.10), 0.005)
    table = _table(hole=(-0.04, 0.04, -0.04, 0.04))
    neighbours = np.concatenate(
        [
            _thin(_box((0.12, 0.05, 0.04), (0.05, 0.05, 0.08)), 0.01),
            _thin(_box((-0.1, -0.1, 0.03), (0.06, 0.04, 0.06)), 0.01),
        ]
    )
    grasps = _grasps_around(np.array([0.0, 0.0, 0.05]), 500)
    obstacles = {"table": table, "objects": neighbours}

    started = time.perf_counter()
    verdicts = xarm.check(grasps, mug, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN)
    seconds = time.perf_counter() - started

    assert len(verdicts) == 500
    assert 0 < sum(verdict.clear for verdict in verdicts) < 500
    assert seconds < 2.0


def _thin(points: NDArray, cell: float) -> NDArray:
    """One point per cube of edge ``cell`` (metres)."""
    _, first = np.unique(np.floor(points / cell).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(first)]


# Sliding grasps along their approach to a depth that passes.


def _lying_cylinder(radius: float, length: float) -> NDArray:
    """Points about 2 mm apart on a cylinder lying on the table along X, centred on the origin."""
    angles = np.linspace(0.0, 2 * np.pi, int(2 * np.pi * radius / 0.002), endpoint=False)
    along = np.arange(-length / 2.0, length / 2.0 + 1e-9, 0.002)
    a, x = np.meshgrid(angles, along)
    return np.stack([x.ravel(), radius * np.sin(a.ravel()), radius + radius * np.cos(a.ravel())], 1)


# A marker lying flat: 2 cm across, 12 cm long.
MARKER = _lying_cylinder(0.01, 0.12)
MARKER_TABLE = _table(hole=(-0.06, 0.06, -0.01, 0.01))
SHIFTS = depth_shifts(0.03, 0.005)


@pytest.mark.self_hosted
def test_xarm_depth_search_lifts_a_grasp_on_a_flat_marker_off_the_table(
    xarm: GripperContactModel,
) -> None:
    from dimos.robot.manipulators.xarm.config import XARM_APPROACH_OFFSETS

    # The closed pads' centre on the marker's middle puts the fingertips through the table.
    grasp = _top_down(0.0, 0.0, 0.01 - XARM_APPROACH_OFFSETS.pad_behind_tcp)
    obstacles = {"table": MARKER_TABLE}

    [where_given] = xarm.check(
        grasp[None], MARKER, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN, MIN_GAP_POINTS
    )
    [found] = xarm.fit_depth(
        grasp[None], MARKER, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN, MIN_GAP_POINTS, SHIFTS
    )

    assert "table" in where_given.touches
    assert found.clear
    # Backed off, but not so far that the pads close above the marker.
    assert -0.03 < found.depth_shift < 0.0


@pytest.mark.self_hosted
def test_xarm_depth_search_leaves_a_passing_grasp_where_it_is(xarm: GripperContactModel) -> None:
    # High on the box, the pads hold only its top, but the grasp passes.
    grasp = _top_down(0.0, 0.0, 0.055)

    [found] = xarm.fit_depth(
        grasp[None], BOX, {"table": BOX_TABLE}, CONTACT_MARGIN, OBSTACLE_MARGIN, 0, SHIFTS
    )

    assert found == _check(xarm, grasp, BOX, table=BOX_TABLE) == CLEAR


@pytest.mark.self_hosted
def test_xarm_depth_search_keeps_the_given_verdict_when_no_depth_passes(
    xarm: GripperContactModel,
) -> None:
    # Pads closing beside the box hold nothing at any depth.
    beside = _top_down(0.05, 0.0, 0.03)
    obstacles = {"table": BOX_TABLE}

    given = xarm.check(
        beside[None], BOX, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN, MIN_GAP_POINTS
    )
    found = xarm.fit_depth(
        beside[None], BOX, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN, MIN_GAP_POINTS, SHIFTS
    )

    assert found == given == [EMPTY_GAP]


@pytest.mark.self_hosted
def test_xarm_depth_search_with_no_shifts_is_the_check(xarm: GripperContactModel) -> None:
    heights = np.linspace(0.0, 0.12, 25)
    grasps = np.stack([_top_down(0.0, 0.0, z) for z in heights])
    obstacles = {"table": BOX_TABLE}

    assert xarm.fit_depth(
        grasps, BOX, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN, MIN_GAP_POINTS, ()
    ) == xarm.check(grasps, BOX, obstacles, CONTACT_MARGIN, OBSTACLE_MARGIN, MIN_GAP_POINTS)


@pytest.mark.self_hosted
def test_xarm_depth_search_backs_a_deep_grasp_off_only_until_it_passes(
    xarm: GripperContactModel,
) -> None:
    # The tool tip 2 cm under the table, as a grasp source centring the marker
    # in the open fingers gives it: the nearest depth that passes is the lowest.
    grasp = _top_down(0.0, 0.0, -0.02)

    [found] = xarm.fit_depth(
        grasp[None],
        MARKER,
        {"table": MARKER_TABLE},
        CONTACT_MARGIN,
        OBSTACLE_MARGIN,
        MIN_GAP_POINTS,
        depth_shifts(0.05, 0.0025),
        floor=0.0,
    )

    assert found.clear
    # Low enough that the fingers meet the 2 cm marker's sides, not just its top.
    assert -0.02 - found.depth_shift < 0.01


@pytest.mark.self_hosted
def test_xarm_floor_rejects_a_grasp_from_under_the_table(xarm: GripperContactModel) -> None:
    # Pointing up from 1 cm under the table top, below a marker that sinks
    # 2 mm into the table: no table point lies near the gripper.
    sunk = MARKER - np.array([0.0, 0.0, 0.002])
    from_below = _top_down(0.0, 0.0, -0.01)
    from_below[:3, :3] = from_below[:3, :3] @ np.diag([1.0, -1.0, -1.0])

    def check(floor: float | None) -> GraspContact:
        return xarm.check(
            from_below[None],
            sunk,
            {"table": MARKER_TABLE},
            CONTACT_MARGIN,
            OBSTACLE_MARGIN,
            floor=floor,
        )[0]

    assert check(None) == CLEAR
    assert check(0.0) == GraspContact(("floor",))
    # From above with the open fingertips 2 mm into the table: below the floor too.
    into = _top_down(0.0, 0.0, -0.002 - XARM_TIP_BEHIND_TCP)
    short_box = _box((0.0, 0.0, 0.015), (0.04, 0.04, 0.03))
    assert (
        "floor"
        in xarm.check(into[None], short_box, {}, CONTACT_MARGIN, OBSTACLE_MARGIN, floor=0.0)[
            0
        ].touches
    )
    # A grasp from above, its fingertips just over the table, is above the floor.
    assert _check(xarm, _top_down(0.0, 0.0, 0.03), BOX, table=BOX_TABLE) == CLEAR
    assert xarm.check(
        _top_down(0.0, 0.0, 0.03)[None],
        BOX,
        {"table": BOX_TABLE},
        CONTACT_MARGIN,
        OBSTACLE_MARGIN,
        floor=0.0,
    ) == [CLEAR]


@pytest.mark.self_hosted
def test_xarm_depth_search_stops_once_enough_grasps_pass(xarm: GripperContactModel) -> None:
    # 120 grasps that all pass on the box: the first 50 are checked, the rest are not.
    grasps = np.stack([_top_down(0.0, 0.0, 0.03, yaw=0.01 * i) for i in range(120)])

    found = xarm.fit_depth(
        grasps, BOX, {"table": BOX_TABLE}, CONTACT_MARGIN, OBSTACLE_MARGIN, 0, SHIFTS, enough=10
    )

    assert all(verdict.clear for verdict in found[:50])
    assert found[50:] == [GraspContact(("unchecked",))] * 70


def test_depth_shifts_run_through_zero() -> None:
    assert depth_shifts(0.015, 0.005) == pytest.approx(
        [-0.015, -0.01, -0.005, 0.0, 0.005, 0.01, 0.015]
    )
    assert depth_shifts(0.0, 0.005) == [0.0]


def _cloud(points: NDArray) -> PointCloud2:
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id="world", timestamp=1.0)


@pytest.mark.self_hosted
@pytest.mark.parametrize(
    "target, radius, aimed_only",
    [
        # A tall box, 5 x 5 x 16 cm: at mid-height the tool tip leaves the palm
        # on the box's top.
        (_box((0.0, 0.0, 0.08), (0.05, 0.05, 0.16)), 0.025, True),
        # A short can, 6 cm across and 3.3 cm tall.
        (_cylinder(0.03, 0.033), 0.03, False),
        # A mug-like cylinder, 8 cm across and 10 cm tall.
        (_cylinder(0.04, 0.10), 0.04, False),
    ],
    ids=["tall box", "short can", "mug"],
)
def test_xarm_heuristic_grasps_aiming_the_pads_pass_the_contact_check(
    xarm: GripperContactModel, target: NDArray, radius: float, aimed_only: bool
) -> None:
    from dimos.robot.manipulators.xarm.config import XARM_APPROACH_OFFSETS

    table = _table(hole=(-radius, radius, -radius, radius))

    def checked(config: HeuristicGraspConfig, min_gap_points: int = 0) -> list[GraspContact]:
        grasps = heuristic_grasps(_cloud(target), config).candidates
        poses = np.stack([pose_to_matrix(grasp.pose) for grasp in grasps])
        return xarm.check(
            poses, target, {"table": table}, CONTACT_MARGIN, OBSTACLE_MARGIN, min_gap_points
        )

    aiming_pads = HeuristicGraspConfig(**XARM_APPROACH_OFFSETS._asdict())
    aimed = checked(aiming_pads)
    at_tool_tip = checked(HeuristicGraspConfig(heuristic_aim_pads=False))

    assert aimed == [CLEAR] * len(aimed)
    assert checked(aiming_pads, MIN_GAP_POINTS) == aimed
    assert (at_tool_tip[0] != CLEAR) is aimed_only


# A made-up three-finger hand: a palm and three fingers 120 degrees apart, each
# hinged at the palm's rim. Open, the fingers splay 0.4 rad outward; closed, they
# lean 0.4 rad inward, so their tips swing forward and back as they close.

_FINGER_LENGTH = 0.08
_HAND_URDF = """<?xml version="1.0"?>
<robot name="hand">
  <link name="palm">
    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.1 0.1 0.02"/></geometry></collision>
  </link>
  <link name="tool"/>
  <joint name="tool_joint" type="fixed">
    <parent link="palm"/><child link="tool"/><origin xyz="0 0 0.09" rpy="0 0 0"/>
  </joint>
  {fingers}
</robot>
"""
_FINGER = """
  <link name="mount_{i}"/>
  <joint name="mount_{i}_joint" type="fixed">
    <parent link="palm"/><child link="mount_{i}"/><origin xyz="0 0 0" rpy="0 0 {yaw}"/>
  </joint>
  <link name="finger_{i}">
    <collision>
      <origin xyz="0 0 0.04" rpy="0 0 0"/><geometry><box size="0.01 0.015 0.08"/></geometry>
    </collision>
  </link>
  <joint name="finger_{i}_joint" type="revolute">
    <parent link="mount_{i}"/><child link="finger_{i}"/>
    <origin xyz="0.04 0 0.01" rpy="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-0.4" upper="0.4" effort="1" velocity="1"/>
    {mimic}
  </joint>
"""


@pytest.fixture(scope="module")
def hand_urdf(tmp_path_factory: pytest.TempPathFactory) -> Path:
    fingers = "".join(
        _FINGER.format(
            i=i,
            yaw=i * 2.0 * np.pi / 3.0,
            mimic="" if i == 0 else '<mimic joint="finger_0_joint" multiplier="1" offset="0"/>',
        )
        for i in range(3)
    )
    path: Path = tmp_path_factory.mktemp("hand") / "hand.urdf"
    path.write_text(_HAND_URDF.format(fingers=fingers))
    return path


def _hand(path: Path, closing_steps: int = 4) -> GripperContactModel:
    # Each finger's inner face lies at x = -0.005 in its own frame. The patch
    # covers it from 2 cm up to the tip, grown 6 mm along the face and past
    # the tip, 8 mm out toward the object and 4 mm into the finger.
    spec = GripperSpec(
        contact_patches=tuple(
            ContactPatch(
                link=f"finger_{i}", center=(-0.007, 0.0, 0.053), size=(0.012, 0.027, 0.066)
            )
            for i in range(3)
        ),
        open_joints={"finger_0_joint": -0.4},
        closed_joints={"finger_0_joint": 0.4},
        closing_steps=closing_steps,
    )
    return GripperContactModel(RobotModel.from_file(path).load(), spec, "tool")


@pytest.fixture(scope="module")
def hand(hand_urdf: Path) -> GripperContactModel:
    return _hand(hand_urdf)


# A can 4 cm across and 5 cm tall, standing at the origin.
CAN = _cylinder(0.02, 0.05)
CAN_TABLE = _table(hole=(-0.02, 0.02, -0.02, 0.02))


def _lowest_fingertip(hand: GripperContactModel, step: int) -> float:
    """How far (metres) the fingers reach past the tool tip, along the approach, at one pose."""
    corners = [
        (hand.tip_from_link(step, link) @ np.array([x, y, z, 1.0]))[2]
        for link in hand.links
        if link.startswith("finger")
        for x in (-0.005, 0.005)
        for y in (-0.0075, 0.0075)
        for z in (0.0, _FINGER_LENGTH)
    ]
    return float(max(corners))


def test_three_finger_hand_keeps_a_grasp_around_the_can(hand: GripperContactModel) -> None:
    assert hand.links == ["finger_0", "finger_1", "finger_2", "palm"]
    assert _check(hand, _top_down(0.0, 0.0, 0.04), CAN, table=CAN_TABLE) == CLEAR


def test_three_finger_hand_rejects_the_palm_on_the_can(hand: GripperContactModel) -> None:
    tall_can = _cylinder(0.02, 0.10)
    # The palm's face is 8 cm behind the tool tip: 1 mm above the can's top.
    assert _check(hand, _top_down(0.0, 0.0, 0.021), tall_can, table=CAN_TABLE) == GraspContact(
        ("target",)
    )


def test_three_finger_hand_sweep_catches_the_fingertips_swinging_into_the_table(
    hand: GripperContactModel, hand_urdf: Path
) -> None:
    swing = _lowest_fingertip(hand, 2) - _lowest_fingertip(hand, 0)
    assert swing > 0.004
    # Open, the fingertips clear the table by 4 mm; halfway through the close they reach it.
    grasp = _top_down(0.0, 0.0, 0.004 + _lowest_fingertip(hand, 0))
    open_only = _hand(hand_urdf, closing_steps=0)

    assert _check(hand, grasp, CAN, table=CAN_TABLE) == GraspContact(("table",), while_closing=True)
    assert _check(open_only, grasp, CAN, table=CAN_TABLE) == CLEAR


def test_three_finger_hand_depth_search_backs_the_sweep_off_the_table(
    hand: GripperContactModel,
) -> None:
    # Open, the fingertips clear the table by 4 mm; closing, they reach it.
    grasp = _top_down(0.0, 0.0, 0.004 + _lowest_fingertip(hand, 0))

    [found] = hand.fit_depth(
        grasp[None],
        CAN,
        {"table": CAN_TABLE},
        CONTACT_MARGIN,
        OBSTACLE_MARGIN,
        MIN_GAP_POINTS,
        SHIFTS,
    )

    assert found.clear
    assert found.depth_shift < 0.0


def test_approach_offsets_of_the_three_finger_hand(hand_urdf: Path) -> None:
    spec = GripperSpec(
        contact_patches=tuple(
            ContactPatch(
                link=f"finger_{i}", center=(-0.007, 0.0, 0.053), size=(0.012, 0.027, 0.066)
            )
            for i in range(3)
        ),
        open_joints={"finger_0_joint": -0.4},
        closed_joints={"finger_0_joint": 0.4},
    )

    offsets = approach_offsets(RobotModel.from_file(hand_urdf).load(), spec, "tool")

    # The tool tip is 9 cm above the palm's centre and each finger hinges 1 cm
    # above it. Closed, a finger leans 0.4 rad inward, taking its patch centre
    # (x -0.007, z 0.053 in the finger's frame) to this height above the hinge.
    patch_up = 0.053 * math.cos(0.4) - 0.007 * math.sin(0.4)
    assert offsets.pad_behind_tcp == pytest.approx(0.09 - 0.01 - patch_up)
    # Upright the fingertip is level with the tool tip; leaning a little, its
    # 1 cm wide end reaches a hair farther.
    assert offsets.fingertip_reach == pytest.approx(math.hypot(0.08, 0.005) - 0.08, abs=1e-5)
    # The palm's face, 1 cm above its centre.
    assert offsets.palm_behind_tcp == pytest.approx(0.08)


def test_three_finger_hand_rejects_a_neighbour_under_a_finger(hand: GripperContactModel) -> None:
    # Finger 0 hangs along +x; a neighbour stands right where it comes down.
    neighbour = _box((0.07, 0.0, 0.03), (0.03, 0.03, 0.06))

    assert _check(
        hand, _top_down(0.0, 0.0, 0.04), CAN, table=CAN_TABLE, objects=neighbour
    ) == GraspContact(("objects",))


def test_three_finger_hand_gap_is_around_the_open_pads(hand: GripperContactModel) -> None:
    # The fingers close toward a point, not toward each other: the gap is the
    # space the three open pads surround, 6 cm deep from about the tool tip back.
    def check(grasp: NDArray) -> GraspContact:
        return _check(hand, grasp, CAN, table=CAN_TABLE, min_gap_points=MIN_GAP_POINTS)

    assert check(_top_down(0.0, 0.0, 0.04)) == CLEAR
    assert check(_top_down(0.1, 0.0, 0.04)) == EMPTY_GAP
    assert check(_top_down(0.0, 0.0, 0.1)) == EMPTY_GAP
    assert hand.in_gap(np.array([[0.0, 0.0, -0.03], [0.0, 0.0, -0.07]])).tolist() == [True, False]


def test_a_spec_naming_a_link_the_urdf_lacks_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "hand.urdf"
    path.write_text(_HAND_URDF.format(fingers=""))
    spec = GripperSpec(
        contact_patches=(ContactPatch(link="finger_9", center=(0, 0, 0), size=(1, 1, 1)),)
    )

    with pytest.raises(ValueError, match="finger_9"):
        GripperContactModel(RobotModel.from_file(path).load(), spec, "tool")


def test_gripper_spec_steps_from_open_to_closed() -> None:
    spec = GripperSpec(
        contact_patches=(ContactPatch(link="pad", center=(0, 0, 0), size=(0.01, 0.01, 0.01)),),
        open_joints={"drive": 0.0},
        closed_joints={"drive": 0.8},
    )

    assert [step["drive"] for step in spec.joint_positions()] == pytest.approx(
        [0.0, 0.2, 0.4, 0.6, 0.8]
    )
    with pytest.raises(ValueError, match="same joints"):
        GripperSpec(contact_patches=spec.contact_patches, open_joints={"drive": 0.0})
