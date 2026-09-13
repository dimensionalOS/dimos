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

import itertools
import math

import numpy as np
import pytest

from dimos.teleop.memory_world.route import LETHAL, SNAP_RADIUS_M, RoutePlanner

VOXEL = 0.1
BODY_Z = 0.4  # the robot's base height above the floor at z = 0


def _path(*waypoints: tuple[float, float]) -> np.ndarray:
    """The robot's base along straight legs between waypoints, a point every 5 cm."""
    points = []
    for (x0, y0), (x1, y1) in itertools.pairwise(waypoints):
        n = max(int(math.dist((x0, y0), (x1, y1)) / 0.05), 2)
        for t in np.linspace(0, 1, n):
            points.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, BODY_Z))
    return np.asarray(points)


# Drove from the left half through the doorway into the right half and back down.
DOORWAY_DRIVE = _path((1, 1), (1, 4.6), (9, 4.6), (9, 1))


def _floor(x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
    xs = np.arange(x0, x1, VOXEL)
    ys = np.arange(y0, y1, VOXEL)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)


def _wall(x0: float, x1: float, y0: float, y1: float, height: float = 1.5) -> np.ndarray:
    xs = np.arange(x0, x1, VOXEL)
    ys = np.arange(y0, y1, VOXEL)
    zs = np.arange(VOXEL, height, VOXEL)
    gx, gy, gz = np.meshgrid(xs, ys, zs, indexing="ij")
    return np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)


def _room_with_doorway() -> np.ndarray:
    """A 10 x 6 m floor with a wall across x = 5 that has a 1.2 m gap at y in [4, 5.2]."""
    return np.concatenate(
        [
            _floor(0, 10, 0, 6),
            _wall(4.9, 5.1, 0, 4.0),
            _wall(4.9, 5.1, 5.2, 6.0),
        ]
    )


def test_costmap_marks_corridor_free_and_walls_and_unseen_space_lethal() -> None:
    planner = RoutePlanner.from_voxels(_room_with_doorway(), DOORWAY_DRIVE, voxel_size=VOXEL)
    assert planner.costs[planner.cell_of((1.5, 2.0))] == 0
    assert 0 <= planner.costs[planner.cell_of((5.0, 4.6))] < LETHAL, (
        "the doorway itself is passable"
    )
    assert planner.costs[planner.cell_of((5.0, 3.5))] == LETHAL, "the wall"
    # Within the robot's radius of the wall is lethal too, further out only costly.
    assert planner.costs[planner.cell_of((5.3, 3.5))] == LETHAL
    assert 0 < planner.costs[planner.cell_of((5.7, 4.0))] < LETHAL
    assert planner.costs[0, 0] == LETHAL, "space the robot never came near is not planned over"
    assert planner.costs[planner.cell_of((3.0, 2.0))] == LETHAL, "nor floor beyond the corridor"
    assert abs(planner.floor_at((1.5, 2.0)) - (BODY_Z - 0.2)) < 1e-9


def test_route_goes_through_the_doorway() -> None:
    planner = RoutePlanner.from_voxels(_room_with_doorway(), DOORWAY_DRIVE, voxel_size=VOXEL)
    route = planner.plan((1.0, 1.0), (9.0, 1.0))
    assert route is not None
    xs = np.asarray([p[0] for p in route.points])
    ys = np.asarray([p[1] for p in route.points])
    crossing = ys[np.argmin(np.abs(xs - 5.0))]
    assert 4.0 < crossing < 5.2, f"crossed the wall at y={crossing}"
    assert route.length_m > math.dist((1, 1), (9, 1)) + 2, (
        "the detour must be longer than the straight line"
    )
    assert all(abs(p[2] - (BODY_Z - 0.2)) < 1e-9 for p in route.points), (
        "drawn a little under the base height"
    )
    assert route.cells == len(route.points)


def test_goal_inside_an_obstacle_snaps_to_the_nearest_free_cell() -> None:
    voxels = np.concatenate([_floor(0, 6, 0, 6), _wall(2.8, 3.4, 2.8, 3.4, height=0.9)])  # a crate
    planner = RoutePlanner.from_voxels(
        voxels, _path((0.5, 0.5), (0.5, 5.5), (5.5, 5.5), (5.5, 0.5), (0.5, 0.5)), voxel_size=VOXEL
    )
    route = planner.plan((0.5, 0.5), (3.1, 3.1))
    assert route is not None
    end = route.points[-1]
    assert 0.3 < math.dist(end[:2], (3.1, 3.1)) < 1.5, "ends next to the crate, not inside it"


def test_no_route_when_walled_off() -> None:
    voxels = np.concatenate(
        [_floor(0, 10, 0, 6), _wall(4.9, 5.1, -1.0, 7.0)]
    )  # no doorway, wall past the floor edges
    left_only = _path((1, 1), (1, 5), (4, 5), (4, 1))  # the robot stayed on its side
    planner = RoutePlanner.from_voxels(voxels, left_only, voxel_size=VOXEL)
    assert planner.plan((1.0, 1.0), (9.0, 1.0)) is None


def test_the_robots_own_body_between_two_samples_is_not_a_wall() -> None:
    """The other half of the sibling below, and the two must not be traded for each other.

    Erasing only at the sample POINTS is wrong whenever the samples are further apart
    than the robot is wide: on a recording sampled every metre, the robot's own body --
    and the people walking beside it -- sit between the samples and are then read as
    walls. A straight 10 m corridor with one such voxel at each sample midpoint planned
    NO route at all, where a 9.20 m one runs down the middle of it.

    Bridging everything is the sibling's bug; bridging nothing is this one.
    `bridgeable` tells a drive from a jump by the legs around it.
    """
    walls = np.concatenate([_wall(-1.0, 11.0, 0.40, 0.55), _wall(-1.0, 11.0, -0.55, -0.40)])
    floor = _floor(-1, 11, -1, 1)
    # Sampled every metre, as a downsampled recording is.
    driven = np.asarray([[float(x), 0.0, BODY_Z] for x in range(11)])
    # The robot's own body, seen between the samples and nowhere else.
    body = np.asarray([[x + 0.5, 0.0, BODY_Z] for x in range(10)])

    planner = RoutePlanner.from_voxels(
        np.concatenate([floor, walls, body]), driven, voxel_size=VOXEL
    )
    route = planner.plan((0.2, 0.0), (9.5, 0.0))
    assert route is not None, "the robot's own body walled off the corridor it drove down"
    assert route.length_m < 12.0, f"routed around its own body: {route.length_m}"


def test_a_drive_outnumbered_by_stillness_is_still_a_drive() -> None:
    """A leg is judged against the legs near it, and stillness must not be one of them.

    Two ways a real recording makes the driving a minority of its own window without the
    robot slowing down at all:

    The pose source is slower than the scan stream. `replay._held_through_gaps` repeats
    the previous pose whenever tf has no sample within tolerance, so a 1 Hz tf chain
    against 10 Hz scans is NINE exact repeats per real step -- and counting them, every
    real leg was outnumbered and the corridor the robot drove down planned no route.

    Or the robot parks either side of the drive rather than only at the end, which is
    what a tour of three places looks like. Half a second of stillness each side is
    enough.

    Counting only the MOVING legs is what fixes both, and it is safe precisely because
    that threshold decides which legs INFORM the comparison and never which are bridged.
    """
    from dimos.teleop.memory_world.route import bridgeable

    still = 0.0015  # a millimetre of jitter, as a real stop looks

    def path_of(gaps: list[float]) -> np.ndarray:
        out = np.zeros((len(gaps) + 1, 3))
        out[1:, 0] = np.cumsum(gaps)
        return out

    shapes = {
        # Nine held repeats per real step: a 1 Hz tf chain against 10 Hz scans.
        "1 Hz tf, 10 Hz scans": [g for _ in range(10) for g in ([1.0] + [0.0] * 9)],
        "parked either side": [still] * 30 + [1.0] * 10 + [still] * 30,
        "a tour of three stops": ([1.0] * 4 + [still] * 30 + [1.0] * 3 + [still] * 30 + [1.0] * 3),
    }
    for label, gaps in shapes.items():
        mask = bridgeable(path_of(gaps), 0.1)
        drove = np.isclose(np.asarray(gaps), 1.0)
        assert mask[drove].all(), f"{label}: a real drive leg was outnumbered by stillness"


def test_a_jump_at_either_end_of_the_drive_is_still_a_jump() -> None:
    """The window has to be padded at the ends, and how it is padded decides the answer.

    Padding with the edge leg ITSELF -- scipy's "nearest" -- makes a jump in the first or
    last few legs most of its own neighbourhood, so it looks ordinary and gets bridged.
    Six legs is enough: `[1.2, .05, .05, .05, .05, .05]` bridged the 1.2 m jump, took the
    wall it crosses from cost 100 to 90, and planned a 4.5 m route straight through it.

    Nothing is padded with data now: an end leg is judged against the legs it actually
    has on one side, which is what "the legs around it" can honestly mean at an end.
    Every padding that invents a leg invents it out of the very leg being judged --
    see the sibling test below, which "mirror" also failed.
    """
    from dimos.teleop.memory_world.route import bridgeable

    fast = 0.05
    for label, gaps in (
        ("jump first", [1.2] + [fast] * 5),
        ("jump last", [fast] * 5 + [1.2]),
        ("jump first, long drive", [1.2] + [fast] * 79),
        ("jump last, long drive", [fast] * 79 + [1.2]),
    ):
        # A straight path whose legs are exactly those lengths.
        path = np.zeros((len(gaps) + 1, 3))
        path[1:, 0] = np.cumsum(gaps)
        mask = bridgeable(path, 0.1)
        jump = np.asarray(gaps) > 1.0
        assert not mask[jump].any(), f"{label}: the jump was bridged"
        assert mask[~jump].all(), f"{label}: a drive leg was not bridged"


def test_a_jump_reflected_into_its_own_window_is_still_a_jump() -> None:
    """The sixteenth shape. "mirror" padding does not repeat the EDGE leg -- and still
    hands a leg near an end a copy of itself, two places away, which is just as good a
    vote. Parked, one 1.2 m relocalisation, parked: the jump is the only moving leg in
    the recording, its own reflection was the only thing near it that moved, so it
    vouched for itself, the wall it crosses dropped from 100 to 90, and a 1.2 m route
    ran through it.

    An end leg now sees only what is really beside it, and when that is nothing, the
    rest of the path -- where nothing else moves either, so the jump stands alone.
    """
    from dimos.teleop.memory_world.route import bridgeable

    voxels = np.concatenate([_floor(0, 10, 0, 6), _wall(4.9, 5.1, -1.0, 7.0)])  # no doorway
    parked = [[4.4, 3.0, BODY_Z]] * 2 + [[5.6, 3.0, BODY_Z]] * 20

    mask = bridgeable(np.asarray(parked), VOXEL)
    assert not mask[1], "the jump was bridged by its own reflection"

    planner = RoutePlanner.from_voxels(voxels, np.asarray(parked), voxel_size=VOXEL)
    assert planner.costs[planner.cell_of((5.0, 3.0))] == LETHAL, (
        "the wall the jump's own reflection opened a door in"
    )


def test_a_drive_with_no_moving_leg_near_it_is_still_a_drive() -> None:
    """The seventeenth shape, and the case a purely local rule cannot decide.

    `_held_through_gaps` repeats the pose through a tf gap, and at eleven repeats per
    pose -- a 1 Hz tf chain against an 11 Hz scan stream -- the steps either side of a
    real one fall OUTSIDE the 21-leg window. Every step of the drive was then alone in a
    window of pure stillness, took the one-cell answer, and the corridor the robot drove
    planned no route. One and ten repeats, where a neighbour is still in reach, worked.

    Alone, a leg is judged against the rest of the path's driving instead: here, ten
    other one-metre steps.
    """
    walls = np.concatenate([_wall(-1.0, 11.0, 0.40, 0.55), _wall(-1.0, 11.0, -0.55, -0.40)])
    body = np.asarray([[x + 0.5, 0.0, BODY_Z] for x in range(10)])
    voxels = np.concatenate([_floor(-1, 11, -1, 1), walls, body])

    for repeats in (1, 10, 11, 40):
        held = [[float(x), 0.0, BODY_Z] for x in range(11) for _ in range(repeats)]
        planner = RoutePlanner.from_voxels(voxels, np.asarray(held), voxel_size=VOXEL)
        route = planner.plan((0.2, 0.0), (9.5, 0.0))
        assert route is not None, f"{repeats} repeats per pose: the drive was refused"
        assert route.length_m < 12.0, f"{repeats} repeats: routed around its own body"


def test_a_path_that_mostly_stands_still_is_still_a_drive() -> None:
    """`replay._held_through_gaps` REPEATS the previous pose exactly through a tf gap.

    So the path handed to the planner is not the "real odometry never repeats a pose"
    the pause fix assumed: a tf stream that stops partway through leaves a tail of
    zero-length legs, and a tour that parks at three places for 45 s each is 93 per cent
    stationary with no tf defect at all. A 90th percentile falls into that noise the
    moment the stationary samples pass 90 per cent -- and then nothing is bridged, the
    robot's own body reads as walls, and the corridor it drove down plans no route.

    The fourth shape, and the one that took two wrong statistics to find.
    """
    walls = np.concatenate([_wall(-1.0, 11.0, 0.40, 0.55), _wall(-1.0, 11.0, -0.55, -0.40)])
    body = np.asarray([[x + 0.5, 0.0, BODY_Z] for x in range(10)])
    voxels = np.concatenate([_floor(-1, 11, -1, 1), walls, body])
    driven = [[float(x), 0.0, BODY_Z] for x in range(11)]

    jitter = lambda n: [  # noqa: E731 - a millimetre of it, which is what a real stop looks like
        [10.0 + 0.001 * ((i % 3) - 1), 0.0, BODY_Z] for i in range(n)
    ]
    shapes = {
        # A tf gap: the pose is held, so the legs are exactly zero.
        "91% exact repeats": driven + [[10.0, 0.0, BODY_Z]] * 100,
        "93% parked": driven + jitter(140),
        # Fifteen minutes parked at 10 Hz. Nine thousand millimetre legs accumulate 9 m
        # of "distance", which outweighs the 10 m actually driven -- so weighting the
        # statistic by distance does not save it either. Only a LOCAL comparison does.
        "15 minutes parked": driven + jitter(9000),
    }
    for label, path in shapes.items():
        planner = RoutePlanner.from_voxels(voxels, np.asarray(path), voxel_size=VOXEL)
        route = planner.plan((0.2, 0.0), (9.5, 0.0))
        assert route is not None, f"{label}: standing still walled off the corridor it drove"
        assert route.length_m < 12.0, f"{label}: routed around its own body ({route.length_m})"


def test_a_finely_sampled_drive_still_does_not_bridge_a_jump() -> None:
    """The third shape, and the one that broke the second fix for the second.

    `_path` samples every 5 cm and the cell is 10 cm, so filtering out legs below one
    cell -- which is how the pause was first handled -- removed EVERY drive leg and left
    only the jump. The jump then WAS the median, got bridged, and the wall it crosses
    went from cost 100 to 90 with a 7.90 m route straight through it: the exact defect
    the wall test below pins, reintroduced by the fix for the corridor test above.

    Three shapes, three tests, and no statistic that fails any of them survives all
    three. The shipped one is the 90th percentile, which needs no filter.
    """
    voxels = np.concatenate([_floor(0, 10, 0, 6), _wall(4.9, 5.1, -1.0, 7.0)])  # no doorway

    def driven(step: float) -> np.ndarray:
        def leg(x0: float, x1: float) -> list[list[float]]:
            n = max(int(abs(x1 - x0) / step), 2)
            return [[x0 + (x1 - x0) * t, 3.0, BODY_Z] for t in np.linspace(0, 1, n)]

        return np.asarray(leg(1.0, 4.4) + leg(5.6, 9.0))  # a 1.2 m jump in the middle

    # Every sampling rate, including one FINER than any threshold a filter could use:
    # 5 mm legs defeated "the median of legs over a centimetre" exactly the way 5 cm legs
    # defeated "the median of legs over a cell", one scale up. Weighting by distance has
    # no threshold to be finer than.
    for step in (0.05, 0.005):
        planner = RoutePlanner.from_voxels(voxels, driven(step), voxel_size=VOXEL)
        assert planner.plan((1.0, 3.0), (9.0, 3.0)) is None, (
            f"sampled every {step} m, a 1.2 m jump was bridged and opened the wall"
        )


def test_a_pause_in_the_drive_does_not_wall_off_the_corridor() -> None:
    """A pause is made of tiny legs, and they must not set the scale for the driving.

    Real odometry never reports the same pose twice, so standing still for five seconds
    fills the list with millimetre legs and drags the median to nothing -- and then
    nothing is bridged, the robot's own body between the drive samples is read as walls,
    and the corridor the sibling test below protects disappears anyway. Same voxels, same
    drive, same endpoints; the only difference is that the robot stopped for a moment.

    Filtering `> 0` caught only the perfectly still case, which is the synthetic one.
    """
    walls = np.concatenate([_wall(-1.0, 11.0, 0.40, 0.55), _wall(-1.0, 11.0, -0.55, -0.40)])
    floor = _floor(-1, 11, -1, 1)
    body = np.asarray([[x + 0.5, 0.0, BODY_Z] for x in range(10)])
    voxels = np.concatenate([floor, walls, body])

    driven = [[float(x), 0.0, BODY_Z] for x in range(11)]
    # A five-second stop halfway, sampled at 10 Hz, with a millimetre of jitter.
    paused = (
        driven[:6]
        + [[5.0 + 0.001 * ((i % 3) - 1), 0.001 * ((i % 2) - 0.5), BODY_Z] for i in range(50)]
        + driven[6:]
    )

    for label, path in (("no pause", driven), ("with a pause", paused)):
        planner = RoutePlanner.from_voxels(voxels, np.asarray(path), voxel_size=VOXEL)
        route = planner.plan((0.2, 0.0), (9.5, 0.0))
        assert route is not None, f"{label}: the corridor it drove down was walled off"
        assert route.length_m < 12.0, f"{label}: routed around its own body ({route.length_m})"


def test_one_pose_jump_across_a_wall_does_not_open_a_door_in_it() -> None:
    """A SLAM relocalisation is a teleport, and `densify` draws a straight line through
    it. Those fabricated points were counted as "the robot was here", and the near-path
    clause then erased the real wall voxels the line passed through -- so the planner
    routed through the hole its own interpolation had made.

    The same drive, the only difference being one 2 m jump in the recorded path:
    continuous, the wall's cells cost 100 and there is no route; with the jump, they
    dropped to 88-90 and a 7.90 m route ran straight through x = 5.

    `densify` itself stays: the corridor and the floor height need it, and the sibling
    test above needs ~3 m of it. Only the claim about the robot's own body is now made
    from the samples the robot actually reported.
    """
    voxels = np.concatenate([_floor(0, 10, 0, 6), _wall(4.9, 5.1, -1.0, 7.0)])  # no doorway

    # The robot drove up the left side and back, never crossing.
    continuous = _path((1, 1), (1, 5), (4, 5), (4, 1))
    assert (
        RoutePlanner.from_voxels(voxels, continuous, voxel_size=VOXEL).plan((1.0, 1.0), (9.0, 1.0))
        is None
    )

    # The same drive with ONE extra pose on the far side of the wall: a relocalisation,
    # not a drive, so there are no samples in between -- 2.83 m from the last real one.
    # `_path` would lay samples every 5 cm along that leg, which is a robot driving
    # through, not a jump, so the sample is appended by hand.
    jumped = np.concatenate([continuous, np.asarray([[6.0, 3.0, BODY_Z]])])
    planner = RoutePlanner.from_voxels(voxels, jumped, voxel_size=VOXEL)
    assert planner.plan((1.0, 1.0), (9.0, 1.0)) is None, (
        "the route went through the wall that one jump interpolated a line across"
    )


def test_sparse_path_samples_still_make_one_corridor() -> None:
    """A path sampled far apart still has to carve ONE connected corridor.

    `[::20]` leaves a 1.03 m gap, which CORRIDOR_M = 1.5 bridges on its own -- so
    `densify` was a no-op for that fixture and the test passed with the whole function
    replaced by `return path`. At `[::60]` the gap is 3.09 m, wider than the corridor,
    and the samples become a chain of islands the planner cannot cross: measured route
    length 11.27 m with densify against 12.70 m without, and at `[::100]` (5.03 m) there
    is no route at all without it. So this samples where the behaviour is load-bearing.
    """
    sparse = DOORWAY_DRIVE[::60]  # ~3 m between samples: wider than CORRIDOR_M
    planner = RoutePlanner.from_voxels(_room_with_doorway(), sparse, voxel_size=VOXEL)
    route = planner.plan((1.0, 1.0), (9.0, 1.0))
    assert route is not None and route.length_m > 8
    # The corridor is continuous, not a chain of islands bridged by a longer way round.
    assert route.length_m < 12.0, (
        f"route detoured around a gap densify should have filled: {route.length_m}"
    )


def test_the_goal_is_snapped_only_into_space_the_start_can_reach() -> None:
    """`plan` snaps the goal `within=reachable_from(start)`, and that mask had no test.

    The nearest free cell to an object can sit on a floor island, on another level, or --
    here -- in a room the robot reached only by being carried. The point of the mask is
    NOT that an unreachable goal is refused: A* would refuse that anyway. It is that a
    goal is snapped to the nearest cell the start can REACH, rather than to the nearest
    cell of any kind, so an object standing against a wall is approached from the side
    the viewer is on instead of being declared unroutable.

    So the goal here sits just past the wall: its nearest free cell is 0.2 m away on the
    far side, and the nearest reachable one is 0.75 m away on the near side. Both are
    well inside SNAP_RADIUS_M, which is what makes the mask, and only the mask, decide.
    My first version of this test put the goal deep in the far room, where A* refused on
    its own and removing the mask changed nothing -- a test with no power over the line
    it was written for.
    """
    voxels = np.concatenate([_floor(0, 10, 0, 6), _wall(4.9, 5.1, -1.0, 7.0)])  # no doorway
    left = _path((1, 1), (1, 5), (4, 5), (4, 1))
    right = _path((6, 1), (6, 5), (9, 5), (9, 1))  # reached by being carried, not driven
    planner = RoutePlanner.from_voxels(voxels, np.concatenate([left, right]), voxel_size=VOXEL)

    here = (1.0, 1.0)
    against_the_far_face = (5.35, 3.0)  # a thing on the wall, seen from the other room

    reachable = planner.reachable_from(here)
    assert reachable is not None
    # The two rooms really are separate, or this fixture proves nothing.
    row, col = planner.cell_of((9.0, 1.0))
    assert planner.passable(row, col) and not reachable[row, col]

    # Unmasked, the nearest free cell is the far one; masked, it is the near one.
    unmasked = planner.snap(against_the_far_face, SNAP_RADIUS_M)
    masked = planner.snap(against_the_far_face, SNAP_RADIUS_M, within=reachable)
    assert unmasked is not None and masked is not None
    # `snap` answers in world coordinates, so ask the grid which cell that is.
    assert not reachable[planner.cell_of(unmasked)], (
        "the unmasked snap already lands somewhere reachable; the fixture proves nothing"
    )
    assert reachable[planner.cell_of(masked)]
    assert masked != unmasked

    # And that is the difference between a route and no route at all.
    route = planner.plan(here, against_the_far_face)
    assert route is not None, "the goal was snapped into the room the start cannot reach"
    assert route.points[-1][0] < 4.9, "the route ended on the far side of the wall"


def test_unreachable_snap_returns_none() -> None:
    planner = RoutePlanner.from_voxels(
        _floor(0, 2, 0, 2), _path((0.5, 0.5), (1.5, 1.5)), voxel_size=VOXEL
    )
    assert planner.snap((30.0, 30.0)) is None


def test_snap_measures_from_the_point_asked_about_not_the_cell_it_fell_in() -> None:
    """Every cell in a ring is the same number of hops away; they are not the same distance.

    `snap` ranked candidates by CELL INDEX, so a diagonal neighbour and an orthogonal one
    were 2 and 1 hops and argmin always took the orthogonal -- however far away it actually
    was. `radius_m` was a bound on hops too, so the cell handed back could be further than
    the caller allowed.

    The grid is built by hand because the room fixtures cannot show it: `snap` returns the
    point unchanged when its own cell is passable, and in a room the free neighbours of a
    blocked cell are all on one side, where hops and metres happen to agree. Here the only
    free cells are the diagonal at the corner the query sits in, and an orthogonal one on
    the far side.
    """
    res = 1.0
    costs = np.full((3, 3), LETHAL, dtype=np.int8)
    costs[0, 0] = 0  # diagonal, up-left of centre
    costs[1, 2] = 0  # orthogonal, to the right of centre
    planner = RoutePlanner(costs, np.zeros((3, 3)), origin_xy=(0.0, 0.0), resolution=res)

    # Hard against the up-left corner of the blocked centre cell, whose centre is (1.5,1.5).
    asked = (1.05, 1.05)
    diagonal = planner.world_of(0, 0)  # (0.5, 0.5)
    orthogonal = planner.world_of(1, 2)  # (2.5, 1.5)
    assert math.dist(asked, diagonal) < math.dist(asked, orthogonal), "fixture is not asymmetric"

    got = planner.snap(asked, radius_m=5.0)

    assert got == diagonal, (
        f"snapped to {got} ({math.dist(asked, got):.2f} m) over {diagonal} "
        f"({math.dist(asked, diagonal):.2f} m): ranked by hops, not metres"
    )
    # ...and radius_m bounds METRES: the nearest free cell is 0.78 m away, so 0.5 refuses.
    assert planner.snap(asked, radius_m=0.5) is None, "returned a cell outside the radius asked for"


def test_a_point_below_the_origin_lands_on_a_negative_cell() -> None:
    """`cell_of` floors rather than truncating, and only negative coordinates show it.

    Its own comment says why it exists: int() truncates toward zero, so a point up to one
    cell BELOW the origin landed on cell 0 instead of -1 and then PASSED the bounds check
    every caller makes. Every fixture in this file puts the origin below every query point,
    where floor and int agree exactly, so swapping one for the other changed nothing any
    test could see.
    """
    planner = RoutePlanner.from_voxels(
        _floor(0, 2, 0, 2), _path((0.5, 0.5), (1.5, 1.5)), voxel_size=VOXEL
    )
    ox, oy = planner.origin_xy

    # Half a cell below the origin on both axes: floor gives -1, int() gives 0.
    row, col = planner.cell_of((ox - planner.resolution / 2, oy - planner.resolution / 2))

    assert (row, col) == (-1, -1), "truncated toward zero, so an outside point read as inside"
    # ...and a point that far out must NOT pass the bounds check every caller makes.
    assert not planner.passable(row, col)
    # world_of is floor's inverse, so a round trip lands back in the same cell.
    assert planner.cell_of(planner.world_of(row, col)) == (row, col)


def test_city_scale_map_plans_on_coarse_cells() -> None:
    # A 4 km straight road: at 10 cm cells the grid would be 40k cells across.
    road = _floor(0, 4000, 0, 4)
    drive = _path((1, 2), (3999, 2))
    planner = RoutePlanner.from_voxels(road[::7], drive, voxel_size=VOXEL)
    assert 0.9 < planner.resolution < 1.2
    assert planner.costs.shape[1] < 4200
    route = planner.plan((10, 2), (3990, 2))
    assert route is not None and route.length_m > 3900


def test_mls_planner_routes_through_the_doorway() -> None:
    pytest.importorskip("dimos_mls_planner")
    from dimos.teleop.memory_world.route import MlsRoutePlanner

    planner = MlsRoutePlanner(_room_with_doorway(), voxel_size=VOXEL)
    assert planner.surface_cells > 100
    route = planner.plan((1.0, 1.0, 0.0), (9.0, 1.0, 0.0))
    assert route is not None and route.planner == "mls"
    xs = np.asarray([p[0] for p in route.points])
    ys = np.asarray([p[1] for p in route.points])
    crossing = ys[np.argmin(np.abs(xs - 5.0))]
    assert 4.0 < crossing < 5.2, f"crossed the wall at y={crossing}"


def test_the_drawn_route_runs_through_cell_centres_not_corners() -> None:
    """The polyline must sit on the cells it was planned through.

    `OccupancyGrid.grid_to_world` is `origin + cell * resolution`, which is the cell's
    CORNER, while `snap()` and `world_of()` work in centres. Taken as-is every waypoint
    sat half a cell down and left of the cell A* chose: 4 cm at this voxel size, but half
    a metre on a city map, where that is more than the robot's radius.
    """
    planner = RoutePlanner.from_voxels(_room_with_doorway(), DOORWAY_DRIVE, voxel_size=VOXEL)
    route = planner.plan((1.0, 1.0), (9.0, 1.0))
    assert route is not None
    for x, y, _ in route.points:
        # cell_of, not snap: snap looks for the nearest FREE cell, which for a waypoint
        # beside a wall is a different cell entirely. The question here is only whether a
        # waypoint sits at the centre of the cell it is in.
        row, col = planner.cell_of((x, y))
        assert planner.world_of(row, col) == pytest.approx((x, y), abs=1e-9), (
            f"waypoint ({x}, {y}) is not the centre of the cell it lands in"
        )
