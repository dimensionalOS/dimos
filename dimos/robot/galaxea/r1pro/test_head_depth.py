# Copyright 2025-2026 Dimensional Inc.
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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.robot.galaxea.r1pro.head_depth import (
    HeadFloorPriorConfig,
    footprint_floor,
    onto_floor,
    view_floor,
)

# A box on the floor ahead of the robot, in floor_frame: x, y extents and its height above the floor.
BOX_X, BOX_Y, BOX_HEIGHT = (1.2, 1.6), (-0.2, 0.2), 0.25
# Optical frame (z forward, y down) to a base whose x is forward and z up.
OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def _pose(rotation: np.ndarray, translation: list[float]) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = rotation, translation
    return pose


def _in_floor(floor_from_camera: np.ndarray, points: np.ndarray) -> np.ndarray:
    moved: np.ndarray = points @ floor_from_camera[:3, :3].T + floor_from_camera[:3, 3]
    return moved


def _scene(floor_from_camera: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Camera-frame points where each ray first meets the floor or the box (true depths), and which rays hit the box."""
    u, v = np.meshgrid(np.linspace(-0.6, 0.6, 41), np.linspace(-0.3, 0.9, 41))
    rays = np.stack([u.ravel(), v.ravel(), np.ones(u.size)], axis=1)
    origin, direction = floor_from_camera[:3, 3], rays @ floor_from_camera[:3, :3].T
    rays, direction = rays[direction[:, 2] < -0.05], direction[direction[:, 2] < -0.05]
    t_floor = -origin[2] / direction[:, 2]
    # Ray against the box (slab method), whose top is BOX_HEIGHT over the floor.
    lo = np.array([BOX_X[0], BOX_Y[0], 0.0])
    hi = np.array([BOX_X[1], BOX_Y[1], BOX_HEIGHT])
    with np.errstate(divide="ignore"):
        t1, t2 = (lo - origin) / direction, (hi - origin) / direction
    t_near, t_far = np.minimum(t1, t2).max(axis=1), np.maximum(t1, t2).min(axis=1)
    box = (t_near <= t_far) & (t_near > 0) & (t_near < t_floor)
    t = np.where(box, t_near, t_floor)
    keep = t < 6.0
    return rays[keep] * t[keep, None], box[keep]


@pytest.mark.parametrize(
    ("offset_m", "slides"),
    [(0.03, True), (-0.1, True), (0.1, False), (-0.3, False)],
    ids=["just-above", "under", "obstacle-height", "drop"],
)
def test_points_near_the_floor_slide_onto_it(offset_m: float, slides: bool) -> None:
    """Within floor_tolerance_m above or min_height_m below the floor a point is floor; higher is an obstacle, and deeper
    is a drop, which stays where it is (and is cut by the height band) rather than becoming floor."""
    floor_from_camera = _pose(
        Rotation.from_euler("y", 30, degrees=True).as_matrix() @ OPTICAL, [0.1, 0.0, 1.1]
    )
    truth, box = _scene(floor_from_camera)
    # Each ray stopped (or carried on) until it met a plane offset_m from the floor.
    moved = truth[~box] * ((offset_m - 1.1) / -1.1)

    heights = _in_floor(
        floor_from_camera, onto_floor(moved, floor_from_camera, HeadFloorPriorConfig())
    )[:, 2]

    np.testing.assert_allclose(heights, 0.0 if slides else offset_m, atol=1e-6)


def test_the_footprint_is_a_disc_of_floor() -> None:
    under = footprint_floor(0.325, 0.05)
    assert np.hypot(under[:, 0], under[:, 1]).max() <= 0.325
    assert (under[:, 2] == 0.0).all()
    assert len(under) == pytest.approx(np.pi * 0.325**2 / 0.05**2, rel=0.15)


def test_unseen_floor_is_filled_only_where_the_nearest_thing_seen_is_floor() -> None:
    """Floor seen 0.8 m out on bearings 0-30 deg and beyond a box at 0.6 m on 40-50 deg; nothing seen behind."""
    angles = np.radians(np.arange(0.0, 31.0, 1.0))
    floor = np.c_[0.8 * np.cos(angles), 0.8 * np.sin(angles), np.zeros(len(angles))]
    box_angles = np.radians(np.arange(40.0, 51.0, 1.0))
    box = np.c_[0.6 * np.cos(box_angles), 0.6 * np.sin(box_angles), np.full(len(box_angles), 0.3)]
    behind_box = np.c_[
        0.8 * np.cos(box_angles), 0.8 * np.sin(box_angles), np.zeros(len(box_angles))
    ]

    filled = view_floor(np.concatenate([floor, box, behind_box]), 0.325, 1.5, 5.0, 0.05)

    radius = np.hypot(filled[:, 0], filled[:, 1])
    bearing = np.degrees(np.arctan2(filled[:, 1], filled[:, 0]))
    assert (filled[:, 2] == 0.0).all()
    assert ((radius > 0.325) & (radius < 0.8)).all()
    assert ((bearing > 10) & (bearing < 15) & (radius > 0.7)).any()
    assert not ((bearing > 40) & (bearing < 50)).any()
    assert ((bearing >= -0.01) & (bearing < 35)).all()
