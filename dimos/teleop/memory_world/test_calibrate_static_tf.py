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

"""The pure parts of the mount calibration: which edge, which stream, which sample.

The fit itself needs a real recording and is exercised by running the command; what
is pinned here is everything that decides WHERE the measurement is applied, because
a right number written to the wrong edge is worse than no measurement at all.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.teleop.memory_world.calibrate_static_tf import (
    _nearest,
    _rigid_frames,
    _rotation,
    _thinned,
    camera_mount_edge,
)
from dimos.teleop.memory_world.tf_tree import TfTree

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def cart_tree() -> TfTree:
    """The cart's own shape: two sensor chains that meet at base_link."""
    tree = TfTree()
    for parent, child in (
        ("map", "odom"),
        ("odom", "base_link"),
        ("base_link", "sensor_mount_link"),
        ("sensor_mount_link", "livox_link"),
        ("livox_link", "livox_frame"),
        ("base_link", "handle_bar_link"),
        ("handle_bar_link", "camera_link"),
        ("camera_link", "camera_depth_optical_frame"),
        ("camera_depth_optical_frame", "camera_color_optical_frame"),
    ):
        tree.add(parent, child, 0.0, (0.1, 0.0, 0.0), IDENTITY, static=True)
    return tree


def test_the_mount_is_the_first_edge_descending_towards_the_camera() -> None:
    """Correcting it carries colour, depth and infra together and nothing else."""
    mount, root = camera_mount_edge(cart_tree(), "camera_depth_optical_frame", "livox_frame")
    assert (mount, root) == ("base_link", "handle_bar_link")


def test_a_camera_named_nothing_like_a_camera_still_finds_its_mount() -> None:
    """The edge is found by walking the tf path, never by what a frame is called."""
    tree = TfTree()
    for parent, child in (
        ("odom", "body"),
        ("body", "lidar_mount"),
        ("lidar_mount", "lidar"),
        ("body", "realsense_link"),
        ("realsense_link", "realsense_depth_frame"),
        ("realsense_depth_frame", "realsense_depth_optical_frame"),
    ):
        tree.add(parent, child, 0.0, (0.1, 0.0, 0.0), IDENTITY, static=True)
    assert camera_mount_edge(tree, "realsense_depth_optical_frame", "lidar") == (
        "body",
        "realsense_link",
    )


def test_an_unreachable_camera_names_no_mount() -> None:
    tree = cart_tree()
    assert camera_mount_edge(tree, "camera_depth_optical_frame", "nowhere") == (
        None,
        "camera_depth_optical_frame",
    )


def test_only_frames_bolted_to_the_body_count_as_the_sensor_s_own() -> None:
    """A map frame is somebody's child too, so being a child proves nothing."""
    rigid = _rigid_frames(cart_tree())
    assert "livox_frame" in rigid and "camera_link" in rigid
    assert "odom" not in rigid and "map" not in rigid  # a cloud in odom cannot calibrate


def test_with_no_body_every_known_frame_is_allowed() -> None:
    """Better to try the fit than to refuse a rig that does not use base_link."""
    tree = TfTree()
    tree.add("odom", "sensor", 0.0, (0.0, 0.0, 0.0), IDENTITY, static=True)
    assert _rigid_frames(tree) == set(tree.frames)


def test_the_nearest_sample_wins_not_the_first_in_the_window() -> None:
    """Anything else absorbs the robot's motion during the window into the mount."""

    class Stream:
        def at(self, ts: float, tolerance: float) -> list[object]:
            return [obs for obs in samples if abs(obs.ts - ts) <= tolerance]

    class Sample:
        def __init__(self, ts: float) -> None:
            self.ts = ts

    samples = [Sample(0.9), Sample(1.0), Sample(1.1)]
    assert _nearest(Stream(), 1.0, 0.2).ts == 1.0
    assert _nearest(Stream(), 1.09, 0.2).ts == 1.1
    with pytest.raises(LookupError):
        _nearest(Stream(), 5.0, 0.2)


def test_the_rotation_helper_is_a_rotation_and_turns_the_right_way() -> None:
    for angles in ((0.0, 0.0, 0.0), (0.3, -1.1, 2.0), (np.pi / 2, 0.0, -np.pi / 4)):
        r = _rotation(*angles)
        assert np.allclose(r.T @ r, np.eye(3), atol=1e-12)
        assert np.linalg.det(r) == pytest.approx(1.0)
    # A quarter turn about z takes x to y, which fixes the sign convention.
    assert np.allclose(
        _rotation(0.0, 0.0, np.pi / 2) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12
    )


def test_thinning_keeps_one_point_per_cube_and_obeys_the_cap() -> None:
    dense = np.repeat(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]), 50, axis=0)
    assert len(_thinned(dense, 0.5, 100)) == 3  # three cubes, however many points
    spread = np.arange(3000, dtype=np.float64).reshape(-1, 1) * [[1.0, 1.0, 1.0]]
    thinned = _thinned(spread, 0.5, 500)
    assert len(thinned) == 500  # capped, and every survivor is one of the originals
    assert {tuple(p) for p in thinned} <= {tuple(p) for p in spread}
