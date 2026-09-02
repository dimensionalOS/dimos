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

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from dimos.experimental.memory_belief.locate import PinholeFisheye, locate_detections
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3


@dataclass
class FakeCameraInfo:
    K: tuple[float, ...] = (400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0)
    D: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)
    width: int = 640
    height: int = 480
    distortion_model: str = "plumb_bob"


@dataclass
class FakeDetection:
    bbox: tuple[float, float, float, float]
    mask: np.ndarray | None = None


@pytest.fixture
def camera() -> PinholeFisheye:
    return PinholeFisheye(FakeCameraInfo())


@pytest.fixture
def identity_pose() -> Transform:
    """Camera at the world origin, looking down +z."""
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        child_frame_id="camera_optical",
    )


def _cube(centre: tuple[float, float, float], half: float, n: int = 6) -> np.ndarray:
    axis = np.linspace(-half, half, n)
    grid = np.stack(np.meshgrid(axis, axis, axis), -1).reshape(-1, 3)
    return grid + np.asarray(centre)


class TestRefusingIsAResult:
    def test_no_points_yields_no_placement(self, camera, identity_pose):
        det = FakeDetection(bbox=(300, 220, 340, 260))
        out = locate_detections(
            [det], np.zeros((0, 3)), world_from_camera=identity_pose, camera=camera
        )
        assert out == [None]

    def test_too_few_points_yields_no_placement(self, camera, identity_pose):
        # Three returns inside the box, but min_points is eight.
        points = _cube((0.0, 0.0, 3.0), 0.02, n=2)[:3]
        det = FakeDetection(bbox=(0, 0, 640, 480))
        out = locate_detections(
            [det], points, world_from_camera=identity_pose, camera=camera, min_points=8
        )
        assert out == [None]

    def test_points_behind_the_camera_do_not_place(self, camera, identity_pose):
        behind = _cube((0.0, 0.0, -3.0), 0.3)
        det = FakeDetection(bbox=(0, 0, 640, 480))
        out = locate_detections([det], behind, world_from_camera=identity_pose, camera=camera)
        assert out == [None]


class TestPlacementIsMeasured:
    def test_position_matches_the_cube_it_was_built_from(self, camera, identity_pose):
        points = _cube((0.0, 0.0, 3.0), 0.25)
        det = FakeDetection(bbox=(0, 0, 640, 480))
        (placed,) = locate_detections([det], points, world_from_camera=identity_pose, camera=camera)
        assert placed is not None
        assert placed.position == pytest.approx((0.0, 0.0, 3.0), abs=0.05)
        assert placed.depth_m == pytest.approx(3.0, abs=0.05)

    def test_extent_is_reported_not_just_a_point(self, camera, identity_pose):
        points = _cube((0.0, 0.0, 3.0), 0.25)
        det = FakeDetection(bbox=(0, 0, 640, 480))
        (placed,) = locate_detections([det], points, world_from_camera=identity_pose, camera=camera)
        assert placed is not None
        # A 0.5 m cube, so every side should measure about half a metre. Without
        # extent, "the bottle is on the desk" cannot be derived at query time.
        assert placed.extent == pytest.approx((0.5, 0.5, 0.5), abs=0.05)

    def test_support_counts_the_inliers(self, camera, identity_pose):
        points = _cube((0.0, 0.0, 3.0), 0.25, n=5)
        det = FakeDetection(bbox=(0, 0, 640, 480))
        (placed,) = locate_detections([det], points, world_from_camera=identity_pose, camera=camera)
        assert placed is not None
        assert placed.support == len(points)


class TestBackgroundIsNotTheObject:
    def test_a_far_wall_inside_the_box_is_excluded(self, camera, identity_pose):
        near = _cube((0.0, 0.0, 2.0), 0.2)
        far = _cube((0.0, 0.0, 9.0), 0.2)
        det = FakeDetection(bbox=(0, 0, 640, 480))
        (placed,) = locate_detections(
            [det],
            np.vstack([near, far]),
            world_from_camera=identity_pose,
            camera=camera,
            depth_band_m=0.6,
        )
        assert placed is not None
        # Depth must land on one surface, not average the two into empty air.
        assert placed.depth_m < 3.0
        assert placed.extent[2] < 1.0


class TestBoxIsNotTheOnlyEvidence:
    def test_a_mask_narrows_the_selection(self, camera, identity_pose):
        left = _cube((-1.0, 0.0, 3.0), 0.15)
        right = _cube((1.0, 0.0, 3.0), 0.15)
        points = np.vstack([left, right])
        mask = np.zeros((480, 640), np.uint8)
        mask[:, :320] = 1  # only the left half of the image
        det = FakeDetection(bbox=(0, 0, 640, 480), mask=mask)
        (placed,) = locate_detections([det], points, world_from_camera=identity_pose, camera=camera)
        assert placed is not None
        assert placed.position[0] < 0.0

    def test_a_mask_that_misses_the_box_falls_back_to_it(self, camera, identity_pose):
        """The fallback has to measure the overlap it is named for.

        The guard counted mask hits across the whole frame, so a mask covering
        a different object passed it while leaving this box nearly empty. The
        narrowed selection then fell under `min_points` and the detection was
        dropped -- the box alone would have placed it.
        """
        subject = _cube((0.0, 0.0, 3.0), 0.15)
        elsewhere = _cube((2.0, 0.0, 3.0), 0.15)
        points = np.vstack([subject, elsewhere])
        # Covers only where `elsewhere` projects, far to the right of the box.
        mask = np.zeros((480, 640), np.uint8)
        mask[:, 560:] = 1
        box = FakeDetection(bbox=(280, 200, 360, 280), mask=mask)
        (placed,) = locate_detections([box], points, world_from_camera=identity_pose, camera=camera)
        assert placed is not None, "a mask covering another object dropped this detection"
        assert placed.position[0] == pytest.approx(0.0, abs=0.2)
