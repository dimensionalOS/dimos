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

from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.teleop.memory_world.camera import CameraModel, Rectifier
from dimos.teleop.memory_world.objects import (
    LocateConfig,
    Look,
    Sighting,
    box_from_points,
    carve,
    detector_phrasings,
    locate,
    object_points,
    render_depth,
)

CAMERA = CameraModel(
    width=640, height=480, fx=400.0, fy=400.0, cx=320.0, cy=240.0, distortion=(0.0,) * 4
)
# The camera at the origin looking along world +y: optical x is world x, optical y is
# world -z, optical z is world +y.
WORLD_T_CAMERA = np.eye(4)
WORLD_T_CAMERA[:3, :3] = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)


class _BoxDetector:
    """Answers with one box on the left half of the frame, or refuses when told to."""

    def __init__(self, refuse_first: int = 0) -> None:
        self.refuse_first = refuse_first
        self.calls = 0
        self.queries: list[list[str]] = []

    def query_detections(self, image: Image, queries: list[str], threshold: float) -> Any:
        self.calls += 1
        self.queries.append(list(queries))
        if self.calls <= self.refuse_first:
            return ImageDetections2D(image=image, detections=[])
        box = Detection2DBBox(
            bbox=(40.0, 100.0, 300.0, 380.0),
            track_id=-1,
            class_id=0,
            confidence=0.8,
            name=queries[0],
            ts=image.ts,
            image=image,
        )
        return ImageDetections2D(image=image, detections=[box])


def _wall(x0: float, x1: float, z0: float, z1: float, y: float) -> np.ndarray:
    xs = np.arange(x0, x1, 0.08)
    zs = np.arange(z0, z1, 0.08)
    return np.array([[x, y, z] for x in xs for z in zs], dtype=np.float32)


def _look(frame_id: int, cloud: np.ndarray) -> Look:
    image = Image.from_opencv(np.zeros((480, 640, 3), np.uint8), ts=100.0 + frame_id)
    depth = render_depth(cloud, WORLD_T_CAMERA, CAMERA, max_depth_m=20.0)
    return Look(frame_id, 100.0 + frame_id, image, WORLD_T_CAMERA, depth)


def test_rendered_depth_is_the_map_seen_through_the_camera() -> None:
    depth = render_depth(_wall(-3.0, 3.0, -1.0, 2.0, 6.0), WORLD_T_CAMERA, CAMERA, max_depth_m=20.0)

    assert depth.shape == (240, 320)
    window = depth[100:140, 140:180]
    assert (window > 0).mean() > 0.08
    assert np.median(window[window > 0]) == pytest.approx(6.0, abs=0.05)
    assert depth[0, 0] == 0.0


def test_object_points_keep_the_box_median_surface_and_drop_the_background() -> None:
    depth = np.zeros((240, 320), np.float32)
    depth[60:180, 40:140] = 3.0  # the object
    depth[60:180, 140:160] = 8.0  # a wall behind it, inside the box
    measured = object_points(
        (80, 120, 320, 360), (640, 480), depth, CAMERA, max_depth_m=20.0, band_m=0.5, min_pixels=20
    )

    assert measured and measured.median == pytest.approx(3.0)
    assert measured.points is not None
    assert np.allclose(measured.points[:, 2], 3.0)
    assert len(measured.points) == 120 * 100
    far = object_points(
        (80, 120, 320, 360), (640, 480), depth, CAMERA, max_depth_m=2.0, band_m=0.5, min_pixels=20
    )
    assert not far and "past 2 m" in far.why
    # With a pose, readings on the ground under the object are left out of the median.
    grounded = depth.copy()
    grounded[150:180, 40:160] = 2.0  # the ground in front, nearer than the object
    camera_down = np.eye(4)
    camera_down[:3, :3] = WORLD_T_CAMERA[:3, :3]
    camera_down[2, 3] = 0.8
    measured = object_points(
        (80, 120, 320, 360),
        (640, 480),
        grounded,
        CAMERA,
        max_depth_m=20.0,
        band_m=0.5,
        min_pixels=20,
        world_t_camera=camera_down,
        ground_z=0.4,
    )
    assert measured and measured.median == pytest.approx(3.0)


def test_box_from_points_lands_in_the_world_frame() -> None:
    # A 2 m wide, 1 m tall, 0.5 m deep slab in front of the camera.
    points = np.array(
        [[x, y, z] for x in np.linspace(-1.0, 1.0, 9) for y in (0.5, -0.5) for z in (6.0, 6.5)]
    )
    box = box_from_points(points, WORLD_T_CAMERA, 6.2, trim_percentile=0.0)

    assert box.centre == pytest.approx((0.0, 6.25, 0.0))
    assert box.extent == pytest.approx((2.0, 0.5, 1.0))
    assert box.yaw == pytest.approx(0.0, abs=1e-6)
    assert box.depth_m == 6.2 and box.pixels == len(points)


def test_box_turns_to_the_objects_long_axis() -> None:
    # A 4 x 1.5 m car footprint parked at 30 degrees to the world axes.
    yaw = np.radians(30.0)
    local = np.array(
        [[x, y, z] for x in np.linspace(-2.0, 2.0, 17) for y in (-0.75, 0.75) for z in (0.0, 1.4)]
    )
    world = local.copy()
    world[:, 0] = np.cos(yaw) * local[:, 0] - np.sin(yaw) * local[:, 1] + 10.0
    world[:, 1] = np.sin(yaw) * local[:, 0] + np.cos(yaw) * local[:, 1] - 5.0
    box = box_from_points(world, np.eye(4), 8.0, trim_percentile=0.0)

    assert box.yaw == pytest.approx(yaw, abs=1e-3)
    assert box.extent == pytest.approx((4.0, 1.5, 1.4), abs=1e-3)
    assert box.centre == pytest.approx((10.0, -5.0, 0.7), abs=1e-3)


def test_a_box_measured_on_the_ground_is_refused() -> None:
    # Only a flat patch of ground under a tall box: nothing of the object is in the map.
    ground = np.array(
        [[x, y, 0.0] for x in np.arange(-3.0, 3.0, 0.08) for y in np.arange(2.0, 12.0, 0.08)],
        dtype=np.float32,
    )
    camera_up = np.eye(4)
    camera_up[:3, :3] = WORLD_T_CAMERA[:3, :3]
    camera_up[2, 3] = 0.3
    image = Image.from_opencv(np.zeros((480, 640, 3), np.uint8), ts=100.0)
    depth = render_depth(ground, camera_up, CAMERA, max_depth_m=20.0)
    look = Look(1, 100.0, image, camera_up, depth)

    found = locate("car", [[look]], _BoxDetector(), CAMERA, config=LocateConfig(max_depth_m=20.0))

    assert len(found) == 1 and not found[0].placed
    assert found[0].centre == (0.0, 0.0, 0.3) and found[0].confidence == 0.8
    assert found[0].to_json()["height"] is None


def test_locate_measures_the_object_and_merges_repeated_looks() -> None:
    left = _wall(-3.0, -0.5, 0.0, 1.0, 6.0)
    right = _wall(0.5, 3.0, 0.0, 4.0, 6.0)
    cloud = np.concatenate([left, right])
    detector = _BoxDetector(refuse_first=1)
    places = [[_look(1, cloud), _look(2, cloud)], [_look(3, cloud)]]

    found = locate("wall", places, detector, CAMERA, config=LocateConfig(max_depth_m=20.0))

    assert detector.calls == 3
    assert detector.queries[0] == ["wall"]
    assert len(found) == 1
    wall = found[0]
    assert wall.placed and wall.views == 2 and wall.query == "wall"
    assert wall.extent is not None
    assert wall.centre[0] < -0.5 and wall.centre[1] == pytest.approx(6.0, abs=0.15)
    assert wall.extent[2] < 1.2
    assert wall.to_json()["height"] == pytest.approx(wall.extent[2])


class _LeftHalfSegmenter:
    """Masks the left half of every box, whatever the box says."""

    def segment(self, detections: ImageDetections2D) -> ImageDetections2D:
        masked = []
        for det in detections.detections:
            x1, y1, x2, y2 = (int(v) for v in det.bbox)
            mask = np.zeros((480, 640), np.uint8)
            mask[y1:y2, x1 : (x1 + x2) // 2] = 255
            masked.append(SimpleNamespace(mask=mask, ts=det.ts))
        return ImageDetections2D(image=detections.image, detections=masked)  # type: ignore[arg-type]


def test_a_mask_narrows_the_measurement_to_the_objects_pixels() -> None:
    wall = _wall(-3.0, 3.0, 0.0, 2.0, 6.0)
    detector = _BoxDetector()

    boxed = locate(
        "wall", [[_look(1, wall)]], detector, CAMERA, config=LocateConfig(max_depth_m=20.0)
    )
    masked = locate(
        "wall",
        [[_look(1, wall)]],
        detector,
        CAMERA,
        segmenter=_LeftHalfSegmenter(),
        config=LocateConfig(max_depth_m=20.0),
    )

    assert boxed[0].extent is not None and masked[0].extent is not None
    assert masked[0].extent[0] < 0.6 * boxed[0].extent[0]
    assert masked[0].centre[0] < boxed[0].centre[0]


def test_carving_drops_points_a_second_view_sees_as_background() -> None:
    wall = _wall(-3.0, 3.0, 0.0, 2.0, 6.0)
    look = _look(1, wall)
    points = np.array([[-2.0, 6.0, 1.0], [2.0, 6.0, 1.0], [0.0, 30.0, 1.0]])
    # A view whose box covers only the left of the frame: the right point is seen
    # in front of the wall but outside the box, the far point is not seen at all.
    sighting = Sighting(look, (0.0, 0.0, 320.0, 480.0), 0.9, None, points, 6.0)

    kept = carve(points, [sighting], CAMERA, tolerance_m=0.3)

    assert kept.tolist() == [[-2.0, 6.0, 1.0], [0.0, 30.0, 1.0]]


def test_detector_is_asked_for_the_bare_singular_noun_too() -> None:
    assert detector_phrasings("trees") == ["trees", "tree"]
    assert detector_phrasings("a tree") == ["a tree", "tree"]
    assert detector_phrasings("the parked cars") == ["the parked cars", "parked car"]
    assert detector_phrasings("mailboxes") == ["mailboxes", "mailbox"]
    assert detector_phrasings("batteries") == ["batteries", "battery"]
    assert detector_phrasings("a bus") == ["a bus", "bus"]
    assert detector_phrasings("glass") == ["glass"]
    assert detector_phrasings("people") == ["people", "person"]
    assert detector_phrasings("tree") == ["tree"]


def test_rectifier_is_identity_without_distortion() -> None:
    image = np.random.default_rng(1).integers(0, 255, size=(480, 640, 3), dtype=np.uint8)
    assert Rectifier(CAMERA)(image) is image
    fisheye = CameraModel(**{**CAMERA.__dict__, "distortion": (-0.07, -0.02, 0.0, 0.0)})
    assert Rectifier(fisheye)(image).shape == image.shape
