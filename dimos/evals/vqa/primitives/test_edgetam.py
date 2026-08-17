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

from typing import TYPE_CHECKING, cast

import numpy as np
import pytest

from dimos.evals.vqa.families import InsufficientEvidenceError
from dimos.evals.vqa.preprocessing import CalibratedFrame
from dimos.evals.vqa.primitives.edgetam import EdgeTamLidarRangeEstimator
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

BBox = tuple[float, float, float, float]


class _PointCloud:
    def __init__(self, points: np.ndarray) -> None:
        self._points = points
        self.read_count = 0

    def as_numpy(self) -> tuple[np.ndarray, None]:
        self.read_count += 1
        return self._points, None


class _Detector:
    def __init__(self, boxes: list[BBox]) -> None:
        self._boxes = boxes

    def detect(self, image: Image, object_name: str) -> ImageDetections2D[Detection2DBBox]:
        detections = [
            Detection2DBBox(
                bbox=box,
                track_id=index,
                class_id=0,
                confidence=1.0,
                name=object_name,
                ts=image.ts,
                image=image,
            )
            for index, box in enumerate(self._boxes)
        ]
        return ImageDetections2D(image, detections)


class _Segmenter:
    def __init__(self, mask: np.ndarray) -> None:
        self._mask = mask
        self.prompted_boxes: list[BBox] = []

    def segment(
        self, detections: ImageDetections2D[Detection2DBBox]
    ) -> ImageDetections2D[Detection2DBBox]:
        prompt = detections[0]
        self.prompted_boxes.append(prompt.bbox)
        segmented: list[Detection2DBBox] = [
            Detection2DSeg(
                bbox=prompt.bbox,
                track_id=prompt.track_id,
                class_id=prompt.class_id,
                confidence=prompt.confidence,
                name=prompt.name,
                ts=prompt.ts,
                image=detections.image,
                mask=self._mask,
            )
        ]
        return ImageDetections2D(detections.image, segmented)


def _frame(
    pointcloud: _PointCloud,
    *,
    index: int = 0,
    transform: Transform | None = None,
    fx: float = 2.0,
    fy: float = 2.0,
    cx: float = 5.0,
    cy: float = 5.0,
) -> CalibratedFrame:
    image = Image(data=np.zeros((10, 10, 3), dtype=np.uint8), ts=10.0)
    return CalibratedFrame(
        index=index,
        image=image,
        pointcloud=cast("PointCloud2", pointcloud),
        camera_info=CameraInfo.from_intrinsics(fx, fy, cx, cy, 10, 10),
        pointcloud_to_camera=transform or Transform.identity(),
        image_observation_timestamp=10.0,
        pointcloud_observation_timestamp=9.98,
        calibration_source="synthetic-calibration",
    )


def _full_mask() -> np.ndarray:
    return np.ones((10, 10), dtype=np.uint8)


def test_projects_transformed_points_and_rejects_invalid_projections() -> None:
    pointcloud = _PointCloud(
        np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, 0.0, -2.0],
                [10.0, 0.0, 1.0],
            ]
        )
    )
    frame = _frame(
        pointcloud,
        transform=Transform(translation=Vector3(1.0, 0.0, 0.0)),
        cx=0.0,
        cy=0.0,
    )
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[0, 2] = 1

    evidence = EdgeTamLidarRangeEstimator(
        _Detector([(0.0, 0.0, 3.0, 2.0)]), _Segmenter(mask), min_supporting_points=1
    ).estimate(frame, "crate")

    assert evidence.camera_range_m == pytest.approx(np.sqrt(2.0))
    assert evidence.supporting_point_count == 1


def test_mask_selection_uses_nearest_camera_z_point_per_pixel() -> None:
    pointcloud = _PointCloud(
        np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, 0.0, 3.0],
                [1.0, 0.0, 1.0],
            ]
        )
    )
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[5, 5] = 1

    evidence = EdgeTamLidarRangeEstimator(
        _Detector([(0.0, 0.0, 10.0, 10.0)]), _Segmenter(mask), min_supporting_points=1
    ).estimate(_frame(pointcloud), "chair")

    assert evidence.supporting_point_count == 1
    assert evidence.camera_range_m == pytest.approx(1.0)
    assert evidence.mask_area_px == 1


@pytest.mark.parametrize("boxes", [[], [(0.0, 0.0, 4.0, 4.0), (5.0, 5.0, 9.0, 9.0)]])
def test_requires_exactly_one_valid_detection(boxes: list[BBox]) -> None:
    estimator = EdgeTamLidarRangeEstimator(_Detector(boxes), _Segmenter(_full_mask()))

    with pytest.raises(InsufficientEvidenceError, match="exactly one valid detected"):
        estimator.estimate(_frame(_PointCloud(np.array([[0.0, 0.0, 1.0]]))), "cup")


def test_rejects_too_few_mask_supporting_points() -> None:
    pointcloud = _PointCloud(
        np.array([[-2.0, 0.0, 1.0], [-1.5, 0.0, 1.0], [-1.0, 0.0, 1.0], [-0.5, 0.0, 1.0]])
    )
    estimator = EdgeTamLidarRangeEstimator(
        _Detector([(0.0, 0.0, 10.0, 10.0)]), _Segmenter(_full_mask())
    )

    with pytest.raises(InsufficientEvidenceError, match="at least 5 supporting points, got 4"):
        estimator.estimate(_frame(pointcloud), "bottle")


def test_returns_median_euclidean_range_and_auditable_quartiles() -> None:
    points = np.array([[u * z, 0.0, z] for u, z in zip(range(1, 6), [1, 2, 3, 4, 20], strict=True)])
    expected_ranges = np.linalg.norm(points, axis=1)
    frame = _frame(_PointCloud(points), fx=1.0, fy=1.0, cx=0.0, cy=0.0)

    evidence = EdgeTamLidarRangeEstimator(
        _Detector([(0.0, 0.0, 9.0, 9.0)]), _Segmenter(_full_mask())
    ).estimate(frame, "cone")

    assert evidence.camera_range_m == pytest.approx(np.median(expected_ranges))
    assert evidence.range_quartiles_m == pytest.approx(
        np.quantile(expected_ranges, [0.25, 0.5, 0.75])
    )
    assert evidence.synchronization_delta_s == pytest.approx(0.02)
    assert evidence.calibration_source == "synthetic-calibration"
    assert evidence.model_dump(mode="json")["prompt_bbox_xyxy"] == [0.0, 0.0, 9.0, 9.0]
    assert evidence.model_dump(mode="json")["mask_bbox_xyxy"] == [0.0, 0.0, 9.0, 9.0]


def test_projection_cache_reuses_only_the_same_explicit_frame() -> None:
    points = np.array([[float(2 * x), 0.0, 2.0] for x in range(5)])
    first_cloud = _PointCloud(points)
    second_cloud = _PointCloud(points)
    first_frame = _frame(first_cloud, index=3, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    second_frame = _frame(second_cloud, index=3, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    estimator = EdgeTamLidarRangeEstimator(
        _Detector([(0.0, 0.0, 9.0, 9.0)]), _Segmenter(_full_mask())
    )

    estimator.estimate(first_frame, "box")
    estimator.estimate(first_frame, "box")
    estimator.estimate(second_frame, "box")

    assert first_cloud.read_count == 1
    assert second_cloud.read_count == 1
