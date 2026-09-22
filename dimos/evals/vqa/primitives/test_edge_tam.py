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

from dimos.evals.vqa.contracts import InsufficientEvidenceError
from dimos.evals.vqa.pointcloud_frame import PointCloudFrame
from dimos.evals.vqa.primitives.edge_tam import EdgeTAMObjectMaskPipeline
from dimos.evals.vqa.primitives.range import LidarRangeEstimator
from dimos.models.vl.base import VlModel
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Bbox, Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class _PointCloud:
    def __init__(self, points: np.ndarray) -> None:
        self._points = points
        self.read_count = 0
        self.frame_id = "lidar"
        self.ts = 9.98

    def as_numpy(self) -> tuple[np.ndarray, None]:
        self.read_count += 1
        return self._points, None


class _TestVlModel(VlModel):
    def __init__(self) -> None:
        pass

    def query(self, image: Image, query: str, **kwargs: object) -> str:
        raise NotImplementedError

    def stop(self) -> None:
        pass


class _Detector(_TestVlModel):
    def __init__(self, boxes: list[Bbox]) -> None:
        self._boxes = boxes

    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
        detections = [
            Detection2DBBox(
                bbox=box,
                track_id=index,
                class_id=0,
                confidence=1.0,
                name=query,
                ts=image.ts,
                image=image,
            )
            for index, box in enumerate(self._boxes)
        ]
        return ImageDetections2D(image, detections)


class _NamedDetector(_TestVlModel):
    def __init__(self, boxes: dict[str, Bbox]) -> None:
        self._boxes = boxes

    def query_detections(
        self, image: Image, query: str, **kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
        box = self._boxes[query]
        return ImageDetections2D(
            image,
            [
                Detection2DBBox(
                    bbox=box,
                    track_id=len(query),
                    class_id=0,
                    confidence=1.0,
                    name=query,
                    ts=image.ts,
                    image=image,
                )
            ],
        )


class _Segmenter:
    def __init__(self, mask: np.ndarray | list[np.ndarray]) -> None:
        self._masks = mask if isinstance(mask, list) else [mask]
        self.prompted_boxes: list[Bbox] = []
        self.call_count = 0

    def segment(
        self, detections: ImageDetections2D[Detection2DBBox]
    ) -> ImageDetections2D[Detection2DSeg]:
        self.call_count += 1
        self.prompted_boxes.extend(prompt.bbox for prompt in detections)
        masks = (
            self._masks * len(detections)
            if len(self._masks) == 1
            else self._masks[: len(detections)]
        )
        segmented: list[Detection2DSeg] = [
            Detection2DSeg(
                bbox=prompt.bbox,
                track_id=prompt.track_id,
                class_id=prompt.class_id,
                confidence=prompt.confidence,
                name=prompt.name,
                ts=prompt.ts,
                image=detections.image,
                mask=mask,
            )
            for prompt, mask in zip(detections, masks, strict=True)
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
) -> PointCloudFrame:
    image = Image(data=np.zeros((10, 10, 3), dtype=np.uint8), ts=10.0)
    return PointCloudFrame(
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


def _range_estimator(
    detector: _Detector | _NamedDetector,
    segmenter: _Segmenter,
    min_supporting_points: int = 5,
) -> LidarRangeEstimator:
    return LidarRangeEstimator(
        EdgeTAMObjectMaskPipeline(detector, segmenter),
        min_supporting_points,
    )


def test_mask_estimator_batches_and_caches_without_pointcloud() -> None:
    image = Image(data=np.zeros((10, 10, 3), dtype=np.uint8), ts=10.0)
    left_mask = np.zeros((10, 10), dtype=np.uint8)
    right_mask = np.zeros((10, 10), dtype=np.uint8)
    left_mask[:, :4] = 1
    right_mask[:, 5:] = 1
    segmenter = _Segmenter([left_mask, right_mask])
    estimator = EdgeTAMObjectMaskPipeline(
        _NamedDetector(
            {
                "left person": (0.0, 0.0, 4.0, 9.0),
                "right person": (5.0, 0.0, 9.0, 9.0),
            }
        ),
        segmenter,
    )

    left, right = estimator.estimate_many(image, ("left person", "right person"))
    cached = estimator.estimate(image, "left person")

    assert left.mask_area_px == 40
    assert right.mask_area_px == 50
    assert cached is left
    assert segmenter.call_count == 1

    next_image = Image(data=np.zeros((10, 10, 3), dtype=np.uint8), ts=11.0)
    estimator.estimate(next_image, "left person")

    assert segmenter.call_count == 2


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

    evidence = _range_estimator(
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

    evidence = _range_estimator(
        _Detector([(0.0, 0.0, 10.0, 10.0)]), _Segmenter(mask), min_supporting_points=1
    ).estimate(_frame(pointcloud), "chair")

    assert evidence.supporting_point_count == 1
    assert evidence.camera_range_m == pytest.approx(1.0)
    assert evidence.mask_area_px == 1


@pytest.mark.parametrize("boxes", [[], [(0.0, 0.0, 4.0, 4.0), (5.0, 5.0, 9.0, 9.0)]])
def test_requires_exactly_one_valid_detection(boxes: list[Bbox]) -> None:
    cloud = _PointCloud(np.array([[0.0, 0.0, 1.0]]))
    estimator = _range_estimator(_Detector(boxes), _Segmenter(_full_mask()))

    with pytest.raises(InsufficientEvidenceError, match="exactly one valid detected"):
        estimator.estimate(_frame(cloud), "cup")

    assert cloud.read_count == 0


def test_rejects_too_few_mask_supporting_points() -> None:
    pointcloud = _PointCloud(
        np.array([[-2.0, 0.0, 1.0], [-1.5, 0.0, 1.0], [-1.0, 0.0, 1.0], [-0.5, 0.0, 1.0]])
    )
    estimator = _range_estimator(_Detector([(0.0, 0.0, 10.0, 10.0)]), _Segmenter(_full_mask()))

    with pytest.raises(InsufficientEvidenceError, match="at least 5 supporting points.*got 4"):
        estimator.estimate(_frame(pointcloud), "bottle")


def test_rejects_segmentation_mask_with_wrong_dimensions() -> None:
    estimator = _range_estimator(
        _Detector([(0.0, 0.0, 10.0, 10.0)]),
        _Segmenter(np.ones((9, 10), dtype=np.uint8)),
        min_supporting_points=1,
    )

    with pytest.raises(InsufficientEvidenceError, match="one valid segmentation mask"):
        estimator.estimate(_frame(_PointCloud(np.array([[0.0, 0.0, 1.0]]))), "bottle")


def test_returns_median_euclidean_range_and_auditable_quartiles() -> None:
    points = np.array([[u * z, 0.0, z] for u, z in zip(range(1, 6), [1, 2, 3, 4, 20], strict=True)])
    expected_ranges = np.linalg.norm(points, axis=1)
    frame = _frame(_PointCloud(points), fx=1.0, fy=1.0, cx=0.0, cy=0.0)

    evidence = _range_estimator(
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


def test_projection_cache_reuses_across_objects_only_for_the_same_explicit_frame() -> None:
    points = np.array([[float(2 * x), 0.0, 2.0] for x in range(5)])
    first_cloud = _PointCloud(points)
    second_cloud = _PointCloud(points)
    first_frame = _frame(first_cloud, index=3, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    second_frame = _frame(second_cloud, index=3, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    estimator = _range_estimator(_Detector([(0.0, 0.0, 9.0, 9.0)]), _Segmenter(_full_mask()))

    estimator.estimate(first_frame, "box")
    estimator.estimate(first_frame, "crate")
    estimator.estimate(second_frame, "box")

    assert first_cloud.read_count == 1
    assert second_cloud.read_count == 1


def test_estimate_many_batches_masks_and_reuses_one_projection() -> None:
    cloud = _PointCloud(np.array([[-1.5, 0.0, 1.0], [3.0, 0.0, 3.0]]))
    left_mask = np.zeros((10, 10), dtype=np.uint8)
    right_mask = np.zeros((10, 10), dtype=np.uint8)
    left_mask[5, 2] = 1
    right_mask[5, 7] = 1
    segmenter = _Segmenter([left_mask, right_mask])
    estimator = _range_estimator(
        _NamedDetector(
            {
                "chair": (0.0, 0.0, 4.0, 9.0),
                "table": (5.0, 0.0, 9.0, 9.0),
            }
        ),
        segmenter,
        min_supporting_points=1,
    )

    left, right = estimator.estimate_many(_frame(cloud), ("chair", "table"))

    assert left.object_name == "chair"
    assert right.object_name == "table"
    assert left.camera_range_m < right.camera_range_m
    assert segmenter.call_count == 1
    assert cloud.read_count == 1
