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

"""Behavioral tests for the fan-I/O Detection3DModule (the N-in / M-out proof).

These run without recorded data or a real detector: a fake 2D detector and a
synthetic world pointcloud pin the image/pointcloud ``.align()`` edge, the
timestamped TF lookup, the 2D->3D projection math, and the port-keyed
``Bundle`` tail in the default CI lane. test_moduleDB.py drives the same
pipeline against recorded frames on self-hosted runners.

Scene geometry: the camera sits at the world origin looking down +Z (optical
convention, identity world->camera transform), and a dense point cluster
hovers at CLUSTER_CENTER. With fx=fy=600, cx=320, cy=240 the cluster projects
into BBOX, so a 2D detection with that bbox must fuse to a 3D box at the
cluster center - any sign, axis, or frame mix-up moves it somewhere else.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest

from dimos.memory2.fanio import Bundle
from dimos.memory2.module import StreamModule
from dimos.memory2.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.module3D import Detection3DModule
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

if TYPE_CHECKING:
    from dimos.perception.detection.detectors.base import Detector

CLUSTER_CENTER = (0.5, -0.3, 2.0)
BBOX = (420.0, 100.0, 520.0, 200.0)


class _OneBoxDetector:
    """Stands in for the 2D detector: one fixed bbox per frame, counted."""

    def __init__(self) -> None:
        self.calls = 0

    def process_image(self, image: Image) -> ImageDetections2D:
        self.calls += 1
        return ImageDetections2D(
            image=image,
            detections=[
                Detection2DBBox(
                    bbox=BBOX,
                    track_id=1,
                    class_id=7,
                    confidence=0.9,
                    name="box",
                    ts=image.ts,
                    image=image,
                )
            ],
        )


class _StaticTf:
    """Identity world->camera_optical TF that records timestamped lookups.

    ``available=False`` models "no transform within tolerance of the image
    time" (the stale-TF branch).
    """

    def __init__(self, available: bool = True) -> None:
        self.available = available
        self.calls: list[tuple[str, str, float | None, float | None]] = []

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        self.calls.append((parent_frame, child_frame, time_point, time_tolerance))
        if not self.available:
            return None
        return Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=child_frame,
            child_frame_id=parent_frame,
            ts=time_point or 0.0,
        )

    def stop(self) -> None:
        pass


def _camera_info() -> CameraInfo:
    return CameraInfo.from_intrinsics(
        fx=600.0,
        fy=600.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
        frame_id="camera_optical",
    )


def _image(ts: float) -> Image:
    return Image(
        data=np.zeros((480, 640, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=ts,
    )


def _cluster_cloud(ts: float) -> PointCloud2:
    """Dense 0.2 m cube of points centered on CLUSTER_CENTER, in world frame."""
    axis = np.arange(-0.1, 0.1001, 0.02)
    gx, gy, gz = np.meshgrid(
        axis + CLUSTER_CENTER[0],
        axis + CLUSTER_CENTER[1],
        axis + CLUSTER_CENTER[2],
    )
    points = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    return PointCloud2.from_numpy(points, frame_id="world", timestamp=ts)


def _make_module(detector: Detector, tf: _StaticTf) -> Detection3DModule:
    module = Detection3DModule(detector=lambda: detector, camera_info=_camera_info())
    module._tf = tf
    return module


def _run_pipeline(module: Detection3DModule, image_ts: list[float], cloud_ts: list[float]) -> list:
    """Feed timestamped frames/clouds through pipeline() with the same stream
    wiring start() assembles (the memory2/test_module.py pattern)."""
    with MemoryStore() as store:
        images = store.stream("color_image", Image)
        clouds = store.stream("pointcloud", PointCloud2)
        for ts in image_ts:
            images.append(_image(ts), ts=ts)
        for ts in cloud_ts:
            clouds.append(_cluster_cloud(ts), ts=ts)
        module._in_streams = {"color_image": images, "pointcloud": clouds}
        return module.pipeline(images).to_list()


def test_pipeline_aligns_image_with_cloud_and_bundles_2d_3d_outputs() -> None:
    """One pipeline run over two In ports: the image with a cloud within the
    0.25 s align tolerance fuses into a port-keyed Bundle (2D + 3D + per-object
    cloud) stamped at the image ts; the image whose nearest cloud is 1 s away
    emits nothing. If the align edge silently paired the faraway cloud instead,
    the second tick would publish a 3D box fused against the wrong scene."""
    detector = _OneBoxDetector()
    tf = _StaticTf()
    module = _make_module(detector, tf)
    try:
        # Fan-I/O contract of the migrated module: 2 In, 2+ Out, primary first,
        # and the wiring comes from the base class - not a copied start().
        assert list(module.inputs) == ["color_image", "pointcloud"]
        assert {"detections_2d", "detections_3d"} <= set(module.outputs)
        assert Detection3DModule.start is StreamModule.start

        out = _run_pipeline(
            module,
            image_ts=[10.0, 11.0],
            # 10.05 pairs with the first image; 12.0 is past 11.0 + tolerance,
            # so the second image has no match and the merge can decide without
            # waiting on a (never-arriving) later live cloud.
            cloud_ts=[10.05, 12.0],
        )
    finally:
        module.stop()

    # Both frames were detected in 2D; the align edge gated the unmatched one.
    assert detector.calls == 2
    # Output keeps the primary (image) ts - not the matched cloud's 10.05.
    assert [obs.ts for obs in out] == [10.0]
    # The world->camera transform was looked up at the image time.
    assert tf.calls == [("camera_optical", "world", 10.0, 5.0)]

    bundle = out[0].data
    assert isinstance(bundle, Bundle)
    assert set(bundle.values) <= set(module.outputs)

    d2d = bundle["detections_2d"]
    d3d = bundle["detections_3d"]
    assert isinstance(d2d, Detection2DArray)
    assert isinstance(d3d, Detection3DArray)
    assert d2d.detections_length == 1
    assert d2d.header.timestamp == pytest.approx(10.0)
    assert d3d.detections_length == 1
    assert d3d.header.timestamp == pytest.approx(10.0)

    # Known-input fusion: the 3D box recovers the cluster pose in world frame.
    fused = d3d.detections[0]
    center = fused.bbox.center.position
    assert center.x == pytest.approx(CLUSTER_CENTER[0], abs=0.1)
    assert center.y == pytest.approx(CLUSTER_CENTER[1], abs=0.1)
    assert center.z == pytest.approx(CLUSTER_CENTER[2], abs=0.15)
    # Identity flows through from the 2D detection.
    assert fused.results[0].hypothesis.class_id == "7"
    assert fused.results[0].hypothesis.score == pytest.approx(0.9)

    # The same fused tick also feeds the visualization ports.
    segmented = bundle["detected_pointcloud_0"]
    assert isinstance(segmented, PointCloud2)
    points, _ = segmented.as_numpy()
    assert len(points) > 0
    assert bundle.get("detected_image_0") is not None


def test_missing_transform_degrades_to_empty_3d_while_2d_still_flows() -> None:
    """With no world->camera transform at the image time, the tick still
    emits: 2D detections publish and the 3D array is empty-but-present at the
    image ts ("nothing fused" differs from "port idle"), so a TF outage cannot
    stall the stream or fuse against a stale pose."""
    detector = _OneBoxDetector()
    module = _make_module(detector, _StaticTf(available=False))
    try:
        out = _run_pipeline(module, image_ts=[10.0], cloud_ts=[10.05, 12.0])
    finally:
        module.stop()

    assert [obs.ts for obs in out] == [10.0]
    bundle = out[0].data
    assert bundle["detections_2d"].detections_length == 1
    assert bundle["detections_3d"].detections_length == 0
    assert bundle["detections_3d"].header.timestamp == pytest.approx(10.0)
    # No fused object -> the per-object cloud key is absent (scatter skips it),
    # while the cropped 2D image from the same tick still rides the bundle.
    assert bundle.get("detected_pointcloud_0") is None
    assert bundle.get("detected_image_0") is not None
