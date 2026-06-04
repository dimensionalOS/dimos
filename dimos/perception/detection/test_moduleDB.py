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

"""Streaming-path tests for ObjectDBModule.

The object database suite (test_object3d.py) feeds 3D detections via
add_detections() by hand, which never exercises pipeline(): the QualityWindow
gate, the 2D detector transform, the image/pointcloud .align() edge, the
timestamped TF lookup, the 3D fusion, the object-DB tap, and the port-keyed
Bundle tail. These tests drive that exact path with recorded frames.

Clock domains in the recording: the odom payloads are stamped ~58 s behind
the lidar/video clock, so transforms built from them are rejected by the
pipeline's timestamped ``tf.get(..., image.ts, 5.0)``. The happy-path test
restamps the transforms into the image clock domain (what a clock-synced
robot produces); the degradation test feeds them as-is to pin the
stale-transform branch.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from dimos.memory2.store.memory import MemoryStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.moduleDB import ObjectDBModule
from dimos.robot.unitree.go2 import connection

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.perception.detection.detectors.base import Detector
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

pytestmark = pytest.mark.self_hosted

SEEKS = (10.0, 12.0, 14.0)


class _CountingDetector:
    """Delegates to a real 2D detector while counting how often it runs."""

    def __init__(self, inner: Detector) -> None:
        self._inner = inner
        self.calls = 0

    def process_image(self, image: Image) -> ImageDetections2D:
        self.calls += 1
        return self._inner.process_image(image)


def _make_module(detector: _CountingDetector) -> ObjectDBModule:
    return ObjectDBModule(
        detector=lambda: detector,
        camera_info=connection._camera_info_static(),
    )


def _camera_transforms(moment: dict, ts: float | None = None) -> list[Transform]:
    """Rebuild the moment's world->camera transform chain, optionally restamped.

    ``ts`` restamps the chain into the image clock domain, modeling a robot
    whose odometry and camera share a clock.
    """
    transforms = connection.GO2Connection._odom_to_tf(moment["odom_frame"])
    if ts is not None:
        for transform in transforms:
            transform.ts = ts
    return transforms


def test_streaming_pipeline_fills_object_db_through_tap(get_moment) -> None:
    """One pipeline run over recorded frames must populate the object DB via
    the tap (no manual add_detections), emit one port-keyed Bundle per image
    stamped at the image capture time, and run the 2D detector once per fused
    tick rather than once per consumer."""
    from dimos.perception.detection.detectors.yolo import Yolo2DDetector

    detector = _CountingDetector(Yolo2DDetector(device="cpu"))
    module = _make_module(detector)
    try:
        with MemoryStore() as store:
            images = store.stream("color_image", Image)
            clouds = store.stream("pointcloud", PointCloud2)
            image_ts: list[float] = []
            for seek in SEEKS:
                moment = get_moment(seek=seek)
                image, cloud = moment["image_frame"], moment["lidar_frame"]
                module.tf.receive_transform(*_camera_transforms(moment, ts=image.ts))
                images.append(image, ts=image.ts)
                clouds.append(cloud, ts=cloud.ts)
                image_ts.append(image.ts)
            # Trailing cloud past the last image so the live sibling stream can
            # resolve the nearest cloud for the final tick without blocking.
            tail = get_moment(seek=SEEKS[-1] + 2.0)["lidar_frame"]
            clouds.append(tail, ts=tail.ts)

            # Same wiring start() assembles, minus live transports - the
            # pattern memory2/test_module.py uses to drive pipeline() directly.
            module._in_streams = {"color_image": images, "pointcloud": clouds}
            out = module.pipeline(images).to_list()
    finally:
        module._close_module()

    # One fused Bundle per image window, stamped at the image (primary) ts -
    # not the matched pointcloud's.
    assert [obs.ts for obs in out] == image_ts
    # The 2D detector ran once per fused tick: the DB tap and the Bundle tail
    # share a single pipeline run instead of re-triggering detection.
    assert detector.calls == len(out)

    fused_total = 0
    for obs in out:
        d2d = obs.data["detections_2d"]
        d3d = obs.data["detections_3d"]
        assert isinstance(d2d, Detection2DArray)
        assert isinstance(d3d, Detection3DArray)
        # 2D and 3D arrays from one tick carry the same image capture time.
        assert d2d.header.timestamp == pytest.approx(obs.ts, abs=1e-6)
        assert d3d.header.timestamp == pytest.approx(obs.ts, abs=1e-6)
        fused_total += d3d.detections_length

    assert fused_total > 0, "recorded frames should yield 3D detections"
    # The tap filled the DB as a side effect of the same run: every fused
    # detection was either merged into an existing object or created a new
    # one, so per-object counters conserve the total (nothing dropped, nothing
    # double-added by a second subscription).
    assert module.objects
    merged_total = sum(obj.detections for obj in module.objects.values())
    assert merged_total == fused_total


def test_stale_transform_degrades_to_empty_3d_not_a_stall(get_moment) -> None:
    """When no transform resolves within tolerance of the image time (here:
    the recording's ~58 s odom clock skew, fed unrestamped), the tick must
    still flow: 2D detections publish, the 3D array is empty-but-present
    (nothing detected != port idle), and no object is minted from a stale
    pose."""
    from dimos.perception.detection.detectors.yolo import Yolo2DDetector

    detector = _CountingDetector(Yolo2DDetector(device="cpu"))
    module = _make_module(detector)
    try:
        with MemoryStore() as store:
            images = store.stream("color_image", Image)
            clouds = store.stream("pointcloud", PointCloud2)
            moment = get_moment(seek=SEEKS[0])
            image, cloud = moment["image_frame"], moment["lidar_frame"]
            # As recorded: transform stamps are ~58 s away from image.ts, so
            # tf.get(..., image.ts, 5.0) inside _fuse finds nothing.
            module.tf.receive_transform(*_camera_transforms(moment))
            images.append(image, ts=image.ts)
            clouds.append(cloud, ts=cloud.ts)
            tail = get_moment(seek=SEEKS[0] + 2.0)["lidar_frame"]
            clouds.append(tail, ts=tail.ts)

            module._in_streams = {"color_image": images, "pointcloud": clouds}
            out = module.pipeline(images).to_list()
    finally:
        module._close_module()

    assert [obs.ts for obs in out] == [image.ts]  # the tick was not swallowed
    bundle = out[0].data
    d2d = bundle["detections_2d"]
    d3d = bundle["detections_3d"]
    assert d2d.detections_length > 0  # the 2D path does not depend on TF
    assert d3d.detections_length == 0  # stale pose -> no 3D fusion
    assert d3d.header.timestamp == pytest.approx(image.ts, abs=1e-6)
    assert module.objects == {}  # no objects minted from a stale transform
