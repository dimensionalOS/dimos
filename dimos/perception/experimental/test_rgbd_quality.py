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

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.experimental.rgbd_quality import filter_depth_detections, validate_rgbd


@pytest.fixture
def rgbd():
    color = Image(np.zeros((32, 32, 3), np.uint8), ImageFormat.RGB, frame_id="camera", ts=100.0)
    depth = Image(np.ones((32, 32), np.float32), ImageFormat.DEPTH, frame_id="camera", ts=100.0)
    info = CameraInfo(
        width=32, height=32, K=[100.0, 0, 16, 0, 100.0, 16, 0, 0, 1], frame_id="camera", ts=100.0
    )
    return color, depth, info


def test_coherent_rgbd_passes(rgbd):
    validate_rgbd(*rgbd, 100.1, 0.5, 0.002)


@pytest.mark.parametrize("change", ["stale", "future", "skew", "frame", "shape", "intrinsics"])
def test_invalid_rgbd_is_rejected(rgbd, change):
    color, depth, info = rgbd
    if change == "stale":
        color.ts = 99.0
    elif change == "future":
        depth.ts = 101.0
    elif change == "skew":
        depth.ts = 100.01
    elif change == "frame":
        info.frame_id = "wrong"
    elif change == "shape":
        info.width = 31
    else:
        info.K[0] = float("nan")
    with pytest.raises(ValueError):
        validate_rgbd(color, depth, info, 100.1, 0.5, 0.002)


def segmentation(image, name, mask):
    return Detection2DSeg(
        bbox=(0, 0, 32, 32),
        track_id=-1,
        class_id=0,
        confidence=0.9,
        name=name,
        ts=image.ts,
        image=image,
        mask=mask,
    )


def test_depth_support_is_per_mask_and_does_not_fill_missing_pixels(rgbd):
    color, depth, _ = rgbd
    depth.data[:, :16] = 0
    left = np.zeros((32, 32), np.uint8)
    left[:, 0:12] = 255
    right = np.zeros((32, 32), np.uint8)
    right[:, 20:] = 255
    detections = ImageDetections2D(
        color, [segmentation(color, "bottle", left), segmentation(color, "cup", right)]
    )
    kept, reports, overlay = filter_depth_detections(detections, depth.data, 0.3, 100, True)
    assert [d.name for d in kept.detections] == ["cup"]
    assert reports[0].valid_depth_pixels == 0 and not reports[0].accepted
    assert reports[1].valid_depth_fraction == 1 and reports[1].accepted
    assert np.all(depth.data[:, :16] == 0)
    assert np.any(overlay != color.data)


def test_box_or_wrong_size_mask_cannot_be_used_as_foreground(rgbd):
    color, depth, _ = rgbd
    seg = segmentation(color, "bottle", np.ones((2, 2), np.uint8))
    box = Detection2DBBox(
        bbox=(0, 0, 32, 32),
        track_id=-1,
        class_id=0,
        confidence=0.9,
        name="cup",
        ts=color.ts,
        image=color,
    )
    kept, reports, _ = filter_depth_detections(
        ImageDetections2D(color, [seg, box]), depth.data, 0.3, 10, True
    )
    assert kept.detections == []
    assert [r.reason for r in reports] == [
        "mask dimensions do not match depth",
        "segmentation mask required",
    ]


def test_nonfinite_depth_never_counts_as_foreground(rgbd):
    color, depth, _ = rgbd
    depth.data[:] = np.nan
    seg = segmentation(color, "cup", np.ones((32, 32), np.uint8))
    _, reports, _ = filter_depth_detections(
        ImageDetections2D(color, [seg]), depth.data, 0.3, 10, True
    )
    assert reports[0].valid_depth_pixels == 0 and not reports[0].accepted
