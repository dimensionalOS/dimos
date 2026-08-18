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

from pathlib import Path

import numpy as np
import pytest

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.rgbd import crop_masks, latest_rgbd


def test_latest_rgbd_returns_nearest_calibrated_observation(tmp_path: Path) -> None:
    path = tmp_path / "rgbd.db"
    with SqliteStore(path=str(path)) as store:
        color = store.stream("front_color", Image, codec="pickle")
        depth = store.stream("front_depth", Image, codec="pickle")
        info = store.stream("front_info", CameraInfo)
        tf = store.stream("tf", TFMessage)
        color.append(
            Image(np.zeros((2, 3, 3), dtype=np.uint8), ImageFormat.RGB, "front_optical", 10.0),
            ts=10.0,
        )
        depth.append(
            Image(np.full((2, 3), 2.0, dtype=np.float32), ImageFormat.DEPTH, "front_optical", 9.98),
            ts=9.98,
        )
        info.append(
            CameraInfo.from_intrinsics(100.0, 101.0, 1.5, 1.0, 3, 2, "front_optical").with_ts(
                10.01
            ),
            ts=10.01,
        )
        tf.append(
            TFMessage(
                Transform(
                    translation=Vector3(1.0, 0.0, 0.0),
                    frame_id="world",
                    child_frame_id="front_optical",
                    ts=10.0,
                )
            ),
            ts=10.0,
        )

        observation = latest_rgbd(
            store,
            color_stream="front_color",
            depth_stream="front_depth",
            camera_info_stream="front_info",
            optical_frame="front_optical",
        )

        assert observation.timestamp == 10.0
        assert np.array_equal(observation.depth.data, np.full((2, 3), 2.0, dtype=np.float32))
        assert observation.camera_info.K[0] == 100.0
        assert observation.world_to_optical.frame_id == "front_optical"
        assert observation.world_to_optical.child_frame_id == "world"


def test_latest_rgbd_rejects_unaligned_depth(tmp_path: Path) -> None:
    path = tmp_path / "rgbd.db"
    with SqliteStore(path=str(path)) as store:
        store.stream("color", Image, codec="pickle").append(
            Image(np.zeros((2, 2, 3), dtype=np.uint8), ImageFormat.RGB, "camera", 10.0),
            ts=10.0,
        )
        store.stream("depth", Image, codec="pickle").append(
            Image(np.ones((2, 2), dtype=np.float32), ImageFormat.DEPTH, "camera", 9.0),
            ts=9.0,
        )
        store.stream("info", CameraInfo).append(
            CameraInfo.from_intrinsics(1.0, 1.0, 1.0, 1.0, 2, 2, "camera").with_ts(10.0),
            ts=10.0,
        )

        with pytest.raises(LookupError, match="No 'depth' observation"):
            latest_rgbd(
                store,
                color_stream="color",
                depth_stream="depth",
                camera_info_stream="info",
                optical_frame="camera",
            )


def test_crop_masks_selects_a_normalized_spatial_subregion() -> None:
    image = Image(np.zeros((8, 8, 3), dtype=np.uint8), ImageFormat.RGB, "camera", 10.0)
    mask = np.zeros((8, 8), dtype=np.uint8)
    mask[1:7, 2:6] = 255
    detection = Detection2DSeg.from_sam2_result(
        mask,
        obj_id=4,
        image=image,
        name="stacked handles",
    )

    result = crop_masks(
        ImageDetections2D(image, [detection]),
        x_range=(0.25, 0.75),
        y_range=(0.5, 1.0),
    )

    assert len(result) == 1
    assert result[0].bbox == (3.0, 4.0, 4.0, 6.0)
    assert np.array_equal(np.argwhere(result[0].mask), [*np.argwhere(mask[4:7, 3:5]), 4, 3])
