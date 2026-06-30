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

from collections.abc import Generator

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.msgs.reconstruction_msgs.ReconstructionStatus import ReconstructionStatus
from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.reconstruction.scene_reconstruction import SceneReconstructionModule


def _camera_info(width: int = 4, height: int = 4, frame_id: str = "camera") -> CameraInfo:
    return CameraInfo.from_intrinsics(
        fx=100.0,
        fy=100.0,
        cx=(width - 1) / 2.0,
        cy=(height - 1) / 2.0,
        width=width,
        height=height,
        frame_id=frame_id,
    )


def _depth_image(
    ts: float = 1.0, frame_id: str = "camera", width: int = 4, height: int = 4
) -> Image:
    return Image(
        data=np.full((height, width), 0.6, dtype=np.float32),
        format=ImageFormat.DEPTH,
        frame_id=frame_id,
        ts=ts,
    )


def _module(**kwargs: object) -> SceneReconstructionModule:
    return SceneReconstructionModule(
        target_frame="world",
        workspace_center=(0.0, 0.0, 0.6),
        workspace_size=1.0,
        depth_trunc=2.0,
        **kwargs,
    )


@pytest.fixture(autouse=True)
def _stop_created_modules(mocker: MockerFixture) -> Generator[None, None, None]:
    modules: list[SceneReconstructionModule] = []
    original_init = SceneReconstructionModule.__init__

    def tracked_init(self: SceneReconstructionModule, **kwargs: object) -> None:
        original_init(self, **kwargs)
        modules.append(self)

    mocker.patch.object(SceneReconstructionModule, "__init__", tracked_init)
    yield
    for module in modules:
        module.stop()


def test_set_workspace_stores_min_corner_and_resets_reconstruction() -> None:
    module = _module()
    original_volume = module._volume
    module._integrated_frames = 3
    module._dropped_frames = 2
    module._latest_integration_ts = 123.0

    result = module.set_workspace(1.0, 2.0, 3.0, 2.0)

    status = module.get_reconstruction_status()
    assert result == "Scene reconstruction workspace updated"
    assert module._volume is not original_volume
    assert module._integrated_frames == 0
    assert module._latest_integration_ts is None
    assert module._last_publish_ts is not None
    assert status.integrated_frames == 0
    assert status.workspace_size == 2.0
    assert status.workspace_origin.x == pytest.approx(0.0)
    assert status.workspace_origin.y == pytest.approx(1.0)
    assert status.workspace_origin.z == pytest.approx(2.0)


def test_missing_camera_info_drops_frame_and_publishes_status() -> None:
    module = _module()
    statuses: list[ReconstructionStatus] = []
    module.status.subscribe(statuses.append)

    module._process_depth_image(_depth_image())

    assert module._integrated_frames == 0
    assert module._dropped_frames == 1
    assert len(statuses) == 1
    assert statuses[0].dropped_frames == 1
    assert statuses[0].last_error == "missing depth camera info"


def test_missing_tf_drops_frame_and_publishes_status(mocker: MockerFixture) -> None:
    module = SceneReconstructionModule(target_frame="world")
    module._set_camera_info(_camera_info(frame_id="world"))
    statuses: list[ReconstructionStatus] = []
    module.status.subscribe(statuses.append)
    mocker.patch.object(module.tf, "get", return_value=None)

    module._process_depth_image(_depth_image())

    assert module._integrated_frames == 0
    assert module._dropped_frames == 1
    assert len(statuses) == 1
    assert statuses[0].last_error == "missing transform"


def test_valid_identity_depth_frame_integrates_and_publishes_outputs() -> None:
    module = _module()
    module._set_camera_info(_camera_info(frame_id="world"))
    pointclouds: list[PointCloud2] = []
    tsdfs: list[TSDFGrid] = []
    statuses: list[ReconstructionStatus] = []
    module.scene_pointcloud.subscribe(pointclouds.append)
    module.tsdf.subscribe(tsdfs.append)
    module.status.subscribe(statuses.append)

    module._process_depth_image(_depth_image(ts=1.0, frame_id="world"))

    assert len(pointclouds) == 1
    assert len(tsdfs) == 1
    assert len(statuses) == 1
    assert tsdfs[0].distances.shape == (1, 40, 40, 40)
    assert tsdfs[0].frame_id == "world"
    assert pointclouds[0].frame_id == "world"
    assert statuses[0].integrated_frames == 1
    assert statuses[0].latest_integration_ts == 1.0


def test_publication_throttling_integrates_but_skips_outputs() -> None:
    module = _module(reconstruction_fps=1.0)
    module._set_camera_info(_camera_info(frame_id="world"))
    pointclouds: list[PointCloud2] = []
    tsdfs: list[TSDFGrid] = []
    statuses: list[ReconstructionStatus] = []
    module.scene_pointcloud.subscribe(pointclouds.append)
    module.tsdf.subscribe(tsdfs.append)
    module.status.subscribe(statuses.append)

    module._process_depth_image(_depth_image(ts=1.0, frame_id="world"))
    module._process_depth_image(_depth_image(ts=1.2, frame_id="world"))

    assert module._integrated_frames == 2
    assert len(pointclouds) == 1
    assert len(tsdfs) == 1
    assert len(statuses) == 1
    assert statuses[0].integrated_frames == 1
