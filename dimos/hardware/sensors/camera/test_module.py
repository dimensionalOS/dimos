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

from collections.abc import Iterator

import pytest
import pytest_mock
import reactivex as rx

from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.protocol.rpc.pubsubrpc import LCMRPC


@pytest.fixture
def camera_module(mocker: pytest_mock.MockerFixture) -> Iterator[CameraModule]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)
    hardware = Webcam()
    mocker.patch.object(hardware, "image_stream", return_value=rx.never())
    module = CameraModule(hardware=hardware)
    module.color_image = mocker.MagicMock()  # type: ignore[assignment]
    module.camera_info = mocker.MagicMock()  # type: ignore[assignment]
    module.tf = mocker.MagicMock()  # type: ignore[assignment]
    yield module
    module.stop()


def test_start_publishes_camera_metadata_immediately(camera_module: CameraModule) -> None:
    camera_module.start()

    camera_module.camera_info.publish.assert_called_once()  # type: ignore[attr-defined]
    camera_module.tf.publish.assert_called_once()  # type: ignore[attr-defined]
