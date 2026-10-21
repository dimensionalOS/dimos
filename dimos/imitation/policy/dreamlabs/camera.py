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

"""Expose the existing Webcam/CameraModule through normal blueprint configuration."""

from dimos.core.core import rpc
from dimos.hardware.sensors.camera.module import CameraModule, CameraModuleConfig
from dimos.hardware.sensors.camera.webcam import Webcam


class DreamlabsCameraConfig(CameraModuleConfig):
    device: str


class DreamlabsCameraModule(CameraModule):
    config: DreamlabsCameraConfig

    @rpc
    def start(self) -> None:
        self.config.hardware = Webcam(camera_index=self.config.device)
        super().start()
