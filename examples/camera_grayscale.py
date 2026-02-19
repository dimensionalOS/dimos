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

from dimos.core import In, Module, Out, autoconnect, rpc
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.visualization.rerun.bridge import RerunBridgeModule


class Grayscale(Module):
    color_image: In[Image]
    gray_image: Out[Image]

    @rpc
    def start(self):
        self.color_image.subscribe(self._publish_grayscale)

    def _publish_grayscale(self, image: Image):
        self.gray_image.publish(image.to_grayscale())


if __name__ == "__main__":
    autoconnect(
        CameraModule.blueprint(),
        Grayscale.blueprint(),
        RerunBridgeModule.blueprint(),
    ).build().loop()
