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

"""A camera on the operator laptop that films the robot from outside, as JPEG."""

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image

# The colour node of the RealSense D435if on the operator laptop.
D435IF_COLOR = (
    "/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435if"
    "_Intel_R__RealSense_TM__Depth_Camera_435if-video-index0"
)


class R1ProExternalCameraConfig(ModuleConfig):
    device: str = D435IF_COLOR
    width: int = 640
    height: int = 480
    fps: float = 30.0
    jpeg_quality: int = 70
    frame_id: str = "external_camera"


class R1ProExternalCamera(Module):
    """Read a V4L2 colour camera and publish each frame as JPEG.

    Config:
        device: V4L2 device path of the camera's colour node.
        width, height: Requested frame size, in pixels.
        fps: Requested frame rate, in frames per second.
        jpeg_quality: JPEG quality, 1-100. 70 at 640x480 is about 36 kB a frame.
        frame_id: Frame name stamped on every image.
    """

    config: R1ProExternalCameraConfig
    color: Out[CompressedImage]

    @rpc
    def start(self) -> None:
        super().start()
        camera = Webcam(
            camera_index=self.config.device,
            width=self.config.width,
            height=self.config.height,
            fps=self.config.fps,
        )
        self.register_disposable(camera.image_stream().subscribe(self._publish))

    def _publish(self, image: Image) -> None:
        jpeg = CompressedImage.from_image(image, quality=self.config.jpeg_quality)
        jpeg.frame_id = self.config.frame_id
        self.color.publish(jpeg)
