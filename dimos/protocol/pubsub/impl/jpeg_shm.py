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

from typing import Any

from turbojpeg import TurboJPEG

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.pubsub.impl.shmpubsub import SharedMemoryPubSubBase


class JpegSharedMemory(SharedMemoryPubSubBase):
    def __init__(self, quality: int = 75, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self.jpeg = TurboJPEG()
        self.quality = quality

    def publish(self, topic: str, msg: Any) -> None:
        bgr_image = msg.to_bgr().to_opencv()
        super().publish(topic, self.jpeg.encode(bgr_image, quality=self.quality))

    def subscribe(self, topic: str, callback):  # type: ignore[no-untyped-def]
        return super().subscribe(
            topic,
            lambda message, name: callback(
                Image(data=self.jpeg.decode(message), format=ImageFormat.BGR), name
            ),
        )
