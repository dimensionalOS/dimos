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

"""H.264-encoded Image transport over LCM."""

from __future__ import annotations

from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.encoders import DecodingError, LCMTopicProto, PubSubEncoderMixin
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase
from dimos.protocol.video.h264 import H264Config, H264Decoder, H264Encoder, VideoDecodeGapError


class H264EncoderMixin(PubSubEncoderMixin[LCMTopicProto, Image, bytes]):
    """Encoder mixin for Image streams using H.264 packets on the wire."""

    def __init__(
        self,
        *,
        config: H264Config | None = None,
        decode_images: bool = True,
        **kwargs: object,
    ) -> None:
        super().__init__(**kwargs)  # type: ignore[misc]
        self.h264_config = config or H264Config()
        self.decode_images = decode_images
        self._encoder: H264Encoder | None = None
        self._decoder: H264Decoder | None = None

    def encode(self, msg: Image, topic: LCMTopicProto) -> bytes:
        if self._encoder is None:
            self._encoder = H264Encoder(self.h264_config)
        return self._encoder.encode(msg).lcm_encode()

    def decode(self, msg: bytes, topic: LCMTopicProto) -> Image:
        if topic.topic == "LCM_SELF_TEST":
            raise DecodingError("Ignoring LCM_SELF_TEST topic")
        if topic.lcm_type is not None and not issubclass(topic.lcm_type, Image):
            raise DecodingError(f"H.264 LCM topic {topic.topic!r} is not typed as Image")
        try:
            image = Image.lcm_decode(msg)
        except ValueError as exc:
            raise DecodingError(str(exc)) from exc
        if not self.decode_images:
            return image
        if self._decoder is None:
            self._decoder = H264Decoder(self.h264_config)
        try:
            return self._decoder.decode(image)
        except (VideoDecodeGapError, ValueError) as exc:
            raise DecodingError(str(exc)) from exc


class H264LCM(  # type: ignore[misc]
    H264EncoderMixin,
    LCMPubSubBase,
): ...


__all__ = ["H264LCM", "H264EncoderMixin"]
