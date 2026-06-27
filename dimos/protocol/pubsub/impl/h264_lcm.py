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

"""H.264-compressed Image transport over LCM."""

from __future__ import annotations

from collections.abc import Callable
from typing import cast

from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.encoders import DecodingError, LCMTopicProto, PubSubEncoderMixin
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase
from dimos.protocol.video.h264 import (
    H264Config,
    H264Decoder,
    H264Encoder,
    H264Packet,
    VideoDecodeGapError,
)


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
        if not decode_images:
            raise ValueError("H.264 transport always decodes packets back to Image")
        self.h264_config = config or H264Config()
        # Test override hooks; real publish state is per-topic below and real
        # subscribe state is closure-local per subscription.
        self._encoder: H264Encoder | None = None
        self._decoder: H264Decoder | None = None
        self._encoders: dict[str, H264Encoder] = {}
        self._decoders: dict[str, H264Decoder] = {}

    def encode(self, msg: Image, topic: LCMTopicProto) -> bytes:
        encoder = self._encoder
        if encoder is None:
            encoder = self._encoders.get(topic.topic)
            if encoder is None:
                encoder = H264Encoder(self.h264_config)
                self._encoders[topic.topic] = encoder
        return encoder.encode(msg).to_bytes()

    def decode(self, msg: bytes, topic: LCMTopicProto) -> Image:
        packet = self._parse_packet(msg, topic)
        decoder = self._decoder
        if decoder is None:
            decoder = self._decoders.get(topic.topic)
            if decoder is None:
                decoder = H264Decoder(self.h264_config)
                self._decoders[topic.topic] = decoder
        return self._decode_packet(packet, decoder)

    def subscribe(
        self, topic: LCMTopicProto, callback: Callable[[Image, LCMTopicProto], None]
    ) -> Callable[[], None]:
        """Subscribe with an independent H.264 decoder per callback/topic."""

        decoders: dict[str, H264Decoder] = {}

        def wrapper_cb(encoded_data: bytes, callback_topic: LCMTopicProto) -> None:
            try:
                packet = self._parse_packet(encoded_data, callback_topic)
                decoder = self._decoder
                if decoder is None:
                    decoder = decoders.get(callback_topic.topic)
                    if decoder is None:
                        decoder = H264Decoder(self.h264_config)
                        decoders[callback_topic.topic] = decoder
                decoded_message = self._decode_packet(packet, decoder)
            except DecodingError:
                return
            callback(decoded_message, callback_topic)

        return cast(
            "Callable[[], None]",
            # Intentionally skip PubSubEncoderMixin.subscribe: H.264 decoding
            # needs a fresh decoder per callback, not the shared decode() state.
            super(PubSubEncoderMixin, self).subscribe(topic, wrapper_cb),  # type: ignore[misc]
        )

    def _parse_packet(self, msg: bytes, topic: LCMTopicProto) -> H264Packet:
        if topic.topic == "LCM_SELF_TEST":
            raise DecodingError("Ignoring LCM_SELF_TEST topic")
        if topic.lcm_type is not None and not issubclass(topic.lcm_type, Image):
            raise DecodingError(f"H.264 LCM topic {topic.topic!r} is not typed as Image")
        try:
            return H264Packet.from_bytes(msg)
        except ValueError as exc:
            raise DecodingError(str(exc)) from exc

    @staticmethod
    def _decode_packet(packet: H264Packet, decoder: H264Decoder) -> Image:
        try:
            return decoder.decode(packet)
        except (VideoDecodeGapError, ValueError) as exc:
            raise DecodingError(str(exc)) from exc


class H264LCM(  # type: ignore[misc]
    H264EncoderMixin,
    LCMPubSubBase,
): ...


__all__ = ["H264LCM", "H264EncoderMixin"]
