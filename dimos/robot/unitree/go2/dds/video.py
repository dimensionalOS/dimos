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

"""Stateful h264 decode for the Go2 front camera (``rt/frontvideo/h264``).

The camera streams h264 where each P-frame references earlier frames, so a
packet can't be decoded on its own (unlike the single-frame jpeg
``rt/frontvideo`` path). This transform holds one PyAV decoder across the
*ordered* packet stream and emits decoded ``Image`` observations::

    from dimos.robot.unitree.go2.dds.store import Go2McapStore
    from dimos.robot.unitree.go2.dds.video import H264Decoder

    store = Go2McapStore(path="go2_dds_stairs.mcap")
    for obs in store.streams.color_image_h264.transform(H264Decoder()):
        obs.data  # a BGR dimos Image (1280x720)

Because state carries forward, iterate from the start (or a keyframe). Packets
the decoder can't yet resolve — e.g. P-frames after a mid-GOP seek — are
dropped until the next keyframe re-syncs it.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import TYPE_CHECKING, Any

from dimos.memory2.transform import Transformer
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.unitree.go2.dds.msgs.CompressedVideo import CompressedVideo

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation


class H264Decoder(Transformer[CompressedVideo, Image]):
    """Decode an ordered ``CompressedVideo`` (h264) stream into BGR ``Image`` frames."""

    def __call__(
        self, upstream: Iterator[Observation[CompressedVideo]]
    ) -> Iterator[Observation[Image]]:
        import av  # optional dep (go2/unitree extra)

        decoder = av.codec.CodecContext.create("h264", "r")
        last: Observation[CompressedVideo] | None = None
        for obs in upstream:
            packet = obs.data
            if packet is None:
                continue
            last = obs
            try:
                frames = decoder.decode(av.packet.Packet(packet.data.tobytes()))
            except av.error.FFmpegError:
                continue  # P-frame with no reference yet (e.g. seeked past a keyframe)
            for frame in frames:
                yield self._emit(frame, obs)

        # Flush: delayed frames (B-frames / reordering) still buffered after the
        # last packet. decode(None) drains them so the tail of the stream isn't lost.
        if last is not None:
            for frame in decoder.decode(None):
                yield self._emit(frame, last)

    @staticmethod
    def _emit(frame: Any, obs: Observation[CompressedVideo]) -> Observation[Image]:
        bgr = frame.to_ndarray(format="bgr24")
        return obs.derive(data=Image.from_numpy(bgr, ImageFormat.BGR, obs.data.frame_id, obs.ts))
