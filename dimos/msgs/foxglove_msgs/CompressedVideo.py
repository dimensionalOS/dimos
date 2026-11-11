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

"""foxglove_msgs::msg::CompressedVideo — one encoded video packet.

Just the encoded bytes + codec name. Inter-frame codecs (h264) can't be decoded
one packet at a time — pipe the ordered stream through :func:`decode_video` to
get ``Image`` frames, or let the rerun bridge log it as a ``VideoStream`` and
decode in-viewer.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
import time
from types import MappingProxyType
from typing import TYPE_CHECKING

from dimos_lcm.foxglove_msgs import CompressedVideo as LCMCompressedVideo
import numpy as np
import reactivex as rx
from reactivex.abc import DisposableBase, ObserverBase, SchedulerBase

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from av import Packet, VideoCodecContext
    from reactivex import Observable

    from dimos.visualization.rerun.bridge import RerunData


VIDEO_DECODERS: Mapping[str, str] = MappingProxyType(
    {
        "h264": "h264",
        "h265": "hevc",
        "hevc": "hevc",
        "av1": "libdav1d",
        "vp8": "vp8",
        "vp9": "vp9",
    }
)


class CompressedVideo(Timestamped):
    """One encoded video packet (Annex-B for h264)."""

    msg_name = "foxglove_msgs.CompressedVideo"

    def __init__(
        self,
        data: np.ndarray | bytes,
        format: str = "h264",
        frame_id: str = "",
        ts: float | None = None,
    ) -> None:
        """Initialize a CompressedVideo.

        Args:
            data: The encoded packet, u8
            format: Codec name, e.g. "h264"
            frame_id: Frame ID
            ts: Timestamp in seconds
        """
        self.data = data if isinstance(data, np.ndarray) else np.frombuffer(data, dtype=np.uint8)
        self.format = format
        self.frame_id = frame_id
        self.ts = ts if ts is not None else time.time()

    def lcm_encode(self) -> bytes:
        msg = LCMCompressedVideo()
        msg.timestamp.sec = int(self.ts)
        msg.timestamp.nanosec = int((self.ts - int(self.ts)) * 1e9)
        msg.frame_id = self.frame_id
        msg.data = self.data.tobytes()
        msg.data_length = len(msg.data)
        msg.format = self.format
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> CompressedVideo:
        msg = LCMCompressedVideo.lcm_decode(data)
        return cls(
            data=bytes(msg.data),
            format=msg.format,
            frame_id=msg.frame_id,
            ts=msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
        )

    def to_rerun(self) -> RerunData:
        """Log the encoded packet as a rerun ``VideoStream`` sample (viewer decodes).

        rerun decodes the stream in-viewer, so this stays per-packet and cheap —
        no server-side decode, and the .rrd holds the compressed bytes. Iterate
        from the start (or a keyframe) so the first sample the viewer sees is one.
        """
        import rerun as rr

        codecs = {
            "h264": rr.VideoCodec.H264,
            "h265": rr.VideoCodec.H265,
            "av1": rr.VideoCodec.AV1,
            "vp8": rr.VideoCodec.VP8,
            "vp9": rr.VideoCodec.VP9,
        }
        codec = codecs.get(self.format.lower())
        if codec is None:
            raise ValueError(f"no rerun VideoCodec for format {self.format!r}")
        return rr.VideoStream(codec, sample=self.data.tobytes())

    def __repr__(self) -> str:
        return (
            f"CompressedVideo(format='{self.format}', bytes={self.data.size}, "
            f"frame_id='{self.frame_id}', ts={self.ts})"
        )


_PIXEL_FORMATS = {
    ImageFormat.BGR: "bgr24",
    ImageFormat.RGB: "rgb24",
    ImageFormat.RGBA: "rgba",
    ImageFormat.BGRA: "bgra",
    ImageFormat.GRAY: "gray",
    ImageFormat.GRAY16: "gray16",
}


class _FrameDecoder:
    """One subscription's decoder: packets in, the frames worth publishing out."""

    _keyframes_only: bool
    """Skip every packet that needs an earlier frame."""

    _min_interval: float
    """Seconds of packet time between two published frames; 0 publishes all."""

    _context: VideoCodecContext | None
    """Open for ``_format``; None before the first packet and after ``flush``."""

    _format: str
    """Codec name of the packets ``_context`` was opened for."""

    _last_frame_id: str
    """The last packet's frame of reference, also used when flushing."""

    _published_ts: float
    """Packet time of the last published frame."""

    def __init__(
        self, keyframes_only: bool, min_interval: float, image_format: ImageFormat
    ) -> None:
        self._keyframes_only = keyframes_only
        self._min_interval = min_interval
        self._image_format = image_format
        self._context = None
        self._format = ""
        self._last_frame_id = ""
        self._published_ts = float("-inf")

    def _open(self, format: str) -> VideoCodecContext:
        import av

        name = VIDEO_DECODERS.get(format.lower())
        if name is None:
            raise ValueError(f"no video decoder for format {format!r}")
        context = av.CodecContext.create(name, "r")
        assert isinstance(context, av.VideoCodecContext)
        if name == "libdav1d":
            # dav1d otherwise releases each frame four packets late, and with
            # skip_frame set never releases a keyframe until the stream ends.
            context.options = {"max_frame_delay": "1"}
        if self._keyframes_only:
            context.skip_frame = "NONKEY"
        return context

    def decode(self, packet: CompressedVideo) -> list[Image]:
        import av

        images = []
        if self._context is None or packet.format != self._format:
            images = self.flush()
            self._context = self._open(packet.format)
            self._format = packet.format
        self._last_frame_id = packet.frame_id
        # PTS carries the source packet's timestamp as integer nanoseconds.
        encoded = av.Packet(packet.data.tobytes())
        encoded.pts = round(packet.ts * 1e9)
        return images + self._publishable(self._context, encoded, packet.frame_id)

    def flush(self) -> list[Image]:
        """Close the decoder and return the frames it was still holding."""
        context = self._context
        self._context = None
        if context is None:
            return []
        return self._publishable(context, None, self._last_frame_id)

    def _publishable(
        self, context: VideoCodecContext, encoded: Packet | None, frame_id: str
    ) -> list[Image]:
        import av

        try:
            frames = context.decode(encoded)
        except av.error.FFmpegError:
            return []
        images = []
        for frame in frames:
            if self._keyframes_only and not frame.key_frame:
                continue
            if frame.pts is None:
                continue
            ts = frame.pts / 1e9
            # A timestamp that runs backwards (a replay looping) publishes at once.
            if 0 <= ts - self._published_ts < self._min_interval:
                continue
            self._published_ts = ts
            images.append(
                Image.from_numpy(
                    frame.to_ndarray(format=_PIXEL_FORMATS[self._image_format]),
                    self._image_format,
                    frame_id,
                    ts,
                )
            )
        return images


def decode_video(
    *,
    keyframes_only: bool = False,
    max_hz: float | None = None,
    image_format: ImageFormat = ImageFormat.BGR,
) -> Callable[[Observable[CompressedVideo]], Observable[Image]]:
    """Decode an ordered packet stream into ``Image`` frames.

    ``image_format`` selects BGR (default), RGB, RGBA, BGRA, GRAY, or GRAY16.
    Depth formats are not supported.

    Each subscription owns its decoder, opened for the codec the packets name
    and reopened when that name changes. Packets that yield no frame are
    dropped: the ones before the first keyframe, and with ``keyframes_only``
    every packet that is not a keyframe. Frames the decoder still holds when
    the codec changes or the stream completes are published first. ``max_hz``
    caps the frames published, by packet timestamp; every packet is still
    decoded.

    A missing packet corrupts every frame up to the next keyframe, so feed
    this from ``In.pure_observable()``, never the latest-wins ``observable()``,
    unless ``keyframes_only`` is set.
    """
    if max_hz is not None and max_hz <= 0:
        raise ValueError("max_hz must be positive")
    if image_format not in _PIXEL_FORMATS:
        raise ValueError(f"unsupported video image format: {image_format}")
    min_interval = 0.0 if max_hz is None else 1.0 / max_hz

    def _operator(source: Observable[CompressedVideo]) -> Observable[Image]:
        def _subscribe(
            observer: ObserverBase[Image], scheduler: SchedulerBase | None = None
        ) -> DisposableBase:
            decoder = _FrameDecoder(keyframes_only, min_interval, image_format)

            def on_next(packet: CompressedVideo) -> None:
                try:
                    images = decoder.decode(packet)
                except Exception as error:
                    observer.on_error(error)
                    return
                for image in images:
                    observer.on_next(image)

            def on_completed() -> None:
                for image in decoder.flush():
                    observer.on_next(image)
                observer.on_completed()

            return source.subscribe(on_next, observer.on_error, on_completed, scheduler=scheduler)

        return rx.create(_subscribe)

    return _operator
