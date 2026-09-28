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

"""RtspCamera: an RTSP H.265 camera as dimos streams.

Publishes the encoded stream untouched (every access unit: dropping one breaks decode
downstream), a capped decoded ``color_image`` for consumers on the same machine, and a
small JPEG for a slow operator link. Nothing is re-encoded; a lower bitrate has to come
from the camera's own encoder settings.

Every frame is stamped at the moment the packet was read, minus ``capture_latency_s``, on
the system clock, so a frame can be aligned with anything else stamped on that clock.

``url`` is an RTSP URL, a local file (a capture replayed through the same code), or
``"synthetic"``: a short clip generated at start (a white square parked at the image
centre), so a simulator has a camera without any file on disk.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, replace
from pathlib import Path
import tempfile
import threading
import time
from typing import Any, Literal

import av
from av.bitstream import BitStreamFilterContext
from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.rtsp.synthetic import write_synthetic_h265
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SYNTHETIC_URL = "synthetic"
_RECONNECT_WAIT_S = 2.0
_AV_TIMEOUT_S = 3.0  # av.open's open and read timeouts
_MAX_PENDING_STAMPS = 64


def _advance(last: float, now: float, period: float) -> float:
    """Reference time of a rate gate after a publish. Stepping by the period instead of to
    ``now`` keeps a frame that arrives a hair early from costing a whole source frame; the
    half-period floor stops a double publish after a gap."""
    return max(last + period, now - period / 2)


@dataclass
class _StreamStat:
    """Per-stream counters; see the ``sensor_stats`` rpc."""

    received: int = 0
    published: int = 0
    dropped: int = 0
    errors: int = 0
    bytes_in: int = 0
    last_mono: float = 0.0


class RtspCameraConfig(ModuleConfig):
    # RTSP URL of the camera, a local file path to replay a capture, or "synthetic" for a
    # generated clip (simulator, tests).
    url: str
    rtsp_transport: Literal["tcp", "udp"] = Field(default="tcp")
    # ffmpeg's RTSP reorder buffer. Larger only if the link to the camera drops packets.
    rtsp_latency_ms: int = Field(default=50)
    frame_id: str = Field(default="camera_optical")
    # Decoded frames are 2.8 MB each at 720p: cap them and keep them on this machine.
    color_hz: float = Field(default=10.0)
    # Startup rate of color_jpeg; set_jpeg_rate() changes it at runtime.
    jpeg_hz: float = Field(default=2.0)
    jpeg_quality: int = Field(default=50)
    jpeg_max_width: int = Field(default=640)
    # Seconds between the sensor exposure and our packet read (camera encode + RTSP
    # buffer); subtracted from the read time for the stamp.
    capture_latency_s: float = Field(default=0.0)
    # File replay: pace by the clip's frame rate and loop, so a 2 s capture stands in for a
    # live camera.
    replay_realtime: bool = Field(default=True)
    replay_loop: bool = Field(default=True)
    sensor_stats_interval_s: float = Field(default=10.0)


class RtspCamera(Module, perception.Image):
    """RTSP H.265 camera relay: encoded passthrough, capped decoded frames, link JPEGs."""

    # Decoding 720p H.265 in software is most of a core; keep it away from every other module.
    dedicated_worker = True

    config: RtspCameraConfig

    video: Out[CompressedVideo]
    color_image: Out[Image]
    color_jpeg: Out[CompressedImage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []
        self._stats: dict[str, _StreamStat] = {}
        self._stats_lock = threading.Lock()
        # Runtime switches, written by the rpcs and read by the relay thread.
        self._video_enabled = True
        self._jpeg_hz = self.config.jpeg_hz
        self._last_color_pub = 0.0
        self._last_jpeg_pub = 0.0
        # What av.open gets: the URL, the file, or the generated clip once start() wrote it.
        self._source = self.config.url
        self._tmp: tempfile.TemporaryDirectory[str] | None = None

    # Lifecycle

    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        if cfg.url == SYNTHETIC_URL:
            self._tmp = tempfile.TemporaryDirectory(prefix="rtsp-synthetic-")
            clip = Path(self._tmp.name) / "synthetic.mp4"
            write_synthetic_h265(clip, width=320, height=180, fps=25, seconds=2.0, centered=True)
            self._source = str(clip)
        self._stop_event.clear()
        self._threads = [threading.Thread(target=self._relay_loop, name="rtsp-relay", daemon=True)]
        if cfg.sensor_stats_interval_s > 0:
            self._threads.append(
                threading.Thread(target=self._stats_report_loop, name="rtsp-stats", daemon=True)
            )
        for t in self._threads:
            t.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for t in self._threads:
            # The relay thread can sit in av.open for two av timeouts in a row (the RTSP
            # handshake, then the stream probe); it checks the stop flag after that.
            t.join(timeout=2 * _AV_TIMEOUT_S + DEFAULT_THREAD_JOIN_TIMEOUT)
        self._threads.clear()
        if self._tmp is not None:
            self._tmp.cleanup()
            self._tmp = None
        super().stop()

    # Relay

    def _is_file(self) -> bool:
        return "://" not in self._source

    def _relay_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.relay_once()
            except (av.FFmpegError, OSError, IndexError, ValueError) as exc:
                self._count("video", errors=1)
                logger.warning("camera stream unavailable", url=self._source, error=str(exc))
                self._stop_event.wait(_RECONNECT_WAIT_S)
                continue
            if self._stop_event.is_set() or (self._is_file() and not self.config.replay_loop):
                return
            if not self._is_file():
                # The server closed a live stream: reopen as after an error, not in a busy loop.
                self._count("video", errors=1)
                logger.warning("camera stream ended", url=self._source)
                self._stop_event.wait(_RECONNECT_WAIT_S)

    def relay_once(self) -> int:
        """One pass over the stream (until it ends or stop is requested). Returns packets read."""
        cfg = self.config
        options: dict[str, str] = {}
        if not self._is_file():
            options = {
                "rtsp_transport": cfg.rtsp_transport,
                "fflags": "nobuffer",
                "max_delay": str(cfg.rtsp_latency_ms * 1000),
            }
        packets = 0
        # A frame comes out of the decoder from a later packet than its own (lookahead),
        # so stamps are kept by pts and handed to the frame they belong to.
        stamps: dict[int | None, float] = {}
        timeout = (_AV_TIMEOUT_S, _AV_TIMEOUT_S)
        with av.open(self._source, options=options, timeout=timeout) as container:
            if self._stop_event.is_set():  # before a first read blocks for another timeout
                return packets
            stream = container.streams.video[0]
            if stream.codec_context.name != "hevc":
                raise ValueError(f"expected H.265, got {stream.codec_context.name}")
            # RTSP delivers Annex-B, which is what a viewer decodes; an mp4 stores
            # length-prefixed units with the parameter sets out of band.
            annexb = BitStreamFilterContext("hevc_mp4toannexb", stream) if self._is_file() else None
            fps = float(stream.average_rate or 25)
            t_start = time.monotonic()
            for packet in container.demux(stream):
                if self._stop_event.is_set():
                    break
                if not packet.size or packet.is_corrupt:
                    self._count("video", dropped=1)
                    continue
                if self._is_file() and cfg.replay_realtime:
                    due = t_start + packets / fps
                    delay = due - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                ts = time.time() - cfg.capture_latency_s
                packets += 1
                self._count("video", received=1, bytes_in=packet.size)
                # Live: forward first, so the passthrough never waits on the decoder.
                if annexb is None:
                    self._publish_video((packet,), ts)
                stamps[packet.pts] = ts
                if len(stamps) > _MAX_PENDING_STAMPS:  # packets that never gave a frame
                    del stamps[next(iter(stamps))]
                frames = self._decode(stream, packet)
                # File: decode first, the filter takes ownership of the packet's buffer.
                if annexb is not None:
                    self._publish_video(annexb.filter(packet), ts)
                for frame in frames:
                    self._publish_decoded(frame, stamps.pop(frame.pts, ts))
            if not self._stop_event.is_set():
                # Flush the decoder's lookahead so a replayed clip yields every frame.
                flush_ts = time.time() - cfg.capture_latency_s
                for frame in self._decode(stream, None):
                    self._publish_decoded(frame, stamps.pop(frame.pts, flush_ts))
        return packets

    def _decode(self, stream: av.VideoStream, packet: av.Packet | None) -> list[av.VideoFrame]:
        try:
            return stream.decode(packet)
        except av.FFmpegError:  # a damaged access unit must not end the passthrough
            self._count("color_image", errors=1)
            return []

    def _publish_video(self, units: Iterable[av.Packet], ts: float) -> None:
        if not self._video_enabled:
            self._count("video", dropped=1)
            return
        for unit in units:
            self.video.publish(
                CompressedVideo(bytes(unit), format="h265", frame_id=self.config.frame_id, ts=ts)
            )
        self._count("video", published=1)

    def _publish_decoded(self, frame: av.VideoFrame, ts: float) -> None:
        cfg = self.config
        now = time.monotonic()
        self._count("color_image", received=1)
        jpeg_hz = self._jpeg_hz
        want_color = cfg.color_hz > 0 and now - self._last_color_pub >= 1.0 / cfg.color_hz
        want_jpeg = jpeg_hz > 0 and now - self._last_jpeg_pub >= 1.0 / jpeg_hz
        if not (want_color or want_jpeg):
            self._count("color_image", dropped=1)
            return
        image = Image(
            data=frame.to_ndarray(format="rgb24"),
            format=ImageFormat.RGB,
            frame_id=cfg.frame_id,
            ts=ts,
        )
        if want_color:
            self._last_color_pub = _advance(self._last_color_pub, now, 1.0 / cfg.color_hz)
            self.color_image.publish(image)
            self._count("color_image", published=1)
        if want_jpeg:
            self._last_jpeg_pub = _advance(self._last_jpeg_pub, now, 1.0 / jpeg_hz)
            try:
                self.color_jpeg.publish(
                    CompressedImage.from_image(
                        image, quality=cfg.jpeg_quality, max_width=cfg.jpeg_max_width
                    )
                )
                self._count("color_jpeg", published=1)
            except (ValueError, OSError):
                self._count("color_jpeg", errors=1)
                logger.exception("jpeg encode failed")
            except RuntimeError as exc:  # libturbojpeg is not installed
                self._jpeg_hz = 0.0
                self._count("color_jpeg", errors=1)
                logger.error("color_jpeg disabled", error=str(exc))

    # Diagnostics

    def _count(self, stream: str, **inc: int) -> None:
        with self._stats_lock:
            st = self._stats.setdefault(stream, _StreamStat())
            for k, v in inc.items():
                setattr(st, k, getattr(st, k) + v)
            st.last_mono = time.monotonic()

    def _stats_snapshot(self) -> dict[str, _StreamStat]:
        with self._stats_lock:
            return {k: replace(v) for k, v in self._stats.items()}

    def _stats_report_loop(self) -> None:
        interval = self.config.sensor_stats_interval_s
        prev = self._stats_snapshot()
        prev_t = time.monotonic()
        while not self._stop_event.wait(interval):
            cur = self._stats_snapshot()
            now = time.monotonic()
            dt = now - prev_t
            rates = {
                name: f"rx={(s.received - prev.get(name, _StreamStat()).received) / dt:.1f}/s "
                f"pub={(s.published - prev.get(name, _StreamStat()).published) / dt:.1f}/s"
                for name, s in sorted(cur.items())
            }
            logger.info("RtspCamera stream rates", window_s=round(dt), **rates)
            prev, prev_t = cur, now

    # RPCs

    @rpc
    def set_video_enabled(self, enabled: bool) -> bool:
        """Publish or withhold ``video``; the decoded outputs keep running. Returns the state."""
        self._video_enabled = bool(enabled)
        return self._video_enabled

    @rpc
    def set_jpeg_rate(self, hz: float) -> float:
        """``color_jpeg`` rate in Hz; 0 stops it. Returns the rate now in effect."""
        self._jpeg_hz = max(0.0, float(hz))
        return self._jpeg_hz

    @rpc
    def sensor_stats(self) -> dict[str, Any]:
        """Per-stream cumulative counters and last-message age."""
        now = time.monotonic()
        return {
            name: {
                "received": s.received,
                "published": s.published,
                "dropped": s.dropped,
                "errors": s.errors,
                "bytes_in": s.bytes_in,
                "age_s": (now - s.last_mono if s.last_mono else -1.0),
            }
            for name, s in self._stats_snapshot().items()
        }
