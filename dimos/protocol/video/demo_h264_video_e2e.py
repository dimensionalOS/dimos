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

"""Synthetic end-to-end H.264 image transport and memory2 storage demo."""

from __future__ import annotations

import os
from pathlib import Path
import sqlite3
import tempfile
import threading
import time

import cv2
import numpy as np

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import H264LcmTransport, pSHMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.memory2.module import OnExisting, Recorder
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.pubsub.impl.h264_lcm import H264LCM
from dimos.protocol.video.h264 import H264Config
from dimos.utils.logging_config import setup_logger
from dimos.visualization.vis_module import vis_module

logger = setup_logger()


class SyntheticVideoSourceConfig(ModuleConfig):
    width: int = 160
    height: int = 120
    fps: float = 10.0
    frame_count: int = 90
    output_frame_id: str = "h264_e2e_camera"
    seed: int = 7


class SyntheticVideoSource(Module):
    """Deterministic RGB image source for H.264 transport/storage QA."""

    config: SyntheticVideoSourceConfig
    color_image: Out[Image]

    _thread: threading.Thread | None = None
    _stop_event: threading.Event | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        logger.info(
            "Started synthetic H.264 video source: %sx%s @ %.2f FPS for %s frames",
            self.config.width,
            self.config.height,
            self.config.fps,
            self.config.frame_count,
        )

    @rpc
    def stop(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    def _publish_loop(self) -> None:
        assert self._stop_event is not None
        period = 1.0 / max(self.config.fps, 0.1)
        next_publish = time.monotonic()
        for seq in range(self.config.frame_count):
            if self._stop_event.is_set():
                break
            frame = self._make_frame(seq)
            self.color_image.publish(frame)
            next_publish += period
            time.sleep(max(0.0, next_publish - time.monotonic()))
        logger.info("Synthetic H.264 source finished publishing frames")

    def _make_frame(self, seq: int) -> Image:
        yy, xx = np.indices((self.config.height, self.config.width), dtype=np.uint16)
        base = (xx + (yy * 3) + (seq * 5) + self.config.seed) % 256
        data = np.stack(
            (base, (base + 85) % 256, (base + 170) % 256),
            axis=2,
        ).astype(np.uint8)
        return Image(
            data=data,
            format=ImageFormat.RGB,
            frame_id=self.config.output_frame_id,
            ts=time.time(),
        )


class H264E2ERecorder(Recorder):
    """Recorder with a typed image input for the synthetic H.264 demo."""

    color_image: In[Image]


class H264WebcamRecorder(Recorder):
    """Recorder with a typed image input for webcam H.264 QA."""

    color_image: In[Image]


class JpegBenchmarkRecorder(Recorder):
    """Recorder for the JPEG side of the storage-size benchmark."""

    jpeg_image: In[Image]


class H264BenchmarkRecorder(Recorder):
    """Recorder for the H.264 side of the storage-size benchmark."""

    h264_image: In[Image]


class H264StorageBenchmarkSourceConfig(SyntheticVideoSourceConfig):
    video_path: str = ""
    width: int = 320
    height: int = 240
    fps: float = 15.0
    frame_count: int = 150
    output_frame_id: str = "h264_storage_benchmark_camera"


class H264StorageBenchmarkSource(Module):
    """Publish identical raw frames to JPEG and H.264 recording paths."""

    config: H264StorageBenchmarkSourceConfig
    jpeg_image: Out[Image]
    h264_image: Out[Image]

    _thread: threading.Thread | None = None
    _stop_event: threading.Event | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        video_path = self._configured_video_path()
        source = str(video_path) if video_path is not None else "synthetic pattern"
        logger.info(
            "Started H.264/JPEG storage benchmark source: %s, %sx%s @ %.2f FPS for up to %s frames",
            source,
            self.config.width,
            self.config.height,
            self.config.fps,
            self.config.frame_count,
        )

    @rpc
    def stop(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    def _publish_loop(self) -> None:
        assert self._stop_event is not None
        video_path = self._configured_video_path()
        if video_path is not None:
            self._publish_video_file(video_path)
            return

        period = 1.0 / max(self.config.fps, 0.1)
        next_publish = time.monotonic()
        for seq in range(self.config.frame_count):
            if self._stop_event.is_set():
                break
            frame = self._make_frame(seq)
            self.jpeg_image.publish(frame)
            self.h264_image.publish(frame.copy())
            next_publish += period
            time.sleep(max(0.0, next_publish - time.monotonic()))
        logger.info("H.264/JPEG storage benchmark source finished publishing frames")

    def _configured_video_path(self) -> Path | None:
        value = self.config.video_path or os.environ.get("DIMOS_H264_BENCHMARK_VIDEO", "")
        return Path(value).expanduser() if value else None

    def _publish_video_file(self, video_path: Path) -> None:
        assert self._stop_event is not None
        if not video_path.exists():
            logger.error("Benchmark video file does not exist: %s", video_path)
            return

        capture = cv2.VideoCapture(str(video_path))
        try:
            if not capture.isOpened():
                logger.error("Failed to open benchmark video file: %s", video_path)
                return

            period = 1.0 / max(self.config.fps, 0.1)
            next_publish = time.monotonic()
            published = 0
            for seq in range(self.config.frame_count):
                if self._stop_event.is_set():
                    break
                ok, frame_bgr = capture.read()
                if not ok:
                    break
                frame = self._image_from_video_frame(frame_bgr)
                self.jpeg_image.publish(frame)
                self.h264_image.publish(frame.copy())
                published = seq + 1
                next_publish += period
                time.sleep(max(0.0, next_publish - time.monotonic()))
            logger.info(
                "H.264/JPEG storage benchmark video source published %s frames from %s",
                published,
                video_path,
            )
        finally:
            capture.release()

    def _image_from_video_frame(self, frame_bgr: np.ndarray) -> Image:
        if self.config.width > 0 and self.config.height > 0:
            frame_bgr = cv2.resize(
                frame_bgr,
                (self.config.width, self.config.height),
                interpolation=cv2.INTER_AREA,
            )
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        return Image(
            data=frame_rgb,
            format=ImageFormat.RGB,
            frame_id=self.config.output_frame_id,
            ts=time.time(),
        )

    def _make_frame(self, seq: int) -> Image:
        yy, xx = np.indices((self.config.height, self.config.width), dtype=np.uint16)
        base = (xx + (yy * 2) + (seq * 4) + self.config.seed) % 256
        marker = ((xx // 20 + yy // 20 + seq) % 2) * 35
        data = np.stack(
            (base, (base + 70 + marker) % 256, (base + 145) % 256),
            axis=2,
        ).astype(np.uint8)
        return Image(
            data=data,
            format=ImageFormat.RGB,
            frame_id=self.config.output_frame_id,
            ts=time.time(),
        )


class H264StorageBenchmarkReporterConfig(ModuleConfig):
    jpeg_db_path: str = "benchmark_jpeg.db"
    h264_db_path: str = "benchmark_h264.db"
    min_wait_seconds: float = 12.0
    wait_seconds: float = 18.0
    stable_seconds: float = 2.0
    poll_seconds: float = 0.5


class H264StorageBenchmarkReporter(Module):
    """Log the JPEG vs H.264 SQLite DB size comparison."""

    config: H264StorageBenchmarkReporterConfig

    _thread: threading.Thread | None = None
    _stop_event: threading.Event | None = None
    _last_summary: str | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._report_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    @rpc
    def summary(self) -> str:
        """Return the most recent JPEG-vs-H.264 storage benchmark summary."""
        return self._last_summary or "benchmark summary not available yet"

    def _report_loop(self) -> None:
        assert self._stop_event is not None
        started_at = time.monotonic()
        deadline = time.monotonic() + self.config.wait_seconds
        stable_since: float | None = None
        last_sizes: tuple[int, int] | None = None
        jpeg_path = Path(self.config.jpeg_db_path)
        h264_path = Path(self.config.h264_db_path)

        while time.monotonic() < deadline and not self._stop_event.is_set():
            if jpeg_path.exists() and h264_path.exists():
                sizes = (
                    _sqlite_snapshot_size(jpeg_path),
                    _sqlite_snapshot_size(h264_path),
                )
                if sizes == last_sizes:
                    stable_since = stable_since or time.monotonic()
                    recording_window_elapsed = (
                        time.monotonic() - started_at >= self.config.min_wait_seconds
                    )
                    if (
                        recording_window_elapsed
                        and time.monotonic() - stable_since >= self.config.stable_seconds
                    ):
                        self._log_sizes(sizes[0], sizes[1])
                        return
                else:
                    last_sizes = sizes
                    stable_since = None
            time.sleep(self.config.poll_seconds)

        if jpeg_path.exists() and h264_path.exists():
            self._log_sizes(
                _sqlite_snapshot_size(jpeg_path),
                _sqlite_snapshot_size(h264_path),
            )
        else:
            missing = [str(path) for path in (jpeg_path, h264_path) if not path.exists()]
            self._last_summary = f"benchmark DB size unavailable; missing={missing}"
            logger.warning(self._last_summary)

    def _log_sizes(self, jpeg_bytes: int, h264_bytes: int) -> None:
        ratio = h264_bytes / jpeg_bytes if jpeg_bytes else float("inf")
        saved = jpeg_bytes - h264_bytes
        saved_pct = (saved / jpeg_bytes * 100.0) if jpeg_bytes else 0.0
        self._last_summary = (
            "H.264/JPEG storage benchmark: "
            f"jpeg={jpeg_bytes} bytes ({jpeg_bytes / 1024 / 1024:.2f} MiB), "
            f"h264={h264_bytes} bytes ({h264_bytes / 1024 / 1024:.2f} MiB), "
            f"h264/jpeg={ratio:.3f}, saved={saved} bytes ({saved_pct:.1f}%)"
        )
        logger.info(self._last_summary)
        print(self._last_summary, flush=True)


def _sqlite_snapshot_size(path: Path) -> int:
    """Return compact SQLite DB size, even while WAL sidecars are active."""
    if not path.exists():
        return 0
    try:
        with tempfile.NamedTemporaryFile(prefix=f"{path.stem}-", suffix=".db") as tmp:
            source = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
            target = sqlite3.connect(tmp.name)
            try:
                source.backup(target)
            finally:
                target.close()
                source.close()
            return Path(tmp.name).stat().st_size
    except sqlite3.Error:
        return _sqlite_live_file_size(path)


def _sqlite_live_file_size(path: Path) -> int:
    total = path.stat().st_size if path.exists() else 0
    for suffix in ("-wal", "-shm"):
        sidecar = Path(f"{path}{suffix}")
        if sidecar.exists():
            total += sidecar.stat().st_size
    return total


class H264MemoryReplayConfig(ModuleConfig):
    db_path: str = "webcam_h264.db"
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    loop: bool = False


class H264MemoryReplay(Module):
    """Replay a memory2 H.264 image stream as decoded `Image` frames."""

    config: H264MemoryReplayConfig
    color_image: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        store = self.register_disposable(SqliteStore(path=self.config.db_path, must_exist=True))
        replay = store.replay(
            speed=self.config.speed,
            seek=self.config.seek,
            duration=self.config.duration,
            loop=self.config.loop,
        )

        def publish_decoded(image: Image) -> None:
            self.color_image.publish(image)

        def on_error(error: Exception) -> None:
            logger.error("H.264 replay pipeline error: %s", error, exc_info=True)

        self.register_disposable(
            replay.streams.color_image.observable().subscribe(
                on_next=publish_decoded,
                on_error=on_error,
            )
        )


class H264VideoProbe(Module):
    """Probe decoded H.264 `Image` delivery and report QA status."""

    color_image: In[Image]

    _lock: threading.Lock
    _received: int
    _last_ts: float | None
    _dimensions: tuple[int, int] | None
    _frame_id: str | None
    _failures: list[str]

    @rpc
    def start(self) -> None:
        super().start()
        self._lock = threading.Lock()
        self._received = 0
        self._last_ts = None
        self._dimensions = None
        self._frame_id = None
        self._failures = []
        self.color_image.subscribe(self._on_image)

    def _on_image(self, image: Image) -> None:
        with self._lock:
            if self._last_ts is not None and image.ts < self._last_ts:
                self._failures.append(f"timestamp regressed: {image.ts} < {self._last_ts}")
            dims = (image.width, image.height)
            if self._dimensions is None:
                self._dimensions = dims
            elif self._dimensions != dims:
                self._failures.append(f"dimension changed: {dims} != {self._dimensions}")
            if self._frame_id is None:
                self._frame_id = image.frame_id
            elif self._frame_id != image.frame_id:
                self._failures.append(f"frame_id changed: {image.frame_id} != {self._frame_id}")
            self._last_ts = image.ts
            self._received += 1

            if self._received % 10 == 0:
                logger.info(
                    "H.264 video probe received %s decoded frames",
                    self._received,
                )

    @rpc
    def summary(self) -> str:
        """Return decoded-frame QA status for the synthetic H.264 demo."""
        with self._lock:
            status = "ok" if not self._failures else "failed"
            return (
                f"status={status} received={self._received} "
                f"dimensions={self._dimensions} frame_id={self._frame_id!r} "
                f"last_ts={self._last_ts} failures={self._failures}"
            )


_h264_config = H264Config(bitrate=1_000_000, target_fps=10, keyframe_interval=15)
_webcam_h264_config = H264Config(bitrate=2_000_000, target_fps=15, keyframe_interval=30)
_benchmark_h264_config = H264Config(bitrate=1_500_000, target_fps=15, keyframe_interval=30)
_inter_machine_h264_topic = "/demo_h264_inter_machine/color_image"


def _webcam() -> Webcam:
    return Webcam(camera_index=0, width=640, height=480, fps=15.0)


demo_h264_video_e2e = autoconnect(
    SyntheticVideoSource.blueprint(),
    H264E2ERecorder.blueprint(
        db_path="h264_video_e2e.db",
        on_existing=OnExisting.OVERWRITE,
        stream_codecs={"color_image": "h264"},
    ),
    H264VideoProbe.blueprint(),
).transports(
    {
        ("color_image", Image): H264LcmTransport(
            "/demo_h264_video_e2e/color_image",
            Image,
            config=_h264_config,
        )
    }
)


demo_h264_storage_benchmark = autoconnect(
    H264StorageBenchmarkSource.blueprint(),
    JpegBenchmarkRecorder.blueprint(
        db_path="benchmark_jpeg.db",
        on_existing=OnExisting.OVERWRITE,
    ),
    H264BenchmarkRecorder.blueprint(
        db_path="benchmark_h264.db",
        on_existing=OnExisting.OVERWRITE,
        stream_codecs={"h264_image": "h264"},
    ),
    H264StorageBenchmarkReporter.blueprint(
        jpeg_db_path="benchmark_jpeg.db",
        h264_db_path="benchmark_h264.db",
    ),
).transports(
    {
        ("h264_image", Image): H264LcmTransport(
            "/demo_h264_storage_benchmark/h264_image",
            Image,
            config=_benchmark_h264_config,
        )
    }
)


demo_h264_webcam_record = autoconnect(
    CameraModule.blueprint(hardware=_webcam, transform=None, frequency=15.0),
    H264WebcamRecorder.blueprint(
        db_path="webcam_h264.db",
        on_existing=OnExisting.OVERWRITE,
        stream_codecs={"color_image": "h264"},
    ),
).transports(
    {
        ("color_image", Image): pSHMTransport(
            "/demo_h264_webcam_record/color_image",
            default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE,
        )
    }
)


demo_h264_webcam_rerun = autoconnect(
    CameraModule.blueprint(hardware=_webcam, transform=None, frequency=15.0),
    H264VideoProbe.blueprint(),
    vis_module(
        "rerun",
        rerun_config={"pubsubs": [H264LCM(config=_webcam_h264_config)]},
    ),
).transports(
    {
        ("color_image", Image): H264LcmTransport(
            "/demo_h264_webcam_rerun/color_image",
            Image,
            config=_webcam_h264_config,
        )
    }
)


demo_h264_webcam_publish = autoconnect(
    CameraModule.blueprint(hardware=_webcam, transform=None, frequency=15.0),
).transports(
    {
        ("color_image", Image): H264LcmTransport(
            _inter_machine_h264_topic,
            Image,
            config=_webcam_h264_config,
        )
    }
)


demo_h264_rerun_subscribe = autoconnect(
    H264VideoProbe.blueprint(),
    vis_module(
        "rerun",
        rerun_config={"pubsubs": [H264LCM(config=_webcam_h264_config)]},
    ),
).transports(
    {
        ("color_image", Image): H264LcmTransport(
            _inter_machine_h264_topic,
            Image,
            config=_webcam_h264_config,
        )
    }
)


demo_h264_webcam_replay = autoconnect(
    H264MemoryReplay.blueprint(db_path="webcam_h264.db"),
    H264VideoProbe.blueprint(),
    vis_module(
        "rerun",
        rerun_config={"pubsubs": [H264LCM(config=_webcam_h264_config)]},
    ),
).transports(
    {
        ("color_image", Image): H264LcmTransport(
            "/demo_h264_webcam_replay/color_image",
            Image,
            config=_webcam_h264_config,
        )
    }
)
