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

import threading
import time

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import H264LcmTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.memory2.module import OnExisting, Recorder
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.video.h264 import H264ImageStorageConfig
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


class H264MemoryReplayConfig(ModuleConfig):
    db_path: str = "webcam_h264.db"
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    loop: bool = False


class H264MemoryReplay(Module):
    """Replay a memory2 H.264 image stream as normal `Image` frames."""

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

        def on_error(error: Exception) -> None:
            logger.error("H.264 replay pipeline error: %s", error, exc_info=True)

        self.register_disposable(
            replay.streams.color_image.observable().subscribe(
                on_next=self.color_image.publish,
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
                logger.info("H.264 video probe received %s decoded frames", self._received)

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


def _webcam() -> Webcam:
    return Webcam(camera_index=0, width=640, height=480, fps=15.0)


demo_h264_video_e2e = autoconnect(
    SyntheticVideoSource.blueprint(),
    H264E2ERecorder.blueprint(
        db_path="h264_video_e2e.db",
        on_existing=OnExisting.OVERWRITE,
        image_storage={
            "color_image": H264ImageStorageConfig(codec=_h264_config),
        },
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


demo_h264_webcam_record = autoconnect(
    CameraModule.blueprint(hardware=_webcam, transform=None, frequency=15.0),
    H264WebcamRecorder.blueprint(
        db_path="webcam_h264.db",
        on_existing=OnExisting.OVERWRITE,
        image_storage={
            "color_image": H264ImageStorageConfig(codec=_webcam_h264_config),
        },
    ),
).transports(
    {
        ("color_image", Image): H264LcmTransport(
            "/demo_h264_webcam_record/color_image",
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
