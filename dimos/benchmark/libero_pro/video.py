"""Side-by-side MP4 recording for normal LIBERO camera streams."""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any

import cv2
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image

VIDEO_FPS = 20.0
CAMERA_WIDTH = 128
CAMERA_HEIGHT = 128
MAX_PENDING_TIMESTAMPS = 8


class LiberoVideoRecorderConfig(ModuleConfig):
    output_path: Path
    fps: float = VIDEO_FPS


class LiberoVideoRecorder(Module):
    """Record synchronized public camera observations as one diagnostic MP4."""

    config: LiberoVideoRecorderConfig
    agentview_color_image: In[Image]
    eye_in_hand_color_image: In[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._video: SideBySideVideo | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._video = SideBySideVideo(self.config.output_path, fps=self.config.fps)
        self.register_disposable(
            Disposable(
                self.agentview_color_image.subscribe(lambda image: self._add("agentview", image))
            )
        )
        self.register_disposable(
            Disposable(
                self.eye_in_hand_color_image.subscribe(
                    lambda image: self._add("robot0_eye_in_hand", image)
                )
            )
        )

    @rpc
    def stop(self) -> None:
        video, self._video = self._video, None
        try:
            super().stop()
        finally:
            if video is not None:
                video.finish()

    def _add(self, camera: str, image: Image) -> None:
        if self._video is not None:
            self._video.add(camera, image)


class SideBySideVideo:
    """Pair same-timestamp camera frames and atomically publish an MP4."""

    def __init__(self, output_path: Path, *, fps: float = VIDEO_FPS) -> None:
        self.output_path = output_path
        self.partial_path = output_path.with_name(f"{output_path.stem}.partial{output_path.suffix}")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.unlink(missing_ok=True)
        self.partial_path.unlink(missing_ok=True)
        self._writer = _open_writer(self.partial_path, fps)
        if not self._writer.isOpened():
            self._writer.release()
            raise RuntimeError(f"Failed to open LIBERO video writer for {self.output_path}")
        self._lock = threading.Lock()
        self._pending: dict[float, dict[str, Image]] = {}
        self._last_timestamp = float("-inf")
        self._frames_written = 0
        self._finished = False

    @property
    def frames_written(self) -> int:
        return self._frames_written

    def add(self, camera: str, image: Image) -> None:
        if camera not in {"agentview", "robot0_eye_in_hand"}:
            raise ValueError(f"Unknown LIBERO camera: {camera}")
        with self._lock:
            if self._finished or image.ts <= self._last_timestamp:
                return
            pair = self._pending.setdefault(image.ts, {})
            pair[camera] = image
            if pair.keys() >= {"agentview", "robot0_eye_in_hand"}:
                frame = _compose(pair["agentview"], pair["robot0_eye_in_hand"])
                self._writer.write(frame)
                self._frames_written += 1
                self._last_timestamp = image.ts
                self._pending = {
                    timestamp: value
                    for timestamp, value in self._pending.items()
                    if timestamp > self._last_timestamp
                }
            while len(self._pending) > MAX_PENDING_TIMESTAMPS:
                self._pending.pop(min(self._pending))

    def finish(self) -> None:
        with self._lock:
            if self._finished:
                return
            self._finished = True
            self._writer.release()
            if self._frames_written == 0:
                self.partial_path.unlink(missing_ok=True)
                raise RuntimeError("LIBERO trial produced no synchronized video frames")
            self.partial_path.replace(self.output_path)


def _compose(agentview: Image, eye_in_hand: Image) -> np.ndarray[Any, np.dtype[np.uint8]]:
    frames = []
    for name, image in (("agentview", agentview), ("robot0_eye_in_hand", eye_in_hand)):
        if image.data.shape != (CAMERA_HEIGHT, CAMERA_WIDTH, 3):
            raise ValueError(
                f"{name} frame must be {CAMERA_WIDTH}x{CAMERA_HEIGHT} RGB, got {image.data.shape}"
            )
        frames.append(np.asarray(image.to_opencv(), dtype=np.uint8))
    return np.ascontiguousarray(np.concatenate(frames, axis=1))


def _open_writer(path: Path, fps: float) -> Any:
    return cv2.VideoWriter(
        str(path),
        cv2.VideoWriter.fourcc(*"mp4v"),
        fps,
        (CAMERA_WIDTH * 2, CAMERA_HEIGHT),
    )
