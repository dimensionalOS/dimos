"""DimOS glue: publish frames from an image or video file as ``color_image``.

A hardware-free camera source for testing the subject tracker end to end with
no robot and no webcam: point it at a photo or a short clip that contains the
subject, then describe it from the DimOS CLI. Loops the file so the stream is
continuous. Imports DimOS, so it is not unit tested.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import cv2

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Config(ModuleConfig):
    """Configuration for the file image source."""

    path: str = ""  # image or video file to publish
    fps: float = 15.0
    loop: bool = True  # restart the video (or keep republishing the image)


class FileImageSource(Module):
    """Publish frames from an image/video file on ``color_image``."""

    config: Config

    color_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._publish_frames, name="file-image-source", daemon=True
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None
        super().stop()

    def _publish(self, frame: Any) -> None:
        self.color_image.publish(Image.from_opencv(frame, ts=time.time()))

    def _wait(self) -> None:
        if self.config.fps > 0:
            self._stop.wait(1.0 / self.config.fps)

    def _publish_frames(self) -> None:
        still = cv2.imread(self.config.path)
        if still is not None:
            while not self._stop.is_set():
                self._publish(still)
                self._wait()
            return
        capture = cv2.VideoCapture(self.config.path)
        try:
            if not capture.isOpened():
                logger.error(
                    "FileImageSource: cannot open %r", self.config.path)
                return
            produced = False
            while not self._stop.is_set():
                ok, frame = capture.read()
                if not ok:
                    # Loop a real video back to the start; but if not a single
                    # frame ever decoded, the file is unusable -- stop instead of
                    # busy-spinning forever on the reset.
                    if self.config.loop and produced:
                        capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    break
                produced = True
                self._publish(frame)
                self._wait()
        finally:
            capture.release()
