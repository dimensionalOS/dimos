# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Persistent V4L2 camera streams for YAM policy observations."""

from __future__ import annotations

import threading
import time
from typing import Any

import cv2
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class OpenPIYAMCameraConfig(ModuleConfig):
    top_device: str = "/dev/video10"
    left_wrist_device: str = "/dev/video16"
    right_wrist_device: str = "/dev/video4"
    width: int = Field(default=640, gt=0)
    height: int = Field(default=480, gt=0)
    fps: float = Field(default=15.0, gt=0)


class OpenPIYAMCameraBridge(Module):
    """Capture three persistent V4L2 devices into named RGB streams."""

    config: OpenPIYAMCameraConfig
    top_image: Out[Image]
    left_wrist_image: Out[Image]
    right_wrist_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._capture_loop, name="openpi-yam-cameras", daemon=True
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        super().stop()

    def _open_capture(self, device: str) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Cannot open YAM camera {device}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _capture_loop(self) -> None:
        devices = (
            self.config.top_device,
            self.config.left_wrist_device,
            self.config.right_wrist_device,
        )
        outputs = (self.top_image, self.left_wrist_image, self.right_wrist_image)
        while not self._stop.is_set():
            captures: list[cv2.VideoCapture] = []
            try:
                for device in devices:
                    captures.append(self._open_capture(device))
                while not self._stop.is_set():
                    started = time.monotonic()
                    images = []
                    # Grab from all devices before decoding to minimize camera skew.
                    for device, capture in zip(devices, captures, strict=True):
                        if not capture.grab():
                            raise RuntimeError(f"Cannot grab frame from {device}")
                    captured_at = time.time()
                    for device, capture in zip(devices, captures, strict=True):
                        ok, frame = capture.retrieve()
                        if not ok:
                            raise RuntimeError(f"Cannot decode frame from {device}")
                        images.append(
                            Image.from_numpy(
                                cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                                format=ImageFormat.RGB,
                                ts=captured_at,
                            )
                        )
                    if self._stop.is_set():
                        break
                    for output, image in zip(outputs, images, strict=True):
                        output.publish(image)
                    self._stop.wait(max(0.0, 1.0 / self.config.fps - (time.monotonic() - started)))
            except Exception:
                if not self._stop.is_set():
                    logger.exception("YAM camera capture failed; reconnecting")
            finally:
                for capture in captures:
                    capture.release()
            self._stop.wait(0.5)


openpi_yam_camera_bridge = OpenPIYAMCameraBridge.blueprint
