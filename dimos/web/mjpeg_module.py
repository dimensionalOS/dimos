"""Expose color_image as an MJPEG HTTP stream + single-frame snapshot.

Usage:
    dimos --simulation run unitree-go2-basic camera-mjpeg-module
    # MJPEG:    http://127.0.0.1:7780/video_feed/color_image
    # snapshot: http://127.0.0.1:7780/snapshot/color_image
"""

import threading
from typing import Any

import cv2
from fastapi import HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response
import numpy as np
import reactivex as rx
from reactivex import operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()


class CameraMjpegConfig(ModuleConfig):
    port: int = 7780
    stream_key: str = "color_image"


class CameraMjpegModule(Module):
    config: CameraMjpegConfig
    color_image: In[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._subject: rx.subject.Subject = rx.subject.Subject()
        self._web: RobotWebInterface | None = None
        self._thread: threading.Thread | None = None
        self._latest_jpeg: bytes | None = None
        self._latest_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self._web = RobotWebInterface(
            port=self.config.port,
            **{self.config.stream_key: self._subject.pipe(ops.share())},
        )
        self._web.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["GET"],
            allow_headers=["*"],
        )

        stream_key = self.config.stream_key

        @self._web.app.get(f"/snapshot/{stream_key}")
        def snapshot() -> Response:
            with self._latest_lock:
                buf = self._latest_jpeg
            if buf is None:
                raise HTTPException(status_code=503, detail="no frame yet")
            return Response(content=buf, media_type="image/jpeg")

        self._thread = threading.Thread(target=self._web.run, daemon=True)
        self._thread.start()
        self.color_image.subscribe(self._on_image)
        logger.info(
            f"MJPEG: http://127.0.0.1:{self.config.port}/video_feed/{stream_key}  "
            f"| snapshot: http://127.0.0.1:{self.config.port}/snapshot/{stream_key}"
        )

    def _on_image(self, img: Image) -> None:
        arr = img.as_numpy()
        if arr.ndim == 3 and arr.shape[2] == 3:
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        self._subject.on_next(arr)

        ok, jpg = cv2.imencode(".jpg", arr)
        if ok:
            with self._latest_lock:
                self._latest_jpeg = np.asarray(jpg).tobytes()

    @rpc
    def stop(self) -> None:
        if self._web is not None:
            try:
                self._web.shutdown()
            except Exception:
                pass
        super().stop()
