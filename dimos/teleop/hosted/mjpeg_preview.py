#!/usr/bin/env python3
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

"""Watch what the remote operator is being sent, from the robot's own network.

Taps the composited frame on its way to the WebRTC track and serves it as
MJPEG, so anyone on the LAN can open a browser and see exactly the picture the
headset is getting -- camera selection, mux layout and all -- without joining
the broker session and without a headset.

Diagnostic only: it encodes at a few frames a second and only while somebody is
watching, so it stays clear of the video encoder it is meant to observe.
"""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_BOUNDARY = "dimosframe"


class MjpegPreviewConfig(ModuleConfig):
    port: int = 8099
    # Deliberately well under the operator's frame rate: this is for looking at,
    # not for driving on, and every frame costs a JPEG encode.
    preview_fps: float = 5.0
    jpeg_quality: int = 60


class MjpegPreviewModule(Module):
    """Serve the latest frame on ``image_in`` as MJPEG over plain HTTP."""

    config: MjpegPreviewConfig

    image_in: In[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest: Image | None = None
        self._lock = threading.Lock()
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.image_in.subscribe(self._on_image)))
        self._serve()

    @rpc
    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None
        super().stop()

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            self._latest = msg

    def _snapshot(self) -> Image | None:
        with self._lock:
            return self._latest

    def _serve(self) -> None:
        module = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def log_message(self, *_args: Any) -> None:
                """Keep the module's own logging the only output."""

            def do_GET(self) -> None:
                if self.path not in ("/", "/preview"):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={_BOUNDARY}")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                period = 1.0 / max(module.config.preview_fps, 0.1)
                try:
                    while True:
                        image = module._snapshot()
                        if image is not None:
                            jpeg = image.to_jpeg_bytes(quality=module.config.jpeg_quality)
                            self.wfile.write(f"--{_BOUNDARY}\r\n".encode())
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                        time.sleep(period)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # viewer closed the tab

        self._server = ThreadingHTTPServer(("0.0.0.0", self.config.port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True, name="MjpegPreview"
        )
        self._thread.start()
        logger.info("operator view mirrored at http://<robot>:%d/preview", self.config.port)
