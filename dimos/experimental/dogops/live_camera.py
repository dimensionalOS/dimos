from __future__ import annotations

import base64
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any


DEFAULT_DIMOS_ROOT = os.environ.get("DIMOS_ROOT", "")
DEFAULT_COLOR_IMAGE_TOPIC = os.environ.get("DOGOPS_CAMERA_TOPIC", "/color_image")
LIVE_CAMERA_MAX_AGE_S = 5.0


class DogOpsLiveCameraAdapter:
    """Best-effort bridge from the DimOS color_image topic into dashboard JPEGs."""

    def __init__(self, *, topic: str = DEFAULT_COLOR_IMAGE_TOPIC) -> None:
        self.topic = topic
        self._lock = threading.RLock()
        self._started = False
        self._error = ""
        self._unsubscribe: Any | None = None
        self._transport: Any | None = None
        self._latest: tuple[float, Any] | None = None

    def status(self) -> dict[str, Any]:
        self.start()
        with self._lock:
            latest = self._latest
            error = self._error
        now = time.time()
        age_s = now - latest[0] if latest is not None else None
        fresh = age_s is not None and age_s <= LIVE_CAMERA_MAX_AGE_S
        frame = latest[1] if fresh else None
        return {
            "ok": fresh,
            "source": "DimOS color_image",
            "topic": self.topic,
            "status": "receiving" if fresh else "stale_frame" if latest is not None else "waiting_for_frame",
            "error": error,
            "received": fresh,
            "stale": latest is not None and not fresh,
            "age_s": round(age_s, 3) if age_s is not None else None,
            "width": int(getattr(frame, "width", 0) or 0) if frame is not None else None,
            "height": int(getattr(frame, "height", 0) or 0) if frame is not None else None,
            "format": _format_name(getattr(frame, "format", None)) if frame is not None else None,
            "frame_id": str(getattr(frame, "frame_id", "") or "") if frame is not None else "",
        }

    def frame_jpeg(self, *, quality: int = 75) -> bytes | None:
        self.start()
        with self._lock:
            latest = self._latest
        if latest is None or time.time() - latest[0] > LIVE_CAMERA_MAX_AGE_S:
            return None
        frame = latest[1]
        if hasattr(frame, "to_base64"):
            return base64.b64decode(frame.to_base64(quality=quality))
        raise TypeError(f"latest color_image frame is not a DimOS Image: {type(frame)!r}")

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._started = True
        try:
            JpegLcmTransport, Image = _import_dimos_camera_types()
            transport = JpegLcmTransport(self.topic, Image)
            unsubscribe = transport.subscribe(self._record)
        except Exception as exc:
            with self._lock:
                self._error = (
                    "DimOS camera imports or subscription unavailable in this Python environment. "
                    f"Run the dashboard from the full DimOS checkout/env or set DOGOPS_CAMERA_TOPIC. {exc}"
                )
            return

        with self._lock:
            self._transport = transport
            self._unsubscribe = unsubscribe

    def _record(self, msg: Any) -> None:
        with self._lock:
            self._latest = (time.time(), msg)
            self._error = ""

    def stop(self) -> None:
        with self._lock:
            unsubscribe = self._unsubscribe
            transport = self._transport
            self._unsubscribe = None
            self._transport = None
            self._latest = None
            self._started = False
        if unsubscribe is not None:
            try:
                unsubscribe()
            except Exception:
                pass
        if transport is not None:
            try:
                transport.stop()
            except Exception:
                pass


def _import_dimos_camera_types() -> tuple[Any, Any]:
    try:
        from dimos.core.transport import JpegLcmTransport
        from dimos.msgs.sensor_msgs.Image import Image
    except ModuleNotFoundError:
        _extend_dimos_package_path()
        from dimos.core.transport import JpegLcmTransport
        from dimos.msgs.sensor_msgs.Image import Image
    return JpegLcmTransport, Image


def _extend_dimos_package_path() -> None:
    if not DEFAULT_DIMOS_ROOT:
        return
    dimos_root = Path(DEFAULT_DIMOS_ROOT).expanduser()
    package_root = dimos_root / "dimos"
    if not package_root.exists():
        return
    if str(dimos_root) not in sys.path:
        sys.path.append(str(dimos_root))
    import dimos

    package_path = getattr(dimos, "__path__", None)
    if package_path is not None and str(package_root) not in package_path:
        package_path.append(str(package_root))


def _format_name(value: Any) -> str:
    return str(getattr(value, "value", value) or "")
