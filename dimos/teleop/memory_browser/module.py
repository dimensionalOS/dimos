"""Memory Browser module — VR memory UI built on top of QuestTeleopModule.

Adds three things on top of the parent's WebSocket + controller plumbing:

1. A reverse-channel WebSocket at ``/ws_memory`` that pushes thumbnails and
   focus updates from the server to the headset.
2. A loader that samples ~N frames from a memory store on connect and ships
   them all to the client up front. After that, scrub is just an integer
   cursor — no per-frame DB hits.
3. A gesture handler that integrates ``palm_roll_delta`` events into a
   timeline cursor, and clears focus on ``swipe``.

The client (Three.js) renders the thumbnails as a curved arc and highlights
whichever index the server reports as active.
"""

from __future__ import annotations

import asyncio
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from dimos.core.core import rpc
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import throttle
from dimos.teleop.memory_browser.messages import (
    MSG_GLOBAL_MAP,
    MSG_THUMBNAIL,
    decode_text,
    encode_binary,
    encode_text,
)
from dimos.teleop.quest.quest_teleop_module import QuestTeleopConfig, QuestTeleopModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "web" / "static"


@dataclass(eq=False)
class _ClientConn:
    """One connected memory-browser client.

    Owns an asyncio queue drained by a sender coroutine on the event loop.
    Producers from worker threads push via :meth:`send_threadsafe` so the
    WebSocket itself is only ever touched from the event-loop thread.
    """

    ws: WebSocket
    loop: asyncio.AbstractEventLoop
    queue: asyncio.Queue[bytes | str] = field(default_factory=lambda: asyncio.Queue(maxsize=512))

    def send_threadsafe(self, msg: bytes | str) -> None:
        try:
            self.loop.call_soon_threadsafe(self._enqueue, msg)
        except RuntimeError:
            # Event loop is gone (client disconnected mid-publish). Drop silently.
            pass

    def _enqueue(self, msg: bytes | str) -> None:
        try:
            self.queue.put_nowait(msg)
        except asyncio.QueueFull:
            # Backpressure — drop oldest, keep this one.
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            self.queue.put_nowait(msg)


class MemoryBrowserConfig(QuestTeleopConfig):
    """Config for the Memory Browser.

    ``store_path`` points at a SQLite memory store — typically
    ``data/go2_bigoffice.db``. ``stream_name`` selects which stream's
    observations populate the timeline ribbon.
    """

    store_path: str = "data/go2_bigoffice.db"
    stream_name: str = "color_image"
    n_thumbnails: int = 20
    thumbnail_max_size: int = 256
    thumbnail_jpeg_quality: int = 70
    # Optional path to a pickled PointCloud2 to render as a top-down minimap.
    # Empty string disables the minimap.
    global_map_path: str = "data/unitree_go2_bigoffice_map.pickle"
    # Z slab to keep when projecting the cloud to 2D. Helps drop floor/ceiling.
    map_z_min: float = -0.2
    map_z_max: float = 2.2
    map_image_size: int = 384
    # Sensitivity: π (one half-rotation of the wrist) spans the full timeline.
    # Lower number = faster scrub.
    scrub_radians_per_full_span: float = 3.141592653  # π
    client_route: str = "/memory_browser"
    ws_route: str = "/ws_memory"
    # Bind on all interfaces by default — the headset connects over LAN/Wi-Fi.
    # Override to "127.0.0.1" if you want to restrict to local-only.
    listen_host: str = "0.0.0.0"


class MemoryBrowserModule(QuestTeleopModule):
    """VR memory-browser module.

    See :mod:`dimos.teleop.memory_browser` for the architectural overview.
    """

    config: MemoryBrowserConfig

    def __init__(self, **kwargs: Any) -> None:
        # Initialise our own state BEFORE super().__init__() because the parent's
        # constructor calls self._setup_routes() at the end, and our override
        # references these attributes.
        self._memory_clients: set[_ClientConn] = set()
        self._clients_lock = threading.RLock()

        # Cursor state, in "ribbon index" units (0..N-1). Float so a partial
        # roll between two indices can still pick the nearest int on emit.
        self._cursor_index: float = 0.0
        self._last_emitted_index: int = -1
        self._scrub_active: bool = False
        # When True, server is showing a highlighted frame. Cleared by a swipe;
        # re-set on the next engage. Prevents stale rolls from drifting focus.
        self._focus_active: bool = False
        self._cursor_lock = threading.RLock()

        # Lazily-opened store + cached thumbnails so reconnects are cheap.
        self._store: SqliteStore | None = None
        self._cached_thumbnails: list[bytes] | None = None
        # Per-index metadata mirrored to clients on connect: id, ts, pose,
        # tags, brightness, sharpness, dims. Computed once during the
        # thumbnail build (the image is already in memory there).
        self._thumbnail_meta: list[dict[str, Any]] = []
        # Cached global-map render: (jpeg_bytes, bounds_dict).
        self._cached_map: tuple[bytes, dict[str, float]] | None = None

        super().__init__(**kwargs)

        # Override the underlying web server's bind host. The parent's
        # RobotWebInterface picks up the global default (loopback), which is
        # wrong for a service the Quest reaches over Wi-Fi.
        self._web_server.host = self.config.listen_host

    # ---- routes ------------------------------------------------------------

    def _setup_routes(self) -> None:
        """Register memory-browser routes alongside the parent's teleop routes."""
        super()._setup_routes()

        app = self._web_server.app

        @app.get(self.config.client_route, response_class=HTMLResponse)
        async def memory_browser_index() -> HTMLResponse:
            index_path = STATIC_DIR / "index.html"
            return HTMLResponse(content=index_path.read_text())

        if STATIC_DIR.is_dir():
            app.mount(
                "/static_mb",
                StaticFiles(directory=str(STATIC_DIR)),
                name="memory_browser_static",
            )

        @app.websocket(self.config.ws_route)
        async def ws_memory(ws: WebSocket) -> None:
            await self._handle_ws(ws)

    # ---- websocket handling ------------------------------------------------

    async def _handle_ws(self, ws: WebSocket) -> None:
        await ws.accept()
        loop = asyncio.get_running_loop()
        conn = _ClientConn(ws=ws, loop=loop)
        with self._clients_lock:
            self._memory_clients.add(conn)
        logger.info("memory-browser client connected (now %d)", len(self._memory_clients))

        sender = asyncio.create_task(self._sender_loop(conn))
        # Push the timeline payload on a worker thread (cv2 + DB are sync).
        threading.Thread(
            target=self._send_initial_payload,
            args=(conn,),
            daemon=True,
            name="MemoryBrowserInitialLoad",
        ).start()

        try:
            while True:
                raw = await ws.receive_text()
                msg = decode_text(raw)
                if msg:
                    self._on_client_message(conn, msg)
        except WebSocketDisconnect:
            logger.info("memory-browser client disconnected")
        except Exception:
            logger.exception("memory-browser ws error")
        finally:
            sender.cancel()
            with self._clients_lock:
                self._memory_clients.discard(conn)

    async def _sender_loop(self, conn: _ClientConn) -> None:
        try:
            while True:
                msg = await conn.queue.get()
                if isinstance(msg, bytes):
                    await conn.ws.send_bytes(msg)
                else:
                    await conn.ws.send_text(msg)
        except asyncio.CancelledError:
            return
        except Exception:
            # WebSocket closed under us — that's fine, the read loop will clean up.
            return

    # ---- initial thumbnail load -------------------------------------------

    def _ensure_store(self) -> SqliteStore:
        if self._store is None:
            self._store = SqliteStore(path=self.config.store_path)
            logger.info("opened memory store at %s", self.config.store_path)
        return self._store

    def _send_initial_payload(self, conn: _ClientConn) -> None:
        """Compute (or reuse cached) thumbnails and push them to one client."""
        try:
            thumbnails = self._cached_thumbnails
            if thumbnails is None:
                thumbnails, self._thumbnail_meta = self._build_thumbnails()
                self._cached_thumbnails = thumbnails
                logger.info("built %d thumbnails for memory browser", len(thumbnails))
            # 1) timeline summary first so client knows how many slots to lay out
            if self._thumbnail_meta:
                t_start = self._thumbnail_meta[0]["ts"]
                t_end = self._thumbnail_meta[-1]["ts"]
            else:
                t_start = t_end = 0.0
            conn.send_threadsafe(
                encode_text(
                    "timeline_summary",
                    n=len(thumbnails),
                    t_start=t_start,
                    t_end=t_end,
                )
            )
            # 2) global map (top-down render) if available — the client will
            # show it as a minimap to the left of the preview pane.
            self._send_global_map(conn)
            # 3) per-frame metadata mirror so the client can populate the
            # right-hand dashboard without further server roundtrips.
            conn.send_threadsafe(encode_text("frame_meta_batch", entries=self._thumbnail_meta))
            # 4) each thumbnail as a binary frame with index in the header.
            for i, jpeg in enumerate(thumbnails):
                meta = self._thumbnail_meta[i] if i < len(self._thumbnail_meta) else {}
                conn.send_threadsafe(
                    encode_binary(
                        MSG_THUMBNAIL,
                        {
                            "index": i,
                            "id": meta.get("id", 0),
                            "ts": meta.get("ts", 0.0),
                            "stream": self.config.stream_name,
                        },
                        jpeg,
                    )
                )
            conn.send_threadsafe(encode_text("ready"))
        except Exception:
            logger.exception("failed to build/send thumbnails")
            conn.send_threadsafe(encode_text("error", message="thumbnail load failed"))

    def _send_global_map(self, conn: _ClientConn) -> None:
        if not self.config.global_map_path:
            return
        try:
            if self._cached_map is None:
                self._cached_map = self._build_global_map()
            if self._cached_map is None:
                return
            jpeg, bounds = self._cached_map
            conn.send_threadsafe(encode_binary(MSG_GLOBAL_MAP, bounds, jpeg))
        except Exception:
            logger.exception("failed to send global map")

    def _build_thumbnails(self) -> tuple[list[bytes], list[dict[str, Any]]]:
        """Sample N frames evenly and JPEG-encode each, collecting metadata.

        We use ``throttle(span / N)`` rather than naive index slicing so frames
        are time-spaced — the underlying observation store may be non-uniform.
        Per-observation metadata (pose, tags, brightness, sharpness, dims) is
        captured here while the image is already in memory, then mirrored to
        clients so the dashboard pane can render without further DB hits.
        """
        store = self._ensure_store()
        stream = store.streams[self.config.stream_name]

        first = stream.first()
        last = stream.last()
        t_start, t_end = float(first.ts), float(last.ts)
        span = max(t_end - t_start, 1e-3)

        n = max(2, int(self.config.n_thumbnails))
        interval = span / n
        max_size = int(self.config.thumbnail_max_size)
        quality = int(self.config.thumbnail_jpeg_quality)

        thumbnails: list[bytes] = []
        meta: list[dict[str, Any]] = []
        for obs in stream.transform(throttle(interval)):
            try:
                img = obs.data
                # Capture image properties from the full-res image before
                # resizing — brightness/sharpness aren't size-invariant.
                brightness = float(getattr(img, "brightness", 0.0))
                sharpness = float(getattr(img, "sharpness", 0.0))
                width = int(getattr(img, "width", 0))
                height = int(getattr(img, "height", 0))

                if hasattr(img, "resize_to_fit"):
                    img, _ = img.resize_to_fit(max_size, max_size)
                bgr = img.to_bgr().to_opencv() if hasattr(img, "to_bgr") else img
                ok, buf = cv2.imencode(
                    ".jpg",
                    bgr,
                    [int(cv2.IMWRITE_JPEG_QUALITY), quality],
                )
                if not ok:
                    continue

                pose = getattr(obs, "pose", None)
                pose_list = list(pose) if pose is not None else None
                thumbnails.append(buf.tobytes())
                meta.append(
                    {
                        "index": len(meta),
                        "id": int(getattr(obs, "id", 0)),
                        "ts": float(obs.ts),
                        "pose": pose_list,
                        "tags": dict(getattr(obs, "tags", {}) or {}),
                        "brightness": brightness,
                        "sharpness": sharpness,
                        "width": width,
                        "height": height,
                    }
                )
                if len(thumbnails) >= n:
                    break
            except Exception:
                logger.exception("skipping thumbnail at ts=%s", getattr(obs, "ts", "?"))
                continue

        return thumbnails, meta
    
    def _build_global_map(self) -> tuple[bytes, dict[str, float]] | None:
        """Render a top-down JPEG of the point cloud at ``global_map_path``.

        Returns ``(jpeg_bytes, bounds)`` where ``bounds`` carries
        ``{x_min, x_max, y_min, y_max, width_px, height_px}`` — enough for the
        client to project an observation's world-XY pose onto pixel coords.
        """
        import pickle

        path = Path(self.config.global_map_path)
        if not path.exists():
            logger.info("no global map at %s; skipping minimap", path)
            return None

        try:
            obj = pickle.loads(path.read_bytes())
        except Exception:
            logger.exception("failed to load global map pickle")
            return None

        # PointCloud2 → numpy (xyz, optional colors)
        as_np = getattr(obj, "as_numpy", None)
        if not callable(as_np):
            logger.warning("global map is not a PointCloud2 (no as_numpy()); skip")
            return None
        xyz, _colors = as_np()
        if xyz is None or xyz.size == 0:
            return None

        import numpy as np

        # Filter by Z slab (drop floor/ceiling).
        z = xyz[:, 2]
        m = (z >= self.config.map_z_min) & (z <= self.config.map_z_max)
        xy = xyz[m, :2]
        if xy.size == 0:
            xy = xyz[:, :2]

        x_min, x_max = float(xy[:, 0].min()), float(xy[:, 0].max())
        y_min, y_max = float(xy[:, 1].min()), float(xy[:, 1].max())
        # Square out the bounds so pixel aspect doesn't squash the world.
        cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2
        half = max(x_max - x_min, y_max - y_min) / 2 * 1.05  # 5% margin
        x_min, x_max, y_min, y_max = cx - half, cx + half, cy - half, cy + half

        size = int(self.config.map_image_size)
        # Bin into a 2D histogram. Saturate to make obstacles pop.
        hist, _, _ = np.histogram2d(
            xy[:, 0], xy[:, 1], bins=size, range=[[x_min, x_max], [y_min, y_max]]
        )
        norm = np.clip(hist / max(np.percentile(hist, 99), 1.0), 0.0, 1.0)
        # Y axis: histogram2d rows are increasing X, cols increasing Y. We want
        # an image where +Y is up. Transpose then flip rows.
        gray = (norm.T * 255).astype(np.uint8)
        gray = np.flipud(gray)
        # Light blue map on dark background to match the UI.
        rgb = np.zeros((size, size, 3), dtype=np.uint8)
        rgb[..., 0] = (gray.astype(np.uint16) * 90 // 255).astype(np.uint8)
        rgb[..., 1] = (gray.astype(np.uint16) * 180 // 255).astype(np.uint8)
        rgb[..., 2] = (gray.astype(np.uint16) * 255 // 255).astype(np.uint8)

        ok, buf = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        )
        if not ok:
            return None
        bounds = {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "width_px": size,
            "height_px": size,
        }
        logger.info("built global map: bounds=%s", bounds)
        return buf.tobytes(), bounds

    # ---- gesture handling --------------------------------------------------

    def _on_client_message(self, conn: _ClientConn, msg: dict[str, Any]) -> None:
        kind = msg.get("type")
        if kind == "engage":
            logger.info("[client] engage")
            self._on_engage()
        elif kind == "disengage":
            logger.info("[client] disengage")
            self._on_disengage()
        elif kind == "palm_roll_delta":
            try:
                delta = float(msg.get("value", 0.0))
            except (TypeError, ValueError):
                return
            logger.info("[client] palm_roll_delta=%.4f rad", delta)
            self._on_palm_roll_delta(delta)
        elif kind == "swipe":
            logger.info("[client] swipe hand=%s value=%s", msg.get("hand"), msg.get("value"))
            self._on_swipe()
        elif kind == "ping":
            conn.send_threadsafe(encode_text("pong"))
        elif kind == "diag":
            # Free-form diagnostic from the JS client. Useful for figuring out
            # whether the render loop, input adapter, or texture pipeline is
            # actually running on the headset side.
            logger.info(
                "[client/diag] %s %s",
                msg.get("event", "?"),
                {k: v for k, v in msg.items() if k not in ("type", "event")},
            )
        else:
            logger.warning("[client] unknown msg kind=%r full=%r", kind, msg)

    def _on_engage(self) -> None:
        with self._cursor_lock:
            self._scrub_active = True
            self._focus_active = True
            # Re-emit the current index so the client picks up the highlight even
            # if it had been dismissed.
            self._broadcast_active_index(force=True)

    def _on_disengage(self) -> None:
        with self._cursor_lock:
            self._scrub_active = False

    def _on_palm_roll_delta(self, delta_radians: float) -> None:
        with self._cursor_lock:
            if not self._scrub_active or not self._focus_active:
                return
            n = len(self._cached_thumbnails or ())
            if n < 2:
                return
            # Map a full radians-per-span to the full ribbon length.
            scale = (n - 1) / max(self.config.scrub_radians_per_full_span, 1e-3)
            self._cursor_index = max(
                0.0,
                min(float(n - 1), self._cursor_index + delta_radians * scale),
            )
            self._broadcast_active_index()

    def _on_swipe(self) -> None:
        with self._cursor_lock:
            if not self._focus_active:
                return
            self._focus_active = False
            self._broadcast_to_clients(encode_text("clear_focus"))

    # ---- broadcast helpers -------------------------------------------------

    def _broadcast_active_index(self, *, force: bool = False) -> None:
        idx = int(round(self._cursor_index))
        if not force and idx == self._last_emitted_index:
            return
        self._last_emitted_index = idx
        self._broadcast_to_clients(encode_text("active_index", index=idx))

    def _broadcast_to_clients(self, msg: bytes | str) -> None:
        with self._clients_lock:
            clients = list(self._memory_clients)
        for conn in clients:
            conn.send_threadsafe(msg)

    # ---- lifecycle ---------------------------------------------------------

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            store = self._store
            self._store = None
            if store is not None:
                try:
                    store.stop()
                except Exception:
                    logger.exception("error closing memory store")
