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

"""Memory World module — spawns the user inside a recorded point cloud.

On WebSocket connect we:

1. Load the pickled :class:`PointCloud2` global map, voxel-downsample it to
   stay under ``max_points``, and push it as one binary frame (positions +
   per-point RGB).
2. Sample the ``color_image`` stream and push each capture pose as a
   Street-View-style marker. The headset can later pinch one to surface the
   image at that location.
3. Push the odom trail as a polyline.

All locomotion (smooth walk, snap turn, teleport, scale) is client-side —
the server is a one-shot data push plus diagnostics.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from pathlib import Path
import threading
from typing import Any

import cv2
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import numpy as np

from dimos.core.core import rpc
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import throttle
from dimos.teleop.memory_world.messages import (
    MSG_IMAGE_POSES,
    MSG_IMAGE_THUMBNAIL,
    MSG_ODOM_TRAIL,
    MSG_POINT_CLOUD,
    MSG_TOP_DOWN_MAP,
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
    """One connected memory-world client. Mirrors memory_browser._ClientConn."""

    ws: WebSocket
    loop: asyncio.AbstractEventLoop
    queue: asyncio.Queue[bytes | str] = field(default_factory=lambda: asyncio.Queue(maxsize=512))

    def send_threadsafe(self, msg: bytes | str) -> None:
        try:
            self.loop.call_soon_threadsafe(self._enqueue, msg)
        except RuntimeError:
            pass

    def _enqueue(self, msg: bytes | str) -> None:
        try:
            self.queue.put_nowait(msg)
        except asyncio.QueueFull:
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            self.queue.put_nowait(msg)


class MemoryWorldConfig(QuestTeleopConfig):
    """Config for the Memory World."""

    store_path: str = "data/go2_bigoffice.db"
    # Where the world point cloud comes from:
    #   "pickle" — load the prebuilt PointCloud2 at global_map_path (default,
    #              has true RGB if the map was captured with colour).
    #   "lidar"  — accumulate a voxel map live from the lidar stream via
    #              VoxelMapTransformer. No static pickle dependency, rebuilds
    #              from whatever is currently in the store. Coloured by height.
    cloud_source: str = "pickle"
    # Pickled PointCloud2 — typically the same map used by memory_browser.
    global_map_path: str = "data/unitree_go2_bigoffice_map.pickle"
    # Voxel size for downsampling before shipping to the headset (metres).
    # 0.05m on a typical office map gives ~150k points; raise if your map is
    # bigger, lower for finer detail at the cost of bandwidth. This same value
    # is sent to the client so rendered point size matches voxel spacing.
    voxel_size: float = 0.05
    # How to colour the cloud:
    #   "height" — turbo rainbow ramp by Z (violet floor → red ceiling).
    #   "rgb"    — true captured colour if the source has it (pickle only);
    #              falls back to height when the source is colourless (lidar).
    color_mode: str = "height"
    # How the client draws the cloud:
    #   "cubes"  — solid instanced voxel cubes (crisp, connected look).
    #   "points" — flat camera-facing sprites (cheaper, grainier).
    # The headset can flip this live with the right B button regardless.
    voxel_render: str = "cubes"
    # Hard cap so an unexpectedly dense map doesn't try to ship 5M points.
    max_points: int = 250_000
    # cloud_source="lidar": which stream to accumulate + how many scans to
    # sample. <= 0 means use EVERY lidar frame (densest map, slowest build).
    # The output cloud is deduped by voxel_size, so more scans improves the
    # map without growing the wire payload — only build time goes up.
    lidar_stream_name: str = "lidar"
    n_voxel_scans: int = 150
    # Set True if the stored lidar scans are ALREADY in the map/world frame
    # (e.g. SLAM-registered). Then we must NOT re-apply each scan's pose —
    # doing so double-transforms them into scattered noise. Leave False if
    # scans are in the sensor frame and need their pose applied.
    lidar_world_frame: bool = False
    # Z slab applied at load time to drop the floor/ceiling from the cloud.
    # The user stands on the floor in VR; rendering it as points is just noise.
    map_z_min: float = -0.2
    map_z_max: float = 2.4
    # color_image stream is sampled for "Street View" capture-pose markers.
    image_stream_name: str = "color_image"
    n_image_markers: int = 200
    # Thumbnail params for the per-pose images that get textured onto quads in
    # 3D world space. Smaller = less bandwidth, lower res in headset.
    thumbnail_max_size: int = 192
    thumbnail_jpeg_quality: int = 70
    # odom stream is used to draw the robot's path as a polyline.
    odom_stream_name: str = "odom"
    n_odom_samples: int = 400
    # Top-down density map (GTA-style minimap + ground projection). Computed
    # from the same point cloud — Z-slab histogram into a square image.
    map_image_size: int = 512
    map_z_min_floor: float = 0.05    # avoid floor speckle
    map_z_max_floor: float = 1.8
    client_route: str = "/memory_world"
    ws_route: str = "/ws_memory_world"
    # Bind on all interfaces by default — the headset connects over Wi-Fi.
    listen_host: str = "0.0.0.0"


class MemoryWorldModule(QuestTeleopModule):
    """VR memory-world module.

    See :mod:`dimos.teleop.memory_world` for the architectural overview.
    """

    config: MemoryWorldConfig

    def __init__(self, **kwargs: Any) -> None:
        self._world_clients: set[_ClientConn] = set()
        self._clients_lock = threading.RLock()

        self._store: SqliteStore | None = None
        # Cached payloads so reconnects are cheap.
        self._cached_cloud: tuple[dict[str, Any], bytes] | None = None
        self._cached_image_poses: tuple[dict[str, Any], bytes] | None = None
        # Per-pose JPEG thumbnails parallel to image_poses indices.
        self._cached_thumbnails: list[bytes] | None = None
        self._cached_odom: tuple[dict[str, Any], bytes] | None = None
        self._cached_top_down: tuple[dict[str, Any], bytes] | None = None

        super().__init__(**kwargs)

        self._web_server.host = self.config.listen_host

    # ---- routes ------------------------------------------------------------

    def _setup_routes(self) -> None:
        super()._setup_routes()

        app = self._web_server.app

        @app.get(self.config.client_route, response_class=HTMLResponse)
        async def memory_world_index() -> HTMLResponse:
            index_path = STATIC_DIR / "index.html"
            return HTMLResponse(content=index_path.read_text())

        if STATIC_DIR.is_dir():
            app.mount(
                "/static_mw",
                StaticFiles(directory=str(STATIC_DIR)),
                name="memory_world_static",
            )

        @app.websocket(self.config.ws_route)
        async def ws_world(ws: WebSocket) -> None:
            await self._handle_ws(ws)

    # ---- websocket handling ------------------------------------------------

    async def _handle_ws(self, ws: WebSocket) -> None:
        await ws.accept()
        loop = asyncio.get_running_loop()
        conn = _ClientConn(ws=ws, loop=loop)
        with self._clients_lock:
            self._world_clients.add(conn)
        logger.info("memory-world client connected (now %d)", len(self._world_clients))

        sender = asyncio.create_task(self._sender_loop(conn))
        threading.Thread(
            target=self._send_initial_payload,
            args=(conn,),
            daemon=True,
            name="MemoryWorldInitialLoad",
        ).start()

        try:
            while True:
                raw = await ws.receive_text()
                msg = decode_text(raw)
                if msg:
                    self._on_client_message(conn, msg)
        except WebSocketDisconnect:
            logger.info("memory-world client disconnected")
        except Exception:
            logger.exception("memory-world ws error")
        finally:
            sender.cancel()
            with self._clients_lock:
                self._world_clients.discard(conn)

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
            return

    # ---- initial payload ---------------------------------------------------

    def _ensure_store(self) -> SqliteStore:
        if self._store is None:
            self._store = SqliteStore(path=self.config.store_path)
            logger.info("opened memory store at %s", self.config.store_path)
        return self._store

    def _send_initial_payload(self, conn: _ClientConn) -> None:
        try:
            if self._cached_cloud is None:
                self._cached_cloud = self._build_cloud()
            if self._cached_image_poses is None:
                self._cached_image_poses, self._cached_thumbnails = self._build_image_poses()
            if self._cached_odom is None:
                self._cached_odom = self._build_odom_trail()
            if self._cached_top_down is None:
                self._cached_top_down = self._build_top_down_map()

            cloud_header, cloud_payload = self._cached_cloud
            conn.send_threadsafe(encode_text("world_summary", **cloud_header))
            conn.send_threadsafe(encode_binary(MSG_POINT_CLOUD, cloud_header, cloud_payload))

            # Send top-down map next — both the ground plane and the HUD
            # minimap need it, so render asap on the client.
            if self._cached_top_down is not None:
                map_header, map_payload = self._cached_top_down
                conn.send_threadsafe(encode_binary(MSG_TOP_DOWN_MAP, map_header, map_payload))

            poses_header, poses_payload = self._cached_image_poses
            conn.send_threadsafe(encode_binary(MSG_IMAGE_POSES, poses_header, poses_payload))

            # One MSG_IMAGE_THUMBNAIL frame per pose. Indices match poses_header.
            if self._cached_thumbnails:
                for i, jpeg in enumerate(self._cached_thumbnails):
                    if not jpeg:
                        continue
                    conn.send_threadsafe(
                        encode_binary(MSG_IMAGE_THUMBNAIL, {"index": i}, jpeg)
                    )

            odom_header, odom_payload = self._cached_odom
            conn.send_threadsafe(encode_binary(MSG_ODOM_TRAIL, odom_header, odom_payload))

            conn.send_threadsafe(encode_text("ready"))
        except Exception:
            logger.exception("failed to build/send world payload")
            conn.send_threadsafe(encode_text("error", message="world load failed"))

    def _build_cloud(self) -> tuple[dict[str, Any], bytes]:
        """Dispatch on cloud_source. Falls back to pickle if lidar build fails."""
        if self.config.cloud_source == "lidar":
            built = self._build_voxel_cloud_from_lidar()
            if built is not None:
                return built
            logger.warning("voxel-from-lidar produced no cloud; falling back to pickle")
        return self._build_point_cloud()

    def _build_voxel_cloud_from_lidar(self) -> tuple[dict[str, Any], bytes] | None:
        """Accumulate a voxel map from the lidar stream and pack it for the wire.

        Each lidar scan is transformed into the world frame via its ``pose``,
        then fed to :class:`VoxelMapTransformer`. The final accumulated cloud
        is height-coloured (cyan low → amber high) so the user gets depth cues
        without true RGB. Same payload layout as :meth:`_build_point_cloud`.
        """
        from dimos.mapping.voxels import VoxelMapTransformer
        from dimos.memory2.transform import FnTransformer
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        try:
            store = self._ensure_store()
            stream = store.streams[self.config.lidar_stream_name]
            first, last = stream.first(), stream.last()
            span = max(float(last.ts) - float(first.ts), 1e-3)
            n_scans = int(self.config.n_voxel_scans)
            use_all = n_scans <= 0

            def to_world_frame(obs: Any) -> Any:
                # If scans are already registered to the map frame, applying
                # the pose again double-transforms them into scattered noise.
                if self.config.lidar_world_frame:
                    return obs
                pose = getattr(obs, "pose", None)
                if pose is None:
                    return None
                p = list(pose)
                if len(p) < 7:
                    return None
                tf = Transform(
                    translation=Vector3(float(p[0]), float(p[1]), float(p[2])),
                    rotation=Quaternion(float(p[3]), float(p[4]), float(p[5]), float(p[6])),
                )
                return obs.derive(data=obs.data.transform(tf))

            # emit_every=0 → only yield the final accumulated map on exhaustion.
            # Throttle to n_scans unless use_all (then feed every frame).
            pipeline = stream if use_all else stream.transform(throttle(span / n_scans))
            result = (
                pipeline.transform(FnTransformer(to_world_frame))
                .transform(VoxelMapTransformer(emit_every=0, voxel_size=self.config.voxel_size))
                .last()
            )
            if result is None or result.data is None:
                return None
            xyz, _ = result.data.as_numpy()
            if xyz is None or xyz.size == 0:
                return None

            z = xyz[:, 2]
            m = (z >= self.config.map_z_min) & (z <= self.config.map_z_max)
            xyz = xyz[m]
            if xyz.size == 0:
                return None
            if xyz.shape[0] > self.config.max_points:
                stride = xyz.shape[0] // self.config.max_points + 1
                xyz = xyz[::stride]

            positions = np.ascontiguousarray(xyz.astype(np.float32))
            # Lidar has no RGB, so always height-colour.
            rgb = self._height_colors(positions)
            header = self._cloud_header(positions)
            payload = positions.tobytes() + rgb.tobytes()
            logger.info("built voxel cloud (%s scans): n=%d",
                        "all" if use_all else str(n_scans), positions.shape[0])
            return header, payload
        except Exception:
            logger.exception("voxel-from-lidar build failed")
            return None

    def _height_colors(self, positions: "np.ndarray") -> "np.ndarray":
        """Map Z (robot up) to a full turbo rainbow (deep blue floor → red ceiling).

        Uses the height SLAB bounds (map_z_min..map_z_max) rather than the
        cloud's own min/max so the colour of a given height is stable and the
        whole ramp is used even if the cloud doesn't span the full slab.
        Returns N×3 uint8, C-contiguous (RGB order).
        """
        zc = positions[:, 2]
        lo = float(self.config.map_z_min)
        hi = float(self.config.map_z_max)
        if hi - lo < 1e-3:
            lo, hi = float(zc.min()), float(zc.max()) + 1e-3
        t = np.clip((zc - lo) / (hi - lo), 0.0, 1.0)
        t8 = (t * 255).astype(np.uint8).reshape(-1, 1)
        # cv2 returns BGR; flip to RGB.
        bgr = cv2.applyColorMap(t8, cv2.COLORMAP_TURBO).reshape(-1, 3)
        rgb = np.ascontiguousarray(bgr[:, ::-1])
        return rgb

    def _cloud_header(self, positions: "np.ndarray") -> dict[str, Any]:
        """Common header: count, colour flag, voxel size, render mode, bounds."""
        return {
            "n": int(positions.shape[0]),
            "has_colors": True,
            "voxel_size": float(self.config.voxel_size),
            "render": self.config.voxel_render,
            "bounds": {
                "x_min": float(positions[:, 0].min()), "x_max": float(positions[:, 0].max()),
                "y_min": float(positions[:, 1].min()), "y_max": float(positions[:, 1].max()),
                "z_min": float(positions[:, 2].min()), "z_max": float(positions[:, 2].max()),
            },
        }

    def _build_point_cloud(self) -> tuple[dict[str, Any], bytes]:
        """Load the pickled PointCloud2, downsample, return (header, payload).

        Payload layout: ``N*12 bytes float32 positions || N*3 bytes uint8 RGB``.
        Header carries N and whether colours are present.
        """
        import pickle

        path = Path(self.config.global_map_path)
        if not path.exists():
            logger.warning("no global map at %s; sending empty cloud", path)
            return {"n": 0, "has_colors": False}, b""

        obj = pickle.loads(path.read_bytes())
        as_np = getattr(obj, "as_numpy", None)
        if not callable(as_np):
            logger.warning("global map is not a PointCloud2; sending empty cloud")
            return {"n": 0, "has_colors": False}, b""

        # Voxel-downsample before pulling to numpy so we don't materialise a
        # huge array just to throw most of it away.
        try:
            obj = obj.voxel_downsample(self.config.voxel_size)
        except Exception:
            logger.exception("voxel downsample failed; using raw cloud")

        xyz, colors = obj.as_numpy()
        if xyz is None or xyz.size == 0:
            return {"n": 0, "has_colors": False}, b""

        # Z-slab filter to drop floor/ceiling.
        z = xyz[:, 2]
        m = (z >= self.config.map_z_min) & (z <= self.config.map_z_max)
        xyz = xyz[m]
        if colors is not None:
            colors = colors[m]

        # Hard cap so we never blow the WS buffer.
        if xyz.shape[0] > self.config.max_points:
            stride = xyz.shape[0] // self.config.max_points + 1
            xyz = xyz[::stride]
            if colors is not None:
                colors = colors[::stride]

        positions = np.ascontiguousarray(xyz.astype(np.float32))
        # color_mode="rgb" uses the captured colour when present; otherwise
        # (and for "height") we ramp by Z.
        if self.config.color_mode == "rgb" and colors is not None:
            rgb = np.ascontiguousarray(np.clip(colors * 255.0, 0, 255).astype(np.uint8))
        else:
            rgb = self._height_colors(positions)

        header = self._cloud_header(positions)
        payload = positions.tobytes() + rgb.tobytes()
        logger.info("built point cloud: n=%d color_mode=%s bytes=%d",
                    positions.shape[0], self.config.color_mode, len(payload))
        return header, payload

    def _build_image_poses(self) -> tuple[tuple[dict[str, Any], bytes], list[bytes]]:
        """Sample N capture poses and JPEG thumbnails from the color_image stream.

        Returns ((header, packed_pose_payload), list_of_jpegs).
        Pose payload: ``N*12 bytes float32`` xyz, then ``N*16 bytes float32`` quat.
        Thumbnails are sent as separate MSG_IMAGE_THUMBNAIL frames so each
        decode happens lazily on the client.
        """
        try:
            store = self._ensure_store()
            stream = store.streams[self.config.image_stream_name]
            first, last = stream.first(), stream.last()
            span = max(float(last.ts) - float(first.ts), 1e-3)
            n = max(2, int(self.config.n_image_markers))
            interval = span / n
            max_size = int(self.config.thumbnail_max_size)
            quality = int(self.config.thumbnail_jpeg_quality)

            positions: list[tuple[float, float, float]] = []
            quats: list[tuple[float, float, float, float]] = []
            timestamps: list[float] = []
            ids: list[int] = []
            thumbnails: list[bytes] = []
            for obs in stream.transform(throttle(interval)):
                pose = getattr(obs, "pose", None)
                if pose is None:
                    continue
                p = list(pose)
                if len(p) < 3:
                    continue
                positions.append((float(p[0]), float(p[1]), float(p[2])))
                if len(p) >= 7:
                    quats.append((float(p[3]), float(p[4]), float(p[5]), float(p[6])))
                else:
                    quats.append((0.0, 0.0, 0.0, 1.0))
                timestamps.append(float(obs.ts))
                ids.append(int(getattr(obs, "id", 0)))

                # JPEG-encode the matching color image.
                try:
                    img = obs.data
                    if hasattr(img, "resize_to_fit"):
                        img, _ = img.resize_to_fit(max_size, max_size)
                    bgr = img.to_bgr().to_opencv() if hasattr(img, "to_bgr") else img
                    ok, buf = cv2.imencode(
                        ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
                    )
                    thumbnails.append(buf.tobytes() if ok else b"")
                except Exception:
                    logger.exception("thumbnail encode failed at ts=%s", obs.ts)
                    thumbnails.append(b"")

                if len(positions) >= n:
                    break

            pos_arr = np.asarray(positions, dtype=np.float32)
            quat_arr = np.asarray(quats, dtype=np.float32)
            header = {
                "n": int(pos_arr.shape[0]),
                "timestamps": timestamps,
                "ids": ids,
            }
            payload = pos_arr.tobytes() + quat_arr.tobytes()
            logger.info("built %d image-pose markers + thumbnails", header["n"])
            return (header, payload), thumbnails
        except Exception:
            logger.exception("failed to build image poses")
            return ({"n": 0, "timestamps": [], "ids": []}, b""), []

    def _build_top_down_map(self) -> tuple[dict[str, Any], bytes] | None:
        """Render a top-down density map from the same pickled PointCloud2.

        Used for two things on the client: a GTA-style HUD minimap and a
        ground-pasted texture (so the user sees walls "drawn" on the floor).
        """
        import pickle

        path = Path(self.config.global_map_path)
        if not path.exists():
            logger.info("no global map at %s; skipping top-down render", path)
            return None
        try:
            obj = pickle.loads(path.read_bytes())
        except Exception:
            logger.exception("failed to load global map pickle for top-down render")
            return None

        as_np = getattr(obj, "as_numpy", None)
        if not callable(as_np):
            return None
        xyz, _colors = as_np()
        if xyz is None or xyz.size == 0:
            return None

        z = xyz[:, 2]
        m = (z >= self.config.map_z_min_floor) & (z <= self.config.map_z_max_floor)
        xy = xyz[m, :2]
        if xy.size == 0:
            xy = xyz[:, :2]

        x_min, x_max = float(xy[:, 0].min()), float(xy[:, 0].max())
        y_min, y_max = float(xy[:, 1].min()), float(xy[:, 1].max())
        cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2
        half = max(x_max - x_min, y_max - y_min) / 2 * 1.05
        x_min, x_max, y_min, y_max = cx - half, cx + half, cy - half, cy + half

        size = int(self.config.map_image_size)
        hist, _, _ = np.histogram2d(
            xy[:, 0], xy[:, 1], bins=size, range=[[x_min, x_max], [y_min, y_max]]
        )
        norm = np.clip(hist / max(np.percentile(hist, 99), 1.0), 0.0, 1.0)
        gray = (norm.T * 255).astype(np.uint8)
        gray = np.flipud(gray)
        # Light cyan walls on dark navy background — matches the world theme.
        rgb = np.zeros((size, size, 3), dtype=np.uint8)
        rgb[..., 0] = (gray.astype(np.uint16) * 76 // 255).astype(np.uint8)
        rgb[..., 1] = (gray.astype(np.uint16) * 217 // 255).astype(np.uint8)
        rgb[..., 2] = (gray.astype(np.uint16) * 255 // 255).astype(np.uint8)
        ok, buf = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        )
        if not ok:
            return None
        header = {
            "x_min": x_min, "x_max": x_max,
            "y_min": y_min, "y_max": y_max,
            "width_px": size, "height_px": size,
        }
        logger.info("built top-down map: %dx%d bounds=%s", size, size, header)
        return header, buf.tobytes()

    def _build_odom_trail(self) -> tuple[dict[str, Any], bytes]:
        """Subsample odom to a small polyline payload."""
        try:
            store = self._ensure_store()
            stream = store.streams[self.config.odom_stream_name]
            first, last = stream.first(), stream.last()
            span = max(float(last.ts) - float(first.ts), 1e-3)
            n = max(2, int(self.config.n_odom_samples))
            interval = span / n

            positions: list[tuple[float, float, float]] = []
            for obs in stream.transform(throttle(interval)):
                pose = getattr(obs, "pose", None)
                if pose is None:
                    continue
                p = list(pose)
                if len(p) < 3:
                    continue
                positions.append((float(p[0]), float(p[1]), float(p[2])))
                if len(positions) >= n:
                    break

            pos_arr = np.asarray(positions, dtype=np.float32)
            header = {"n": int(pos_arr.shape[0])}
            payload = pos_arr.tobytes()
            logger.info("built odom trail with %d points", header["n"])
            return header, payload
        except Exception:
            logger.exception("failed to build odom trail")
            return {"n": 0}, b""

    # ---- client messages (mostly diagnostics) ------------------------------

    def _on_client_message(self, conn: _ClientConn, msg: dict[str, Any]) -> None:
        kind = msg.get("type")
        if kind == "ping":
            conn.send_threadsafe(encode_text("pong"))
        elif kind == "diag":
            logger.info(
                "[client/diag] %s %s",
                msg.get("event", "?"),
                {k: v for k, v in msg.items() if k not in ("type", "event")},
            )
        elif kind in (
            "locomote", "yaw", "teleport_aim", "teleport_commit", "teleport_cancel",
            "scale_delta", "reset_view", "toggle_images", "toggle_render",
        ):
            # Client-side view gestures, echoed only as telemetry. Debug-level
            # so they don't spam the console (scale_delta fires every frame).
            logger.debug("[client] %s", kind)
        else:
            logger.warning("[client] unknown msg kind=%r full=%r", kind, msg)

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
