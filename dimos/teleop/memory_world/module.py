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

1. Accumulate the recording's lidar stream into a voxel map and push it as one
   binary frame (positions + per-point RGB).
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
import io
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Annotated, Any, Literal
import uuid

import cv2
from fastapi import HTTPException, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import numpy as np
from pydantic import Field as PydanticField

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.transform import throttle
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.teleop.memory_world.messages import (
    MSG_IMAGE_POSES,
    MSG_IMAGE_THUMBNAIL,
    MSG_ODOM_TRAIL,
    MSG_POINT_CLOUD,
    MSG_QUERY_IMAGE,
    MSG_TOP_DOWN_MAP,
    decode_text,
    encode_binary,
    encode_text,
)
from dimos.teleop.memory_world.query import (
    MEMORY_ANALYSIS_BOOTSTRAP,
    RESULT_SENTINEL,
    HighlightPath,
    HighlightPoint,
    MemoryQueryResult,
)
from dimos.teleop.memory_world.tf_tree import TfTree, pose_matrix
from dimos.teleop.memory_world.visual_search import (
    SIGLIP2_MODEL_NAME,
    PatchHit,
    Place,
    VisualMemoryIndex,
    body_style_quaternion,
    cluster_hits,
    cluster_places,
    hot_patches,
    patch_world_position,
    search_phrase,
)
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "web" / "static"


@dataclass(eq=False)
class _ClientConn:
    """One connected memory-world client."""

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


# A body frame (x forward, z up) seen as a camera optical frame (z forward,
# y down): the standard ROS optical rotation. Used only for recordings that
# carry no tf tree and stamp body poses on their images.
OPTICAL_FROM_BODY = pose_matrix((0.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5))

# Height ramp stops (RGB), floor to ceiling: purple, blue, cyan. The cool half
# of the wheel only, so yellow, orange, red and green stay free for
# highlights; the hue moves as well as the brightness because the lit voxel
# material flattens brightness alone.
HEIGHT_COLOR_STOPS = np.array([[120.0, 20.0, 150.0], [40.0, 80.0, 235.0], [120.0, 245.0, 255.0]])


class MemoryWorldConfig(ModuleConfig):
    """Config for the Memory World."""

    store_path: str = "data/go2_bigoffice.db"
    server_port: int = 8443
    # Voxel size for downsampling before shipping to the headset (metres).
    # 0.05m on a typical office map gives ~150k points; raise if your map is
    # bigger, lower for finer detail at the cost of bandwidth. This same value
    # is sent to the client so rendered point size matches voxel spacing.
    voxel_size: float = 0.05
    # Hard cap so an unexpectedly dense map doesn't try to ship 5M points.
    max_points: int = 250_000
    # Which lidar stream to accumulate and how many scans to sample. <= 0 means
    # use every lidar frame (densest map, slowest build).
    # The output cloud is deduped by voxel_size, so more scans improves the
    # map without growing the wire payload — only build time goes up.
    lidar_stream_name: str = "lidar"
    n_voxel_scans: int = 150
    # Set True if the stored lidar scans are ALREADY in the map/world frame
    # (e.g. SLAM-registered). Then we must NOT re-apply each scan's pose —
    # doing so double-transforms them into scattered noise. Leave False if
    # scans are in the sensor frame and need their pose applied. None detects
    # this from the point cloud frame_id.
    lidar_world_frame: bool | None = None
    # Z slab applied at load time to drop the floor/ceiling from the cloud.
    # The user stands on the floor in VR; rendering it as points is just noise.
    map_z_min: float = -0.2
    map_z_max: float = 2.4
    # Height colour ramp: floor (7th percentile of z) to floor + this. Room
    # height, roughly; anything taller saturates to the top colour.
    height_ramp_span_m: float = PydanticField(default=2.2, gt=0.0)
    # color_image stream is sampled for "Street View" capture-pose markers.
    image_stream_name: str = "color_image"
    n_image_markers: int = 200
    # Thumbnail params for the per-pose images that get textured onto quads in
    # 3D world space. Smaller = less bandwidth, lower res in headset.
    thumbnail_max_size: int = 192
    thumbnail_jpeg_quality: int = 70
    # The frames behind an answer are shown full-size in the world, so they
    # get more pixels than the capture-pose thumbnails.
    query_image_max_size: int = 640
    query_image_distance_m: float = PydanticField(default=1.0, gt=0.0)
    # ---- poses: the tf tree, and nothing else --------------------------------
    # Every pose the world needs (camera frames, lidar scans, the path) is a
    # tf lookup at the observation's timestamp. Recordings without a tf stream
    # fall back to the pose stamped on each image, read as a body pose.
    tf_stream_name: str = "tf"
    world_frame: str = "world"
    # The image stream's own frame_id by default.
    camera_optical_frame: str | None = None
    # A lookup fails when the nearest tf sample is further away than this.
    tf_tolerance_s: float = PydanticField(default=0.1, gt=0.0)
    # Added to image timestamps before the tf lookup, for recorders whose
    # camera and tf clocks disagree. 0 trusts the stamps.
    camera_time_offset_s: float = 0.0
    # The camera's path is drawn as a polyline sampled from tf.
    n_trail_samples: int = 400
    # Top-down density map (GTA-style minimap + ground projection). Computed
    # from the same point cloud — Z-slab histogram into a square image.
    map_image_size: int = 512
    map_z_min_floor: float = 0.05  # avoid floor speckle
    map_z_max_floor: float = 1.8
    client_route: str = "/memory_world"
    ws_route: str = "/ws_memory_world"
    # Bind on all interfaces by default — the headset connects over Wi-Fi.
    listen_host: str = "0.0.0.0"
    background_mode: Literal["black", "passthrough"] = "black"
    memory_analysis_max_output_chars: int = PydanticField(default=64_000, gt=0)
    # ---- spoken "where did I see X" search --------------------------------
    # SigLIP 2 per-patch index over the image stream. Building it is the slow
    # part and happens once per recording, in the background, into the
    # recording itself (~1.7 MB per indexed frame at fp16).
    siglip_model_name: str = SIGLIP2_MODEL_NAME
    image_index_stream_name: str = "image_siglip2_patches"
    # Every Nth frame. The recording is ~15fps, so 3 keeps sub-metre coverage
    # at a third of the embedding cost.
    image_index_stride: int = PydanticField(default=3, ge=1)
    build_image_index_on_start: bool = True
    # Two hits closer together than this are one place, not two answers.
    place_radius_m: float = PydanticField(default=2.5, gt=0.0)
    max_places: int = PydanticField(default=6, ge=1)
    # How many best-scoring frames are kept before clustering into places.
    search_top_k: int = PydanticField(default=200, ge=1)
    # faster-whisper model size for the spoken query.
    whisper_model: str = "base.en"
    # ---- putting the answer on the object, not on the robot -----------------
    # With a depth stream and intrinsics, each place is moved from the capture
    # pose to the point the winning patch actually looked at.
    depth_stream_name: str | None = None
    camera_info_stream_name: str | None = None
    depth_tolerance_s: float = PydanticField(default=0.02, gt=0.0)
    # Best frames whose hot patches are raycast, and how close two raycast
    # hits must land to be the same object.
    locate_frames: int = PydanticField(default=12, ge=1)
    object_radius_m: float = PydanticField(default=0.75, gt=0.0)


class MemoryWorldModule(Module):
    """VR memory-world module.

    See :mod:`dimos.teleop.memory_world` for the architectural overview.
    """

    config: MemoryWorldConfig

    def __init__(self, **kwargs: Any) -> None:
        self._world_clients: set[_ClientConn] = set()
        self._clients_lock = threading.Lock()

        self._store: SqliteStore | None = None
        # Cached payloads so reconnects are cheap.
        self._cached_cloud: tuple[dict[str, Any], bytes] | None = None
        self._cached_image_poses: tuple[dict[str, Any], bytes] | None = None
        # Per-pose JPEG thumbnails parallel to image_poses indices.
        self._cached_thumbnails: list[bytes] | None = None
        self._cached_odom: tuple[dict[str, Any], bytes] | None = None
        self._cached_top_down: tuple[dict[str, Any], bytes] | None = None
        self._viewer_position: tuple[float, float, float] | None = None
        self._visual_index: VisualMemoryIndex | None = None
        self._index_lock = threading.Lock()
        # The world caches are built lazily by whichever client connects first;
        # without this, two clients arriving together each voxelise the whole
        # recording.
        self._world_cache_lock = threading.Lock()
        self._index_progress = "not started"
        self._whisper: Any = None
        self._tf_tree_cache: TfTree | None = None
        self._tf_missing = False
        self._active_query_result: dict[str, Any] | None = None
        self._active_query_images: list[tuple[dict[str, Any], bytes]] = []
        self._query_revision = 0
        self._web_server: RobotWebInterface | None = None
        self._web_server_thread: threading.Thread | None = None

        super().__init__(**kwargs)
        self.config.store_path = str(self._resolve_store_path(self.config.store_path))

    @staticmethod
    def _resolve_store_path(name_or_path: str) -> Path:
        """Resolve explicit paths directly and bare names through the data registry."""
        path = Path(name_or_path).expanduser()
        if not path.is_absolute() and path.parts[:1] != ("data",):
            return get_data(name_or_path).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"memory store not found at {path}")
        return path.resolve()

    # ---- routes ------------------------------------------------------------

    def _setup_routes(self) -> None:
        assert self._web_server is not None
        app = self._web_server.app

        @app.get(self.config.client_route, response_class=HTMLResponse)  # type: ignore[misc]
        async def memory_world_index() -> HTMLResponse:
            index_path = STATIC_DIR / "index.html"
            content = index_path.read_text().replace(
                "__BACKGROUND_MODE__", self.config.background_mode
            )
            return HTMLResponse(content=content)

        if STATIC_DIR.is_dir():
            app.mount(
                "/static_mw",
                StaticFiles(directory=str(STATIC_DIR)),
                name="memory_world_static",
            )

        @app.websocket(self.config.ws_route)  # type: ignore[misc]
        async def ws_world(ws: WebSocket) -> None:
            await self._handle_ws(ws)

        @app.post(f"{self.config.client_route}/voice")  # type: ignore[misc]
        async def memory_world_voice(audio: UploadFile) -> dict[str, Any]:
            """Transcribe a spoken query and highlight the answer in VR.

            The Quest browser exposes no Web Speech API, so the headset records
            with MediaRecorder and posts the blob here instead.
            """
            raw = await audio.read()
            if not raw:
                raise HTTPException(status_code=400, detail="empty recording")
            transcript = await asyncio.to_thread(self._transcribe, raw)
            if not transcript:
                return {"transcript": "", "answer": "Nothing was said"}
            self._broadcast(encode_text("voice_transcript", text=transcript))
            outcome = await asyncio.to_thread(self.find_in_memory, transcript)
            return {
                "transcript": transcript,
                "success": outcome.success,
                "answer": outcome.message,
                "metadata": outcome.metadata,
            }

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
        except Exception as error:
            # The socket is usually already gone; the ws handler's finally
            # cleans up. Say so rather than vanishing without a trace.
            logger.debug("memory-world sender stopped: %s", error)

    # ---- initial payload ---------------------------------------------------

    def _ensure_store(self) -> SqliteStore:
        if self._store is None:
            self._store = SqliteStore(path=self.config.store_path, must_exist=True)
            logger.info("opened memory store at %s", self.config.store_path)
        return self._store

    def _ensure_world_cache(self) -> None:
        """Build the cloud, top-down map, markers and trail once, whoever asks first."""
        with self._world_cache_lock:
            if self._cached_cloud is None:
                self._cached_cloud = self._build_cloud()
            if self._cached_top_down is None:
                self._cached_top_down = self._build_top_down_map(self._cached_cloud)
            if self._cached_image_poses is None:
                self._cached_image_poses, self._cached_thumbnails = self._build_image_poses()
            if self._cached_odom is None:
                self._cached_odom = self._build_trail()

    def _send_initial_payload(self, conn: _ClientConn) -> None:
        try:
            self._ensure_world_cache()
            assert self._cached_cloud is not None  # built above; narrows the type
            assert self._cached_image_poses is not None
            assert self._cached_odom is not None
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
                    conn.send_threadsafe(encode_binary(MSG_IMAGE_THUMBNAIL, {"index": i}, jpeg))

            odom_header, odom_payload = self._cached_odom
            conn.send_threadsafe(encode_binary(MSG_ODOM_TRAIL, odom_header, odom_payload))

            conn.send_threadsafe(encode_text("ready"))
            with self._clients_lock:
                active_query_result = self._active_query_result
            if active_query_result is not None:
                conn.send_threadsafe(encode_text("query_result", **active_query_result))
                for header, jpeg in self._active_query_images:
                    conn.send_threadsafe(encode_binary(MSG_QUERY_IMAGE, header, jpeg))
        except Exception:
            logger.exception("failed to build/send world payload")
            conn.send_threadsafe(encode_text("error", message="world load failed"))

    def _build_cloud(self) -> tuple[dict[str, Any], bytes]:
        """Build a voxel cloud from the recording's lidar stream."""
        built = self._build_voxel_cloud_from_lidar()
        if built is None:
            raise RuntimeError("voxel-from-lidar produced no cloud")
        return built

    def _build_voxel_cloud_from_lidar(self) -> tuple[dict[str, Any], bytes] | None:
        """Accumulate a voxel map from the lidar stream and pack it for the wire.

        Each lidar scan is transformed into the world frame by a tf lookup at
        its stamp, then fed to :class:`VoxelMapTransformer`. The final accumulated cloud
        is height-coloured (violet low → red high) so the user gets depth cues
        without true RGB.
        """
        from dimos.mapping.voxels.module import VoxelMapTransformer
        from dimos.memory.transform import FnTransformer
        from dimos.msgs.geometry_msgs.Transform import Transform

        try:
            store = self._ensure_store()
            stream = store.streams[self.config.lidar_stream_name]
            first, last = stream.first(), stream.last()
            span = max(float(last.ts) - float(first.ts), 1e-3)
            n_scans = int(self.config.n_voxel_scans)
            use_all = n_scans <= 0
            lidar_world_frame = self.config.lidar_world_frame
            if lidar_world_frame is None:
                frame_id = str(getattr(first.data, "frame_id", "")).lower().lstrip("/")
                lidar_world_frame = frame_id in {"map", "odom", "world"}
                logger.info(
                    "lidar frame %r detected as %s",
                    frame_id,
                    "world-aligned" if lidar_world_frame else "sensor-relative",
                )

            def to_world_frame(obs: Any) -> Any:
                # If scans are already registered to the map frame, applying
                # a pose again double-transforms them into scattered noise.
                if lidar_world_frame:
                    return obs
                scan_frame = str(getattr(obs.data, "frame_id", "") or "").lstrip("/")
                matrix = self._frame_pose_at(scan_frame, float(obs.ts))
                if matrix is None:
                    pose = getattr(obs, "pose_tuple", None)  # no tf tree: the stamped pose
                    if pose is None or self._tf_tree() is not None:
                        return None
                    matrix = pose_matrix(tuple(pose[:3]), tuple(pose[3:7]))
                tf = Transform.from_matrix(matrix)
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
            logger.info(
                "built voxel cloud (%s scans): n=%d",
                "all" if use_all else str(n_scans),
                positions.shape[0],
            )
            return header, payload
        except Exception:
            logger.exception("voxel-from-lidar build failed")
            return None

    def _height_colors(self, positions: np.ndarray) -> np.ndarray:
        """Map Z (robot up) onto a purple-blue-cyan ramp.

        The map deliberately stays inside one cool hue band: floor is deep
        indigo, ceiling is pale cyan, and everything between is a blue. That
        keeps the warm colours (yellow, orange, red, magenta) and green free
        for highlights, so a voxel painted by a query reads as "the answer"
        rather than "a slightly different height".

        The ramp starts at the floor, taken as the 7th percentile of the
        cloud's heights (recordings whose odometry frame is not floor-aligned
        would clip a fixed slab to one end), and spans ``height_ramp_span_m``
        above it. Anchoring the top to the floor rather than to the highest
        voxel keeps ceiling fixtures and stray returns from stretching the
        ramp until the walls all look alike. Returns N x 3 uint8 RGB.
        """
        zc = positions[:, 2]
        lo = float(np.percentile(zc, 7)) if zc.size else 0.0
        hi = lo + max(float(self.config.height_ramp_span_m), 1e-3)
        t = np.clip((zc - lo) / (hi - lo), 0.0, 1.0)
        stops = np.linspace(0.0, 1.0, len(HEIGHT_COLOR_STOPS))
        rgb = np.stack([np.interp(t, stops, HEIGHT_COLOR_STOPS[:, c]) for c in range(3)], axis=1)
        return np.ascontiguousarray(np.rint(rgb).astype(np.uint8))

    def _cloud_header(self, positions: np.ndarray) -> dict[str, Any]:
        """Common header: count, colour flag, voxel size, and bounds."""
        return {
            "n": int(positions.shape[0]),
            "has_colors": True,
            "voxel_size": float(self.config.voxel_size),
            "bounds": {
                "x_min": float(positions[:, 0].min()),
                "x_max": float(positions[:, 0].max()),
                "y_min": float(positions[:, 1].min()),
                "y_max": float(positions[:, 1].max()),
                "z_min": float(positions[:, 2].min()),
                "z_max": float(positions[:, 2].max()),
            },
        }

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
            for obs in stream.transform(throttle(interval)):  # type: ignore[var-annotated]
                optical = self._camera_pose_of(obs)
                if optical is None:
                    continue
                # Markers stand where the camera was and face the way it looked.
                positions.append(tuple(float(v) for v in optical[:3, 3]))  # type: ignore[arg-type]
                quats.append(body_style_quaternion(optical))
                timestamps.append(float(obs.ts))
                ids.append(int(getattr(obs, "id", 0)))

                try:
                    thumbnails.append(self._encode_jpeg(obs.data, max_size, quality))
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

    @staticmethod
    def _encode_jpeg(img: Any, max_size: int, quality: int) -> bytes:
        if hasattr(img, "resize_to_fit"):
            img, _ = img.resize_to_fit(max_size, max_size)
        bgr = img.to_bgr().to_opencv() if hasattr(img, "to_bgr") else img
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return buf.tobytes() if ok else b""

    def _build_top_down_map(
        self, cloud: tuple[dict[str, Any], bytes]
    ) -> tuple[dict[str, Any], bytes] | None:
        """Render a top-down density map from the same point cloud shown in VR.

        Used for two things on the client: a GTA-style HUD minimap and a
        ground-pasted texture (so the user sees walls "drawn" on the floor).
        """
        cloud_header, cloud_payload = cloud
        n = int(cloud_header.get("n", 0))
        xyz = np.frombuffer(cloud_payload, dtype=np.float32, count=n * 3).reshape(n, 3)
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
        density_scale = max(float(np.percentile(hist, 99)), 1.0)
        norm = np.clip(hist / density_scale, 0.0, 1.0)
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
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "width_px": size,
            "height_px": size,
        }
        logger.info("built top-down map: %dx%d bounds=%s", size, size, header)
        return header, buf.tobytes()

    def _build_trail(self) -> tuple[dict[str, Any], bytes]:
        """The camera's path as a small polyline, sampled from tf."""
        try:
            n = max(2, int(self.config.n_trail_samples))
            positions: list[tuple[float, float, float]] = []
            tree = self._tf_tree()
            if tree is not None:
                span = tree.span(self.config.world_frame, self._camera_frame())
                if span is not None:
                    for ts in np.linspace(span[0], span[1], n):
                        matrix = self._frame_pose_at(self._camera_frame(), float(ts))
                        if matrix is not None:
                            positions.append(tuple(float(v) for v in matrix[:3, 3]))  # type: ignore[arg-type]
            else:
                store = self._ensure_store()
                stream = store.streams[self.config.image_stream_name]
                span_s = max(float(stream.last().ts) - float(stream.first().ts), 1e-3)
                for obs in stream.transform(throttle(span_s / n)):  # type: ignore[var-annotated]
                    matrix = self._camera_pose_of(obs)
                    if matrix is not None:
                        positions.append(tuple(float(v) for v in matrix[:3, 3]))  # type: ignore[arg-type]
                    if len(positions) >= n:
                        break

            pos_arr = np.asarray(positions, dtype=np.float32)
            header = {"n": int(pos_arr.shape[0])}
            logger.info("built camera trail with %d points", header["n"])
            return header, pos_arr.tobytes()
        except Exception:
            logger.exception("failed to build the camera trail")
            return {"n": 0}, b""

    @skill
    def analyze_memory(
        self,
        code: str,
        timeout: Annotated[float, PydanticField(gt=0.0, le=100.0)] = 100.0,
    ) -> SkillResult:
        """Analyze the recorded memory and display validated spatial results in VR.

        Run complete Python code in a fresh process with ``store`` (the mem2
        SqliteStore), ``np`` (NumPy), and ``viewer_position`` available. Inspect
        streams with ``store.list_streams()``, ``store.summary()``, and
        ``store.streams[name]``. Observations expose ``pose_tuple``, ``data``,
        and ``id``. ``store.read_stream`` does not exist. For a bounded xyz
        trajectory use ``sample_pose_path("odom", max_points=200)``. Assign a
        dictionary to ``result`` with a required ``answer`` and optional fields:
        ``focus_point`` [x,y,z], ``regions`` (polygon point lists),
        ``evidence_paths`` (path point lists), ``points``, and
        ``observation_ids``. Every point must be [x,y,z] in the world frame.
        A route from the current VR position is added automatically when
        ``focus_point`` and ``global_costmap`` are available.

        Args:
            code: Complete Python source that assigns the result dictionary.
            timeout: Maximum execution time in seconds, up to 100 seconds.
        """
        started = time.monotonic()
        with self._clients_lock:
            viewer_position = self._viewer_position
        try:
            completed = subprocess.run(
                [
                    sys.executable,
                    "-c",
                    MEMORY_ANALYSIS_BOOTSTRAP,
                    self.config.store_path,
                    json.dumps(viewer_position),
                ],
                input=code,
                capture_output=True,
                text=True,
                check=False,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            return SkillResult.fail(
                "EXECUTION_TIMEOUT", f"Memory analysis timed out after {timeout:g} seconds"
            )

        marker = completed.stdout.rfind(RESULT_SENTINEL)
        if marker < 0:
            detail = (completed.stderr or completed.stdout or "analysis returned no result").strip()
            return SkillResult.fail("EXECUTION_FAILED", self._cap_analysis_output(detail))

        encoded = completed.stdout[marker + len(RESULT_SENTINEL) :].splitlines()[0]
        if len(encoded) > self.config.memory_analysis_max_output_chars:
            return SkillResult.fail(
                "RESULT_TOO_LARGE",
                "Memory result exceeds the configured output limit of "
                f"{self.config.memory_analysis_max_output_chars} characters",
            )
        try:
            result = MemoryQueryResult.model_validate_json(encoded)
            self._add_route_to_result(result)
        except Exception as exc:
            return SkillResult.fail("EXECUTION_FAILED", f"Invalid memory result: {exc}")

        query_id = self._publish_query_result(result)

        return SkillResult(
            success=True,
            message=result.answer,
            duration_ms=(time.monotonic() - started) * 1000,
            metadata={
                "query_id": query_id,
                "regions": len(result.regions),
                "evidence_paths": len(result.evidence_paths),
                "observation_ids": len(result.observation_ids),
                "route": result.route is not None,
            },
        )

    def _broadcast(self, message: bytes | str) -> None:
        with self._clients_lock:
            clients = tuple(self._world_clients)
        for client in clients:
            client.send_threadsafe(message)

    def _publish_query_result(self, result: MemoryQueryResult) -> str:
        """Send a result to every connected viewer and remember it for reconnects."""
        with self._clients_lock:
            query_id = uuid.uuid4().hex
            self._query_revision += 1
            payload = result.model_dump(mode="json")
            payload.update(query_id=query_id, revision=self._query_revision)
            self._active_query_result = payload
            self._active_query_images = []
        self._broadcast(encode_text("query_result", **payload))
        return query_id

    # ---- spoken visual search ---------------------------------------------

    def _ensure_visual_index(self) -> VisualMemoryIndex:
        if self._visual_index is None:
            self._visual_index = VisualMemoryIndex(
                self._ensure_store(),
                pose_of=self._camera_pose_of,
                image_stream_name=self.config.image_stream_name,
                index_stream_name=self.config.image_index_stream_name,
                model_name=self.config.siglip_model_name,
            )
        return self._visual_index

    def _build_visual_index(self) -> None:
        """Embed the recording's frames. Slow and one-shot; runs off the request path."""
        with self._index_lock:
            index = self._ensure_visual_index()
            existing = index.count()
            self._index_progress = f"building (had {existing} frames)"
            try:
                added = index.build(stride=self.config.image_index_stride)
            except Exception as error:
                self._index_progress = f"failed: {error}"
                logger.exception("visual index build failed")
                return
            self._index_progress = f"ready ({index.count()} frames)"
            logger.info("visual index ready: %d frames (+%d new)", index.count(), added)
        # Warm both model loads here, off the request path: cold they add ~18s
        # to whichever query comes first, which is the one being demoed.
        index.model.embed_text("warmup")
        _ = self.whisper
        logger.info("voice query path warm")

    @skill
    def find_in_memory(self, query: str) -> SkillResult:
        """Find the distinct places something was seen and highlight them in VR.

        Answers questions like "where did I see a car" by comparing the phrase
        against precomputed SigLIP 2 embeddings of the recording's camera
        frames, then reducing the matches to one marker per distinct location.

        Args:
            query: What to look for, e.g. "a car" or "a whiteboard".
        """
        started = time.monotonic()
        phrase = search_phrase(query)
        if not phrase:
            return SkillResult.fail("INVALID_QUERY", "The query text is empty")

        index = self._ensure_visual_index()
        if index.count() == 0:
            return SkillResult.fail(
                "INDEX_NOT_READY",
                f"The SigLIP index for {self.config.store_path} holds no frames "
                f"({self._index_progress}). Build it with "
                f"`python -m dimos.teleop.memory_world.visual_search {self.config.store_path}`.",
            )

        places = self._locate_objects(phrase)
        located = bool(places)
        if not located:
            # No depth or extrinsics: answer with the poses the frames were taken from.
            places = cluster_places(
                index.search(phrase, k=self.config.search_top_k),
                radius=self.config.place_radius_m,
                max_places=self.config.max_places,
            )
        if not places:
            return SkillResult.fail("NOT_FOUND", f"Nothing in the recording matches {phrase!r}")

        result = MemoryQueryResult(
            answer=f"Found {phrase} in {len(places)} place(s), best match {places[0].similarity:+.3f}",
            focus_point=places[0].position,
            points=[
                HighlightPoint(
                    position=place.position,
                    label=f"{phrase} ({place.similarity:+.3f}, {place.views} view"
                    f"{'s' if place.views != 1 else ''})",
                    radius=self.config.object_radius_m if located else None,
                )
                for place in places
            ],
            observation_ids=self._markers_near([place.position for place in places]),
        )
        self._add_route_to_result(result)
        query_id = self._publish_query_result(result)
        self._publish_query_images(query_id, phrase, places)

        return SkillResult(
            success=True,
            message=result.answer,
            duration_ms=(time.monotonic() - started) * 1000,
            metadata={
                "query_id": query_id,
                "query": phrase,
                "places": [
                    {
                        "position": place.position,
                        "similarity": place.similarity,
                        "views": place.views,
                    }
                    for place in places
                ],
                "located": located,
            },
        )

    def _publish_query_images(self, query_id: str, phrase: str, places: list[Place]) -> None:
        """Send the frame behind each place, posed where its camera stood.

        The header carries the camera position, its forward and up directions
        and its field of view, so the viewer can hang the picture on the
        camera's image plane.
        """
        store = self._ensure_store()
        images = store.streams[self.config.image_stream_name]
        hfov_deg = 70.0
        if self.config.camera_info_stream_name is not None:
            info = store.streams[self.config.camera_info_stream_name].first().data
            hfov_deg = float(np.degrees(2.0 * np.arctan2(info.width / 2.0, info.K[0])))

        sent: list[tuple[dict[str, Any], bytes]] = []
        for index, place in enumerate(places):
            try:
                frame = images.at(place.ts, tolerance=0.005).first()
                jpeg = self._encode_jpeg(
                    frame.data, self.config.query_image_max_size, self.config.thumbnail_jpeg_quality
                )
            except Exception:
                logger.exception("could not fetch the frame behind place %d", index)
                continue
            camera = pose_matrix(place.camera_position or place.position, place.orientation)
            forward, up = camera[:3, 2], -camera[:3, 1]  # optical: z forward, y down
            height, width = frame.data.shape[:2]
            header = {
                "query_id": query_id,
                "index": index,
                "label": f"{phrase} ({place.similarity:+.3f})",
                "position": [float(v) for v in camera[:3, 3]],
                "forward": [float(v) for v in forward],
                "up": [float(v) for v in up],
                "hfov_deg": hfov_deg,
                "aspect": float(width) / float(height),
                "distance_m": float(self.config.query_image_distance_m),
            }
            sent.append((header, jpeg))
        with self._clients_lock:
            self._active_query_images = sent
        for header, jpeg in sent:
            self._broadcast(encode_binary(MSG_QUERY_IMAGE, header, jpeg))

    # ---- poses -------------------------------------------------------------

    def _tf_tree(self) -> TfTree | None:
        """The recording's tf tree, loaded once; None when the recording has none."""
        if self._tf_tree_cache is None and not self._tf_missing:
            store = self._ensure_store()
            if self.config.tf_stream_name not in store.list_streams():
                self._tf_missing = True
                logger.warning(
                    "no %r stream; falling back to the poses stamped on images",
                    self.config.tf_stream_name,
                )
                return None
            tree = TfTree.from_stream(store.streams[self.config.tf_stream_name])
            logger.info("tf tree: %d transforms over %d frames", len(tree), len(tree.frames))
            self._tf_tree_cache = tree
        return self._tf_tree_cache

    def _camera_frame(self) -> str:
        if self.config.camera_optical_frame:
            return self.config.camera_optical_frame
        first = self._ensure_store().streams[self.config.image_stream_name].first()
        return str(getattr(first.data, "frame_id", "") or "").lstrip("/")

    def _frame_pose_at(self, frame: str, ts: float) -> np.ndarray | None:
        """world_T_frame at *ts* from tf, or None."""
        tree = self._tf_tree()
        if tree is None:
            return None
        matrix: np.ndarray | None = tree.lookup(
            self.config.world_frame, frame, ts, self.config.tf_tolerance_s
        )
        return matrix

    def _camera_pose_of(self, obs: Any) -> np.ndarray | None:
        """world_T_optical for an image observation.

        From tf at the image's stamp (plus ``camera_time_offset_s``); without a
        tf stream, from the body pose stamped on the image, turned into the
        optical convention.
        """
        if self._tf_tree() is not None:
            return self._frame_pose_at(
                self._camera_frame(), float(obs.ts) + self.config.camera_time_offset_s
            )
        pose = getattr(obs, "pose_tuple", None)
        if pose is None:
            return None
        body = pose_matrix(tuple(pose[:3]), tuple(pose[3:7]) if len(pose) >= 7 else (0, 0, 0, 1))
        return np.asarray(body @ OPTICAL_FROM_BODY)

    def _locate_objects(self, phrase: str) -> list[Place]:
        """Raycast the hot patches of the best frames through depth and group the hits.

        Empty when the recording has no depth stream or intrinsics, or when
        no hot patch lands on valid depth.
        """
        if self.config.depth_stream_name is None or self.config.camera_info_stream_name is None:
            return []
        store = self._ensure_store()
        depth_stream = store.streams[self.config.depth_stream_name]
        k = store.streams[self.config.camera_info_stream_name].first().data.K
        intrinsics = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))

        hits: list[PatchHit] = []
        for frame in self._ensure_visual_index().frame_patches(phrase, k=self.config.locate_frames):
            try:
                depth = depth_stream.at(frame.ts, tolerance=self.config.depth_tolerance_s).first()
            except LookupError:
                continue
            depth_mm = np.asarray(depth.data.data)
            camera_to_world = pose_matrix(frame.position, frame.orientation)
            for image_uv, score in hot_patches(frame.similarity, frame.rows, frame.cols):
                position = patch_world_position(image_uv, depth_mm, intrinsics, camera_to_world)
                if position is not None:
                    hits.append(
                        PatchHit(
                            position=position,
                            similarity=score,
                            source_id=frame.source_id,
                            ts=frame.ts,
                            camera_position=frame.position,
                        )
                    )
        return cluster_hits(
            hits, radius=self.config.object_radius_m, max_places=self.config.max_places
        )

    def _markers_near(self, positions: list[tuple[float, float, float]]) -> list[int]:
        """Ids of the capture-pose markers closest to each place.

        The viewer only holds thumbnails for the ``n_image_markers`` poses it was
        sent, and a matching frame is usually not one of them. Highlighting the
        nearest marker instead puts a visible photo at each answer location.
        """
        if self._cached_image_poses is None:
            return []
        header, payload = self._cached_image_poses
        n = int(header.get("n", 0))
        ids = header.get("ids") or []
        if n == 0 or len(ids) < n:
            return []
        marker_xyz = np.frombuffer(payload, dtype=np.float32, count=n * 3).reshape(n, 3)
        nearest = {
            int(ids[int(np.argmin(np.linalg.norm(marker_xyz - np.asarray(p, np.float32), axis=1)))])
            for p in positions
        }
        return sorted(nearest)

    @property
    def whisper(self) -> Any:
        if self._whisper is None:
            from faster_whisper import WhisperModel

            self._whisper = WhisperModel(
                self.config.whisper_model, device="auto", compute_type="int8"
            )
            logger.info("loaded faster-whisper %s", self.config.whisper_model)
        return self._whisper

    def _transcribe(self, audio: bytes) -> str:
        """Transcribe a browser audio recording with faster-whisper.

        Decoding goes through faster-whisper's own resampler rather than a
        temp-file handoff, so whatever container MediaRecorder chose (webm/opus
        on Chromium and the Quest browser, mp4/aac on Safari) is handled the
        same way.
        """
        from faster_whisper import decode_audio

        samples = decode_audio(io.BytesIO(audio), sampling_rate=16_000)
        segments, _ = self.whisper.transcribe(samples, language="en")
        return " ".join(segment.text for segment in segments).strip()

    def _cap_analysis_output(self, output: str) -> str:
        limit = self.config.memory_analysis_max_output_chars
        if len(output) <= limit:
            return output
        return output[:limit] + f"\n... [truncated, {len(output)} chars total]"

    def _add_route_to_result(self, result: MemoryQueryResult) -> None:
        # Routes are server-owned: only the planner may label one collision-aware.
        result.route = None
        with self._clients_lock:
            viewer_position = self._viewer_position
        if result.focus_point is None or viewer_position is None:
            return
        try:
            store = self._ensure_store()
            if "global_costmap" not in store.list_streams():
                return
            costmap = store.streams.global_costmap.last().data
            route = min_cost_astar(
                costmap,
                goal=result.focus_point[:2],
                start=viewer_position[:2],
            )
            if route is None:
                return
            points = [(pose.x, pose.y, pose.z + 0.08) for pose in route.poses]
            if len(points) >= 2:
                result.route = HighlightPath(
                    points=points,
                    label="Route to answer",
                    color="#64ff8f",
                )
        except Exception:
            logger.exception("failed to build route to memory query result")

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
        elif kind == "viewer_pose":
            position = msg.get("position")
            if (
                isinstance(position, list)
                and len(position) == 3
                and all(isinstance(value, int | float) and np.isfinite(value) for value in position)
            ):
                with self._clients_lock:
                    self._viewer_position = (
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    )
        elif kind in (
            "locomote",
            "yaw",
            "teleport_aim",
            "teleport_commit",
            "teleport_cancel",
            "scale_delta",
            "reset_view",
            "toggle_images",
            "toggle_cloud",
        ):
            # Client-side view gestures, echoed only as telemetry. Debug-level
            # so they don't spam the console (scale_delta fires every frame).
            logger.debug("[client] %s", kind)
        else:
            logger.warning("[client] unknown msg kind=%r full=%r", kind, msg)

    # ---- lifecycle ---------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        self._web_server = RobotWebInterface(
            host=self.config.listen_host,
            port=self.config.server_port,
        )
        self._setup_routes()
        self._web_server_thread = threading.Thread(
            target=self._web_server.run,
            kwargs={"ssl": True, "ssl_certs_dir": DIMOS_PROJECT_ROOT / "assets" / "teleop_certs"},
            daemon=True,
            name="MemoryWorldWebServer",
        )
        self._web_server_thread.start()
        logger.info(
            "memory-world server started on https://%s:%d",
            self.config.listen_host,
            self.config.server_port,
        )
        if self.config.build_image_index_on_start:
            threading.Thread(
                target=self._build_visual_index,
                daemon=True,
                name="MemoryWorldVisualIndex",
            ).start()

    @rpc
    def stop(self) -> None:
        try:
            if self._web_server is not None:
                self._web_server.shutdown()
            if self._web_server_thread is not None:
                self._web_server_thread.join(timeout=3)
                self._web_server_thread = None
        finally:
            if self._visual_index is not None:
                self._visual_index.stop()
                self._visual_index = None
            store = self._store
            self._store = None
            if store is not None:
                try:
                    store.stop()
                except Exception:
                    logger.exception("error closing memory store")
            super().stop()
