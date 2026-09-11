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

1. Push the latest voxel map from the ray tracing mapper as one binary frame
   (positions + per-point RGB), and again whenever the mapper grows it.
2. Sample the ``color_image`` stream and push each capture pose as a
   Street-View-style marker. The headset can later pinch one to surface the
   image at that location.
3. Push the camera trail as a polyline.

Routes come from the MLS planner. A goal set through the navigation skills is
planned from the robot's last recorded pose, and every path the planner emits
is drawn on the active answer.

All locomotion (smooth walk, snap turn, teleport, scale) is client-side —
the server is a data push plus diagnostics.
"""

from __future__ import annotations

import asyncio
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass, field
import functools
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Annotated, Any, Literal, TypeVar
import uuid

import cv2
from fastapi import HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import HTMLResponse, Response
from fastapi.staticfiles import StaticFiles
import numpy as np
from pydantic import Field as PydanticField
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.memory.store.base import Store
from dimos.memory.transform import throttle
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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
from dimos.teleop.memory_world.recording import detect_streams, open_recording
from dimos.teleop.memory_world.replay import ReplayRecorder, VoxelReplay
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
from dimos.types.robot_location import RobotLocation
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "web" / "static"
MAX_QUERY_FRAMES = 12
ROUTE_LIFT_M = 0.08


def _xyz_text(position: tuple[float, float, float]) -> str:
    return "({:.1f}, {:.1f}, {:.1f})".format(*position)


def _heading(
    position: tuple[float, float, float], orientation: tuple[float, float, float, float]
) -> float:
    """Yaw of an optical camera pose's forward axis, in world radians."""
    forward = pose_matrix(position, orientation)[:3, 2]
    return float(np.arctan2(forward[1], forward[0]))


class _RevalidatedStaticFiles(StaticFiles):
    """Static files the browser must revalidate (ETag) on every load.

    Without this a phone kept a stale scene.js beside a fresh main.js for
    hours and every new control on the page was a TypeError.
    """

    async def get_response(self, path: str, scope: Any) -> Any:
        response = await super().get_response(path, scope)
        response.headers["Cache-Control"] = "no-cache"
        return response


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
T = TypeVar("T")


def _serialized(method: Callable[..., T]) -> Callable[..., T]:
    """Run one search at a time; the index is read on one store connection."""

    @functools.wraps(method)
    def wrapper(self: MemoryWorldModule, *args: Any, **kwargs: Any) -> T:
        with self._search_lock:
            return method(self, *args, **kwargs)

    return wrapper


# A timeline whose last snapshot is this close to the recording's end counts as complete.
REPLAY_COMPLETE_MARGIN_S = 15.0
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
    # The lidar stream the timeline replay is built from.
    lidar_stream_name: str = "lidar"
    # Viewers get the mapper's latest map at most this often.
    map_push_interval_s: float = PydanticField(default=10.0, gt=0.0)
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
    # Top-down density map for the HUD minimap. Computed
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
    # Empty means "named after siglip_model_name", so two models never share one.
    image_index_stream_name: str = ""
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
    # ---- timeline replay ------------------------------------------------------
    # Keyframe and per-scan diff streams written into the recording once (see
    # replay.py); the viewer scrubs by fetching one keyframe's segment at a
    # time. A longer interval means fewer, larger segments.
    replay_keyframe_interval_s: float = PydanticField(default=30.0, gt=0.0)
    # The camera frame shown while scrubbing, fetched one at a time.
    replay_frame_max_size: int = 480
    replay_frame_jpeg_quality: int = 60


class MemoryWorldModule(Module):
    """VR memory-world module.

    See :mod:`dimos.teleop.memory_world` for the architectural overview.
    """

    config: MemoryWorldConfig

    global_map: In[PointCloud2]
    path: In[NavPath]

    def __init__(self, **kwargs: Any) -> None:
        self._world_clients: set[_ClientConn] = set()
        self._clients_lock = threading.Lock()

        self._store: Store | None = None
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
        self._search_lock = threading.Lock()
        # The world caches are built lazily by whichever client connects first;
        # without this, two clients arriving together each voxelise the whole
        # recording.
        self._world_cache_lock = threading.Lock()
        self._index_progress = "not started"
        self._whisper: Any = None
        self._tf_tree_cache: TfTree | None = None
        self._tf_missing = False
        self._replay: VoxelReplay | None = None
        self._recorder: ReplayRecorder | None = None
        self._replay_complete = False
        self._replay_floor = 0.0
        self._replay_frames_json: list[float] | None = None
        self._replay_lock = threading.Lock()
        # The store's sqlite connection is not safe to read from two threads
        # at once, and a scrubbing viewer fetches segments and frames together.
        self._replay_read_lock = threading.Lock()
        self._replay_progress = "not started"
        self._replay_index: dict[str, Any] | None = None
        self._replay_frames: OrderedDict[int, tuple[bytes, dict[str, Any]]] = OrderedDict()
        self._camera_hfov_deg: float | None = None
        self._active_query_result: dict[str, Any] | None = None
        self._active_query_images: list[tuple[dict[str, Any], bytes]] = []
        self._query_revision = 0
        self._last_route: HighlightPath | None = None
        self._map_push_lock = threading.Lock()
        self._map_push_timer: threading.Timer | None = None
        self._latest_map: np.ndarray | None = None
        self._last_map_push = float("-inf")
        self._tagged_locations: dict[str, RobotLocation] = {}
        self._recording_start_ts: float | None = None
        self._web_server: RobotWebInterface | None = None
        self._web_server_thread: threading.Thread | None = None
        self._prepare_thread: threading.Thread | None = None

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
            # The newest static file stamps the script URLs, so a reload never
            # pairs a fresh main.js with a scene.js the browser cached earlier.
            asset_version = max(path.stat().st_mtime_ns for path in STATIC_DIR.iterdir())
            content = (
                index_path.read_text()
                .replace("__BACKGROUND_MODE__", self.config.background_mode)
                .replace("__ASSET_VERSION__", str(asset_version))
            )
            return HTMLResponse(content=content)

        if STATIC_DIR.is_dir():
            app.mount(
                "/static_mw",
                _RevalidatedStaticFiles(directory=str(STATIC_DIR)),
                name="memory_world_static",
            )

        @app.websocket(self.config.ws_route)  # type: ignore[misc]
        async def ws_world(ws: WebSocket) -> None:
            await self._handle_ws(ws)

        # Replay segments are int16 grids and uint32 slots: they halve under gzip.
        app.add_middleware(GZipMiddleware, minimum_size=1024)

        @app.get(f"{self.config.client_route}/replay/index")  # type: ignore[misc]
        async def memory_world_replay_index() -> dict[str, Any]:
            """Scan and keyframe stamps: everything the viewer needs to seek."""
            try:
                return await asyncio.to_thread(self._replay_read, self._replay_index_json)
            except Exception as error:
                raise HTTPException(
                    status_code=503, detail=f"replay {self._replay_progress}"
                ) from error

        @app.get(f"{self.config.client_route}/replay/segment/{{number}}")  # type: ignore[misc]
        async def memory_world_replay_segment(number: int, request: Request) -> Response:
            """One keyframe plus the diffs up to the next, see VoxelReplay.segment.

            Segments are gzipped once when first built: compressing a megabyte
            per request cost more than sending it.
            """
            replay = await asyncio.to_thread(self._ensure_replay)
            if not 0 <= number < len(replay.index.keyframe_scan):
                raise HTTPException(status_code=404, detail="no such segment")
            start, end = replay.index.segment_scans(number)
            headers = self._replay_cache_headers(replay, f"{number}-{end - start}")
            if request.headers.get("if-none-match") == headers["ETag"]:
                return Response(status_code=304, headers=headers)
            raw, gzipped = await asyncio.to_thread(
                self._replay_read, replay.encoded_segment, number
            )
            if "gzip" in request.headers.get("accept-encoding", ""):
                headers["Content-Encoding"] = "gzip"
                return Response(
                    content=gzipped, media_type="application/octet-stream", headers=headers
                )
            return Response(content=raw, media_type="application/octet-stream", headers=headers)

        @app.get(f"{self.config.client_route}/replay/frame")  # type: ignore[misc]
        async def memory_world_replay_frame(t: float, request: Request) -> Response:
            """The camera frame nearest *t* as JPEG; its pose rides in a header."""
            replay = await asyncio.to_thread(self._ensure_replay)
            found = await asyncio.to_thread(self._replay_read, self._replay_frame, t)
            if found is None:
                raise HTTPException(status_code=404, detail="no frame near that time")
            jpeg, meta = found
            headers = self._replay_cache_headers(replay, meta["ts"])
            if request.headers.get("if-none-match") == headers["ETag"]:
                return Response(status_code=304, headers=headers)
            headers["X-Camera-Pose"] = json.dumps(meta)
            return Response(content=jpeg, media_type="image/jpeg", headers=headers)

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

    def _ensure_store(self) -> Store:
        if self._store is None:
            self._store = open_recording(self.config.store_path)
            logger.info("opened memory store at %s", self.config.store_path)
            self._name_streams(self._store)
        return self._store

    def _name_streams(self, store: Store) -> None:
        """Fill in stream names the recording does not actually have.

        The defaults suit a Go2 recording (``color_image``, ``lidar``); this
        rig says ``realsense_color_image`` and ``pointlio_lidar``, and someone
        else's robot says something else again. A name given on the command
        line is kept as-is — only names that are missing get detected, so a
        recording with two cameras can still be pointed at one of them.
        """
        present = set(store.list_streams())
        detected = detect_streams(store)
        for role, setting in (
            ("image", "image_stream_name"),
            ("lidar", "lidar_stream_name"),
            ("depth", "depth_stream_name"),
            ("camera_info", "camera_info_stream_name"),
            ("tf", "tf_stream_name"),
        ):
            configured = getattr(self.config, setting)
            if configured in present or detected[role] is None:
                continue
            setattr(self.config, setting, detected[role])
            logger.info(
                "%s: using %r (no %r in the recording)", setting, detected[role], configured
            )

    def _ensure_world_cache(self) -> None:
        """Build the markers and trail once, whoever asks first. The map comes from the mapper."""
        with self._world_cache_lock:
            if self._cached_image_poses is None:
                self._cached_image_poses, self._cached_thumbnails = self._build_image_poses()
            if self._cached_odom is None:
                self._cached_odom = self._build_trail()

    def _send_initial_payload(self, conn: _ClientConn) -> None:
        try:
            self._ensure_world_cache()
            assert self._cached_image_poses is not None
            assert self._cached_odom is not None
            with self._world_cache_lock:
                cloud, top_down = self._cached_cloud, self._cached_top_down
            # Before the mapper's first emission there is no map yet; it
            # reaches this client with the next push.
            if cloud is not None:
                self._send_map(conn.send_threadsafe, cloud, top_down)

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

    # ---- map from the mapper ----------------------------------------------

    def _on_global_map(self, cloud: PointCloud2) -> None:
        xyz, _ = cloud.as_numpy()
        if xyz is None or xyz.size == 0:
            return
        with self._map_push_lock:
            self._latest_map = np.asarray(xyz, dtype=np.float32)
        self._schedule_map_push()
        try:
            self._record_snapshot(self._latest_map, float(cloud.ts))
        except Exception:
            logger.exception("recording the mapper snapshot into the timeline failed")

    def _schedule_map_push(self) -> None:
        """Push the latest map to every viewer, at most every map_push_interval_s."""
        with self._map_push_lock:
            if self._map_push_timer is not None:
                return
            delay = self.config.map_push_interval_s - (time.monotonic() - self._last_map_push)
            self._map_push_timer = threading.Timer(max(0.0, delay), self._push_map)
            self._map_push_timer.daemon = True
            self._map_push_timer.start()

    def _push_map(self) -> None:
        """Pack the mapper's latest map, cache it for new viewers, and send it to the current ones."""
        with self._map_push_lock:
            self._map_push_timer = None
            self._last_map_push = time.monotonic()
            xyz, self._latest_map = self._latest_map, None
        if xyz is None:
            return
        packed = self._pack_cloud(xyz)
        if packed is None:
            return
        top_down = self._build_top_down_map(packed)
        with self._world_cache_lock:
            first = self._cached_cloud is None
            self._cached_cloud = packed
            self._cached_top_down = top_down
        if first:
            logger.info("first map from the mapper: n=%d", packed[0]["n"])
        self._send_map(self._broadcast, packed, top_down)

    @staticmethod
    def _send_map(
        send: Callable[[bytes | str], None],
        cloud: tuple[dict[str, Any], bytes],
        top_down: tuple[dict[str, Any], bytes] | None,
    ) -> None:
        header, payload = cloud
        send(encode_text("world_summary", **header))
        send(encode_binary(MSG_POINT_CLOUD, header, payload))
        if top_down is not None:
            map_header, map_payload = top_down
            send(encode_binary(MSG_TOP_DOWN_MAP, map_header, map_payload))

    def _pack_cloud(self, xyz: np.ndarray) -> tuple[dict[str, Any], bytes] | None:
        """Clip a world-frame cloud to the height slab, cap it, and pack it for the wire."""
        z = xyz[:, 2]
        xyz = xyz[(z >= self.config.map_z_min) & (z <= self.config.map_z_max)]
        if xyz.size == 0:
            return None
        if xyz.shape[0] > self.config.max_points:
            stride = xyz.shape[0] // self.config.max_points + 1
            xyz = xyz[::stride]
        positions = np.ascontiguousarray(xyz.astype(np.float32))
        # Lidar has no RGB, so always height-color.
        rgb = self._height_colors(positions)
        header = self._cloud_header(positions)
        return header, positions.tobytes() + rgb.tobytes()

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

            # One indexed read per marker rather than a pass over every frame:
            # on an mcap a full pass decompresses every image chunk (minutes),
            # while a read from a stamp touches only the chunk that holds it.
            def sampled() -> Any:
                for k in range(n):
                    found = stream.after(float(first.ts) + k * interval - 1e-6).limit(1).to_list()
                    if found:
                        yield found[0]

            for obs in sampled():
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

        The client shows it as the GTA-style HUD minimap.
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
        The planner's current route, if any, is drawn on every result.

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
            result.route = self._last_route
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
        if self.config.image_stream_name not in self._ensure_store().list_streams():
            # Nothing to index; saves loading a 3.7 GB model to find that out.
            self._index_progress = f"no {self.config.image_stream_name!r} stream"
            logger.warning("visual index skipped: %s", self._index_progress)
            return
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
    @_serialized
    def find_in_memory(self, query: str) -> SkillResult:
        """Find where something was seen, and when it was last seen, and highlight it in VR.

        Answers "where did I see X" and "when did you last see X" by comparing
        the phrase against precomputed SigLIP 2 embeddings of the recording's
        camera frames, reduced to one marker per distinct location.

        Args:
            query: What to look for, e.g. "a car" or "a whiteboard".
        """
        started = time.monotonic()
        phrase = search_phrase(query)
        if not phrase:
            return SkillResult.fail("INVALID_QUERY", "The query text is empty")
        not_ready = self._index_not_ready()
        if not_ready is not None:
            return not_ready

        places, located = self._search_places(phrase)
        if not places:
            return SkillResult.fail("NOT_FOUND", f"Nothing in the recording matches {phrase!r}")

        latest = max(places, key=lambda place: place.ts)
        answer = (
            f"Found {phrase} in {len(places)} place(s), best match {places[0].similarity:+.3f}. "
            f"Last seen {self._describe_time(latest.ts)} at {_xyz_text(latest.position)}."
        )
        query_id = self._show_places(phrase, places, located, answer)

        return SkillResult(
            success=True,
            message=answer,
            duration_ms=(time.monotonic() - started) * 1000,
            metadata={
                "query_id": query_id,
                "query": phrase,
                "places": [
                    {
                        "position": place.position,
                        "similarity": place.similarity,
                        "views": place.views,
                        "ts": place.ts,
                        "offset_s": self._offset_s(place.ts),
                        "frame_id": place.source_id,
                    }
                    for place in places
                ],
                "located": located,
                "last_seen": {
                    "ts": latest.ts,
                    "offset_s": self._offset_s(latest.ts),
                    "frame_id": latest.source_id,
                    "position": latest.position,
                },
            },
        )

    @skill
    @_serialized
    def show_frames_in_memory(self, query: str, count: int = 5) -> SkillResult:
        """Show the camera frames that best match a query, each hung where its camera stood.

        Answers "show me the top N images of X". Frames are ranked by SigLIP 2
        patch similarity, so neighboring frames of one view can rank together.

        Args:
            query: What to look for, e.g. "a fountain".
            count: How many frames to show, 1 to 12.
        """
        started = time.monotonic()
        phrase = search_phrase(query)
        if not phrase:
            return SkillResult.fail("INVALID_QUERY", "The query text is empty")
        not_ready = self._index_not_ready()
        if not_ready is not None:
            return not_ready

        count = max(1, min(int(count), MAX_QUERY_FRAMES))
        frames = self._ensure_visual_index().search(phrase, k=count)
        if not frames:
            return SkillResult.fail("NOT_FOUND", f"Nothing in the recording matches {phrase!r}")

        newest = max(frames, key=lambda frame: frame.ts)
        answer = (
            f"Showing the {len(frames)} frames that best match {phrase}; "
            f"the most recent is from {self._describe_time(newest.ts)}."
        )
        result = MemoryQueryResult(
            answer=answer,
            focus_point=frames[0].position,
            points=[
                HighlightPoint(
                    position=frame.position,
                    label=f"{phrase} #{rank} ({frame.similarity:+.3f}, {self._describe_time(frame.ts)})",
                )
                for rank, frame in enumerate(frames, 1)
            ],
            observation_ids=self._markers_near([frame.position for frame in frames]),
        )
        query_id = self._publish_query_result(result)
        self._publish_query_images(query_id, phrase, frames)

        return SkillResult(
            success=True,
            message=answer,
            duration_ms=(time.monotonic() - started) * 1000,
            metadata={
                "query_id": query_id,
                "query": phrase,
                "frames": [
                    {
                        "rank": rank,
                        "frame_id": frame.source_id,
                        "ts": frame.ts,
                        "offset_s": self._offset_s(frame.ts),
                        "similarity": frame.similarity,
                        "position": frame.position,
                    }
                    for rank, frame in enumerate(frames, 1)
                ],
            },
        )

    # ---- spatial memory interface ------------------------------------------

    @rpc
    @_serialized
    def query_by_text(self, text: str, limit: int = 5) -> list[dict]:  # type: ignore[type-arg]
        """The places matching a text, best first, as spatial memory results.

        Each result carries a distance of 1 - similarity and one metadata entry
        with the world position and heading of the camera that saw it. The
        places are highlighted in the viewer as a side effect.
        """
        phrase = search_phrase(text)
        if not phrase or self._index_not_ready() is not None:
            return []
        places, located = self._search_places(phrase)
        if not places:
            return []
        self._show_places(
            phrase,
            places,
            located,
            f"Found {phrase} in {len(places)} place(s), best match {places[0].similarity:+.3f}.",
        )
        return [
            {
                "id": place.source_id,
                "distance": 1.0 - place.similarity,
                "metadata": [
                    {
                        "pos_x": place.position[0],
                        "pos_y": place.position[1],
                        "pos_z": place.position[2],
                        "rot_z": _heading(place.position, place.orientation),
                        "ts": place.ts,
                        "frame_id": place.source_id,
                    }
                ],
            }
            for place in places[:limit]
        ]

    @rpc
    def tag_location(self, robot_location: RobotLocation) -> bool:
        self._tagged_locations[robot_location.name.strip().lower()] = robot_location
        return True

    @rpc
    def query_tagged_location(self, query: str) -> RobotLocation | None:
        return self._tagged_locations.get(query.strip().lower())

    def _index_not_ready(self) -> SkillResult | None:
        """Why a search cannot run yet, or None when the index is complete.

        A search reads the index stream on the connection the builder is
        writing to, so nothing is served while the build runs.
        """
        progress = self._index_progress
        if progress.startswith("building"):
            return SkillResult.fail(
                "INDEX_NOT_READY",
                f"The SigLIP index for {self.config.store_path} is still being built "
                f"({progress}); the terminal reports when it is ready.",
            )
        if progress.startswith("failed"):
            return SkillResult.fail(
                "INDEX_FAILED", f"The SigLIP index build {progress}; restart to rebuild it."
            )
        if self._ensure_visual_index().count() > 0:
            return None
        return SkillResult.fail(
            "INDEX_NOT_READY",
            f"The SigLIP index for {self.config.store_path} holds no frames "
            f"({progress}). Build it with "
            f"`python -m dimos.teleop.memory_world.visual_search {self.config.store_path}`.",
        )

    def _search_places(self, phrase: str) -> tuple[list[Place], bool]:
        """Distinct places for a phrase: on the object through depth, else where it was seen from."""
        places = self._locate_objects(phrase)
        if places:
            return places, True
        places = cluster_places(
            self._ensure_visual_index().search(phrase, k=self.config.search_top_k),
            radius=self.config.place_radius_m,
            max_places=self.config.max_places,
        )
        return places, False

    def _show_places(self, phrase: str, places: list[Place], located: bool, answer: str) -> str:
        """Highlight the places and the frames behind them in every viewer."""
        result = MemoryQueryResult(
            answer=answer,
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
        query_id = self._publish_query_result(result)
        self._publish_query_images(query_id, phrase, places)
        return query_id

    def _offset_s(self, ts: float) -> float:
        if self._recording_start_ts is None:
            images = self._ensure_store().streams[self.config.image_stream_name]
            self._recording_start_ts = float(images.first().ts)
        return ts - self._recording_start_ts

    def _describe_time(self, ts: float) -> str:
        minutes, seconds = divmod(int(self._offset_s(ts)), 60)
        return f"{minutes}:{seconds:02d} into the recording"

    def _publish_query_images(self, query_id: str, phrase: str, places: list[Place]) -> None:
        """Send the frame behind each place, posed where its camera stood.

        The header carries the camera position, its forward and up directions
        and its field of view, so the viewer can hang the picture on the
        camera's image plane.
        """
        store = self._ensure_store()
        images = store.streams[self.config.image_stream_name]
        hfov_deg = self._camera_hfov()

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

    def _camera_hfov(self) -> float:
        """Horizontal field of view of the image stream, from camera_info when present."""
        if self._camera_hfov_deg is None:
            self._camera_hfov_deg = 70.0
            if self.config.camera_info_stream_name is not None:
                store = self._ensure_store()
                info = store.streams[self.config.camera_info_stream_name].first().data
                self._camera_hfov_deg = float(
                    np.degrees(2.0 * np.arctan2(info.width / 2.0, info.K[0]))
                )
        return self._camera_hfov_deg

    # ---- timeline replay -----------------------------------------------------

    def _ensure_replay(self) -> VoxelReplay:
        """The recorded timeline, or LookupError until the mapper has produced one."""
        with self._replay_lock:
            if self._replay is None:
                raise LookupError(f"replay {self._replay_progress}")
            return self._replay

    def _open_replay(self) -> None:
        """Keep the timeline of an earlier run when it covers the whole recording."""
        store = self._ensure_store()
        with self._replay_lock:
            if self._replay is not None:
                return
            self._replay_progress = "waiting for the mapper"
            if not VoxelReplay.matches(
                store,
                voxel_size=self.config.voxel_size,
                keyframe_interval_s=self.config.replay_keyframe_interval_s,
            ):
                return
            replay = VoxelReplay(store, z_min=self.config.map_z_min, z_max=self.config.map_z_max)
            if not replay.covers(self._recording_end() - REPLAY_COMPLETE_MARGIN_S):
                return
            self._replay = replay
            self._replay_complete = True
            self._replay_floor = self._floor_of(replay.keyframes.last().data.points_f32())
            self._replay_progress = "ready"
            logger.info("timeline from an earlier run covers the recording, keeping it")

    def _record_snapshot(self, xyz: np.ndarray, ts: float) -> None:
        """Fold a mapper snapshot into the timeline, unless a complete one is on disk."""
        with self._replay_lock:
            if self._replay_complete:
                return
            store = self._ensure_store()
            with self._replay_read_lock:
                if self._recorder is None:
                    self._recorder = ReplayRecorder(
                        store,
                        voxel_size=self.config.voxel_size,
                        keyframe_interval_s=self.config.replay_keyframe_interval_s,
                    )
                    self._replay_progress = "recording"
                    logger.info("recording the mapper's timeline into %s", self.config.store_path)
                keyframe = self._recorder.add_snapshot(xyz, ts)
                if keyframe:
                    self._replay_floor = self._floor_of(xyz)
                if self._replay is None:
                    self._replay = VoxelReplay(
                        store, z_min=self.config.map_z_min, z_max=self.config.map_z_max
                    )
                else:
                    self._replay.extend(ts, keyframe)

    def _recording_end(self) -> float:
        """Stamp of the last lidar scan, or infinity without a lidar stream."""
        store = self._ensure_store()
        if self.config.lidar_stream_name not in store.list_streams():
            return math.inf
        return float(store.streams[self.config.lidar_stream_name].last().ts)

    @staticmethod
    def _floor_of(points: np.ndarray) -> float:
        return float(np.percentile(points[:, 2], 7)) if len(points) else 0.0

    @staticmethod
    def _replay_cache_headers(replay: VoxelReplay, key: object) -> dict[str, str]:
        """Browsers revalidate against the build stamp, so a rebuilt timeline is never stale."""
        built_at = replay.index.stream_tags["built_at"]
        return {"ETag": f'"{built_at}-{key}"', "Cache-Control": "no-cache"}

    def _replay_read(self, fn: Any, *args: Any) -> Any:
        """Run one store-reading replay call at a time."""
        with self._replay_read_lock:
            return fn(*args)

    def _replay_index_json(self) -> dict[str, Any]:
        """Scan and keyframe stamps plus what the viewer needs to draw and seek."""
        replay = self._ensure_replay()
        store = self._ensure_store()
        if self._replay_frames_json is None:
            # Listing every camera stamp is a pass over the image stream, so once.
            frames: list[float] = []
            if self.config.image_stream_name in store.list_streams():
                images = store.streams[self.config.image_stream_name]
                frames = [round(float(obs.ts), 4) for obs in images]
            self._replay_frames_json = frames
        payload = replay.index.to_json()
        payload["complete"] = self._replay_complete or replay.covers(
            self._recording_end() - REPLAY_COMPLETE_MARGIN_S
        )
        payload["frames"] = self._replay_frames_json
        payload["hfov_deg"] = self._camera_hfov()
        payload["height"] = {
            "floor": self._replay_floor,
            "span": float(self.config.height_ramp_span_m),
        }
        payload["colors"] = (HEIGHT_COLOR_STOPS / 255.0).round(4).tolist()
        return payload

    def _replay_frame(self, ts: float) -> tuple[bytes, dict[str, Any]] | None:
        """JPEG and camera pose of the image nearest *ts*, kept in a small LRU."""
        images = self._ensure_store().streams[self.config.image_stream_name]
        candidates = list(images.at(ts, tolerance=0.25))
        if not candidates:
            return None
        obs = min(candidates, key=lambda o: abs(float(o.ts) - ts))
        key = int(obs.id)
        cached = self._replay_frames.get(key)
        if cached is not None:
            self._replay_frames.move_to_end(key)
            return cached
        jpeg = self._encode_jpeg(
            obs.data, self.config.replay_frame_max_size, self.config.replay_frame_jpeg_quality
        )
        meta: dict[str, Any] = {"ts": round(float(obs.ts), 4), "hfov_deg": self._camera_hfov()}
        camera = self._camera_pose_of(obs)
        if camera is not None:
            meta.update(
                position=[float(v) for v in camera[:3, 3]],
                forward=[float(v) for v in camera[:3, 2]],
                up=[float(v) for v in -camera[:3, 1]],
            )
        self._replay_frames[key] = (jpeg, meta)
        while len(self._replay_frames) > 600:
            self._replay_frames.popitem(last=False)
        return jpeg, meta

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
                            camera_orientation=frame.orientation,
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

    # ---- route from the planner --------------------------------------------

    def _on_path(self, path: NavPath) -> None:
        """Draw the planner's latest route on the active answer. An empty path clears it."""
        points = [
            (pose.position.x, pose.position.y, pose.position.z + ROUTE_LIFT_M)
            for pose in path.poses
        ]
        route = (
            HighlightPath(points=points, label="Route to answer", color="#64ff8f")
            if len(points) >= 2
            else None
        )
        with self._clients_lock:
            if route == self._last_route:
                return
            self._last_route = route
            active = self._active_query_result
        logger.info("route from the planner: %s", f"{len(points)} waypoints" if route else "none")
        if active is None:
            self._publish_query_result(
                MemoryQueryResult(answer="Route to the goal" if route else "No route to the goal")
            )
            return
        with self._clients_lock:
            self._query_revision += 1
            payload = dict(
                active,
                route=route.model_dump(mode="json") if route is not None else None,
                revision=self._query_revision,
            )
            self._active_query_result = payload
        self._broadcast(encode_text("query_result", **payload))

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
        if self.global_map.transport is not None:
            self.register_disposable(Disposable(self.global_map.subscribe(self._on_global_map)))
        if self.path.transport is not None:
            self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
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
        self._prepare_thread = threading.Thread(
            target=self._prepare, daemon=True, name="MemoryWorldPrepare"
        )
        self._prepare_thread.start()

    def _prepare(self) -> None:
        """Build what every client needs, in order of urgency, on one thread.

        Each step is a pass over the recording; run together they starve each
        other (on an mcap every pass decompresses the image chunks), so the
        world cache goes first, the replay second and the slow SigLIP index
        last. A client that connects mid-way waits on the world cache lock.
        """
        try:
            self._ensure_world_cache()
        except Exception:
            logger.exception("world cache build failed")
        try:
            self._open_replay()
        except Exception:
            logger.exception("opening the recorded timeline failed")
        if self.config.build_image_index_on_start:
            self._build_visual_index()

    @rpc
    def stop(self) -> None:
        with self._map_push_lock:
            if self._map_push_timer is not None:
                self._map_push_timer.cancel()
                self._map_push_timer = None
        try:
            if self._web_server is not None:
                self._web_server.shutdown()
            if self._web_server_thread is not None:
                self._web_server_thread.join(timeout=3)
                self._web_server_thread = None
            if self._prepare_thread is not None:
                self._prepare_thread.join(timeout=10)
                self._prepare_thread = None
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
