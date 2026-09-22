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

1. Push the recording's map as one binary frame (positions + per-point RGB):
   the stored final map at once when an earlier run mapped the recording,
   otherwise the ray tracing mapper's map, again whenever it grows.
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
from collections import deque
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
from fastapi import HTTPException, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from langchain_core.messages import BaseMessage
import numpy as np
from pydantic import Field as PydanticField, ValidationError
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.agents.utils import ChatEntry, chat_entries
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory.store.base import Store
from dimos.memory.transform import throttle
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.teleop.memory_world.camera import CameraModel, Rectifier
from dimos.teleop.memory_world.messages import (
    MSG_IMAGE_POSES,
    MSG_IMAGE_THUMBNAIL,
    MSG_ODOM_TRAIL,
    MSG_POINT_CLOUD,
    MSG_QUERY_IMAGE,
    decode_text,
    encode_binary,
    encode_text,
)
from dimos.teleop.memory_world.objects import LocateConfig, Look, locate, render_depth
from dimos.teleop.memory_world.query import (
    MEMORY_ANALYSIS_BOOTSTRAP,
    RESULT_SENTINEL,
    HighlightBox,
    HighlightPath,
    HighlightPoint,
    MemoryQueryResult,
    validation_summary,
)
from dimos.teleop.memory_world.recording import detect_streams, open_recording
from dimos.teleop.memory_world.replay import (
    ReplayRecorder,
    final_map,
    timeline_end,
    timeline_matches,
)
from dimos.teleop.memory_world.tf_tree import TfTree, pose_matrix, quaternion_from_matrix
from dimos.teleop.memory_world.visual_search import (
    SIGLIP2_MODEL_NAME,
    Place,
    VisualMemoryIndex,
    body_style_quaternion,
    cluster_places,
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
# Chat rows replayed to a viewer that connects mid-conversation.
CHAT_HISTORY = 400


def _xyz_text(position: tuple[float, float, float]) -> str:
    return "({:.1f}, {:.1f}, {:.1f})".format(*position)


def _heading(
    position: tuple[float, float, float], orientation: tuple[float, float, float, float]
) -> float:
    """Yaw of an optical camera pose's forward axis, in world radians."""
    forward = pose_matrix(position, orientation)[:3, 2]
    return float(np.arctan2(forward[1], forward[0]))


def _merge_results(active: dict[str, Any], fresh: dict[str, Any]) -> dict[str, Any]:
    """The fresh result on top of the canvas so far, each list capped at its schema limit."""
    merged = dict(active)
    for key in ("regions", "boxes", "evidence_paths", "points", "observation_ids"):
        combined = list(active.get(key) or []) + list(fresh.get(key) or [])
        if key == "observation_ids":
            combined = list(dict.fromkeys(combined))
        merged[key] = combined[-_list_cap(key) :]
    merged["answer"] = fresh["answer"]
    merged["focus_point"] = fresh.get("focus_point") or active.get("focus_point")
    merged["route"] = fresh.get("route")
    return merged


def _list_cap(key: str) -> int:
    return max(
        getattr(meta, "max_length", 0) or 0 for meta in MemoryQueryResult.model_fields[key].metadata
    )


def _navigation_goal(place: Place) -> dict[str, float]:
    """Where to stand for a place, facing it.

    A measured object's position is on the object, inside the map's voxels, so the
    goal is where the camera stood when it saw it, which the robot has already
    walked. An unmeasured place is that spot already.
    """
    if place.extent is not None and place.camera_position is not None:
        stand = place.camera_position
        yaw = float(np.arctan2(place.position[1] - stand[1], place.position[0] - stand[0]))
    else:
        stand = place.position
        yaw = _heading(place.position, place.orientation)
    return {"pos_x": stand[0], "pos_y": stand[1], "pos_z": stand[2], "rot_z": yaw}


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
# The stored map stands in for a live mapper: republished often at first so a
# planner that starts later still gets it, then rarely. Its tf, the robot's
# final pose, is published at the rate a robot would.
MAP_PUBLISH_WARMUP_S = 60.0
MAP_PUBLISH_WARMUP_INTERVAL_S = 10.0
MAP_PUBLISH_INTERVAL_S = 60.0
TF_PUBLISH_INTERVAL_S = 0.5
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
    # The detector path: OWLv2 on the best frame of each place, its box placed
    # off the depth stream or, without one, off depth rendered from the lidar map.
    locate_threshold: float = PydanticField(default=0.5, gt=0.0, lt=1.0)
    # Narrow each detector box to an EdgeTAM mask before measuring the map under it.
    segment_boxes: bool = True
    locate_attempts: int = PydanticField(default=2, ge=1)
    locate_max_depth_m: float = PydanticField(default=20.0, gt=0.0)
    # Intrinsics for a recording without a camera_info stream: fx, fy, cx, cy
    # at the recorded resolution, and the distortion of the named model.
    camera_intrinsics: tuple[float, float, float, float] | None = None
    camera_distortion: tuple[float, ...] = ()
    camera_distortion_model: Literal["equidistant", "plumb_bob"] = "equidistant"
    # ---- the stored map -------------------------------------------------------
    # The mapper's snapshots are written into the recording once as keyframe
    # and diff streams (see replay.py). A later run loads the map they end on.
    replay_keyframe_interval_s: float = PydanticField(default=30.0, gt=0.0)
    # The frame whose final recorded pose stands in for the robot's position.
    robot_frame: str = "base_link"


class MemoryWorldModule(Module):
    """VR memory-world module.

    See :mod:`dimos.teleop.memory_world` for the architectural overview.
    """

    config: MemoryWorldConfig

    global_map: In[PointCloud2]
    path: In[NavPath]
    # The stored map and the robot's final pose, published for the planner
    # when no mapper runs.
    map: Out[PointCloud2]
    tf: Out[TFMessage]
    # The agent's conversation, shared with the human CLI: its messages come in,
    # a viewer's typed questions go out.
    agent: In[BaseMessage]
    agent_idle: In[bool]
    human_input: Out[str]

    def __init__(self, **kwargs: Any) -> None:
        self._world_clients: set[_ClientConn] = set()
        self._clients_lock = threading.Lock()
        self._chat_history: deque[ChatEntry] = deque(maxlen=CHAT_HISTORY)
        self._agent_is_idle = True

        self._store: Store | None = None
        # Cached payloads so reconnects are cheap.
        self._cached_cloud: tuple[dict[str, Any], bytes] | None = None
        self._cached_image_poses: tuple[dict[str, Any], bytes] | None = None
        # Per-pose JPEG thumbnails parallel to image_poses indices.
        self._cached_thumbnails: list[bytes] | None = None
        self._cached_odom: tuple[dict[str, Any], bytes] | None = None
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
        self._recorder: ReplayRecorder | None = None
        self._map_complete = False
        self._map_opened = False
        self._map_lock = threading.Lock()
        # The store's sqlite connection is not safe to read from two threads at once.
        self._store_read_lock = threading.Lock()
        self._map_progress = "not started"
        self._map_publisher: threading.Thread | None = None
        self._located: dict[str, list[Place]] = {}
        self._owlv2: Any = None
        self._edge_tam: Any = None
        self._edge_tam_unavailable = False
        self._models_lock = threading.Lock()
        self._map_centers: np.ndarray | None = None
        self._camera_hfov_deg: float | None = None
        self._active_query_result: dict[str, Any] | None = None
        self._active_query_images: list[tuple[dict[str, Any], bytes]] = []
        # An agent turn keeps every result it publishes on one canvas until the
        # agent goes idle. Outside a turn each result replaces the last.
        self._turn_open = False
        self._turn_query_id: str | None = None
        self._query_revision = 0
        self._last_route: HighlightPath | None = None
        self._map_push_lock = threading.Lock()
        self._map_push_timer: threading.Timer | None = None
        self._latest_map: np.ndarray | None = None
        # The mapper can publish faster than a snapshot folds in. Only the newest
        # waits; the worker below folds it into the map push and the timeline.
        self._snapshot_pending: tuple[np.ndarray, float] | None = None
        self._snapshot_wakeup = threading.Event()
        self._snapshot_thread: threading.Thread | None = None
        self._stopping = False
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
                cloud = self._cached_cloud
            # Before the mapper's first emission there is no map yet; it
            # reaches this client with the next push.
            if cloud is not None:
                self._send_map(conn.send_threadsafe, cloud)

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
                chat_history = list(self._chat_history)
                agent_is_idle = self._agent_is_idle
            conn.send_threadsafe(encode_text("chat_history", entries=chat_history))
            conn.send_threadsafe(encode_text("agent_idle", idle=agent_is_idle))
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
            self._snapshot_pending = (np.asarray(xyz, dtype=np.float32), float(cloud.ts))
        self._snapshot_wakeup.set()

    def _snapshot_worker(self) -> None:
        while not self._stopping:
            self._snapshot_wakeup.wait()
            self._snapshot_wakeup.clear()
            self._fold_snapshot()

    def _fold_snapshot(self) -> None:
        """Fold the newest mapper snapshot into the map push and the timeline."""
        with self._map_push_lock:
            pending, self._snapshot_pending = self._snapshot_pending, None
            if pending is not None:
                self._latest_map = pending[0]
        if pending is None:
            return
        self._schedule_map_push()
        try:
            self._record_snapshot(*pending)
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
        with self._world_cache_lock:
            first = self._cached_cloud is None
            self._cached_cloud = packed
        if first:
            logger.info("first map to the viewers: n=%d", packed[0]["n"])
        self._send_map(self._broadcast, packed)

    @staticmethod
    def _send_map(send: Callable[[bytes | str], None], cloud: tuple[dict[str, Any], bytes]) -> None:
        header, payload = cloud
        send(encode_text("world_summary", **header))
        send(encode_binary(MSG_POINT_CLOUD, header, payload))

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
        SqliteStore), ``np`` (NumPy), ``viewer_position``, ``route`` (the
        planner's current route as world xyz points, or None) and ``objects``
        (every object the detector has located so far, with position and size) available. Inspect
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
            route = [list(p) for p in self._last_route.points] if self._last_route else None
        try:
            completed = subprocess.run(
                [
                    sys.executable,
                    "-c",
                    MEMORY_ANALYSIS_BOOTSTRAP,
                    self.config.store_path,
                    json.dumps(viewer_position),
                    json.dumps(route),
                    json.dumps(self._located_json()),
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
        except ValidationError as exc:
            return SkillResult.fail(
                "EXECUTION_FAILED", f"Invalid memory result: {validation_summary(exc)}"
            )
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
                "boxes": len(result.boxes),
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
        """Send a result to every connected viewer and remember it for reconnects.

        Inside an agent turn the result joins the turn's canvas, so the route, the
        boxes and the frames of successive tool calls all stay up together.
        """
        with self._clients_lock:
            result.route = self._last_route
            fresh = result.model_dump(mode="json")
            active = self._active_query_result
            if (
                self._turn_open
                and active is not None
                and self._turn_query_id is not None
                and active.get("query_id") == self._turn_query_id
            ):
                query_id = self._turn_query_id
                payload = _merge_results(active, fresh)
            else:
                query_id = uuid.uuid4().hex
                payload = fresh
                self._active_query_images = []
                if self._turn_open:
                    self._turn_query_id = query_id
            self._query_revision += 1
            payload.update(query_id=query_id, revision=self._query_revision)
            self._active_query_result = payload
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
        # Warm the index and the text model here, off the request path: cold they
        # add half a minute to whichever query comes first, the demoed one.
        with self._search_lock:
            index.warm()
        index.model.embed_text("warmup")
        logger.info("search path warm")

    def _warm_detectors(self) -> None:
        """Load the detector, the segmenter and Whisper and run each once, off the request path."""
        try:
            if self._camera_model() is not None:
                frame = Image.from_opencv(np.zeros((64, 64, 3), np.uint8), ts=0.0)
                self._owlv2_detector().query_detections(frame, ["a thing"], 0.5)
                segmenter = self._segmenter()
                if segmenter is not None:
                    box = Detection2DBBox(
                        bbox=(8.0, 8.0, 56.0, 56.0),
                        track_id=-1,
                        class_id=0,
                        confidence=1.0,
                        name="a thing",
                        ts=0.0,
                        image=frame,
                    )
                    segmenter.segment(ImageDetections2D(image=frame, detections=[box]))
                logger.info("detector path warm")
        except Exception:
            logger.exception("detector warm-up failed")
        try:
            _ = self.whisper
        except ImportError as error:
            logger.warning("voice queries unavailable: %s", error)

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
        if located:
            measured = sum(1 for place in places if place.extent is not None)
            answer = (
                f"The detector measured {measured} {phrase} object(s) on the map, plus "
                f"{len(places) - measured} sighting(s) it could not place, which may repeat "
                f"those or each other. Best confidence {places[0].similarity:.2f}. Last seen "
                f"{self._describe_time(latest.ts)} at {_xyz_text(latest.position)}."
            )
        else:
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
                        "extent": place.extent,
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
                        **_navigation_goal(place),
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
                    f"{'s' if place.views != 1 else ''}"
                    + (f", {place.extent[2]:.1f} m tall" if place.extent else "")
                    + ")",
                    radius=self.config.object_radius_m if place.extent is not None else None,
                    extent=place.extent,
                    yaw=place.yaw,
                )
                for place in places
            ],
            boxes=[
                HighlightBox(
                    center=place.position, extent=place.extent, yaw=place.yaw, label=phrase
                )
                for place in places
                if place.extent is not None
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
            offset = len(self._active_query_images)
            for header, _ in sent:
                header["index"] += offset
            self._active_query_images.extend(sent)
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

    # ---- objects ------------------------------------------------------------------

    def _camera_model(self) -> CameraModel | None:
        """The image stream's intrinsics, from camera_info or the configured values."""
        store = self._ensure_store()
        if self.config.camera_info_stream_name in store.list_streams():
            return CameraModel.from_camera_info(
                store.streams[self.config.camera_info_stream_name].first().data
            )
        if self.config.camera_intrinsics is None:
            return None
        fx, fy, cx, cy = self.config.camera_intrinsics
        first = store.streams[self.config.image_stream_name].first().data
        return CameraModel(
            width=int(first.width),
            height=int(first.height),
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            distortion=tuple(self.config.camera_distortion),
            model=self.config.camera_distortion_model,
        )

    # ---- the stored map --------------------------------------------------------

    def _open_map(self) -> None:
        """Show the map of an earlier run at once when its timeline covers the whole recording."""
        store = self._ensure_store()
        with self._map_lock:
            if self._map_opened:
                return
            self._map_opened = True
            self._map_progress = "waiting for the mapper"
            if not timeline_matches(
                store,
                voxel_size=self.config.voxel_size,
                keyframe_interval_s=self.config.replay_keyframe_interval_s,
            ):
                return
            if timeline_end(store) < self._recording_end() - REPLAY_COMPLETE_MARGIN_S:
                return
            with self._store_read_lock:
                centers, _ = final_map(store, self.config.voxel_size)
            self._map_complete = True
            self._map_progress = "ready"
            self._map_centers = centers
        logger.info("map from an earlier run covers the recording: %d voxels", len(centers))
        with self._map_push_lock:
            self._latest_map = centers
        self._push_map()
        self._start_map_publisher(centers)

    def _record_snapshot(self, xyz: np.ndarray, ts: float) -> None:
        """Fold a mapper snapshot into the timeline, unless a complete one is on disk."""
        self._open_map()
        with self._map_lock:
            if self._map_complete:
                return
            store = self._ensure_store()
            with self._store_read_lock:
                if self._recorder is None:
                    self._recorder = ReplayRecorder(
                        store,
                        voxel_size=self.config.voxel_size,
                        keyframe_interval_s=self.config.replay_keyframe_interval_s,
                    )
                    self._map_progress = "recording"
                    logger.info("recording the mapper's timeline into %s", self.config.store_path)
                self._recorder.add_snapshot(xyz, ts)

    def _recording_end(self) -> float:
        """Stamp of the last lidar scan, or infinity without a lidar stream."""
        store = self._ensure_store()
        if self.config.lidar_stream_name not in store.list_streams():
            return math.inf
        return float(store.streams[self.config.lidar_stream_name].last().ts)

    def _final_robot_pose(self) -> np.ndarray | None:
        """world_T_robot at the end of the recording, from tf."""
        tree = self._tf_tree()
        if tree is None:
            return None
        span = tree.span(self.config.world_frame, self.config.robot_frame)
        if span is None:
            return None
        return self._frame_pose_at(self.config.robot_frame, span[1])

    def _start_map_publisher(self, centers: np.ndarray) -> None:
        """Feed the planner the stored map and the robot's final pose, as a robot would."""
        if self.map.transport is None and self.tf.transport is None:
            return
        pose = self._final_robot_pose()
        if pose is None:
            logger.warning(
                "no final %r pose in tf; the planner has no start", self.config.robot_frame
            )
        else:
            logger.info("robot's final pose stands at %s", _xyz_text(tuple(pose[:3, 3])))
        self._map_publisher = threading.Thread(
            target=self._publish_stored_map,
            args=(centers, pose),
            daemon=True,
            name="MemoryWorldMapPublisher",
        )
        self._map_publisher.start()

    def _publish_stored_map(self, centers: np.ndarray, pose: np.ndarray | None) -> None:
        started = time.monotonic()
        last_map = -math.inf
        transform = None
        if pose is not None:
            translation = Vector3(*(float(v) for v in pose[:3, 3]))
            rotation = Quaternion(*quaternion_from_matrix(pose[:3, :3]))
            transform = (translation, rotation)
        while not self._stopping:
            now = time.time()
            if transform is not None and self.tf.transport is not None:
                self.tf.publish(
                    TFMessage(
                        Transform(
                            *transform, self.config.world_frame, self.config.robot_frame, ts=now
                        )
                    )
                )
            warm = time.monotonic() - started < MAP_PUBLISH_WARMUP_S
            interval = MAP_PUBLISH_WARMUP_INTERVAL_S if warm else MAP_PUBLISH_INTERVAL_S
            if self.map.transport is not None and time.monotonic() - last_map >= interval:
                self.map.publish(
                    PointCloud2.from_numpy(centers, frame_id=self.config.world_frame, timestamp=now)
                )
                last_map = time.monotonic()
            time.sleep(TF_PUBLISH_INTERVAL_S)

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
        """Show the detector the best frame of each place and measure what it boxes.

        Empty when the recording has no intrinsics, when nothing can place a
        box (no depth stream and no complete lidar map), or when the detector
        refuses every frame.
        """
        cached = self._located.get(phrase)
        if cached is not None:
            return cached
        camera = self._camera_model()
        if camera is None:
            return []
        if self.config.depth_stream_name is None and self._map_centers is None:
            return []
        ranked = self._ensure_visual_index().search(phrase, k=self.config.search_top_k)
        rectify = Rectifier(camera)
        images = self._ensure_store().streams[self.config.image_stream_name]
        places_looks: list[list[Look]] = []
        for group in self._frames_by_place(ranked):
            looks = [self._look_at(candidate, images, rectify, camera) for candidate in group]
            if any(look is not None for look in looks):
                places_looks.append([look for look in looks if look is not None])
        config = LocateConfig(
            threshold=self.config.locate_threshold,
            attempts=self.config.locate_attempts,
            max_depth_m=self.config.locate_max_depth_m,
            merge_m=self.config.object_radius_m,
        )
        found = locate(
            phrase,
            places_looks,
            self._owlv2_detector(),
            camera,
            segmenter=self._segmenter(),
            config=config,
        )
        places = [
            Place(
                position=item.centre,
                similarity=item.confidence,
                source_id=item.frame_id,
                ts=item.ts,
                orientation=quaternion_from_matrix(item.world_t_camera[:3, :3]),
                camera_position=tuple(float(v) for v in item.world_t_camera[:3, 3]),  # type: ignore[arg-type]
                views=item.views,
                extent=item.extent,
                yaw=item.yaw,
            )
            for item in found
        ]
        self._located[phrase] = places
        logger.info(
            "located %d %r object(s) from %d place(s) shown to the detector",
            len(places),
            phrase,
            len(places_looks),
        )
        return places

    def _frames_by_place(self, ranked: list[Place]) -> list[list[Place]]:
        """Ranked frames grouped by where they were taken, best first, a few per place."""
        groups: list[list[Place]] = []
        for candidate in ranked:
            here = np.asarray(candidate.position)
            for group in groups:
                if (
                    np.linalg.norm(here - np.asarray(group[0].position))
                    < self.config.place_radius_m
                ):
                    if len(group) < self.config.locate_attempts:
                        group.append(candidate)
                    break
            else:
                if len(groups) < self.config.locate_frames:
                    groups.append([candidate])
        return groups

    def _look_at(
        self, candidate: Place, images: Any, rectify: Rectifier, camera: CameraModel
    ) -> Look | None:
        try:
            obs = images.at(candidate.ts, tolerance=0.005).first()
        except LookupError:
            return None
        pose = self._camera_pose_of(obs)
        if pose is None:
            return None
        image = Image.from_opencv(rectify(obs.data.to_opencv()), ts=float(obs.ts))
        return Look(int(obs.id), float(obs.ts), image, pose, self._depth_for(obs.ts, pose, camera))

    def _depth_for(self, ts: float, pose: np.ndarray, camera: CameraModel) -> np.ndarray | None:
        """Meters on the camera's grid: the recorded depth, else the lidar map through the camera."""
        if self.config.depth_stream_name is not None:
            depth_stream = self._ensure_store().streams[self.config.depth_stream_name]
            try:
                depth = depth_stream.at(ts, tolerance=self.config.depth_tolerance_s).first()
            except LookupError:
                return None
            return np.asarray(depth.data.data, dtype=np.float32) * 0.001
        if self._map_centers is None:
            return None
        return render_depth(
            self._map_centers,
            pose,
            camera,
            max_depth_m=self.config.locate_max_depth_m,
            voxel_m=self.config.voxel_size,
        )

    def _owlv2_detector(self) -> Any:
        with self._models_lock:
            if self._owlv2 is None:
                from dimos.perception.detection.detectors.owlv2 import Owlv2Detector

                self._owlv2 = Owlv2Detector()
                logger.info("loaded OWLv2 for object localization")
            return self._owlv2

    def _segmenter(self) -> Any:
        """EdgeTAM masks for the detector's boxes, or None where it cannot run."""
        if not self.config.segment_boxes:
            return None
        with self._models_lock:
            if self._edge_tam_unavailable:
                return None
            if self._edge_tam is None:
                from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter

                try:
                    self._edge_tam = EdgeTAMImageSegmenter()
                except (RuntimeError, ImportError, OSError) as error:
                    self._edge_tam_unavailable = True
                    logger.warning("objects are measured under boxes, not masks: %s", error)
                    return None
                logger.info("loaded EdgeTAM for object masks")
            return self._edge_tam

    def _located_json(self) -> list[dict[str, Any]]:
        return [
            {
                "label": phrase,
                "position": list(place.position),
                "extent": list(place.extent) if place.extent else None,
                "height": place.extent[2] if place.extent else None,
                "yaw": place.yaw if place.extent else None,
                "confidence": place.similarity,
                "views": place.views,
                "ts": place.ts,
                "best_frame_id": place.source_id,
            }
            for phrase, places in self._located.items()
            for place in places
        ]

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
        elif kind == "ask":
            text = msg.get("text")
            if isinstance(text, str) and text.strip():
                self._ask(conn, text.strip())
        else:
            logger.warning("[client] unknown msg kind=%r full=%r", kind, msg)

    # ---- chat with the agent ---------------------------------------------

    def _ask(self, conn: _ClientConn, text: str) -> None:
        """Send a viewer's typed question to the agent, the way the human CLI does."""
        if self.human_input.transport is None:
            conn.send_threadsafe(encode_text("error", message="no agent is connected"))
            return
        with self._clients_lock:
            self._agent_is_idle = False
            self._begin_turn()
        self.human_input.publish(text)

    def _begin_turn(self) -> None:
        """Start a fresh canvas for the answer to a new question. Callers hold the lock."""
        self._turn_open = True
        self._turn_query_id = None

    def _on_agent_message(self, msg: BaseMessage) -> None:
        entries = chat_entries(msg)
        with self._clients_lock:
            self._chat_history.extend(entries)
            if any(entry.get("role") == "human" for entry in entries):
                self._begin_turn()
        for entry in entries:
            self._broadcast(encode_text("chat", **entry))

    def _on_agent_idle(self, idle: bool) -> None:
        with self._clients_lock:
            self._agent_is_idle = bool(idle)
            if idle:
                self._turn_open = False
        self._broadcast(encode_text("agent_idle", idle=bool(idle)))

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
            self._snapshot_thread = threading.Thread(
                target=self._snapshot_worker, daemon=True, name="MemoryWorldSnapshots"
            )
            self._snapshot_thread.start()
            self.register_disposable(Disposable(self.global_map.subscribe(self._on_global_map)))
        if self.path.transport is not None:
            self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.agent.transport is not None:
            self.register_disposable(Disposable(self.agent.subscribe(self._on_agent_message)))
        if self.agent_idle.transport is not None:
            self.register_disposable(Disposable(self.agent_idle.subscribe(self._on_agent_idle)))
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
        stored map goes first, the world cache second and the slow SigLIP index
        last. A client that connects mid-way waits on the world cache lock. The
        detector and segmenter load on their own thread meanwhile, since they
        never touch the recording.
        """
        threading.Thread(
            target=self._warm_detectors, daemon=True, name="MemoryWorldWarmDetectors"
        ).start()
        try:
            self._open_map()
        except Exception:
            logger.exception("loading the stored map failed")
        try:
            self._ensure_world_cache()
        except Exception:
            logger.exception("world cache build failed")
        if self.config.build_image_index_on_start:
            self._build_visual_index()

    @rpc
    def stop(self) -> None:
        self._stopping = True
        self._snapshot_wakeup.set()
        if self._snapshot_thread is not None:
            self._snapshot_thread.join(timeout=5)
            self._snapshot_thread = None
        if self._map_publisher is not None:
            self._map_publisher.join(timeout=3)
            self._map_publisher = None
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
