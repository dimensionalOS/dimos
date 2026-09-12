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

"""Memory World module: spawns the user inside a recorded point cloud.

On connect it pushes the voxel map (positions + RGB), the camera poses as
markers with thumbnails, and the odom trail; then it answers questions and
serves the replay. Locomotion is client-side.
"""

from __future__ import annotations

import asyncio
from collections import OrderedDict
import contextlib
import gzip
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
from fastapi import HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import HTMLResponse, Response
import numpy as np
from pydantic import Field as PydanticField

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.memory.store.base import Store
from dimos.memory.transform import throttle
from dimos.teleop.memory_world.clients import ClientConn, RevalidatedStaticFiles
from dimos.teleop.memory_world.embed import EmbeddingJob
from dimos.teleop.memory_world.hyperspace_answers import HyperspaceAnswers
from dimos.teleop.memory_world.hyperspace_search import HYPERSPACE_MODEL_NAME
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
    MemoryQueryResult,
    answer_positions,
)
from dimos.teleop.memory_world.recording import (
    build_tf_tree,
    detect_streams,
    open_recording,
    pick_lidar,
    tf_root,
    usable_streams,
)
from dimos.teleop.memory_world.replay import (
    DIFF_STREAM,
    KEYFRAME_STREAM,
    SensorScan,
    VoxelReplay,
    accumulate_scans,
    build_replay_streams,
    sensor_scan,
)
from dimos.teleop.memory_world.replay_serving import HEIGHT_COLOR_STOPS, ReplayServing
from dimos.teleop.memory_world.tf_tree import TfTree, pose_matrix
from dimos.teleop.memory_world.visual_answers import VisualAnswers
from dimos.teleop.memory_world.visual_search import (
    SIGLIP2_MODEL_NAME,
    VisualMemoryIndex,
    body_style_quaternion,
    search_phrase,
)
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "web" / "static"


# A body frame (x forward, z up) seen as a camera optical frame (z forward,
# y down): the standard ROS optical rotation. Used only for recordings that
# carry no tf tree and stamp body poses on their images.
OPTICAL_FROM_BODY = pose_matrix((0.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5))


class MemoryWorldConfig(ModuleConfig):
    """Config for the Memory World."""

    store_path: str = "data/go2_bigoffice.db"
    server_port: int = 8443
    # Voxel size of the map (metres), also the rendered point size. 8 cm is
    # the ray-tracing module's own: its support gate keeps walls and floors
    # and drops the fuzz at this size, where 5 cm loses most of both.
    voxel_size: float = 0.08
    # Cap on the static cloud sent to a viewer. A building is about a million
    # voxels; the viewer's quality governor thins what it cannot draw.
    max_points: int = 1_500_000
    # Which lidar stream, and how many scans (<= 0 uses every frame). The cloud is
    # deduped by voxel_size, so more scans only costs build time. Empty picks the
    # stream whose poses agree with tf (recording.pick_lidar).
    lidar_stream_name: str = ""
    n_voxel_scans: int = 150
    # True: the scans are already in the world frame (SLAM output), so their poses
    # must not be applied twice. None detects it: a scan frame equal to world_frame or
    # a stitched *corrected* one counts as aligned, map/odom/world count as aligned
    # when tf cannot place them, and anything else is placed through tf.
    lidar_world_frame: bool | None = None
    # Heights kept from the cloud, in the recording's own frame. None keeps
    # everything: a recording can be multi-storey and its origin can be the sensor,
    # so there is no floor to assume. A [-0.2, 2.4] slab once threw away 96% of a
    # stairwell.
    map_z_min: float | None = None
    map_z_max: float | None = None
    # Height colour ramp, over the cloud's own range so a multi-storey
    # recording gets a different colour per level. Percentiles, so one stray
    # return below the building does not flatten the rest into one shade.
    height_ramp_low_percentile: float = PydanticField(default=5.0, ge=0.0, le=100.0)
    height_ramp_high_percentile: float = PydanticField(default=95.0, ge=0.0, le=100.0)
    # color_image stream is sampled for "Street View" capture-pose markers.
    # Empty detects it from the recording's message types.
    image_stream_name: str = ""
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
    # Every pose the world needs is a tf lookup at the observation's stamp. A
    # recording with no tf falls back to the body pose stamped on each image.
    tf_stream_name: str = ""  # empty detects it
    # The frame everything is placed in; empty or absent from tf, the tf root is
    # used. Scans stamped with this frame are taken as already aligned.
    world_frame: str = "world"
    # The image stream's own frame_id by default.
    camera_optical_frame: str | None = None
    # A lookup fails when the nearest tf sample is further away than this.
    tf_tolerance_s: float = PydanticField(default=0.1, gt=0.0)
    # The camera's path is drawn as a polyline sampled from tf.
    n_trail_samples: int = 400
    # The frame the viewer's orbit mode circles, sent per replay scan. Falls
    # back to the camera frame when tf does not know it.
    orbit_frame: str = "base_link"
    # Top-down density map (GTA-style minimap + ground projection). Computed
    # from the same point cloud — Z-slab histogram into a square image.
    map_image_size: int = 512
    # The top-down map is a footprint, so it takes the middle of whatever
    # height range the cloud spans — percentiles, not metres, so it works on
    # one storey or several.
    map_z_low_percentile: float = PydanticField(default=10.0, ge=0.0, le=100.0)
    map_z_high_percentile: float = PydanticField(default=90.0, ge=0.0, le=100.0)
    client_route: str = "/memory_world"
    ws_route: str = "/ws_memory_world"
    # Bind on all interfaces by default — the headset connects over Wi-Fi.
    listen_host: str = "0.0.0.0"
    background_mode: Literal["black", "passthrough"] = "black"
    memory_analysis_max_output_chars: int = PydanticField(default=64_000, gt=0)
    # ---- spoken "where did I see X" search --------------------------------
    # SigLIP 2 per-patch index over the image stream, built once per recording in
    # the background, into the recording (~1.7 MB per indexed frame at fp16).
    siglip_model_name: str = SIGLIP2_MODEL_NAME
    # Empty means "named after the image stream and siglip_model_name", so two
    # models, or two cameras, never share one.
    image_index_stream_name: str = ""
    # Every Nth frame. The recording is ~15fps, so 3 keeps sub-metre coverage
    # at a third of the embedding cost.
    image_index_stride: int = PydanticField(default=3, ge=1)
    # Embed the frames here, with the model in-process, when the recording has
    # no vectors. Off by default: the viewer offers "Add embeddings" (siglipify)
    # instead, and an index that already exists is loaded either way.
    build_image_index_on_start: bool = False
    # The viewer's "Add embeddings" button runs siglipify from this flake over
    # the recording, which writes the vectors back into it (see embed.py).
    siglipify_flake: str = "github:jeff-hykin/siglipify"
    # Hyperspace (dimos.mapping.hyperspace) answers once the recording has keyframes.
    hyperspace_model_name: str = HYPERSPACE_MODEL_NAME
    hyperspace_voxel_size: float = PydanticField(default=0.1, gt=0.0)
    hyperspace_device: str = "cpu"  # text tower; MPS aborts inside a dimos worker
    hyperspace_ingest_device: str = "auto"  # the ingest subprocess may use the GPU
    hyperspace_ingest_hz: float = PydanticField(default=5.0, gt=0.0)
    hyperspace_segments: bool = True
    # Hyperspace's refine chain ("default" = its own; falls back to heat next to the map +
    # own clusters when it keeps nothing), "occupancy" for that path only, "none" for the raw map
    hyperspace_refine: str = "default"
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
    # A missing, empty or uncalibrated (zero focal length) camera_info leaves the
    # default field of view and skips the depth raycast.
    camera_info_stream_name: str | None = None
    depth_tolerance_s: float = PydanticField(default=0.02, gt=0.0)
    # Best frames whose hot patches are raycast, and how close two raycast
    # hits must land to be the same object.
    locate_frames: int = PydanticField(default=12, ge=1)
    object_radius_m: float = PydanticField(default=0.75, gt=0.0)
    # ---- timeline replay ------------------------------------------------------
    # Keyframe and per-scan diff streams written once (replay.py); the viewer scrubs
    # a segment at a time. A longer interval means fewer, larger segments.
    replay_keyframe_interval_s: float = PydanticField(default=5.0, gt=0.0)
    build_replay_on_start: bool = True
    # Rays longer than this are not cast. The replay is ray-traced (each scan
    # clears the voxels its rays pass through), and its final keyframe is the
    # static map, so this bounds both.
    replay_max_range_m: float = PydanticField(default=30.0, gt=0.0)
    # The camera frame shown while scrubbing, fetched one at a time.
    replay_frame_max_size: int = 480
    replay_frame_jpeg_quality: int = 60


class MemoryWorldModule(HyperspaceAnswers, ReplayServing, VisualAnswers, Module):
    """VR memory-world module.

    See :mod:`dimos.teleop.memory_world` for the architectural overview.
    """

    config: MemoryWorldConfig

    def __init__(self, **kwargs: Any) -> None:
        self._world_clients: set[ClientConn] = set()
        self._clients_lock = threading.Lock()

        self._store: Store | None = None
        # Cached payloads so reconnects are cheap.
        self._cached_cloud: tuple[dict[str, Any], bytes] | None = None
        self._map_xyz: np.ndarray | None = None
        self._init_hyperspace()
        self._cached_image_poses: tuple[dict[str, Any], bytes] | None = None
        # Per-pose JPEG thumbnails parallel to image_poses indices.
        self._cached_thumbnails: list[bytes] | None = None
        self._cached_odom: tuple[dict[str, Any], bytes] | None = None
        self._cached_top_down: tuple[dict[str, Any], bytes] | None = None
        self._viewer_position: tuple[float, float, float] | None = None
        self._visual_index: VisualMemoryIndex | None = None
        self._lidar_world_aligned_cache: bool | None = None
        self._index_lock = threading.RLock()
        self._embed_job = EmbeddingJob(
            on_finished=lambda job: self._broadcast(
                encode_text("index_status", **self._index_status())
            )
        )
        # The world caches are built lazily by whichever client connects first;
        # without this, two clients arriving together each voxelise the whole
        # recording.
        self._world_cache_lock = threading.Lock()
        self._index_progress = "not started"
        self._whisper: Any = None
        self._tf_tree_cache: TfTree | None = None
        self._tf_missing = False
        self._replay: VoxelReplay | None = None
        self._replay_lock = threading.Lock()
        # The store's sqlite connection is not safe to read from two threads at
        # once: a scrubbing viewer fetches segments and frames while an answer's
        # evidence frames are being decoded.
        self._store_lock = threading.RLock()  # re-entered by the world cache build
        self._stopping = threading.Event()
        self._replay_progress = "not started"
        self._replay_error: str | None = None  # a failed build is not retried until a reopen
        self._replay_index: dict[str, Any] | None = None
        self._replay_frames: OrderedDict[float, tuple[bytes, dict[str, Any]]] = OrderedDict()
        self._camera_hfov_deg: float | None = None
        self._camera_frame_cache: str | None = None
        self._active_query_result: dict[str, Any] | None = None
        self._active_query_images: list[tuple[dict[str, Any], bytes]] = []
        self._query_revision = 0
        self._web_server: RobotWebInterface | None = None
        self._web_server_thread: threading.Thread | None = None
        self._prepare_thread: threading.Thread | None = None
        self._replay_thread: threading.Thread | None = None  # a build a route started
        self._workers_lock = threading.Lock()  # publishes that handle; held for a moment

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

        self._setup_hyperspace_routes(app)

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
                RevalidatedStaticFiles(directory=str(STATIC_DIR)),
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
                return await asyncio.to_thread(self._replay_index_json)
            except Exception as error:
                raise HTTPException(
                    status_code=503, detail=f"replay {self._replay_progress}"
                ) from error

        @app.get(f"{self.config.client_route}/replay/segment/{{number}}")  # type: ignore[misc]
        async def memory_world_replay_segment(number: int, request: Request) -> Response:
            """One keyframe plus the diffs up to the next; see VoxelReplay.segment."""

            try:
                replay, _ = await asyncio.to_thread(self._replay_if_ready)
            except Exception as error:
                raise HTTPException(
                    status_code=503, detail=f"replay {self._replay_progress}"
                ) from error
            if not 0 <= number < len(replay.index.keyframe_scan):
                raise HTTPException(status_code=404, detail="no such segment")

            def read() -> bytes:
                if self._replay is not replay:  # reopened meanwhile; the viewer retries
                    raise HTTPException(status_code=503, detail="the recording was reopened")
                return replay.encoded_segment(number)

            gzipped = await asyncio.to_thread(self._replay_read, read)
            headers = {"Cache-Control": "max-age=3600"}
            if "gzip" in request.headers.get("accept-encoding", ""):
                headers["Content-Encoding"] = "gzip"
                content = gzipped
            else:
                content = gzip.decompress(gzipped)
            return Response(content=content, media_type="application/octet-stream", headers=headers)

        @app.get(f"{self.config.client_route}/replay/frame")  # type: ignore[misc]
        async def memory_world_replay_frame(t: float) -> Response:
            """The camera frame nearest *t* as JPEG; its pose rides in a header."""
            found = await asyncio.to_thread(self._replay_read, self._replay_frame, t)
            if found is None:
                raise HTTPException(status_code=404, detail="no frame near that time")
            jpeg, meta = found
            return Response(
                content=jpeg,
                media_type="image/jpeg",
                headers={"X-Camera-Pose": json.dumps(meta), "Cache-Control": "max-age=3600"},
            )

        @app.get(f"{self.config.client_route}/embeddings")  # type: ignore[misc]
        async def memory_world_embeddings() -> dict[str, Any]:
            """Whether the recording can be searched, and how adding embeddings is going."""
            return await asyncio.to_thread(self._index_status)

        @app.post(f"{self.config.client_route}/embeddings")  # type: ignore[misc]
        async def memory_world_add_embeddings() -> dict[str, Any]:
            """Run siglipify over the recording, in the background; poll GET for progress."""
            if not await asyncio.to_thread(self._start_embedding):
                raise HTTPException(status_code=409, detail="embeddings are already being added")
            return await asyncio.to_thread(self._index_status)

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
        conn = ClientConn(ws=ws, loop=loop)
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

    async def _sender_loop(self, conn: ClientConn) -> None:
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
        with self._store_lock:
            if self._store is None:
                # Published before it is named, because naming reads the tf tree and
                # that calls straight back in here -- publishing afterwards opened the
                # recording again for every such read, to a RecursionError. Withdrawn
                # again if naming refuses the recording, so that a refusal is not left
                # behind as a module serving with every role empty.
                self._store = open_recording(self.config.store_path)
                logger.info("opened memory store at %s", self.config.store_path)
                try:
                    self._name_streams(self._store)
                except BaseException:
                    store, self._store = self._store, None
                    with contextlib.suppress(Exception):
                        store.stop()
                    raise
            return self._store

    def _name_streams(self, store: Store) -> None:
        """Name the streams (and the world frame) the config left empty or the recording lacks.

        Roles are filled from the recording's message types, the lidar from
        whichever point-cloud stream agrees with tf, the world frame from the tf
        root; a name given on the command line is kept when the recording has it.
        """
        # A configured colour stream keeps its own camera_info paired to it.
        detected = detect_streams(store, image=self.config.image_stream_name or None)
        usable = usable_streams(store)  # named is not enough; see the note at the check below
        # tf first: naming the lidar needs the tree.
        for role, setting in (
            ("tf", "tf_stream_name"),
            ("image", "image_stream_name"),
            ("depth", "depth_stream_name"),
            ("camera_info", "camera_info_stream_name"),
            ("lidar", "lidar_stream_name"),
        ):
            configured = getattr(self.config, setting)
            if configured and configured not in usable:
                # Dropped, not just outranked: kept when detection had no replacement, an
                # empty `tf` became a non-None zero-frame tree. `""` is what the
                # missing-stream fallbacks look for.
                setattr(self.config, setting, "")
                configured = ""
            if configured or detected[role] is None:
                continue
            chosen = detected[role]
            if role == "lidar" and len(detected["lidar_candidates"]) > 1:
                tree = self._tf_tree()  # tf is named first, so the tree can be read now
                if tree is not None:
                    world = self.config.world_frame
                    if world not in tree.frames:
                        world = tf_root(tree) or world
                    chosen = pick_lidar(store, detected["lidar_candidates"], tree, world) or chosen
            setattr(self.config, setting, chosen)
            logger.info(
                "%s: using %r (%s)",
                setting,
                chosen,
                "detected" if not configured else f"no {configured!r} in the recording",
            )
        tree = self._tf_tree()
        if tree is not None and self.config.world_frame not in tree.frames:
            root = tf_root(tree)
            if root:
                logger.info("world_frame: using %r (the tf root)", root)
                self.config.world_frame = root

    def _ensure_world_cache(
        self,
    ) -> tuple[
        tuple[dict[str, Any], bytes],
        tuple[dict[str, Any], bytes] | None,
        tuple[dict[str, Any], bytes],
        list[bytes] | None,
        tuple[dict[str, Any], bytes],
    ]:
        """Build the cloud, top-down map, markers and trail once, whoever asks first.

        Returns (cloud, top-down map, image poses, thumbnails, trail) as one
        snapshot taken under the lock: a reopen clears the fields meanwhile.
        """
        with self._world_cache_lock:
            if self._cached_cloud is None:
                self._cached_cloud = self._build_cloud()
            if self._cached_top_down is None:
                self._cached_top_down = self._build_top_down_map(self._cached_cloud)
            if self._cached_image_poses is None:
                with self._store_lock:  # walks the image stream
                    self._cached_image_poses, self._cached_thumbnails = self._build_image_poses()
            if self._cached_odom is None:
                with self._store_lock:
                    self._cached_odom = self._build_trail()
            return (
                self._cached_cloud,
                self._cached_top_down,
                self._cached_image_poses,
                self._cached_thumbnails,
                self._cached_odom,
            )

    def _send_initial_payload(self, conn: ClientConn) -> None:
        try:
            if self._cached_cloud is None:  # unlocked hint: this viewer waits for the build
                conn.send_threadsafe(
                    encode_text("status", message="Building the map from the recording…")
                )
            cloud, top_down, poses, thumbnails, odom = self._ensure_world_cache()
            cloud_header, cloud_payload = cloud
            conn.send_threadsafe(encode_text("world_summary", **cloud_header))
            conn.send_threadsafe(encode_binary(MSG_POINT_CLOUD, cloud_header, cloud_payload))

            # Send top-down map next — both the ground plane and the HUD
            # minimap need it, so render asap on the client.
            if top_down is not None:
                map_header, map_payload = top_down
                conn.send_threadsafe(encode_binary(MSG_TOP_DOWN_MAP, map_header, map_payload))

            poses_header, poses_payload = poses
            conn.send_threadsafe(encode_binary(MSG_IMAGE_POSES, poses_header, poses_payload))

            # One MSG_IMAGE_THUMBNAIL frame per pose. Indices match poses_header.
            if thumbnails:
                for i, jpeg in enumerate(thumbnails):
                    if not jpeg:
                        continue
                    conn.send_threadsafe(encode_binary(MSG_IMAGE_THUMBNAIL, {"index": i}, jpeg))

            odom_header, odom_payload = odom
            conn.send_threadsafe(encode_binary(MSG_ODOM_TRAIL, odom_header, odom_payload))

            conn.send_threadsafe(encode_text("ready"))
            conn.send_threadsafe(encode_text("index_status", **self._index_status()))
            with self._clients_lock:  # under the lock: no newer answer slips in
                if self._active_query_result is not None:
                    conn.send_threadsafe(encode_text("query_result", **self._active_query_result))
                    for header, jpeg in self._active_query_images:
                        conn.send_threadsafe(encode_binary(MSG_QUERY_IMAGE, header, jpeg))
        except Exception:
            logger.exception("failed to build/send world payload")
            conn.send_threadsafe(encode_text("error", message="world load failed"))

        self._resend_hyperspace(conn)

    def _build_cloud(self) -> tuple[dict[str, Any], bytes]:
        """Build a voxel cloud from the recording's lidar stream."""
        built = self._build_voxel_cloud_from_lidar()
        if built is None:
            raise RuntimeError("voxel-from-lidar produced no cloud")
        return built

    def _build_voxel_cloud_from_lidar(self) -> tuple[dict[str, Any], bytes] | None:
        """The voxel map, packed for the wire.

        The map is the ray-traced replay's final keyframe: every scan cleared
        the voxels its rays passed through, so what is left is what the last
        look at each place saw (windows, people and reflections do not pile up
        the way they do in a plain accumulation). Without the replay the scans
        are simply accumulated with :class:`VoxelMapTransformer`. The cloud is
        height-coloured so the user gets depth cues without true RGB.
        """
        try:
            xyz: np.ndarray | None
            if self.config.build_replay_on_start:
                try:
                    replay = self._ensure_replay()
                    xyz = self._replay_read(lambda: replay.final_keyframe().data.points_f32())
                    if xyz.size == 0:  # scans tf could not place: accumulate them instead
                        raise RuntimeError("the replay's final keyframe is empty")
                    logger.info("voxel cloud from the ray-traced replay: %d voxels", len(xyz))
                except Exception:
                    if self._stopping.is_set():
                        raise
                    # No seekable replay (a failed build, too few scans): the map still shows.
                    logger.exception("no replay for the cloud; accumulating the scans instead")
                    xyz = self._replay_read(self._accumulated_cloud)
            else:
                xyz = self._replay_read(self._accumulated_cloud)
            if xyz is None or xyz.size == 0:
                return None

            z = xyz[:, 2]
            low = self.config.map_z_min if self.config.map_z_min is not None else -np.inf
            high = self.config.map_z_max if self.config.map_z_max is not None else np.inf
            m = (z >= low) & (z <= high)
            logger.info(
                "cloud z spans %.2f..%.2f; keeping %d of %d voxels",
                float(z.min()),
                float(z.max()),
                int(m.sum()),
                len(z),
            )
            xyz = xyz[m]
            if xyz.size == 0:
                return None
            self._map_xyz = np.ascontiguousarray(
                xyz.astype(np.float32)
            )  # the whole map, for planning
            if xyz.shape[0] > self.config.max_points:
                stride = xyz.shape[0] // self.config.max_points + 1
                xyz = xyz[::stride]

            positions = np.ascontiguousarray(xyz.astype(np.float32))
            # Lidar has no RGB, so always height-colour.
            rgb = self._height_colors(positions)
            header = self._cloud_header(positions)
            payload = positions.tobytes() + rgb.tobytes()
            logger.info("built voxel cloud: n=%d", positions.shape[0])
            return header, payload
        except Exception:
            logger.exception("voxel-from-lidar build failed")
            return None

    def _height_colors(self, positions: np.ndarray) -> np.ndarray:
        """Map Z (robot up) onto the purple-to-green height ramp.

        The ramp spans the cloud's own height range, from the
        ``height_ramp_low_percentile`` to the ``height_ramp_high_percentile``
        of z, so a stairwell or a two-storey building reads as different
        colours per level. Percentiles rather than min/max, so one stray
        return far below the building does not flatten everything else into a
        single shade. Returns N x 3 uint8 RGB.
        """
        zc = positions[:, 2]
        lo = float(np.percentile(zc, self.config.height_ramp_low_percentile)) if zc.size else 0.0
        hi = float(np.percentile(zc, self.config.height_ramp_high_percentile)) if zc.size else 1.0
        hi = max(hi, lo + 1e-3)
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
            ids: list[int] = []  # marker ids, unique per marker
            source_ids: list[int] = []  # the store's own, which analyze_memory names
            source_ids_are_real = not self.config.store_path.endswith(".mcap")
            thumbnails: list[bytes] = []

            # One indexed read per marker rather than a pass over every frame:
            # on an mcap a full pass decompresses every image chunk (minutes),
            # while a read from a stamp touches only the chunk that holds it.
            def sampled() -> Any:
                for k in range(n):
                    found = stream.after(float(first.ts) + k * interval - 1e-6).limit(1).to_list()
                    if found:
                        yield found[0]

            for k, obs in enumerate(sampled()):
                optical = self._camera_pose_of(obs)
                if optical is None:
                    continue
                # Markers stand where the camera was and face the way it looked.
                positions.append(tuple(float(v) for v in optical[:3, 3]))  # type: ignore[arg-type]
                quats.append(body_style_quaternion(optical))
                timestamps.append(float(obs.ts))
                ids.append(k)  # unique per marker: an mcap's observation ids are window-local
                # An mcap numbers each windowed read from zero, so its ids would name
                # the wrong frames; only a db's are the ids analyze_memory means.
                source_ids.append(int(getattr(obs, "id", 0)) if source_ids_are_real else -1)

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
                "source_ids": source_ids,  # what analyze_memory's observation_ids mean
            }
            payload = pos_arr.tobytes() + quat_arr.tobytes()
            logger.info("built %d image-pose markers + thumbnails", header["n"])
            return (header, payload), thumbnails
        except Exception:
            logger.exception("failed to build image poses")
            return ({"n": 0, "timestamps": [], "ids": []}, b""), []

    @staticmethod
    def _encode_jpeg(img: Any, max_size: int, quality: int) -> bytes:
        img, _ = img.resize_to_fit(max_size, max_size)
        bgr = img.to_bgr().to_opencv()
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
        if xyz.size == 0:
            return None

        z = xyz[:, 2]
        low = float(np.percentile(z, self.config.map_z_low_percentile))
        high = float(np.percentile(z, self.config.map_z_high_percentile))
        m = (z >= low) & (z <= high)
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

        Run complete Python code in a fresh process with ``store`` (the recording,
        a mem2 store), ``np`` (NumPy), and ``viewer_position`` available. Inspect
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
        """Send a result to every connected viewer and remember it for reconnects.

        Whatever the engine counted in, the ids that go out are marker ids. The viewer
        holds thumbnails for the markers it was sent and nothing else, so an answer
        naming a frame that is not one of them has no picture to show; the markers
        nearest the answer do.
        """
        # In place, not model_copy: the caller keeps a reference and reports on it.
        # `analyze_memory` answers the agent with len(result.observation_ids), and with a
        # copy that number stayed the agent's OWN INPUT rather than what was lit -- it
        # differs in both directions, since the ids collapse through a set and an answer
        # naming no frame still gets the nearest marker.
        result.observation_ids = self._marker_ids_for(result)
        with self._clients_lock:
            query_id = uuid.uuid4().hex
            self._query_revision += 1
            payload = result.model_dump(mode="json")
            payload.update(query_id=query_id, revision=self._query_revision)
            self._active_query_result = payload
            self._active_query_images = []
            if payload.get("engine") != "hyperspace":  # no heat map or frusta go with it
                self._active_heatmap = self._active_pyramids = None
                self._last_answer = (None, None)
            # Queued under the lock: two answers then reach every viewer in revision order.
            message = encode_text("query_result", **payload)
            for client in tuple(self._world_clients):
                client.send_threadsafe(message)
        return query_id

    def _marker_ids_for(self, result: MemoryQueryResult) -> list[int]:
        """The answer's own ids if they are markers, else the markers nearest it.

        ``analyze_memory`` answers with the store's own observation ids, read from the
        whole recording; the viewer was only sent a couple of hundred markers, so those
        ids are almost never among them. Snapping here means the wire carries one id
        space and the viewer needs no rule for telling them apart.
        """
        # The engine says which space its ids are in; the two overlap numerically (marker
        # 1 and observation 1 are both small integers), so guessing from the values would
        # quietly highlight the wrong photo rather than fail.
        if result.engine != "agent":
            return list(result.observation_ids or [])
        cached = self._cached_image_poses
        if cached is None:  # no markers were published, so no id of ours can name one
            return []
        if not result.observation_ids:
            # An answer can point somewhere without naming a frame. The markers nearest
            # where it points are still its evidence, and returning nothing here left it
            # with no photograph at all.
            return self._markers_near(answer_positions(result))
        header, _ = cached
        sources = {
            source: marker
            for source, marker in zip(
                header.get("source_ids") or [], header.get("ids") or [], strict=False
            )
            if source >= 0  # an mcap numbers each windowed read from zero: no real ids
        }
        # Deduplicated, like the nearest-marker branch below and for the same reason the
        # VIEWER does it: `_selectedImageIds` is a Set, so two ids naming one marker light
        # one photograph. Keeping both told the agent it had lit two. Order is kept because
        # it is the agent's own; the set comprehension below has no order to lose.
        snapped = list(dict.fromkeys(sources[i] for i in result.observation_ids if i in sources))
        if snapped:
            return snapped
        return self._markers_near(answer_positions(result))

    # ---- spoken visual search ---------------------------------------------

    def _ensure_visual_index(self) -> VisualMemoryIndex:
        with self._store_lock, self._index_lock:  # a lazy init that opens the store; store first
            return self._visual_index_unlocked()

    def _visual_index_unlocked(self) -> VisualMemoryIndex:
        if self._visual_index is None:
            self._visual_index = VisualMemoryIndex(
                self._ensure_store(),
                pose_of=self._camera_pose_of,
                image_stream_name=self.config.image_stream_name,
                index_stream_name=self.config.image_index_stream_name,
                model_name=self.config.siglip_model_name,
                world_frame=self.config.world_frame,
            )
        return self._visual_index

    def _reopen_recording(self) -> None:
        """Open the recording afresh: siglipify rewrote the mcap, and the store holds the old file."""
        # Lock order everywhere: planner, world cache, replay, store, index.
        with (
            self._planner_lock,
            self._world_cache_lock,
            self._replay_lock,
            self._store_lock,
            self._index_lock,
        ):
            # Replay bookkeeping first: the open below is slow, and a viewer polling
            # /replay/index meanwhile must not read a stale "build failed".
            self._replay = None
            self._replay_error = None
            self._replay_progress = "not started"
            old = self._store
            self._store = open_recording(self.config.store_path)
            self._replay_index = None
            self._replay_frames.clear()
            self._drop_visual_index()
            self._tf_tree_cache = None  # before naming: the names are picked against the tree
            self._tf_missing = False
            self._name_streams(self._store)
            self._lidar_world_aligned_cache = None
            self._camera_hfov_deg = None
            self._camera_frame_cache = None
            self._cached_cloud = self._cached_image_poses = self._cached_thumbnails = None
            self._cached_odom = self._cached_top_down = self._map_xyz = None
            self._route_planner = None
            self._orbit_cache.clear()
            if old is not None:
                old.stop()
        logger.info("reopened %s", self.config.store_path)

    @skill
    def find_in_memory(self, query: str) -> SkillResult:
        """Find the distinct places something was seen and highlight them in VR:
        "where did I see a car" → Hyperspace when ready, else the SigLIP index.

        Args:
            query: What to look for, e.g. "a car" or "a whiteboard".
        """
        started = time.monotonic()
        phrase = search_phrase(query)
        if not phrase:
            return SkillResult.fail("INVALID_QUERY", "The query text is empty")
        if self._hyperspace_ready():
            return self._find_with_hyperspace(phrase, started)

        try:
            return self._find_with_siglip(phrase, started)
        except Exception as error:  # an index built for another model, camera or frame
            logger.exception("visual index query failed")
            return SkillResult.fail("QUERY_FAILED", f"The SigLIP index cannot answer: {error}")

    # ---- poses: the tf tree ------------------------------------------------
    def _tf_tree(self) -> TfTree | None:
        """The recording's tf tree, loaded once; None when the recording has none."""
        with self._store_lock:
            return self._load_tf_tree()

    def _load_tf_tree(self) -> TfTree | None:
        if self._tf_tree_cache is None and not self._tf_missing:
            store = self._ensure_store()
            if self.config.tf_stream_name not in store.list_streams():
                self._tf_missing = True
                logger.warning(
                    "no %r stream; falling back to the poses stamped on images",
                    self.config.tf_stream_name,
                )
                return None
            tree = build_tf_tree(store, self.config.tf_stream_name)
            logger.info("tf tree: %d transforms over %d frames", len(tree), len(tree.frames))
            self._tf_tree_cache = tree
        return self._tf_tree_cache

    def _camera_frame(self) -> str:
        if self.config.camera_optical_frame is not None:
            return self.config.camera_optical_frame
        if self._camera_frame_cache is None:  # read once, even when it is empty
            try:
                with self._store_lock:  # the payload read too: a reopen closes the old store
                    first = self._ensure_store().streams[self.config.image_stream_name].first()
                    frame = str(getattr(first.data, "frame_id", "") or "")
            except LookupError:  # no image stream, or an empty one
                frame = ""
            self._camera_frame_cache = frame.lstrip("/")
        return self._camera_frame_cache

    def _lidar_world_aligned(self) -> bool:
        """Whether the lidar scans are stored already registered in the world frame."""
        if self._lidar_world_aligned_cache is None:
            aligned = self.config.lidar_world_frame
            if aligned is None:
                with self._store_lock:
                    first = self._ensure_store().streams[self.config.lidar_stream_name].first()
                    frame_id = str(getattr(first.data, "frame_id", "")).lower().lstrip("/")
                world = str(self.config.world_frame or "").lower().lstrip("/")
                # Any other fixed frame goes through tf: map <- odom is not the identity.
                aligned = frame_id == world or "corrected" in frame_id
                if not aligned and frame_id in {"map", "odom", "world"}:
                    tree = self._tf_tree()
                    known = set() if tree is None else {f.lower().lstrip("/") for f in tree.frames}
                    aligned = frame_id not in known  # tf cannot place it: it is the world
                logger.info(
                    "lidar frame %r detected as %s",
                    frame_id,
                    "world-aligned" if aligned else "sensor-relative",
                )
            self._lidar_world_aligned_cache = bool(aligned)
        return self._lidar_world_aligned_cache

    def _accumulated_cloud(self) -> np.ndarray | None:
        """Every ``n_voxel_scans``-th scan voxelised into one cloud, no clearing."""
        world_aligned = self._lidar_world_aligned()

        def to_world(obs: Any) -> Any:
            # A scan already in the map frame must not get its pose applied again.
            return obs if world_aligned else self._scan_to_world(obs)

        stream = self._ensure_store().streams[self.config.lidar_stream_name]
        return accumulate_scans(stream, to_world, self.config.voxel_size, self.config.n_voxel_scans)

    def _scan_frame(self, obs: Any) -> SensorScan | None:
        """A lidar scan in its sensor frame with the sensor's pose, for ray casting.

        The pose is a tf lookup at the scan's stamp. A world-aligned stream uses
        the pose stamped on the scan, else the camera, the nearest frame tf knows.
        """
        world_aligned = self._lidar_world_aligned()
        scan_frame = str(getattr(obs.data, "frame_id", "") or "").lstrip("/")
        stamped = getattr(obs, "pose_tuple", None)
        if world_aligned and stamped is not None:  # stitched scans carry the sensor pose
            return sensor_scan(
                obs.data.points_f32(),
                pose_matrix(tuple(stamped[:3]), tuple(stamped[3:7])),
                in_world=True,
            )
        pose_frame = self._camera_frame() if world_aligned else scan_frame
        matrix = self._frame_pose_at(pose_frame, float(obs.ts))
        if matrix is None:
            pose = getattr(obs, "pose_tuple", None)
            if pose is None or self._tf_tree() is not None:
                return None
            matrix = pose_matrix(tuple(pose[:3]), tuple(pose[3:7]))
        return sensor_scan(obs.data.points_f32(), np.asarray(matrix), in_world=world_aligned)

    def _scan_to_world(self, obs: Any) -> Any:
        """A sensor-frame lidar scan moved into the world frame, or None.

        The pose is a tf lookup at the scan's stamp; only a recording without
        any tf stream falls back to the pose stamped on the observation.
        """
        from dimos.msgs.geometry_msgs.Transform import Transform

        scan_frame = str(getattr(obs.data, "frame_id", "") or "").lstrip("/")
        matrix = self._frame_pose_at(scan_frame, float(obs.ts))
        if matrix is None:
            pose = getattr(obs, "pose_tuple", None)
            if pose is None or self._tf_tree() is not None:
                return None
            matrix = pose_matrix(tuple(pose[:3]), tuple(pose[3:7]))
        return obs.data.transform(Transform.from_matrix(matrix))

    def _camera_hfov(self) -> float:
        with self._store_lock:  # a lazy init that reads the store
            return self._read_camera_hfov()

    def _read_camera_hfov(self) -> float:
        """Horizontal field of view of the image stream: from camera_info when it has a
        focal length, else 70 degrees."""
        if self._camera_hfov_deg is None:
            self._camera_hfov_deg = 70.0
            if self.config.camera_info_stream_name is not None:
                try:
                    store = self._ensure_store()
                    info = store.streams[self.config.camera_info_stream_name].first().data
                    if not info.K[0]:
                        raise LookupError("camera_info has no focal length")
                    self._camera_hfov_deg = float(
                        np.degrees(2.0 * np.arctan2(info.width / 2.0, info.K[0]))
                    )
                except LookupError:  # declared, never published: the default stands
                    logger.warning(
                        "no %r message; using a %.0f degree field of view",
                        self.config.camera_info_stream_name,
                        self._camera_hfov_deg,
                    )
        return self._camera_hfov_deg

    # ---- timeline replay -----------------------------------------------------

    def _ensure_replay(self) -> VoxelReplay:
        """The recording's replay streams, built on first use if missing. Needs the image
        stream too: the index lists its frame stamps. A build that places no scan is
        deleted again and raises; the failure is remembered until a reopen.

        Lock order everywhere: planner, world cache, replay, store, index. The
        build runs under the replay lock alone: the lidar and derived streams
        have their own connections and nothing serves replay data until it is done.
        """
        with self._replay_lock:
            return self._replay_locked()

    def _replay_if_ready(self) -> tuple[VoxelReplay, dict[str, Any]]:
        """The replay and its index json for the serving routes: while a build holds
        the lock they answer 503 instead of holding a request thread for the whole
        build. Both come from the one acquisition: a reopen in between would clear them."""
        if not self._replay_lock.acquire(blocking=False):
            raise RuntimeError(f"replay {self._replay_progress}")
        try:
            if self._replay is None and self._replay_error is None:
                # Never build on a request thread: start it (once) and let the viewer poll.
                with self._workers_lock:  # paired with stop(): nothing starts once it stops
                    if self._stopping.is_set():
                        raise RuntimeError("replay not built: stopping")
                    if self._replay_thread is None or not self._replay_thread.is_alive():
                        thread = threading.Thread(
                            target=self._build_replay, daemon=True, name="MemoryWorldReplay"
                        )
                        thread.start()  # started before it is published: join needs that
                        self._replay_thread = thread
                raise RuntimeError(f"replay {self._replay_progress}")
            replay = self._replay_locked()
            assert self._replay_index is not None  # set together with _replay
            return replay, self._replay_index
        finally:
            self._replay_lock.release()

    def _replay_locked(self) -> VoxelReplay:
        if self._replay is not None:
            return self._replay
        if self._replay_error is not None:
            raise RuntimeError(self._replay_error)
        try:
            with self._store_lock:
                store = self._ensure_store()
                if self.config.image_stream_name not in store.list_streams():
                    named = repr(self.config.image_stream_name or "colour")
                    raise RuntimeError(f"no {named} image stream; the replay needs camera frames")
                available = VoxelReplay.available(
                    store,
                    voxel_size=self.config.voxel_size,
                    lidar_stream_name=self.config.lidar_stream_name,
                    max_range=self.config.replay_max_range_m,
                    world_frame=self.config.world_frame,
                )
            if not available:
                self._replay_progress = "building"
                logger.info("building the voxel replay streams into %s", self.config.store_path)
                stats = build_replay_streams(
                    store,
                    lidar_stream_name=self.config.lidar_stream_name,
                    to_scan=self._scan_frame,
                    voxel_size=self.config.voxel_size,
                    max_range=self.config.replay_max_range_m,
                    keyframe_interval_s=self.config.replay_keyframe_interval_s,
                    cancelled=self._stopping.is_set,
                    world_frame=self.config.world_frame,
                )
                if self._stopping.is_set():  # cut short: the streams lack their last keyframe
                    raise RuntimeError("cancelled")
                if stats.added == 0:  # nothing placed: the streams would pass as finished
                    for name in (DIFF_STREAM, KEYFRAME_STREAM):
                        store.delete_stream(name)
                    raise RuntimeError("no voxel came out of the scans (tf, frame or range)")
                logger.info(
                    "voxel replay built: %d scans, %d keyframes, +%d/-%d edits in %.1f s",
                    stats.scans,
                    stats.keyframes,
                    stats.added,
                    stats.removed,
                    stats.seconds,
                )
            with self._store_lock:
                # The replay shows the same heights as the static map.
                replay = VoxelReplay(
                    store,
                    z_min=self.config.map_z_min if self.config.map_z_min is not None else -np.inf,
                    z_max=self.config.map_z_max if self.config.map_z_max is not None else np.inf,
                )
                if len(replay.index.scan_ts) < 2:  # nothing to seek: the viewer would poll forever
                    raise RuntimeError("fewer than two scans")
                # Listing every camera stamp is a pass over the image stream (on
                # an mcap that decompresses every chunk), so it is done here, once.
                self._replay_index = self._build_replay_index_json(replay)
        except Exception as error:
            # The viewer reads this prefix: it stops polling on a failed build.
            self._replay_progress = f"build failed: {error}"
            # Remembered: otherwise every viewer connect and every /replay/index retry
            # would rebuild from scratch. A cancelled build is retried by the next start.
            if not self._stopping.is_set():
                self._replay_error = self._replay_progress
            raise
        self._replay = replay
        self._replay_progress = "ready"
        return replay

    def _replay_read(self, fn: Any, *args: Any) -> Any:
        """Run one store-reading replay call at a time."""
        with self._store_lock:
            return fn(*args)

    def _build_replay(self) -> None:
        if self._stopping.is_set():  # stop() may have set it after the caller's check
            return
        try:
            self._ensure_replay()
        except Exception:
            logger.exception("voxel replay build failed")  # _replay_locked keeps the reason

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

        From tf at the image's stamp; without a
        tf stream, from the body pose stamped on the image, turned into the
        optical convention.
        """
        if self._tf_tree() is not None:
            world_T_optical = self._frame_pose_at(self._camera_frame(), float(obs.ts))
        else:
            pose = getattr(obs, "pose_tuple", None)
            if pose is None:
                return None
            body = pose_matrix(
                tuple(pose[:3]), tuple(pose[3:7]) if len(pose) >= 7 else (0, 0, 0, 1)
            )
            world_T_optical = np.asarray(body @ OPTICAL_FROM_BODY)
        return world_T_optical

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

    def _on_client_message(self, conn: ClientConn, msg: dict[str, Any]) -> None:
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
        self._prepare_thread = threading.Thread(
            target=self._prepare, daemon=True, name="MemoryWorldPrepare"
        )
        self._prepare_thread.start()

    def _prepare(self) -> None:
        """Build what every client needs, in order of urgency, on one thread: each
        step is a pass over the recording and run together they starve each other."""
        try:
            self._ensure_world_cache()
        except Exception:
            logger.exception("world cache build failed")
        if self._stopping.is_set():
            return
        self._load_hyperspace()
        if self.config.build_replay_on_start and not self._stopping.is_set():
            self._build_replay()
        if self._hyperspace_ready():
            # Say so rather than leaving it "not started": the viewer reads that as a
            # build in flight and polls for it for the whole session.
            self._index_progress = "not needed; Hyperspace answers this recording"
        elif not self._stopping.is_set():
            self._build_visual_index()  # the SigLIP index is the fallback engine

    @rpc
    def stop(self) -> None:
        try:
            if self._web_server is not None:
                self._web_server.shutdown()
            if self._web_server_thread is not None:
                self._web_server_thread.join(timeout=3)
                self._web_server_thread = None
            with self._workers_lock:  # paired with _replay_if_ready: no worker starts after this
                self._stopping.set()  # prepare stops between steps; a replay build per scan
                threads = (self._prepare_thread, self._replay_thread)
            self._embed_job.terminate()
            self._prepare_job.terminate()
            for thread in threads:
                if thread is not None:
                    thread.join(timeout=60)
        finally:
            busy = [t for t in (self._prepare_thread, self._replay_thread) if t and t.is_alive()]
            if busy:
                logger.warning(
                    "%s is still running; its stores are left to the process exit", busy[0].name
                )
            else:
                # ALL of the teardown is under the store lock, store then index, the
                # order every reader uses. The evidence, query and adopt threads are not
                # joined here and each holds it while reading: dismantle the index under
                # one and it reads a dismantled index; close the store under one and every
                # `memworld --stop` after a question prints a page of ProgrammingError.
                # The deadline is because an evidence read decodes many frames and --stop
                # must not wait; past it NOTHING is torn down and the process exit does it
                # all, as the busy branch above already decides for its threads. Tearing
                # half of it down is the one outcome worse than either.
                if self._store_lock.acquire(timeout=5):
                    try:
                        with self._index_lock:
                            if self._hyperspace is not None:
                                self._hyperspace.close()
                                self._hyperspace = None  # not a handle to a closed search
                            if self._visual_index is not None:
                                self._visual_index.stop()
                                self._visual_index = None
                            store, self._store = self._store, None
                            if store is not None:
                                try:
                                    store.stop()
                                except Exception:
                                    logger.exception("error closing memory store")
                    finally:
                        self._store_lock.release()
                else:
                    logger.warning("a read is still in flight; the store is left to exit")
            super().stop()
