#!/usr/bin/env python3

# Copyright 2025-2026 Dimensional Inc.
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

"""Marauder's Map web view.

A Harry-Potter "Marauder's Map" styled floor plan of the area the robot has
mapped, with every detected person plotted at their world position as an
ID-tagged footprint marker. Two display modes (toggled in the browser):

  * ID only           -- just the code, e.g. ``#3``
  * ID + photo         -- the code plus the person's cropped camera thumbnail

Data sources (all already produced by ``unitree-go2-detection``):
  * ``global_costmap``  (OccupancyGrid)   -> the parchment floor plan
  * ``color_image``     (Image)           -> source for per-person crops
  * ``detections``      (Detection2DArray)-> per-person id/class/bbox
  * ``pointcloud``      (PointCloud2, world frame) -> world-frame localization
  * ``odom``            (PoseStamped)      -> the robot's own dot on the map

World localization reuses ``Detection3DPC.from_2d`` (the same projection
``Detection3DModule`` uses) so a person's map position is the centroid of the
world points inside their detection box -- and the original track_id / class /
crop survive the projection.

Same authoring pattern as the reID-marking feature; served on its own port
(default 7782) so it can run alongside the command center (7779) and the
marking view (7781).
"""

import asyncio
import base64
import math
from pathlib import Path as FilePath
import subprocess
import sys
import threading
import time
from typing import Any

import cv2
import numpy as np
from reactivex.disposable import Disposable
import socketio  # type: ignore[import-untyped]
from starlette.applications import Starlette
from starlette.responses import FileResponse, Response
from starlette.routing import Route
import uvicorn

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos_lcm.std_msgs import Bool  # planner stop_movement uses LCM std_msgs/Bool
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_TEMPLATES_DIR = FilePath(__file__).parent / "templates"
_MAP_HTML = _TEMPLATES_DIR / "marauders_map.html"
_SOCKETIO_JS = _TEMPLATES_DIR / "socketio.min.js"
# 100 Harry Potter characters x 10 lines each, baked locally so the map works
# fully offline (the field network has no internet).
_HP_JS = _TEMPLATES_DIR / "hp_characters.js"
# Ambient BGM — "Potion Latch", loop-played in the page. File ships in this
# folder so the offline field network works.
_BGM_MP3 = _TEMPLATES_DIR / "potion_latch.mp3"

PERSON_CLASS_ID = 0  # COCO "person"
DOG_CLASS_ID = 16  # COCO "dog" -- used as the heuristic for "another Go2 quadruped"
# Dog roster ids live in a separate numeric range so they never collide with
# the small long-term person ids assigned by reID.
DOG_ID_BASE = 1_000_000


class ReidMapConfig(ModuleConfig):
    camera_info: CameraInfo

    port: int = 7782
    # The pointcloud frame (global map is published in the world frame).
    world_frame_hint: str = "world"
    # Only plot people; set False to plot every detected object.
    persons_only: bool = True
    # Also flag other Unitree-Go2-like quadrupeds (COCO "dog") with a special icon.
    detect_robot_dogs: bool = True
    robot_dog_class_id: int = DOG_CLASS_ID
    # Throttles (Hz). Map changes slowly; detection localization is CPU heavy.
    map_emit_hz: float = 1.0
    detect_hz: float = 3.0
    # How long a person stays on the map after they were last seen (sec).
    person_ttl_sec: float = 8.0
    # Re-grab a person's photo at most this often (sec).
    photo_refresh_sec: float = 4.0
    photo_quality: int = 70
    photo_max_width: int = 120
    # Larger crop kept per id and served on demand for the "view large" lightbox.
    photo_full_quality: int = 82
    photo_full_max_width: int = 360

    # Per-dog label. If blank, auto-detect from the WiFi SSID this machine is
    # joined to (each Go2 broadcasts its own AP, e.g. "dimair13") so a viewpoint
    # is identified by which dog's network it came from.
    robot_label: str = ""
    wifi_interface: str = "en0"

    # reID: collapse the detector's short-term track_ids into stable long-term
    # IDs (osnet appearance embeddings) so one person == one code, and the
    # roster count reflects distinct people rather than track fragments.
    enable_reid: bool = True
    # Frames a track must accumulate before it can be matched/assigned an ID.
    # Lower = snappier first appearance, higher = more reliable matching.
    reid_min_embeddings: int = 3
    reid_similarity: float = 0.63

    # Spatial-temporal re-association (used when appearance reID is OFF). Keeps
    # the SAME person on one stable id by position continuity: a brand-new
    # detector track that appears within `reassoc_radius_m` of where a recently
    # (within `reassoc_window_s`) lost person was, and that spot is not currently
    # held by another live person, inherits that person's id. Robust to BoT-SORT
    # reassigning track ids across brief occlusions, without appearance merges.
    reassoc_radius_m: float = 1.5
    reassoc_window_s: float = 12.0

    # Web teleop: linear / angular speed when a movement button is pressed.
    teleop_linear_speed: float = 0.5  # m/s
    teleop_angular_speed: float = 0.8  # rad/s


class ReidMapModule(Module):
    """Plots detected people on a Marauder's-Map-styled floor plan web view."""

    config: ReidMapConfig

    global_costmap: In[OccupancyGrid]
    color_image: In[Image]
    detections: In[Detection2DArray]
    pointcloud: In[PointCloud2]
    odom: In[PoseStamped]

    # Web -> robot wiring:
    #   * cmd_vel: teleop Twist, remapped to tele_cmd_vel in the blueprint.
    #   * clicked_point: a synthesized camera-pixel click that lands inside
    #     the chosen person's bbox; flows into the SAME BBoxSelectionModule
    #     input the Rerun camera-view click uses. This lets us drive selection
    #     through jamjam's public API without modifying their module.
    cmd_vel: Out[Twist]
    clicked_point: Out[PointStamped]
    # Click-to-navigate: the user clicks a free point on the parchment map and we
    # publish that world-frame target on goal_request; ReplanningAStarPlanner
    # picks it up and drives via nav_cmd_vel. stop_movement is the kill for that
    # planner (used to make "follow person" and "navigate to point" mutually
    # exclusive — only one drives nav_cmd_vel at a time).
    goal_request: Out[PoseStamped]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._uvicorn_server_thread: threading.Thread | None = None
        self._uvicorn_server: uvicorn.Server | None = None
        self.sio: socketio.AsyncServer | None = None
        self.app: Any = None
        self._broadcast_loop: asyncio.AbstractEventLoop | None = None
        self._broadcast_thread: threading.Thread | None = None
        self._stopped = False

        self._lock = threading.Lock()
        self._latest_image: Image | None = None
        self._latest_pc: PointCloud2 | None = None
        self._last_map_emit = 0.0
        self._last_detect = 0.0
        # Keyed by stable long-term reID id (or track_id if reID disabled):
        #   {x, y, name, class_id, photo, photo_ts, last_seen}
        self._people: dict[int, dict[str, Any]] = {}
        # Cached map metadata so the frontend can map world -> pixel for markers.
        self._map_meta: dict[str, Any] | None = None
        # Long-term appearance reID (lazily built on start if enabled).
        self._idsystem: Any = None
        # Label identifying which dog/WiFi this viewpoint belongs to.
        self._label: str = "go2"
        # Spatial-temporal identity layer (used when appearance reID is off):
        #   _track_to_stable: detector track_id -> stable display id
        #   _stable_pos:      stable id -> {x, y, last_seen} (for re-association)
        self._track_to_stable: dict[int, int] = {}
        self._stable_pos: dict[int, dict[str, float]] = {}
        self._next_stable: int = 0
        # Per-stable-id mapping to the latest raw LCM Detection2D the tracker
        # produced for that person -- used when the web client clicks a
        # character to drive TargetLockModule.
        self._stable_to_raw_det: dict[int, Any] = {}
        self._latest_detection_header: Any = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    @rpc
    def start(self) -> None:
        super().start()
        self._label = self.config.robot_label.strip() or self._detect_wifi_ssid() or "go2"
        logger.info("Marauder's Map: viewpoint label = %s", self._label)
        self._create_server()
        self._start_broadcast_loop()
        self._uvicorn_server_thread = threading.Thread(target=self._run_uvicorn_server, daemon=True)
        self._uvicorn_server_thread.start()
        logger.info(f"Marauder's Map available at http://localhost:{self.config.port}/")

        if self.config.enable_reid:
            try:
                from dimos.models.embedding.treid import TorchReIDModel
                from dimos.perception.detection.reid.embedding_id_system import (
                    EmbeddingIDSystem,
                )

                self._idsystem = EmbeddingIDSystem(
                    model=TorchReIDModel,
                    padding=0,
                    similarity_threshold=self.config.reid_similarity,
                    min_embeddings_for_matching=self.config.reid_min_embeddings,
                )
                logger.info("Marauder's Map: reID (long-term IDs) enabled")
            except Exception:
                logger.exception("Marauder's Map: reID unavailable, falling back to track_id")
                self._idsystem = None

        for stream, handler in (
            (self.global_costmap, self._on_costmap),
            (self.color_image, self._on_color_image),
            (self.pointcloud, self._on_pointcloud),
            (self.detections, self._on_detections),
            (self.odom, self._on_odom),
        ):
            try:
                unsub = stream.subscribe(handler)
                self.register_disposable(Disposable(unsub))
            except Exception:
                logger.exception("Marauder's Map: subscribe failed for %s", handler.__name__)

    @rpc
    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        if self._uvicorn_server:
            self._uvicorn_server.should_exit = True
        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            self._broadcast_loop.call_soon_threadsafe(self._broadcast_loop.stop)
        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._uvicorn_server_thread and self._uvicorn_server_thread.is_alive():
            self._uvicorn_server_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    # ------------------------------------------------------------------ #
    # Server
    # ------------------------------------------------------------------ #
    def _create_server(self) -> None:
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

        async def serve_index(request):  # type: ignore[no-untyped-def]
            if _MAP_HTML.exists():
                return FileResponse(_MAP_HTML, media_type="text/html")
            return Response(content="marauders_map.html not found", status_code=503)

        async def serve_socketio_js(request):  # type: ignore[no-untyped-def]
            if _SOCKETIO_JS.exists():
                return FileResponse(_SOCKETIO_JS, media_type="application/javascript")
            return Response(content="socketio.min.js not found", status_code=503)

        async def serve_hp_js(request):  # type: ignore[no-untyped-def]
            if _HP_JS.exists():
                return FileResponse(_HP_JS, media_type="application/javascript")
            return Response(content="hp_characters.js not found", status_code=503)

        async def serve_bgm(request):  # type: ignore[no-untyped-def]
            """Background ambient music (loop-played in the page).

            Served from this folder so the page works on the offline field
            network. Audio kept under /vendor/* for consistency with the
            other static asset routes.
            """
            if _BGM_MP3.exists():
                return FileResponse(_BGM_MP3, media_type="audio/mpeg")
            return Response(content="bgm.mp3 not found", status_code=503)

        starlette_app = Starlette(
            routes=[
                Route("/", serve_index),
                Route("/vendor/socketio.js", serve_socketio_js),
                Route("/vendor/hp_characters.js", serve_hp_js),
                Route("/vendor/bgm.mp3", serve_bgm),
            ]
        )
        self.app = socketio.ASGIApp(self.sio, starlette_app)

        @self.sio.event  # type: ignore[untyped-decorator]
        async def connect(sid, environ) -> None:  # type: ignore[no-untyped-def]
            logger.info(f"Marauder's Map client connected: {sid}")
            # Send the cached map + current people so a fresh client is not blank.
            if self._map_meta is not None:
                await self.sio.emit("map", self._map_meta, room=sid)  # type: ignore[union-attr]
            await self.sio.emit("people", {"people": self._people_payload()}, room=sid)  # type: ignore[union-attr]

        @self.sio.event  # type: ignore[untyped-decorator]
        async def get_photo(sid, data) -> None:  # type: ignore[no-untyped-def]
            """Client asked for a full-size portrait to view large."""
            try:
                ident = int(data.get("id"))
            except Exception:
                return
            entry = self._people.get(ident)
            full = entry.get("photo_full") if entry else None
            await self.sio.emit(  # type: ignore[union-attr]
                "photo_full",
                {"id": ident, "img": full, "name": entry.get("name") if entry else None},
                room=sid,
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def teleop(sid, data) -> None:  # type: ignore[no-untyped-def]
            """Web button held -> emit Twist on cmd_vel (->tele_cmd_vel topic).

            Payload is a Twist in robot units; the button handler in the page
            scales the axes by the configured linear/angular speed so this
            module stays a thin pipe. An all-zero Twist arrives on button
            release and tells MovementManager to hand back to nav.
            """
            try:
                lx = float(data.get("linear_x", 0.0))
                ly = float(data.get("linear_y", 0.0))
                az = float(data.get("angular_z", 0.0))
            except Exception:
                return
            try:
                self.cmd_vel.publish(
                    Twist(linear=Vector3(lx, ly, 0.0), angular=Vector3(0.0, 0.0, az))
                )
            except Exception:
                logger.exception("Marauder's Map: teleop publish failed")

        @self.sio.event  # type: ignore[untyped-decorator]
        async def select(sid, data) -> None:  # type: ignore[no-untyped-def]
            """Web click on a character -> drive TargetLockModule.
            Also cancels any active nav goal — only one driver of nav_cmd_vel
            at a time (planner OR bbox-follow)."""
            try:
                ident = int(data.get("id"))
            except Exception:
                return
            # Cancel any nav-to-point goal first so the planner stops driving
            # before bbox-follow takes over.
            try:
                self.stop_movement.publish(Bool(data=True))
            except Exception:
                logger.exception("Marauder's Map: stop_movement publish failed (on select)")
            self._publish_selection(ident)

        @self.sio.event  # type: ignore[untyped-decorator]
        async def deselect(sid, data) -> None:  # type: ignore[no-untyped-def]
            """Web clears the selection -> empty Detection2DArray."""
            self._publish_selection(None)

        @self.sio.event  # type: ignore[untyped-decorator]
        async def navigate(sid, data) -> None:  # type: ignore[no-untyped-def]
            """Web click on a free point of the map -> drive the planner.
            Payload: {"wx": <world x in metres>, "wy": <world y in metres>}.
            Cancels any active follow (planner & bbox-follow share nav_cmd_vel
            — exclusivity is enforced here)."""
            try:
                wx = float(data.get("wx"))
                wy = float(data.get("wy"))
            except Exception:
                return
            # Clear any locked person so bbox-follow stops driving nav_cmd_vel.
            try:
                self._publish_selection(None)
            except Exception:
                logger.exception("Marauder's Map: deselect on navigate failed")
            try:
                self.goal_request.publish(
                    PoseStamped(
                        position=(wx, wy, 0.0),
                        orientation=(0.0, 0.0, 0.0, 1.0),
                        frame_id=self.config.world_frame_hint,
                    )
                )
                # Echo back so the UI can render a goal pin without waiting
                # for an odom round-trip.
                await self.sio.emit(  # type: ignore[union-attr]
                    "goal", {"x": wx, "y": wy}
                )
            except Exception:
                logger.exception("Marauder's Map: goal_request publish failed")

        @self.sio.event  # type: ignore[untyped-decorator]
        async def cancel_nav(sid, data) -> None:  # type: ignore[no-untyped-def]
            """Stop the planner (publishes Bool(True) on stop_movement)."""
            try:
                self.stop_movement.publish(Bool(data=True))
                await self.sio.emit("goal", None)  # type: ignore[union-attr]
            except Exception:
                logger.exception("Marauder's Map: cancel_nav publish failed")

    def _start_broadcast_loop(self) -> None:
        def loop_runner() -> None:
            self._broadcast_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._broadcast_loop)
            try:
                self._broadcast_loop.run_forever()
            finally:
                self._broadcast_loop.close()

        self._broadcast_thread = threading.Thread(target=loop_runner, daemon=True)
        self._broadcast_thread.start()

    def _run_uvicorn_server(self) -> None:
        config = uvicorn.Config(
            self.app,
            host=global_config.listen_host,
            port=self.config.port,
            log_level="error",
        )
        self._uvicorn_server = uvicorn.Server(config)
        self._uvicorn_server.run()

    # ------------------------------------------------------------------ #
    # Stream handlers
    # ------------------------------------------------------------------ #
    def _on_color_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image

    def _on_pointcloud(self, pc: PointCloud2) -> None:
        with self._lock:
            self._latest_pc = pc

    def _on_odom(self, msg: PoseStamped) -> None:
        self._emit(
            "robot",
            {"x": float(msg.position.x), "y": float(msg.position.y), "label": self._label},
        )

    def _on_costmap(self, grid: OccupancyGrid) -> None:
        now = time.monotonic()
        if now - self._last_map_emit < 1.0 / max(self.config.map_emit_hz, 0.1):
            return
        self._last_map_emit = now
        try:
            meta = self._encode_map(grid)
        except Exception:
            logger.exception("Marauder's Map: failed to encode occupancy grid")
            return
        self._map_meta = meta
        self._emit("map", meta)

    def _on_detections(self, msg: Detection2DArray) -> None:
        # Header is stashed unconditionally so the web select event can still
        # emit a valid Detection2DArray even on frames we throttle past.
        self._latest_detection_header = msg.header

        # Build raw-detection lookup (by LCM id string == track_id) so a web
        # click for stable_id N can construct a single-detection array from
        # the *current* frame, not the throttled-stale one.
        raw_by_id = {str(getattr(r, "id", "") or ""): r for r in msg.detections}

        now = time.monotonic()
        if now - self._last_detect < 1.0 / max(self.config.detect_hz, 0.1):
            self._refresh_stable_raw(msg.detections, raw_by_id)
            return
        self._last_detect = now

        with self._lock:
            image = self._latest_image
            pc = self._latest_pc
        if image is None or pc is None:
            return

        # World <- camera_optical transform, same lookup Detection3DModule uses.
        try:
            transform = self.tf.get("camera_optical", pc.frame_id, image.ts, 1.0)
        except Exception:
            transform = None
        if not transform:
            return

        try:
            parsed = ImageDetections2D.from_ros_detection2d_array(image, msg)
        except Exception:
            logger.exception("Marauder's Map: failed to parse detections")
            return

        # People (class 0) plus, optionally, other Go2-like quadrupeds (dog).
        persons = [det for det in parsed if int(det.class_id) == PERSON_CLASS_ID]
        dogs = (
            [det for det in parsed if int(det.class_id) == self.config.robot_dog_class_id]
            if self.config.detect_robot_dogs
            else []
        )

        # Same-frame co-occurrence = different people; feed this to reID so two
        # people standing together never collapse into one ID.
        if self._idsystem is not None and len(persons) > 1:
            try:
                self._idsystem.add_negative_constraints([int(d.track_id) for d in persons])
            except Exception:
                logger.exception("Marauder's Map: reID negative-constraint update failed")

        seen_now = time.time()
        for det in persons:
            center = self._project_center(det, pc, transform)
            if center is None:
                continue
            x, y = center
            ident = self._resolve_identity(det, x, y, seen_now)
            if ident is None:
                continue  # appearance reID not confident yet
            self._write_entry(ident, det, x, y, "person", seen_now)
            raw = raw_by_id.get(str(int(det.track_id)))
            if raw is not None:
                self._stable_to_raw_det[ident] = raw

        for det in dogs:
            center = self._project_center(det, pc, transform)
            if center is None:
                continue
            ident = DOG_ID_BASE + int(det.track_id)
            self._write_entry(ident, det, center[0], center[1], "dog", seen_now)
            raw = raw_by_id.get(str(int(det.track_id)))
            if raw is not None:
                self._stable_to_raw_det[ident] = raw

        self._expire_people()
        self._prune_stable(seen_now)
        self._emit("people", {"people": self._people_payload()})

    def _refresh_stable_raw(
        self, raw_dets: list[Any], raw_by_id: dict[str, Any]
    ) -> None:
        """Keep _stable_to_raw_det fresh on frames we throttled past.

        Doesn't run reID / projection — only re-binds the stable ids we
        already know to whatever raw detection currently carries the same
        track id, so a web click in this frame still produces a usable
        single-detection array.
        """
        for sid, prev_raw in list(self._stable_to_raw_det.items()):
            track_id = str(getattr(prev_raw, "id", "") or "")
            if not track_id:
                continue
            cur = raw_by_id.get(track_id)
            if cur is not None:
                self._stable_to_raw_det[sid] = cur

    def _project_center(
        self, det: Any, pc: PointCloud2, transform: Any
    ) -> tuple[float, float] | None:
        """Project a 2D detection to a world-frame (x, y) via the lidar map."""
        try:
            det3d = Detection3DPC.from_2d(
                det,
                world_pointcloud=pc,
                camera_info=self.config.camera_info,
                world_to_optical_transform=transform,
            )
        except Exception:
            det3d = None
        if det3d is None:
            return None
        return float(det3d.center.x), float(det3d.center.y)

    def _resolve_identity(self, det: Any, x: float, y: float, now: float) -> int | None:
        """Stable display id for a person.

        With appearance reID enabled, defer to it. Otherwise use spatial-temporal
        re-association on top of the detector's track id so the SAME person keeps
        one id even when the tracker hands out a fresh track id after a gap.
        """
        if self._idsystem is not None:
            try:
                lid = int(self._idsystem.register_detection(det))
            except Exception:
                logger.exception("Marauder's Map: reID register failed")
                return int(det.track_id)
            return None if lid < 0 else lid

        tid = int(det.track_id)
        sid = self._track_to_stable.get(tid)
        if sid is None:
            sid = self._reassociate(x, y, now)
            if sid is None:
                sid = self._next_stable
                self._next_stable += 1
            self._track_to_stable[tid] = sid
        self._stable_pos[sid] = {"x": x, "y": y, "last_seen": now}
        return sid

    def _reassociate(self, x: float, y: float, now: float) -> int | None:
        """Find a recently-lost (not currently active) stable id near (x, y)."""
        best: int | None = None
        best_d = self.config.reassoc_radius_m
        active_cut = now - 1.0  # an id seen within 1s is live -> never steal it
        for sid, info in self._stable_pos.items():
            last = info["last_seen"]
            if last > active_cut:
                continue
            if now - last > self.config.reassoc_window_s:
                continue
            d = math.hypot(x - info["x"], y - info["y"])
            if d < best_d:
                best_d, best = d, sid
        return best

    def _prune_stable(self, now: float) -> None:
        cutoff = now - self.config.reassoc_window_s
        dead = [s for s, i in self._stable_pos.items() if i["last_seen"] < cutoff]
        for s in dead:
            self._stable_pos.pop(s, None)
        for t in [t for t, s in self._track_to_stable.items() if s in dead]:
            self._track_to_stable.pop(t, None)

    def _write_entry(
        self, ident: int, det: Any, x: float, y: float, kind: str, seen_now: float
    ) -> None:
        """Update a person's roster/map entry."""
        entry = self._people.get(ident, {})
        entry.update(
            {
                "x": x,
                "y": y,
                "name": str(det.name),
                "class_id": int(det.class_id),
                "kind": kind,
                "last_seen": seen_now,
            }
        )
        # Refresh thumbnail + full-size crop occasionally (crop + jpeg is not free).
        if seen_now - entry.get("photo_ts", 0.0) > self.config.photo_refresh_sec:
            try:
                crop = det.cropped_image(padding=8)
                entry["photo"] = crop.to_base64(
                    quality=self.config.photo_quality,
                    max_width=self.config.photo_max_width,
                )
                # Larger version kept server-side; served on demand (not broadcast).
                entry["photo_full"] = crop.to_base64(
                    quality=self.config.photo_full_quality,
                    max_width=self.config.photo_full_max_width,
                )
                entry["photo_ts"] = seen_now
            except Exception:
                pass
        self._people[ident] = entry

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _detect_wifi_ssid(self) -> str:
        """Best-effort current WiFi SSID, used as this dog's label."""
        iface = self.config.wifi_interface
        cmds: list[list[str]] = []
        if sys.platform == "darwin":
            cmds = [
                ["networksetup", "-getairportnetwork", iface],
                ["ipconfig", "getsummary", iface],
            ]
        else:  # linux
            cmds = [["iwgetid", "-r"], ["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"]]

        for cmd in cmds:
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout
            except Exception:
                continue
            if not out:
                continue
            # networksetup: "Current Wi-Fi Network: dimair13"
            if "Current Wi-Fi Network:" in out:
                return out.split("Current Wi-Fi Network:", 1)[1].strip().splitlines()[0]
            # ipconfig getsummary: a line "  SSID : dimair13"
            for line in out.splitlines():
                s = line.strip()
                if s.upper().startswith("SSID") and ":" in s:
                    return s.split(":", 1)[1].strip()
                # nmcli "yes:dimair13"
                if s.startswith("yes:"):
                    return s.split(":", 1)[1].strip()
            # iwgetid -r prints the bare SSID
            if cmd[0] == "iwgetid" and out.strip():
                return out.strip().splitlines()[0]
        return ""

    def _expire_people(self) -> None:
        cutoff = time.time() - self.config.person_ttl_sec
        for tid in [t for t, e in self._people.items() if e.get("last_seen", 0) < cutoff]:
            del self._people[tid]
            self._stable_to_raw_det.pop(tid, None)

    def _people_payload(self) -> list[dict[str, Any]]:
        # Note: the large `photo_full` is intentionally excluded here -- it is
        # served on demand via the `get_photo` event to keep this broadcast lean.
        out = []
        for tid, e in self._people.items():
            out.append(
                {
                    "id": tid,
                    "x": e.get("x"),
                    "y": e.get("y"),
                    "name": e.get("name"),
                    "kind": e.get("kind", "person"),
                    "seen_by": self._label,
                    "photo": e.get("photo"),
                    "age": round(time.time() - e.get("last_seen", time.time()), 1),
                }
            )
        return out

    def _encode_map(self, grid: OccupancyGrid) -> dict[str, Any]:
        """Render the occupancy grid to an RGBA PNG of ink walls (transparent
        elsewhere) and return it plus the world->pixel mapping metadata."""
        g = np.asarray(grid.grid)
        h, w = g.shape
        rgba = np.zeros((h, w, 4), dtype=np.uint8)

        occupied = g >= 50
        free = (g >= 0) & (g < 50)
        # Ink walls (dark sepia), faint inked floor, transparent unknown.
        rgba[occupied] = (40, 30, 22, 255)
        rgba[free] = (90, 70, 45, 40)

        ok, buf = cv2.imencode(".png", cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8") if ok else ""

        origin = grid.info.origin
        return {
            "img": img_b64,
            "width": int(w),
            "height": int(h),
            "resolution": float(grid.info.resolution),
            "origin": [float(origin.position.x), float(origin.position.y)],
            "label": self._label,
        }

    def _publish_selection(self, ident: int | None) -> None:
        """Drive BBoxSelectionModule by synthesizing a camera-pixel click.

        Selecting:   publish a ``PointStamped`` at the *center* of the chosen
                     person's bbox with ``frame_id="color_image/web_click"``.
                     BBoxSelectionModule's ``clicked_point`` handler accepts
                     any frame_id containing ``color_image``, hit-tests the
                     pixel against current detections, and locks onto the
                     matching bbox -- the exact same path Rerun camera clicks
                     take, with no upstream code touched.

        Deselecting: publish a ``PointStamped`` far outside any detection
                     box. BBoxSelectionModule's "click did not hit any bbox"
                     branch then clears the selection naturally.
        """
        ts = time.time()
        if ident is None:
            # A guaranteed miss; finite numbers so the non-finite filter passes
            # but the hit-test rejects -> clears selection.
            click = PointStamped(
                ts=ts, frame_id="color_image/web_click", x=-1.0e6, y=-1.0e6, z=0.0
            )
            try:
                self.clicked_point.publish(click)
            except Exception:
                logger.exception("Marauder's Map: deselect publish failed")
            return

        raw = self._stable_to_raw_det.get(ident)
        if raw is None:
            logger.info("Marauder's Map: no live detection for id=%s", ident)
            return
        try:
            center = raw.bbox.center.position
            click = PointStamped(
                ts=ts,
                frame_id="color_image/web_click",
                x=float(center.x),
                y=float(center.y),
                z=0.0,
            )
            self.clicked_point.publish(click)
        except Exception:
            logger.exception("Marauder's Map: select publish failed")

    def _emit(self, event: str, data: Any) -> None:
        if (
            self.sio is not None
            and self._broadcast_loop is not None
            and not self._broadcast_loop.is_closed()
        ):
            asyncio.run_coroutine_threadsafe(self.sio.emit(event, data), self._broadcast_loop)


__all__ = ["ReidMapConfig", "ReidMapModule"]
