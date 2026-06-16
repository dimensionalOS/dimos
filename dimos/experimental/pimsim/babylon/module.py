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

from __future__ import annotations

import asyncio
from collections.abc import Callable
from contextlib import asynccontextmanager, suppress
import math
import os
from pathlib import Path
import struct
import threading
import time
from typing import Any, Literal

import lcm as lcmlib
import numpy as np
from reactivex.disposable import Disposable
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import FileResponse, HTMLResponse, JSONResponse, Response
from starlette.routing import Mount, Route, WebSocketRoute
from starlette.staticfiles import StaticFiles
from starlette.websockets import WebSocket, WebSocketDisconnect
import uvicorn

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.experimental.pimsim.babylon.browser import STATIC_DIR, index_html
from dimos.experimental.pimsim.babylon.config import (
    CoordinatorControlSpec,
    HumanoidControlSpec,
    MujocoRespawnSpec,
)
from dimos.experimental.pimsim.babylon.geometry import (
    compose_scene_mesh_wxyz,
    dimos_joint_to_mjcf,
    media_type,
    path_contains,
)
from dimos.experimental.pimsim.babylon.robot_meshes import (
    RobotMeshes,
    apply_state,
    load_robot_meshes,
)
from dimos.experimental.pimsim.entity import (
    EntityDescriptor,
    EntityState,
    EntityStateBatch,
    pose_from_wire,
    pose_to_wire,
    twist_to_wire,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEFAULT_BROADCAST_HZ = 30.0
_DEFAULT_PORT = 8091
_POSE_POSITION_EPSILON = 1e-5
_POSE_QUATERNION_EPSILON = 1e-5
# Binary websocket message tags. Pointcloud (0x02) and camera (0x01) were
# removed when /global_map and /camera_image migrated to the LCM<->WS
# bridge; the bridge forwards camera frames as JPEG-encoded Image LCM
# packets (publisher uses JpegLcmTransport) and the browser displays them
# via createObjectURL. Only robot_pose remains on /ws pending browser FK.
_WS_MSG_ROBOT_POSE = 0x03
_WS_ROBOT_POSE_HEADER_BYTES = 16
# Per-message WS send timeout. The browser tab momentarily failing to drain
# TCP (Chrome backgrounding, GC pause, App Nap) shouldn't permanently wedge
# the in-flight gate: time out, drop the client, let the JS reconnect.
_WS_SEND_TIMEOUT_S = 1.0
# Best-effort close after a timed-out send. If the kernel buffer is full the
# close frame may not flush either; uvicorn's keepalive will then reap.
_WS_CLOSE_TIMEOUT_S = 0.5
_NO_CACHE_HEADERS = {
    "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
    "Pragma": "no-cache",
    "Expires": "0",
}


def _asset_token(path: Path) -> str:
    stat = path.stat()
    return f"{stat.st_mtime_ns:x}-{stat.st_size:x}"


def _versioned_asset_name(prefix: str, path: Path) -> str:
    return f"{prefix}-{_asset_token(path)}{path.suffix.lower()}"


def _legacy_asset_name(prefix: str, path: Path) -> str:
    return f"{prefix}{path.suffix.lower()}"


def _matches_asset_name(asset_name: str, prefix: str, path: Path) -> bool:
    suffix = path.suffix.lower()
    return asset_name == _legacy_asset_name(prefix, path) or (
        asset_name.startswith(f"{prefix}-") and asset_name.endswith(suffix)
    )


class BabylonSceneViewerModule(Module):
    # /joint_state and /odom are still consumed by the server because forward
    # kinematics (-> _make_robot_pose_payload) runs server-side. The browser
    # subscribes to these directly via /lcm-ws too, so once FK moves to the
    # browser these In[] decls go away.
    joint_state: In[JointState]
    odom: In[PoseStamped]
    # Entity world (browser is authoritative; these republish for dimos consumers).
    entity_descriptors: Out[EntityDescriptor]
    # Aggregated per-tick snapshot — single source for cross-process
    # consumers like the rust scene_lidar.
    entity_state_batch: Out[EntityStateBatch]
    _mujoco_sim: MujocoRespawnSpec | None = None
    _robot_ctrl: HumanoidControlSpec | None = None
    _coordinator_ctrl: CoordinatorControlSpec | None = None

    def __init__(
        self,
        mjcf_path: str | Path,
        *,
        port: int = _DEFAULT_PORT,
        assets: dict[str, bytes] | None = None,
        scene_path: str | Path | None = None,
        browser_collision_path: str | Path | None = None,
        splat_path: str | Path | None = None,
        splat_alignment: dict[str, Any] | None = None,
        scene_scale: float = 1.0,
        scene_translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
        scene_rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0),
        scene_y_up: bool = True,
        broadcast_hz: float = _DEFAULT_BROADCAST_HZ,
        # Browser-physics-only base. Set enable_sim=True to have the browser
        # own collision/pose integration and publish sim_odom back through
        # this module.
        enable_sim: bool = False,
        sim_rate: float = 100.0,
        vehicle_height: float = 0.75,
        step_offset: float = 0.22,
        support_floor: bool = True,
        support_floor_z: float | None = None,
        support_floor_size: float = 0.0,
        init_x: float = 0.0,
        init_y: float = 0.0,
        init_z: float = 0.0,
        init_yaw: float = 0.0,
        lock_z: bool = True,
        initial_entities: list[dict[str, Any]] | None = None,
        entity_authority: Literal["browser", "external"] = "browser",
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        # "browser": Havok owns entity physics, browser states republish on
        # entity_state_batch (historic behavior). "external": an external
        # sim (MuJoCo) publishes /entity_state_batch; browser entities spawn
        # kinematic and mirror those poses, and browser-reported states are
        # ignored.
        self._entity_authority = entity_authority
        self._mjcf_path = Path(mjcf_path)
        self._assets = assets
        self._port = port
        self._scene_path = Path(scene_path) if scene_path else None
        self._browser_collision_path = (
            Path(browser_collision_path) if browser_collision_path else None
        )
        self._splat_path = Path(splat_path) if splat_path else None
        self._splat_alignment = splat_alignment or {}
        self._scene_scale = scene_scale
        self._scene_translation = scene_translation
        self._scene_rotation_zyx_deg = scene_rotation_zyx_deg
        self._scene_y_up = scene_y_up
        self._broadcast_dt = 1.0 / float(broadcast_hz)

        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_base_pos: np.ndarray | None = None
        self._latest_base_wxyz: np.ndarray | None = None
        self._robot_pose_lock = threading.Lock()
        self._last_robot_pose_values: np.ndarray | None = None

        self._robot: RobotMeshes | None = None
        self._uvicorn_server: uvicorn.Server | None = None
        self._server_thread: threading.Thread | None = None
        self._broadcast_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._server_loop: asyncio.AbstractEventLoop | None = None
        self._clients: set[WebSocket] = set()
        # Per-client send lock. Starlette's WebSocket is not safe for
        # concurrent sends; without serialization the broadcast loop and
        # the initial replay in _websocket can race on the same socket
        # and break Starlette's ASGI state machine. Locks are created /
        # cleaned in _websocket; helpers below assume an entry exists.
        self._ws_send_locks: dict[WebSocket, asyncio.Lock] = {}

        # @dimos/msgs-compatible LCM <-> WebSocket bridge. Lives on /lcm-ws of
        # this same Starlette app so the browser doesn't need a second port.
        # Forwards raw LCM packets in both directions; subscribers/publishers
        # on the bus see no difference between browser-published messages and
        # any other peer.
        self._lcm_bus: lcmlib.LCM | None = None
        self._lcm_subscription: Any = None
        self._lcm_handle_thread: threading.Thread | None = None
        self._lcm_clients: set[WebSocket] = set()
        self._lcm_seq: int = 0
        # Per-client latest-packet-per-channel buffer + wake event + drain
        # task. Replaces the previous "schedule one coroutine per LCM
        # packet" pattern: under load that backed up asyncio with hundreds
        # of pending tasks, putting the browser 10-15s behind real time,
        # and the legacy `websockets` library asserted on concurrent
        # drains when uvicorn's keepalive ping collided with our sends.
        # Now there is exactly one in-flight send per client at a time,
        # and the LCM thread just overwrites the per-channel slot
        # (latest-wins, old packets drop on the floor) so the bridge runs
        # at the WebSocket's drain rate instead of the bus's emit rate.
        self._lcm_pending: dict[WebSocket, dict[str, bytes]] = {}
        self._lcm_wake: dict[WebSocket, asyncio.Event] = {}
        self._lcm_drain_tasks: dict[WebSocket, asyncio.Task[None]] = {}
        # Per-channel forward rate caps (min seconds between packets sent
        # to any one browser client for that channel). The bus keeps
        # emitting at its native rate; we just throttle the WS leg so a
        # heavy producer (e.g. the rust voxel mapper at 10 Hz) doesn't
        # eat the browser's budget. None of this affects in-process
        # consumers like the nav stack or rerun.
        _gm_hz = float(os.environ.get("DIMOS_PIMSIM_GLOBAL_MAP_HZ", "1.0"))
        self._lcm_channel_min_interval_s: dict[str, float] = {
            "/global_map#sensor_msgs.PointCloud2": 1.0 / _gm_hz if _gm_hz > 0 else 0.0,
        }
        self._lcm_last_forward_ts: dict[WebSocket, dict[str, float]] = {}

        self._browser_physics_enabled = enable_sim
        self._browser_sim_rate = float(sim_rate)
        self._browser_vehicle_height = float(vehicle_height)
        self._browser_step_offset = float(step_offset)
        self._browser_support_floor = bool(support_floor)
        self._browser_support_floor_z = float(
            init_z if support_floor_z is None else support_floor_z
        )
        self._browser_support_floor_size = float(support_floor_size)
        self._browser_initial_pose = {
            "x": float(init_x),
            "y": float(init_y),
            "z": float(init_z if not lock_z else init_z + vehicle_height),
            "yaw": float(init_yaw),
            "lockZ": bool(lock_z),
        }
        self._initial_entities = initial_entities or []
        self._entity_asset_paths = self._collect_entity_asset_paths(self._initial_entities)

        # Entity world. The browser owns physics state; this table is a
        # local mirror used for (a) reconnect replay (so a fresh tab can
        # rebuild the world) and (b) `list_entities` queries.
        self._entity_lock = threading.Lock()
        self._entities: dict[str, EntityDescriptor] = {}
        # Latest pose per entity, sourced from browser entity_states msgs.
        # Used to build the aggregated EntityStateBatch the rust scene_lidar
        # subscribes to.
        self._entity_poses: dict[str, Pose] = {}
        self._test_entity_counter = 0

    @rpc
    def start(self) -> None:
        super().start()

        self._robot = load_robot_meshes(self._mjcf_path, assets=self._assets)
        app = self._create_app()
        config = uvicorn.Config(app, host="0.0.0.0", port=self._port, log_level="warning")
        self._uvicorn_server = uvicorn.Server(config)
        self._server_thread = threading.Thread(
            target=self._uvicorn_server.run,
            name="babylon-viewer-server",
            daemon=True,
        )
        self._server_thread.start()

        # joint_state + odom drive server-side FK (-> _make_robot_pose_payload).
        # The browser also subscribes to these on /lcm-ws for its HUD sliders
        # and (eventually) for browser-side FK. /global_map, /nav_path, and
        # /nav_cmd_vel are consumed entirely on the browser via the bridge.
        self.register_disposable(Disposable(self.joint_state.subscribe(self._on_joint_state)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))

        self._install_initial_entities()

        self._broadcast_thread = threading.Thread(
            target=self._broadcast_loop,
            name="babylon-viewer-broadcast",
            daemon=True,
        )
        self._broadcast_thread.start()

        # LCM <-> WS bridge for browser. Subscribes to the bus once, forwards
        # raw packets to every /lcm-ws client. Receives raw LCM packets the
        # browser publishes (encoded with @dimos/msgs) and republishes them.
        self._lcm_bus = lcmlib.LCM()
        self._lcm_subscription = self._lcm_bus.subscribe(".*", self._on_lcm_bus_msg)
        self._lcm_subscription.set_queue_capacity(10000)
        self._lcm_handle_thread = threading.Thread(
            target=self._lcm_handle_loop,
            name="babylon-viewer-lcm-bridge",
            daemon=True,
        )
        self._lcm_handle_thread.start()

        logger.info("Babylon scene viewer: http://localhost:%s/", self._port)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._uvicorn_server is not None:
            self._uvicorn_server.should_exit = True
        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=2.0)
        if self._lcm_handle_thread and self._lcm_handle_thread.is_alive():
            self._lcm_handle_thread.join(timeout=2.0)
        if self._server_thread and self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        super().stop()

    def _create_app(self) -> Starlette:
        @asynccontextmanager
        async def _lifespan(_app: Starlette) -> Any:
            self._server_loop = asyncio.get_running_loop()
            yield

        return Starlette(
            routes=[
                Route("/", self._index),
                Route("/config.json", self._config),
                Route("/robot.json", self._robot_json),
                Route("/arms.json", self._arms_json),
                Route("/viewer_debug", self._viewer_debug, methods=["POST"]),
                Route("/assets/{asset_name:path}", self._asset),
                Mount("/static", app=StaticFiles(directory=STATIC_DIR), name="static"),
                WebSocketRoute("/ws", self._websocket),
                WebSocketRoute("/lcm-ws", self._lcm_websocket),
            ],
            lifespan=_lifespan,
        )

    async def _arms_json(self, request: Request) -> JSONResponse:
        """Joint-limit catalogue so the page can build sliders with real range."""
        if self._robot_ctrl is None:
            return JSONResponse({"joints": []}, headers=_NO_CACHE_HEADERS)
        try:
            limits = self._robot_ctrl.arm_joint_limits()
        except Exception as exc:
            logger.warning("BabylonViewer: arm_joint_limits() failed: %s", exc)
            return JSONResponse({"joints": []}, headers=_NO_CACHE_HEADERS)
        joints = [{"name": name, "min": float(lo), "max": float(hi)} for (name, lo, hi) in limits]
        return JSONResponse({"joints": joints}, headers=_NO_CACHE_HEADERS)

    async def _index(self, request: Request) -> HTMLResponse:
        html = index_html()
        for asset_name in ("style.css", "ui.js", "app.js", "lcm_client.js"):
            asset_path = STATIC_DIR / asset_name
            if not asset_path.exists():
                continue
            token = _asset_token(asset_path)
            html = html.replace(
                f'"/static/{asset_name}"',
                f'"/static/{asset_name}?v={token}"',
            )
        return HTMLResponse(html, headers=_NO_CACHE_HEADERS)

    async def _config(self, request: Request) -> JSONResponse:
        scene_file = None
        scene_bytes = 0
        collision_file = None
        collision_bytes = 0
        scene_wxyz = compose_scene_mesh_wxyz(
            y_up=self._scene_y_up,
            rotation_zyx_deg=self._scene_rotation_zyx_deg,
        )
        if self._scene_path is not None and self._scene_path.exists():
            scene_file = _versioned_asset_name("scene", self._scene_path)
            scene_bytes = self._scene_path.stat().st_size
        if self._browser_collision_path is not None and self._browser_collision_path.exists():
            collision_file = _versioned_asset_name("collision", self._browser_collision_path)
            collision_bytes = self._browser_collision_path.stat().st_size
        splat_file = None
        splat_bytes = 0
        if self._splat_path is not None and self._splat_path.exists():
            splat_file = _versioned_asset_name("splat", self._splat_path)
            splat_bytes = self._splat_path.stat().st_size
        return JSONResponse(
            {
                "sceneFile": scene_file,
                "sceneBytes": scene_bytes,
                "collisionSceneFile": collision_file,
                "collisionSceneBytes": collision_bytes,
                "splatFile": splat_file,
                "splatBytes": splat_bytes,
                "splatAlignment": self._splat_alignment,
                "sceneScale": self._scene_scale,
                "scenePosition": list(self._scene_translation),
                "sceneWxyz": list(scene_wxyz),
                "browserPhysics": self._browser_physics_enabled,
                "entityAuthority": self._entity_authority,
                "browserPhysicsHz": self._browser_sim_rate,
                "browserPhysicsInitialPose": self._browser_initial_pose,
                "vehicleHeight": self._browser_vehicle_height,
                "stepOffset": self._browser_step_offset,
                "supportFloor": self._browser_support_floor,
                "supportFloorZ": self._browser_support_floor_z,
                "supportFloorSize": self._browser_support_floor_size,
            },
            headers=_NO_CACHE_HEADERS,
        )

    async def _viewer_debug(self, request: Request) -> JSONResponse:
        try:
            message = await request.json()
        except Exception:
            return JSONResponse({"ok": False}, status_code=400, headers=_NO_CACHE_HEADERS)

        label = message.get("label", "viewer")
        payload = message.get("payload")
        if isinstance(payload, dict):
            logger.info("BabylonViewer debug", label=label, **payload)
        return JSONResponse({"ok": True}, headers=_NO_CACHE_HEADERS)

    async def _robot_json(self, request: Request) -> JSONResponse:
        robot = self._robot
        if robot is None:
            return JSONResponse({"bodyNames": [], "geoms": []}, headers=_NO_CACHE_HEADERS)

        geoms: list[dict[str, Any]] = []
        for index, geom in enumerate(robot.geoms):
            geoms.append(
                {
                    "id": index,
                    "body": geom.body_name,
                    "vertices": geom.vertices.astype(np.float32).reshape(-1).tolist(),
                    "indices": geom.faces.astype(np.int32).reshape(-1).tolist(),
                    "position": geom.local_pos.astype(np.float32).tolist(),
                    "wxyz": geom.local_wxyz.astype(np.float32).tolist(),
                    "rgba": [float(value) for value in geom.rgba],
                }
            )
        return JSONResponse(
            {"bodyNames": robot.body_names, "geoms": geoms},
            headers=_NO_CACHE_HEADERS,
        )

    async def _asset(self, request: Request) -> Response:
        has_scene_asset = self._scene_path is not None and self._scene_path.exists()
        has_collision_asset = (
            self._browser_collision_path is not None and self._browser_collision_path.exists()
        )
        has_splat_asset = self._splat_path is not None and self._splat_path.exists()
        if (
            not has_scene_asset
            and not has_collision_asset
            and not has_splat_asset
            and not self._entity_asset_paths
        ):
            return Response("scene asset not configured", status_code=404)

        asset_name = request.path_params["asset_name"]
        entity_asset = self._entity_asset_paths.get(asset_name)
        if entity_asset is not None and entity_asset.exists():
            return FileResponse(
                entity_asset,
                media_type=media_type(entity_asset),
                headers=_NO_CACHE_HEADERS,
            )

        if self._splat_path is not None and _matches_asset_name(
            asset_name, "splat", self._splat_path
        ):
            return FileResponse(
                self._splat_path,
                media_type="application/octet-stream",
                headers=_NO_CACHE_HEADERS,
            )

        if self._scene_path is not None and _matches_asset_name(
            asset_name,
            "scene",
            self._scene_path,
        ):
            return FileResponse(
                self._scene_path,
                media_type=media_type(self._scene_path),
                headers=_NO_CACHE_HEADERS,
            )

        if self._browser_collision_path is not None and _matches_asset_name(
            asset_name, "collision", self._browser_collision_path
        ):
            return FileResponse(
                self._browser_collision_path,
                media_type=media_type(self._browser_collision_path),
                headers=_NO_CACHE_HEADERS,
            )

        if self._scene_path is not None and self._scene_path.suffix.lower() == ".gltf":
            candidate = self._scene_path.parent / asset_name
            if path_contains(self._scene_path.parent, candidate) and candidate.exists():
                return FileResponse(
                    candidate,
                    media_type=media_type(candidate),
                    headers=_NO_CACHE_HEADERS,
                )

        if (
            self._browser_collision_path is not None
            and self._browser_collision_path.suffix.lower() == ".gltf"
        ):
            candidate = self._browser_collision_path.parent / asset_name
            if path_contains(self._browser_collision_path.parent, candidate) and candidate.exists():
                return FileResponse(
                    candidate,
                    media_type=media_type(candidate),
                    headers=_NO_CACHE_HEADERS,
                )

        return Response("asset not found", status_code=404)

    async def _websocket(self, websocket: WebSocket) -> None:
        # Control-plane / camera channel. Display topics (pointcloud, path,
        # joint state, nav cmd_vel) and browser-authored state (sim_odom,
        # cmd_vel, clicked_point, point_goal, entity world) flow through
        # /lcm-ws via @dimos/msgs; only the JSON RPC for things that touch
        # in-process Python state (respawn, arm control, coordinator
        # activate/dry-run, multi-tab entity replay) lives here now.
        await websocket.accept()
        self._ws_send_locks[websocket] = asyncio.Lock()
        self._clients.add(websocket)
        logger.info("BabylonViewer: websocket connected", clients=len(self._clients))
        try:
            robot_pose_payload = self._make_robot_pose_payload(force=True)
            if robot_pose_payload is not None:
                await self._send_bytes_locked(websocket, robot_pose_payload)
            # Replay entity descriptors so a fresh tab rebuilds the world
            # the browser-side physics is otherwise oblivious to.
            for spawn in self._entity_spawn_messages():
                await self._send_json_locked(websocket, spawn)
            while True:
                message = await websocket.receive_json()
                self._handle_client_message(message)
        except WebSocketDisconnect:
            pass
        finally:
            self._clients.discard(websocket)
            self._ws_send_locks.pop(websocket, None)
            logger.info("BabylonViewer: websocket disconnected", clients=len(self._clients))

    # ---- LCM <-> WebSocket bridge ---------------------------------------
    #
    # @dimos/msgs in the browser speaks the standard LCM small-message wire
    # format: [magic u32 BE][seq u32 BE][channel utf-8][\0][payload]. The
    # bridge synthesises that on the way out (since the Python LCM library
    # hands us channel + payload separately) and parses it on the way in.
    # Fragmented LCM messages (>~64KB) are not yet supported here.

    _LCM_MAGIC_SHORT = 0x4C433032  # "LC02"

    def _lcm_handle_loop(self) -> None:
        if self._lcm_bus is None:
            return
        while not self._stop_event.is_set():
            try:
                self._lcm_bus.handle_timeout(100)
            except Exception:
                logger.exception("BabylonViewer: lcm handle failed")

    def _on_lcm_bus_msg(self, channel: str, data: bytes) -> None:
        # Runs on the LCM library's reader thread. We don't synthesise the
        # full small-message packet here (the seq counter has to advance
        # in some order, and the per-client drain serialises naturally on
        # the asyncio loop) — just hand channel + payload over via
        # call_soon_threadsafe and let the loop thread do the rest.
        if self._entity_authority == "external" and channel.startswith("/entity_state_batch#"):
            self._mirror_external_entity_states(data)
        if not self._lcm_clients:
            return
        loop = self._server_loop
        if loop is None:
            return
        loop.call_soon_threadsafe(self._lcm_enqueue, channel, data)

    def _mirror_external_entity_states(self, data: bytes) -> None:
        """Track the external sim's entity poses so reconnect replay spawns
        entities where they currently are, not at their cooked poses."""
        try:
            batch = EntityStateBatch.decode(data)
        except Exception:
            return
        with self._entity_lock:
            for descriptor, pose in batch.entries:
                if descriptor.entity_id in self._entities:
                    self._entity_poses[descriptor.entity_id] = pose

    def _lcm_enqueue(self, channel: str, data: bytes) -> None:
        # Runs on the asyncio loop thread. Synthesise the LC02 wire bytes
        # once, then for each client overwrite its latest-packet slot for
        # this channel and wake its drain task. When the bus emits faster
        # than the browser drains, the slot just keeps getting overwritten
        # — old packets drop on the floor (latest-wins, no queue growth).
        if not self._lcm_clients:
            return
        min_interval = self._lcm_channel_min_interval_s.get(channel, 0.0)
        now = time.monotonic() if min_interval > 0.0 else 0.0
        chan_bytes = channel.encode("utf-8")
        packet = (
            struct.pack(">II", self._LCM_MAGIC_SHORT, self._lcm_seq) + chan_bytes + b"\x00" + data
        )
        self._lcm_seq = (self._lcm_seq + 1) & 0xFFFFFFFF
        for websocket in tuple(self._lcm_clients):
            pending = self._lcm_pending.get(websocket)
            wake = self._lcm_wake.get(websocket)
            if pending is None or wake is None:
                continue
            if min_interval > 0.0:
                last = self._lcm_last_forward_ts.get(websocket, {})
                if now - last.get(channel, 0.0) < min_interval:
                    continue  # rate-cap: drop this packet for this client
                last[channel] = now
            pending[channel] = packet
            wake.set()

    async def _lcm_drain_loop(self, websocket: WebSocket) -> None:
        # One of these runs per connected /lcm-ws client. It waits for the
        # wake event, snapshots the per-channel pending dict, clears it,
        # then sends every captured packet through the per-client send
        # lock (so we never compete with our own sends or uvicorn's
        # keepalive ping). If any send fails (timeout or socket error)
        # we close the client and exit; the receive coroutine wakes on
        # the disconnect and runs the normal cleanup.
        pending = self._lcm_pending.get(websocket)
        wake = self._lcm_wake.get(websocket)
        if pending is None or wake is None:
            return
        try:
            while True:
                await wake.wait()
                wake.clear()
                if not pending:
                    continue
                snapshot = list(pending.values())
                pending.clear()
                for packet in snapshot:
                    await self._send_bytes_locked(websocket, packet)
        except asyncio.CancelledError:
            pass
        except Exception:
            logger.warning(
                "BabylonViewer: lcm-ws drain failed, closing client",
                exc_info=True,
            )
            with suppress(Exception):
                await asyncio.wait_for(websocket.close(), timeout=_WS_CLOSE_TIMEOUT_S)

    async def _lcm_websocket(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._ws_send_locks[websocket] = asyncio.Lock()
        self._lcm_pending[websocket] = {}
        self._lcm_wake[websocket] = asyncio.Event()
        self._lcm_last_forward_ts[websocket] = {}
        self._lcm_clients.add(websocket)
        drain_task = asyncio.create_task(self._lcm_drain_loop(websocket))
        self._lcm_drain_tasks[websocket] = drain_task
        logger.info("BabylonViewer: lcm-ws connected", clients=len(self._lcm_clients))
        try:
            while True:
                packet = await websocket.receive_bytes()
                self._publish_lcm_packet(packet)
        except WebSocketDisconnect:
            pass
        except Exception:
            logger.exception("BabylonViewer: lcm-ws receive failed")
        finally:
            self._lcm_clients.discard(websocket)
            self._ws_send_locks.pop(websocket, None)
            self._lcm_pending.pop(websocket, None)
            self._lcm_wake.pop(websocket, None)
            self._lcm_last_forward_ts.pop(websocket, None)
            task = self._lcm_drain_tasks.pop(websocket, None)
            if task is not None:
                task.cancel()
                with suppress(asyncio.CancelledError, Exception):
                    await task
            logger.info("BabylonViewer: lcm-ws disconnected", clients=len(self._lcm_clients))

    def _publish_lcm_packet(self, packet: bytes) -> None:
        if self._lcm_bus is None:
            return
        if len(packet) < 8:
            return
        magic, _seq = struct.unpack(">II", packet[:8])
        if magic != self._LCM_MAGIC_SHORT:
            logger.warning("BabylonViewer: dropping non-small LCM packet from browser")
            return
        try:
            null_idx = packet.index(b"\x00", 8)
        except ValueError:
            return
        channel = packet[8:null_idx].decode("utf-8", errors="replace")
        payload = packet[null_idx + 1 :]
        self._lcm_bus.publish(channel, payload)

    # ---- existing JSON control plane ------------------------------------

    def _handle_client_message(self, message: dict[str, Any]) -> None:
        # All bus-level publishes (cmd_vel, sim_odom -> /odom, clicked_point,
        # point_goal, entity_state_batch) now flow browser -> /lcm-ws -> bus
        # directly. What stays here is JSON RPC for things that touch
        # in-process Python state: sim respawn, arm-joint mutators, the
        # coordinator engage/dry-run toggles, and the multi-tab entity replay.
        message_type = message.get("type")
        if message_type == "respawn":
            logger.info(
                "BabylonViewer: respawn requested",
                has_mujoco_sim=self._mujoco_sim is not None,
            )
            if self._mujoco_sim is not None:
                self._respawn_with_policy_reset(self._mujoco_sim.respawn)
            if self._browser_physics_enabled:
                self._broadcast_json_from_thread(
                    {"type": "sim_respawn", "pose": self._browser_initial_pose}
                )
            return
        if message_type == "respawn_at":
            point = message.get("point")
            if not isinstance(point, list) or len(point) < 2:
                return
            try:
                x = float(point[0])
                y = float(point[1])
                z = float(point[2]) if len(point) > 2 else 0.0
            except (TypeError, ValueError):
                return
            logger.info(
                "BabylonViewer: respawn_at requested",
                x=x,
                y=y,
                z=z,
                has_mujoco_sim=self._mujoco_sim is not None,
            )
            if self._mujoco_sim is not None:
                self._respawn_with_policy_reset(lambda: self._mujoco_sim.respawn_at(x, y))
            if self._browser_physics_enabled:
                pose = dict(self._browser_initial_pose)
                pose.update({"x": x, "y": y, "z": z + self._browser_vehicle_height})
                self._broadcast_json_from_thread({"type": "sim_respawn", "pose": pose})
            return
        if message_type == "arm_joint":
            name = message.get("name")
            position = message.get("position")
            if (
                self._robot_ctrl is None
                or not isinstance(name, str)
                or not isinstance(position, (int, float))
            ):
                return
            self._robot_ctrl.set_arm_joint(name, float(position))
            return
        if message_type == "release_arms":
            if self._robot_ctrl is not None:
                self._robot_ctrl.release_arms()
            return
        if message_type == "set_activated":
            engaged = bool(message.get("engaged", False))
            if self._coordinator_ctrl is None:
                logger.warning("BabylonViewer: set_activated requested but no coordinator wired")
                return
            logger.info("BabylonViewer: set_activated=%s", engaged)
            self._coordinator_ctrl.set_activated(engaged=engaged)
            return
        if message_type == "set_dry_run":
            enabled = bool(message.get("enabled", False))
            if self._coordinator_ctrl is None:
                logger.warning("BabylonViewer: set_dry_run requested but no coordinator wired")
                return
            logger.info("BabylonViewer: set_dry_run=%s", enabled)
            self._coordinator_ctrl.set_dry_run(enabled=enabled)
            return
        if message_type == "entity_states":
            self._publish_entity_states(message.get("states") or [])
            return
        if message_type == "entity_test_add":
            self._handle_entity_test_add(message.get("point") or [0.0, 0.0, 2.0])
            return
        if message_type == "entity_spawn":
            self._handle_entity_spawn(message)
            return
        if message_type == "entity_add_wall":
            self._handle_entity_add_wall(message)
            return
        if message_type == "entity_clear":
            self._handle_entity_clear()
            return
        if message_type == "viewer_debug":
            label = message.get("label", "viewer")
            payload = message.get("payload")
            if isinstance(payload, dict):
                logger.info(
                    "BabylonViewer debug",
                    label=label,
                    **payload,
                )
            return

    def _respawn_with_policy_reset(self, respawn: Callable[[], bool]) -> bool:
        if self._coordinator_ctrl is not None:
            self._coordinator_ctrl.set_activated(engaged=False)
        try:
            return respawn()
        finally:
            if self._coordinator_ctrl is not None:
                self._coordinator_ctrl.set_activated(engaged=True)

    def _broadcast_loop(self) -> None:
        # Still here only because forward kinematics runs server-side and the
        # browser consumes the resulting per-body pose array as a binary
        # frame on /ws. Once the browser does its own FK from /joint_state
        # + /odom (both already available via /lcm-ws), this loop and the
        # robot_pose binary frame both go.
        while not self._stop_event.is_set():
            loop = self._server_loop
            if loop is not None and self._clients:
                robot_pose_payload = self._make_robot_pose_payload()
                if robot_pose_payload is not None:
                    asyncio.run_coroutine_threadsafe(
                        self._broadcast_robot_pose(robot_pose_payload),
                        loop,
                    )
            time.sleep(self._broadcast_dt)

    async def _broadcast_robot_pose(self, robot_pose_payload: bytes) -> None:
        dead: list[WebSocket] = []
        for websocket in tuple(self._clients):
            try:
                await self._send_bytes_locked(websocket, robot_pose_payload)
            except Exception:
                dead.append(websocket)
        await self._drop_clients(dead)

    def _make_robot_pose_payload(self, *, force: bool = False) -> bytes | None:
        robot = self._robot
        if robot is None:
            return None

        with self._state_lock:
            joints = dict(self._latest_joints)
            base_pos = None if self._latest_base_pos is None else self._latest_base_pos.copy()
            base_wxyz = None if self._latest_base_wxyz is None else self._latest_base_wxyz.copy()

        with self._robot_pose_lock:
            apply_state(
                robot,
                base_pos=base_pos,
                base_wxyz=base_wxyz,
                joint_positions=joints,
            )

            body_count = len(robot.body_names)
            poses = np.empty((body_count, 7), dtype=np.float32)
            poses[:, 0:3] = robot.data.xpos[:body_count].astype(np.float32, copy=False)
            poses[:, 3:7] = robot.data.xquat[:body_count].astype(np.float32, copy=False)

            previous = self._last_robot_pose_values
            if not force and previous is not None and previous.shape == poses.shape:
                position_delta = np.max(np.abs(poses[:, 0:3] - previous[:, 0:3]))
                quaternion_delta = np.max(np.abs(poses[:, 3:7] - previous[:, 3:7]))
                if (
                    position_delta <= _POSE_POSITION_EPSILON
                    and quaternion_delta <= _POSE_QUATERNION_EPSILON
                ):
                    return None

            self._last_robot_pose_values = poses.copy()

        header = struct.pack(
            ">B3xId",
            _WS_MSG_ROBOT_POSE,
            body_count,
            time.time(),
        )
        assert len(header) == _WS_ROBOT_POSE_HEADER_BYTES
        return header + np.ascontiguousarray(poses).tobytes()

    def _on_joint_state(self, msg: JointState) -> None:
        with self._state_lock:
            self._latest_joints = {
                dimos_joint_to_mjcf(name): float(position)
                for name, position in zip(msg.name, msg.position, strict=False)
            }

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._state_lock:
            self._latest_base_pos = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
            self._latest_base_wxyz = np.array(
                [
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ],
                dtype=np.float64,
            )

    def _broadcast_bytes_from_thread(self, payload: bytes) -> None:
        loop = self._server_loop
        if loop is None or not self._clients:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_bytes(payload), loop)

    async def _broadcast_bytes(self, payload: bytes) -> None:
        dead: list[WebSocket] = []
        for websocket in tuple(self._clients):
            try:
                await self._send_bytes_locked(websocket, payload)
            except Exception:
                dead.append(websocket)
        await self._drop_clients(dead)

    # ─── Entity world ────────────────────────────────────────────────────
    #
    # Browser (Havok) owns physics state. Python adds/removes/teleports via
    # RPC → JSON over WS; browser publishes per-tick `entity_states` JSON
    # back. We mirror the descriptor table for reconnect replay only.

    @rpc
    def spawn_entity(self, descriptor: EntityDescriptor, pose: Pose) -> bool:
        """Add an entity to the browser sim. Idempotent on entity_id."""
        with self._entity_lock:
            self._entities[descriptor.entity_id] = descriptor
            self._entity_poses[descriptor.entity_id] = pose
        self.entity_descriptors.publish(descriptor)
        self._broadcast_json_from_thread(
            {
                "type": "entity_spawn",
                "descriptor": self._spawn_descriptor_wire(descriptor),
                "pose": pose_to_wire(pose),
            }
        )
        self._publish_entity_snapshot()
        return True

    def _spawn_descriptor_wire(self, descriptor: EntityDescriptor) -> dict[str, Any]:
        """Descriptor as sent to the browser. With an external entity
        authority the browser is a mirror, not a simulator — entities spawn
        kinematic (mass 0) and get their poses from entity_state_batch."""
        wire = descriptor.to_wire()
        if self._entity_authority == "external":
            wire["kind"] = "kinematic"
            wire["mass"] = 0.0
        return wire

    @rpc
    def despawn_entity(self, entity_id: str) -> bool:
        with self._entity_lock:
            existed = self._entities.pop(entity_id, None) is not None
            self._entity_poses.pop(entity_id, None)
        if not existed:
            return False
        self._broadcast_json_from_thread({"type": "entity_despawn", "entity_id": entity_id})
        self._publish_entity_snapshot()
        return True

    @rpc
    def set_entity_pose(self, entity_id: str, pose: Pose) -> bool:
        """Teleport the entity. Browser applies as a kinematic pose write."""
        with self._entity_lock:
            if entity_id not in self._entities:
                return False
            self._entity_poses[entity_id] = pose
        self._broadcast_json_from_thread(
            {"type": "entity_set_pose", "entity_id": entity_id, "pose": pose_to_wire(pose)}
        )
        self._publish_entity_snapshot()
        return True

    @rpc
    def apply_entity_velocity(self, entity_id: str, twist: Twist) -> bool:
        """Set linear+angular velocity on a dynamic entity."""
        with self._entity_lock:
            if entity_id not in self._entities:
                return False
        self._broadcast_json_from_thread(
            {
                "type": "entity_apply_velocity",
                "entity_id": entity_id,
                "twist": twist_to_wire(twist),
            }
        )
        return True

    @rpc
    def list_entities(self) -> list[str]:
        with self._entity_lock:
            return sorted(self._entities.keys())

    def _handle_entity_test_add(self, point: list[float]) -> None:
        """HUD-driven smoke spawn: drops a visible dynamic obstacle at ``point``.

        Stays in the WS handler path (not a public RPC) since it only
        exists to drive the Add button — production callers should
        construct their own EntityDescriptor + Pose and use spawn_entity.
        """
        try:
            x, y, z = (float(point[i]) for i in range(3))
        except (IndexError, TypeError, ValueError):
            logger.warning("BabylonViewer: entity_test_add ignored: bad point %r", point)
            return
        self._test_entity_counter += 1
        descriptor = EntityDescriptor(
            entity_id=f"box_{self._test_entity_counter}",
            kind="dynamic",
            shape_hint="box",
            extents=(0.8, 0.8, 1.2),
            mass=8.0,
        )
        pose = Pose(x, y, z)
        self.spawn_entity(descriptor, pose)

    def _handle_entity_spawn(self, message: dict[str, Any]) -> None:
        """Spawn an arbitrary entity from a wire descriptor + pose.

        The generic WS counterpart to the ``spawn_entity`` ``@rpc`` — this is
        what ``PimSim.add_object`` (the usage API) sends, so a script can drop
        any mesh/primitive into the scene the same way on any backend.
        """
        raw_desc = message.get("descriptor")
        raw_pose = message.get("pose")
        if not isinstance(raw_desc, dict) or not isinstance(raw_pose, dict):
            logger.warning("BabylonViewer: entity_spawn ignored: missing descriptor/pose")
            return
        try:
            descriptor = EntityDescriptor.from_wire(raw_desc)
            pose = pose_from_wire(raw_pose)
        except (KeyError, TypeError, ValueError) as e:
            logger.warning("BabylonViewer: entity_spawn ignored: bad payload: %s", e)
            return
        self.spawn_entity(descriptor, pose)

    def _handle_entity_add_wall(self, message: dict[str, Any]) -> None:
        """Spawn an axis-aligned static box wall between (x1, y1) and (x2, y2).

        Test-control entry — keeps PimSimClient's surface symmetric with
        DimSim's SceneClient.add_wall without exposing a one-off RPC.
        """
        try:
            x1 = float(message["x1"])
            y1 = float(message["y1"])
            x2 = float(message["x2"])
            y2 = float(message["y2"])
        except (KeyError, TypeError, ValueError):
            logger.warning("BabylonViewer: entity_add_wall ignored: bad endpoints %r", message)
            return
        height = float(message.get("height", 1.5))
        thickness = float(message.get("thickness", 0.1))
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length <= 0.0:
            return
        yaw = math.atan2(dy, dx)
        half_yaw = yaw * 0.5
        qw = math.cos(half_yaw)
        qz = math.sin(half_yaw)
        self._test_entity_counter += 1
        descriptor = EntityDescriptor(
            entity_id=f"wall_{self._test_entity_counter}",
            kind="static",
            shape_hint="box",
            extents=(length, thickness, height),
            mass=0.0,
        )
        pose = Pose((x1 + x2) * 0.5, (y1 + y2) * 0.5, height * 0.5, 0.0, 0.0, qz, qw)
        self.spawn_entity(descriptor, pose)

    def _handle_entity_clear(self) -> None:
        with self._entity_lock:
            ids = list(self._entities.keys())
            self._entities.clear()
            self._entity_poses.clear()
        for entity_id in ids:
            self._broadcast_json_from_thread({"type": "entity_despawn", "entity_id": entity_id})
        self._publish_entity_snapshot()

    def _install_initial_entities(self) -> None:
        for raw in self._initial_entities:
            if raw.get("spawn", "initial") != "initial":
                continue
            try:
                descriptor = EntityDescriptor.from_wire(raw["descriptor"])
                pose = pose_from_wire(raw["initial_pose"])
            except (KeyError, TypeError, ValueError) as exc:
                logger.warning("BabylonViewer: dropping bad packaged entity: %s", exc)
                continue
            self.spawn_entity(descriptor, pose)

    @staticmethod
    def _collect_entity_asset_paths(entities: list[dict[str, Any]]) -> dict[str, Path]:
        assets: dict[str, Path] = {}
        for raw in entities:
            descriptor = raw.get("descriptor", {})
            mesh_ref = descriptor.get("mesh_ref")
            visual_path = raw.get("visual_path")
            if not isinstance(mesh_ref, str) or not isinstance(visual_path, str):
                continue
            assets[mesh_ref] = Path(visual_path).expanduser().resolve()
        return assets

    def _publish_entity_states(self, states_wire: list[dict[str, Any]]) -> None:
        """Browser → python entity state batch. Publish the aggregated
        ``entity_state_batch`` for cross-process consumers (rust
        scene_lidar). Entries for entities we don't know about are
        dropped (despawn race)."""
        if self._entity_authority != "browser":
            # External sim owns entity state; browser-reported poses are
            # kinematic-mirror echoes, not authority.
            return
        batch_entries: list[tuple[EntityDescriptor, Pose]] = []
        ts = time.time()
        for raw in states_wire:
            try:
                state = EntityState.from_wire(raw)
            except (KeyError, TypeError, ValueError) as exc:
                logger.warning("BabylonViewer: dropping malformed entity_state: %s", exc)
                continue
            with self._entity_lock:
                desc = self._entities.get(state.entity_id)
                if desc is None:
                    continue
                self._entity_poses[state.entity_id] = state.pose
            batch_entries.append((desc, state.pose))
            ts = max(ts, state.ts)
        self.entity_state_batch.publish(EntityStateBatch(entries=batch_entries, ts=ts))

    def _publish_entity_snapshot(self) -> None:
        with self._entity_lock:
            entries = [
                (desc, self._entity_poses.get(entity_id, Pose()))
                for entity_id, desc in self._entities.items()
            ]
        self.entity_state_batch.publish(EntityStateBatch(entries=entries, ts=time.time()))

    def _entity_spawn_messages(self) -> list[dict[str, Any]]:
        """Replay payload: a fresh tab gets spawn commands for every entity."""
        with self._entity_lock:
            entities = [
                (descriptor, self._entity_poses.get(entity_id, Pose()))
                for entity_id, descriptor in self._entities.items()
            ]
        return [
            {
                "type": "entity_spawn",
                "descriptor": self._spawn_descriptor_wire(d),
                "pose": pose_to_wire(pose),
            }
            for d, pose in entities
        ]

    def _broadcast_json_from_thread(self, payload: dict[str, Any]) -> None:
        """JSON analog of `_broadcast_bytes_from_thread`."""
        loop = self._server_loop
        if loop is None or not self._clients:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_json(payload), loop)

    async def _broadcast_json(self, payload: dict[str, Any]) -> None:
        dead: list[WebSocket] = []
        for websocket in tuple(self._clients):
            try:
                await self._send_json_locked(websocket, payload)
            except Exception:
                dead.append(websocket)
        await self._drop_clients(dead)

    async def _send_bytes_locked(self, websocket: WebSocket, payload: bytes) -> None:
        """Send bytes under the per-client lock with a wall-clock timeout.

        Starlette's WebSocket has a single ASGI send pipe; concurrent
        ``send_*`` calls on the same WS corrupt its state machine. The
        per-client lock serialises sends so only one is in flight at a
        time. The timeout bounds how long any single send can hold the
        lock — without it, a stalled browser would back up every other
        sender behind it forever.
        """
        lock = self._ws_send_locks.get(websocket)
        if lock is None:
            raise RuntimeError("WebSocket no longer registered")
        async with lock:
            await asyncio.wait_for(websocket.send_bytes(payload), timeout=_WS_SEND_TIMEOUT_S)

    async def _send_json_locked(self, websocket: WebSocket, payload: dict[str, Any]) -> None:
        """JSON counterpart to :meth:`_send_bytes_locked`."""
        lock = self._ws_send_locks.get(websocket)
        if lock is None:
            raise RuntimeError("WebSocket no longer registered")
        async with lock:
            await asyncio.wait_for(websocket.send_json(payload), timeout=_WS_SEND_TIMEOUT_S)

    async def _drop_clients(self, dead: list[WebSocket]) -> None:
        """Discard dead clients and best-effort close them so JS reconnects.

        Called on send timeout or exception. Without the close(), the JS
        ``onclose`` doesn't fire and the auto-reconnect loop in
        stream_worker.js never runs until uvicorn's WS keepalive eventually
        reaps the connection ~40s later.
        """
        for websocket in dead:
            self._clients.discard(websocket)
            self._ws_send_locks.pop(websocket, None)
            with suppress(Exception):
                await asyncio.wait_for(websocket.close(), timeout=_WS_CLOSE_TIMEOUT_S)
