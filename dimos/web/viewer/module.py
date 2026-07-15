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

"""Babylon-based browser viewer for any dimos system.

Viewer-only cut of the simulation scene viewer: it renders, it never owns
state. The browser page (Babylon.js) draws a scene mesh and/or gaussian
splat, the robot (server-side FK from ``joint_state`` + ``odom``),
pointclouds, the nav path and camera frames, and offers teleop
(``/cmd_vel``) and click-to-navigate (``point_goal``) — every display
topic and browser-authored message flows through the LCM<->WebSocket
bridge (:mod:`dimos.web.lcm_bridge`), so the browser is just another bus
peer. The browser-physics / entity-authority half of the original module
stayed in the simulation tree; this one composes with real robots and
sims alike.

Add to any blueprint::

    BabylonViewerModule.blueprint(
        mjcf_path=...,          # optional: robot meshes + FK
        scene_path=...,         # optional: visual scene GLB/glTF
        splat_path=...,         # optional: gaussian splat
    )

Then open ``http://<host>:8091/``. All three arguments are optional —
with none you still get pointclouds, path, teleop and camera over a
grid floor.
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager, suppress
import os
from pathlib import Path
import struct
import threading
import time
from typing import Any

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
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.web.lcm_bridge import bridge as lcm_bridge
from dimos.web.viewer.geometry import (
    compose_scene_mesh_wxyz,
    dimos_joint_to_mjcf,
    media_type,
    path_contains,
)

logger = setup_logger()

STATIC_DIR = Path(__file__).with_name("static")
_LCM_CLIENT_PATH = Path(lcm_bridge.__file__).with_name("static") / "lcm_client.js"

_DEFAULT_BROADCAST_HZ = 30.0
_DEFAULT_PORT = 8091
_POSE_POSITION_EPSILON = 1e-5
_POSE_QUATERNION_EPSILON = 1e-5
# Binary websocket message tag. Robot body poses are the only binary frame
# left on /ws — everything else (pointclouds, camera JPEGs, path, teleop)
# flows through the LCM<->WS bridge. Once the browser does its own FK from
# /joint_state + /odom this goes too.
_WS_MSG_ROBOT_POSE = 0x03
_WS_ROBOT_POSE_HEADER_BYTES = 16
# Per-message WS send timeout. The browser tab momentarily failing to drain
# TCP (Chrome backgrounding, GC pause, App Nap) shouldn't permanently wedge
# the in-flight gate: time out, drop the client, let the JS reconnect.
_WS_SEND_TIMEOUT_S = 1.0
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


class BabylonViewerModule(Module):
    # joint_state and odom are consumed server-side because forward
    # kinematics runs here (-> _make_robot_pose_payload). The browser also
    # sees both via /lcm-ws; once FK moves to the browser these go away.
    joint_state: In[JointState]
    odom: In[PoseStamped]

    def __init__(
        self,
        mjcf_path: str | Path | None = None,
        *,
        port: int = _DEFAULT_PORT,
        assets: dict[str, bytes] | None = None,
        mesh_dir: str | Path | None = None,
        scene_path: str | Path | None = None,
        splat_path: str | Path | None = None,
        splat_alignment: dict[str, Any] | None = None,
        scene_scale: float = 1.0,
        scene_translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
        scene_rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0),
        scene_y_up: bool = True,
        broadcast_hz: float = _DEFAULT_BROADCAST_HZ,
        channel_rate_hz: dict[str, float] | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._mjcf_path = Path(mjcf_path) if mjcf_path else None
        self._assets = assets
        # Deferred alternative to `assets`: a directory of mesh files read
        # at start() so blueprint import stays cheap (and LFS-backed dirs
        # download lazily).
        self._mesh_dir = mesh_dir
        self._port = port
        self._scene_path = Path(scene_path) if scene_path else None
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

        self._robot: Any = None
        self._uvicorn_server: uvicorn.Server | None = None
        self._server_thread: threading.Thread | None = None
        self._broadcast_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._server_loop: asyncio.AbstractEventLoop | None = None
        self._clients: set[WebSocket] = set()
        # Per-client send lock. Starlette's WebSocket is not safe for
        # concurrent sends; without serialization the broadcast loop and
        # the initial replay in _websocket can race on the same socket.
        self._ws_send_locks: dict[WebSocket, asyncio.Lock] = {}

        # The bus-to-browser data plane. Default rate cap keeps the rust
        # voxel mapper's multi-MB /global_map from eating the tab's budget;
        # in-process consumers are unaffected.
        _gm_hz = float(os.environ.get("DIMOS_VIEWER_GLOBAL_MAP_HZ", "1.0"))
        rates: dict[str, float] = {}
        if _gm_hz > 0:
            rates["/global_map#sensor_msgs.PointCloud2"] = _gm_hz
        rates.update(channel_rate_hz or {})
        self._ws_bridge = lcm_bridge.LcmWebSocketBridge(channel_rate_hz=rates)

    @rpc
    def start(self) -> None:
        super().start()

        if self._mjcf_path is not None:
            # mujoco is only needed for the robot layer (MJCF parse + FK);
            # a robotless viewer must not require the sim extra.
            from dimos.web.viewer.robot_meshes import load_robot_meshes

            assets = self._assets
            if assets is None and self._mesh_dir is not None:
                assets = {
                    mesh.name: mesh.read_bytes()
                    for mesh in Path(self._mesh_dir).iterdir()
                    # skip .DS_Store / AppleDouble "._*" junk that rides
                    # along in LFS-extracted asset dirs
                    if mesh.is_file() and not mesh.name.startswith(".")
                }
            self._robot = load_robot_meshes(self._mjcf_path, assets=assets)

        config = uvicorn.Config(
            self._create_app(), host="0.0.0.0", port=self._port, log_level="warning"
        )
        self._uvicorn_server = uvicorn.Server(config)
        self._server_thread = threading.Thread(
            target=self._uvicorn_server.run,
            name="babylon-viewer-server",
            daemon=True,
        )
        self._server_thread.start()

        # In a blueprint these ports arrive wired; standalone (tests, bare
        # scripts) they may not be — serve anyway, the robot just holds its
        # default pose until someone wires or publishes.
        for port, callback, name in (
            (self.joint_state, self._on_joint_state, "joint_state"),
            (self.odom, self._on_odom, "odom"),
        ):
            if getattr(port, "transport", None) is None:
                logger.warning("BabylonViewer: %s port not wired; robot pose will be static", name)
                continue
            self.register_disposable(Disposable(port.subscribe(callback)))

        self._broadcast_thread = threading.Thread(
            target=self._broadcast_loop,
            name="babylon-viewer-broadcast",
            daemon=True,
        )
        self._broadcast_thread.start()

        self._ws_bridge.start()

        logger.info("Babylon viewer: http://localhost:%s/", self._port)
        if os.environ.get("DIMOS_BABYLON_OPEN", "1").lower() not in ("0", "false", "no"):
            import webbrowser

            try:
                webbrowser.open(f"http://localhost:{self._port}/")
            except Exception:  # headless environments have no opener
                pass

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self._ws_bridge.stop()
        if self._uvicorn_server is not None:
            self._uvicorn_server.should_exit = True
        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=2.0)
        if self._server_thread and self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        super().stop()

    # ---- HTTP surface ----------------------------------------------------

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
                # The browser client ships with the bridge package; serve it
                # under /static so index.html needs no special-casing. Listed
                # before the Mount so it wins the match.
                Route("/static/lcm_client.js", self._lcm_client_js),
                Mount("/static", app=StaticFiles(directory=STATIC_DIR), name="static"),
                WebSocketRoute("/ws", self._websocket),
                *self._ws_bridge.routes(),
            ],
            lifespan=_lifespan,
        )

    async def _lcm_client_js(self, request: Request) -> FileResponse:
        return FileResponse(
            _LCM_CLIENT_PATH, media_type="text/javascript", headers=_NO_CACHE_HEADERS
        )

    async def _index(self, request: Request) -> HTMLResponse:
        html = (STATIC_DIR / "index.html").read_text()
        for asset_name in ("style.css", "ui.js", "app.js"):
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
        scene_wxyz = compose_scene_mesh_wxyz(
            y_up=self._scene_y_up,
            rotation_zyx_deg=self._scene_rotation_zyx_deg,
        )
        if self._scene_path is not None and self._scene_path.exists():
            scene_file = _versioned_asset_name("scene", self._scene_path)
            scene_bytes = self._scene_path.stat().st_size
        splat_file = None
        splat_bytes = 0
        if self._splat_path is not None and self._splat_path.exists():
            splat_file = _versioned_asset_name("splat", self._splat_path)
            splat_bytes = self._splat_path.stat().st_size
        # Exactly the keys the page reads — this build renders, it never
        # simulates, so there is no browser-physics block.
        return JSONResponse(
            {
                "sceneFile": scene_file,
                "sceneBytes": scene_bytes,
                "splatFile": splat_file,
                "splatBytes": splat_bytes,
                "splatAlignment": self._splat_alignment,
                "sceneScale": self._scene_scale,
                "scenePosition": list(self._scene_translation),
                "sceneWxyz": list(scene_wxyz),
                # "external" = any entity authority lives outside the browser;
                # the page's ENTITY MIRROR stub keys off this.
                "entityAuthority": "external",
            },
            headers=_NO_CACHE_HEADERS,
        )

    async def _arms_json(self, request: Request) -> JSONResponse:
        # Arm-slider RPC surface is a sim-tree feature; the viewer-only
        # build reports no controllable joints so the HUD hides the panel.
        return JSONResponse({"joints": []}, headers=_NO_CACHE_HEADERS)

    async def _viewer_debug(self, request: Request) -> JSONResponse:
        try:
            message = await request.json()
        except Exception:
            return JSONResponse({"ok": False}, status_code=400, headers=_NO_CACHE_HEADERS)
        label = message.get("label", "viewer")
        payload = message.get("payload")
        if isinstance(payload, dict):
            # Client-controlled keys must not be splatted into structlog
            # kwargs (an "event" key would collide with the message itself).
            logger.info("BabylonViewer debug", label=label, payload=payload)
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
        asset_name = request.path_params["asset_name"]

        if self._splat_path is not None and _matches_asset_name(
            asset_name, "splat", self._splat_path
        ):
            return FileResponse(
                self._splat_path,
                media_type="application/octet-stream",
                headers=_NO_CACHE_HEADERS,
            )

        if self._scene_path is not None and _matches_asset_name(
            asset_name, "scene", self._scene_path
        ):
            return FileResponse(
                self._scene_path,
                media_type=media_type(self._scene_path),
                headers=_NO_CACHE_HEADERS,
            )

        # .gltf scenes reference sibling files (textures, .bin buffers).
        if self._scene_path is not None and self._scene_path.suffix.lower() == ".gltf":
            candidate = self._scene_path.parent / asset_name
            if path_contains(self._scene_path.parent, candidate) and candidate.exists():
                return FileResponse(
                    candidate,
                    media_type=media_type(candidate),
                    headers=_NO_CACHE_HEADERS,
                )

        return Response("asset not found", status_code=404)

    # ---- /ws: robot pose broadcast + JSON control ------------------------

    async def _websocket(self, websocket: WebSocket) -> None:
        # Display topics and browser-authored messages (cmd_vel, point_goal,
        # clicked_point) flow through /lcm-ws. /ws carries the robot_pose
        # binary frame plus a JSON control channel the viewer-only build
        # mostly ignores.
        await websocket.accept()
        self._ws_send_locks[websocket] = asyncio.Lock()
        self._clients.add(websocket)
        logger.info("BabylonViewer: websocket connected", clients=len(self._clients))
        try:
            robot_pose_payload = self._make_robot_pose_payload(force=True)
            if robot_pose_payload is not None:
                await self._send_bytes_locked(websocket, robot_pose_payload)
            while True:
                message = await websocket.receive_json()
                self._handle_client_message(message)
        except WebSocketDisconnect:
            pass
        finally:
            self._clients.discard(websocket)
            self._ws_send_locks.pop(websocket, None)
            logger.info("BabylonViewer: websocket disconnected", clients=len(self._clients))

    def _handle_client_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("type")
        # Respawn / arm / coordinator RPCs belong to the simulation tree's
        # authority build. Log-and-ignore keeps old HUD buttons harmless.
        logger.debug("BabylonViewer: ignoring control message", type=message_type)

    # ---- server-side FK --------------------------------------------------

    def _broadcast_loop(self) -> None:
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
        from dimos.web.viewer.robot_meshes import apply_state

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

    # ---- send helpers ------------------------------------------------------

    async def _send_bytes_locked(self, websocket: WebSocket, payload: bytes) -> None:
        lock = self._ws_send_locks.get(websocket)
        if lock is None:
            raise RuntimeError("WebSocket no longer registered")
        async with lock:
            await asyncio.wait_for(websocket.send_bytes(payload), timeout=_WS_SEND_TIMEOUT_S)

    async def _drop_clients(self, dead: list[WebSocket]) -> None:
        for websocket in dead:
            self._clients.discard(websocket)
            self._ws_send_locks.pop(websocket, None)
            with suppress(Exception):
                await asyncio.wait_for(websocket.close(), timeout=_WS_CLOSE_TIMEOUT_S)
