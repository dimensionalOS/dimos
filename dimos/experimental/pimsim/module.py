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
from contextlib import asynccontextmanager
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
from dimos.core.stream import In, Out
from dimos.experimental.pimsim.browser import STATIC_DIR, index_html
from dimos.experimental.pimsim.config import (
    CoordinatorControlSpec,
    HumanoidControlSpec,
    MujocoRespawnSpec,
)
from dimos.experimental.pimsim.entity import (
    EntityDescriptor,
    EntityState,
    EntityStateBatch,
    pose_to_wire,
    twist_to_wire,
)
from dimos.experimental.pimsim.geometry import (
    compose_scene_mesh_wxyz,
    dimos_joint_to_mjcf,
    media_type,
    path_contains,
)
from dimos.experimental.pimsim.robot_meshes import (
    RobotMeshes,
    apply_state,
    load_robot_meshes,
)
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, _get_colormap_lut
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEFAULT_BROADCAST_HZ = 30.0
_DEFAULT_METADATA_HZ = 10.0
_DEFAULT_PORT = 8091
_DEFAULT_POINTCLOUD_HZ = 2.0
_DEFAULT_POINTCLOUD_MAX_POINTS = 70000
_DEFAULT_CAMERA_HZ = 15.0
_DEFAULT_CAMERA_JPEG_QUALITY = 75
_POSE_POSITION_EPSILON = 1e-5
_POSE_QUATERNION_EPSILON = 1e-5
# Binary websocket message tags.
_WS_MSG_CAMERA = 0x01
_WS_MSG_POINTCLOUD = 0x02
_WS_MSG_ROBOT_POSE = 0x03
_WS_POINTCLOUD_HEADER_BYTES = 8
_WS_ROBOT_POSE_HEADER_BYTES = 16
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
    joint_state: In[JointState]
    odom: In[PoseStamped]
    path: In[PathMsg]
    pointcloud_overlay: In[PointCloud2]
    camera_image: In[Image]
    # Optional second camera (e.g. a workspace-facing realsense). If a
    # transport is wired, a second camera panel shows up in the HUD.
    workspace_image: In[Image]
    clicked_point: Out[PointStamped]
    point_goal: Out[PointStamped]
    cmd_vel: Out[Twist]
    # Entity world (browser is authoritative; these republish for dimos consumers).
    entity_descriptors: Out[EntityDescriptor]
    entity_states: Out[EntityState]
    # Aggregated snapshot for cross-process consumers (rust scene_lidar).
    # Published every browser tick alongside the per-entity stream.
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
        scene_scale: float = 1.0,
        scene_translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
        scene_rotation_zyx_deg: tuple[float, float, float] = (0.0, 0.0, 0.0),
        scene_y_up: bool = True,
        broadcast_hz: float = _DEFAULT_BROADCAST_HZ,
        metadata_hz: float = _DEFAULT_METADATA_HZ,
        pointcloud_hz: float = _DEFAULT_POINTCLOUD_HZ,
        pointcloud_max_points: int = _DEFAULT_POINTCLOUD_MAX_POINTS,
        camera_hz: float = _DEFAULT_CAMERA_HZ,
        camera_jpeg_quality: int = _DEFAULT_CAMERA_JPEG_QUALITY,
        camera_name: str = "camera",
        workspace_name: str = "workspace",
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._mjcf_path = Path(mjcf_path)
        self._assets = assets
        self._port = port
        self._scene_path = Path(scene_path) if scene_path else None
        self._browser_collision_path = (
            Path(browser_collision_path) if browser_collision_path else None
        )
        self._scene_scale = scene_scale
        self._scene_translation = scene_translation
        self._scene_rotation_zyx_deg = scene_rotation_zyx_deg
        self._scene_y_up = scene_y_up
        self._broadcast_dt = 1.0 / float(broadcast_hz)
        self._metadata_dt = 1.0 / float(metadata_hz)
        self._pointcloud_min_dt = 1.0 / float(pointcloud_hz)
        self._pointcloud_max_points = pointcloud_max_points
        self._camera_min_dt = 1.0 / float(camera_hz)
        self._camera_jpeg_quality = int(camera_jpeg_quality)
        self._camera_name = camera_name
        self._workspace_name = workspace_name
        self._last_workspace_sent = 0.0

        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_base_pos: np.ndarray | None = None
        self._latest_base_wxyz: np.ndarray | None = None
        self._latest_path: list[list[float]] = []
        self._latest_path_version = 0
        self._last_metadata_broadcast = 0.0
        self._last_metadata_path_version = -1
        self._robot_pose_lock = threading.Lock()
        self._last_robot_pose_values: np.ndarray | None = None
        self._pointcloud_lock = threading.Lock()
        self._latest_pointcloud_payload: bytes | None = None
        self._pointcloud_pending_lock = threading.Lock()
        self._pointcloud_send_pending = False
        self._last_pointcloud_sent = 0.0

        # Camera state. _turbo_jpeg is lazy-initialised so the viewer still
        # imports cleanly on machines without PyTurboJPEG (it's an optional
        # dep). Only the encode path requires it.
        self._camera_lock = threading.Lock()
        self._last_camera_sent = 0.0
        self._turbo_jpeg: Any = None

        self._robot: RobotMeshes | None = None
        self._uvicorn_server: uvicorn.Server | None = None
        self._server_thread: threading.Thread | None = None
        self._broadcast_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._server_loop: asyncio.AbstractEventLoop | None = None
        self._clients: set[WebSocket] = set()

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

        self.register_disposable(Disposable(self.joint_state.subscribe(self._on_joint_state)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        self.register_disposable(
            Disposable(self.pointcloud_overlay.subscribe(self._on_pointcloud_overlay))
        )
        self.register_disposable(Disposable(self.camera_image.subscribe(self._on_camera_image)))
        try:
            self.register_disposable(
                Disposable(self.workspace_image.subscribe(self._on_workspace_image))
            )
        except Exception:
            logger.debug("BabylonViewer: workspace_image not wired; skipping second camera")

        self._broadcast_thread = threading.Thread(
            target=self._broadcast_loop,
            name="babylon-viewer-broadcast",
            daemon=True,
        )
        self._broadcast_thread.start()
        logger.info("Babylon scene viewer: http://localhost:%s/", self._port)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._uvicorn_server is not None:
            self._uvicorn_server.should_exit = True
        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=2.0)
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
                Route("/assets/{asset_name:path}", self._asset),
                Mount("/static", app=StaticFiles(directory=STATIC_DIR), name="static"),
                WebSocketRoute("/ws", self._websocket),
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
        app_js = STATIC_DIR / "app.js"
        style_css = STATIC_DIR / "style.css"
        if app_js.exists():
            html = html.replace(
                'src="/static/app.js"',
                f'src="/static/app.js?v={_asset_token(app_js)}"',
            )
        if style_css.exists():
            html = html.replace(
                'href="/static/style.css"',
                f'href="/static/style.css?v={_asset_token(style_css)}"',
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
        return JSONResponse(
            {
                "sceneFile": scene_file,
                "sceneBytes": scene_bytes,
                "collisionSceneFile": collision_file,
                "collisionSceneBytes": collision_bytes,
                "sceneScale": self._scene_scale,
                "scenePosition": list(self._scene_translation),
                "sceneWxyz": list(scene_wxyz),
            },
            headers=_NO_CACHE_HEADERS,
        )

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
        if (self._scene_path is None or not self._scene_path.exists()) and (
            self._browser_collision_path is None or not self._browser_collision_path.exists()
        ):
            return Response("scene asset not configured", status_code=404)

        asset_name = request.path_params["asset_name"]
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
        await websocket.accept()
        self._clients.add(websocket)
        logger.info("BabylonViewer: websocket connected", clients=len(self._clients))
        try:
            robot_pose_payload = self._make_robot_pose_payload(force=True)
            if robot_pose_payload is not None:
                await websocket.send_bytes(robot_pose_payload)
            await websocket.send_json(self._make_state_payload())
            with self._pointcloud_lock:
                pointcloud_payload = self._latest_pointcloud_payload
            if pointcloud_payload is not None:
                await websocket.send_bytes(pointcloud_payload)
            # Replay entity descriptors so a fresh tab rebuilds the world
            # the browser-side physics is otherwise oblivious to.
            for spawn in self._entity_spawn_messages():
                await websocket.send_json(spawn)
            while True:
                message = await websocket.receive_json()
                self._handle_client_message(message)
        except WebSocketDisconnect:
            pass
        finally:
            self._clients.discard(websocket)
            logger.info("BabylonViewer: websocket disconnected", clients=len(self._clients))

    def _handle_client_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("type")
        if message_type == "respawn":
            logger.info(
                "BabylonViewer: respawn requested",
                has_mujoco_sim=self._mujoco_sim is not None,
            )
            if self._mujoco_sim is not None:
                self._respawn_with_policy_reset(self._mujoco_sim.respawn)
            self.cmd_vel.publish(Twist.zero())
            return
        if message_type == "respawn_at":
            point = message.get("point")
            if not isinstance(point, list) or len(point) < 2:
                return
            try:
                x = float(point[0])
                y = float(point[1])
            except (TypeError, ValueError):
                return
            logger.info(
                "BabylonViewer: respawn_at requested",
                x=x,
                y=y,
                has_mujoco_sim=self._mujoco_sim is not None,
            )
            if self._mujoco_sim is not None:
                self._respawn_with_policy_reset(lambda: self._mujoco_sim.respawn_at(x, y))
            self.cmd_vel.publish(Twist.zero())
            return
        if message_type == "cmd_vel":
            twist = self._parse_twist(message)
            if twist is not None:
                self.cmd_vel.publish(twist)
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
        if message_type == "entity_clear":
            self._handle_entity_clear()
            return
        if message_type not in {"clicked_point", "point_goal"}:
            return
        point = message.get("point")
        if not isinstance(point, list) or len(point) != 3:
            return
        try:
            x, y, z = (float(value) for value in point)
        except (TypeError, ValueError):
            return
        stamped = PointStamped(x=x, y=y, z=z, frame_id="map")
        if message_type == "clicked_point":
            self.clicked_point.publish(stamped)
        else:
            self.point_goal.publish(stamped)

    def _respawn_with_policy_reset(self, respawn: Callable[[], bool]) -> bool:
        if self._coordinator_ctrl is not None:
            self._coordinator_ctrl.set_activated(engaged=False)
        try:
            return respawn()
        finally:
            if self._coordinator_ctrl is not None:
                self._coordinator_ctrl.set_activated(engaged=True)

    @staticmethod
    def _parse_twist(message: dict[str, Any]) -> Twist | None:
        linear = message.get("linear", [0.0, 0.0, 0.0])
        angular = message.get("angular", [0.0, 0.0, 0.0])
        if not isinstance(linear, list) or not isinstance(angular, list):
            return None
        if len(linear) != 3 or len(angular) != 3:
            return None
        try:
            return Twist(
                linear=Vector3(*(float(value) for value in linear)),
                angular=Vector3(*(float(value) for value in angular)),
            )
        except (TypeError, ValueError):
            return None

    def _broadcast_loop(self) -> None:
        while not self._stop_event.is_set():
            loop = self._server_loop
            if loop is not None and self._clients:
                robot_pose_payload = self._make_robot_pose_payload()
                state_payload = self._make_metadata_payload_if_due()
                if robot_pose_payload is not None or state_payload is not None:
                    asyncio.run_coroutine_threadsafe(
                        self._broadcast_state(state_payload, robot_pose_payload),
                        loop,
                    )
            time.sleep(self._broadcast_dt)

    async def _broadcast_state(
        self,
        state_payload: dict[str, Any] | None,
        robot_pose_payload: bytes | None,
    ) -> None:
        dead: list[WebSocket] = []
        for websocket in tuple(self._clients):
            try:
                if robot_pose_payload is not None:
                    await websocket.send_bytes(robot_pose_payload)
                if state_payload is not None:
                    await websocket.send_json(state_payload)
            except Exception:
                dead.append(websocket)
        for websocket in dead:
            self._clients.discard(websocket)

    def _make_metadata_payload_if_due(self) -> dict[str, Any] | None:
        now = time.monotonic()
        with self._state_lock:
            path_version = self._latest_path_version

        if (
            now - self._last_metadata_broadcast < self._metadata_dt
            and path_version == self._last_metadata_path_version
        ):
            return None

        self._last_metadata_broadcast = now
        self._last_metadata_path_version = path_version
        return self._make_state_payload()

    def _make_state_payload(self) -> dict[str, Any]:
        with self._state_lock:
            joints = dict(self._latest_joints)
            path_points = [point[:] for point in self._latest_path]
            path_version = self._latest_path_version

        # Normalise joint names to the short form the slider HUD uses:
        #   * strip a leading "<hwid>/" prefix (e.g. "g1/left_shoulder_pitch"
        #     → "left_shoulder_pitch") for coordinator-style names.
        #   * strip a trailing "_joint" suffix (e.g. "left_shoulder_pitch_joint"
        #     → "left_shoulder_pitch") for URDF-style names.
        def _canon(k: str) -> str:
            if "/" in k:
                k = k.split("/", 1)[1]
            if k.endswith("_joint"):
                k = k[: -len("_joint")]
            return k

        joint_positions = {_canon(k): float(v) for k, v in joints.items()}
        return {
            "type": "state",
            "time": time.time(),
            "path": path_points,
            "path_version": path_version,
            "joints": joint_positions,
        }

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

    def _on_path(self, msg: PathMsg) -> None:
        with self._state_lock:
            self._latest_path = [[pose.x, pose.y, pose.z] for pose in msg.poses]
            self._latest_path_version += 1

    def _on_pointcloud_overlay(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_pointcloud_sent < self._pointcloud_min_dt:
            return

        payload = self._make_pointcloud_payload(msg)
        if payload is None:
            return

        with self._pointcloud_lock:
            self._latest_pointcloud_payload = payload
            self._last_pointcloud_sent = now
        self._broadcast_pointcloud_from_thread(payload)

    def _on_camera_image(self, msg: Image) -> None:
        self._broadcast_camera(msg, self._camera_name, is_workspace=False)

    def _on_workspace_image(self, msg: Image) -> None:
        self._broadcast_camera(msg, self._workspace_name, is_workspace=True)

    def _broadcast_camera(self, msg: Image, name: str, *, is_workspace: bool) -> None:
        # Rate-limit per-camera to avoid saturating the websocket with multi-MB
        # frames when the publisher pushes at 30+ Hz.
        now = time.monotonic()
        with self._camera_lock:
            last_attr = "_last_workspace_sent" if is_workspace else "_last_camera_sent"
            if now - getattr(self, last_attr) < self._camera_min_dt:
                return
            setattr(self, last_attr, now)

        try:
            jpeg = self._encode_jpeg(msg)
        except Exception as exc:
            logger.warning("BabylonViewer: camera JPEG encode failed: %s", exc)
            return
        if jpeg is None:
            return

        # Binary frame layout:
        #   byte 0:      _WS_MSG_CAMERA (0x01)
        #   bytes 1-2:   name length (big-endian uint16)
        #   bytes 3..:   utf-8 camera name, then JPEG payload
        name_bytes = name.encode("utf-8")[:65535]
        header = bytes([_WS_MSG_CAMERA]) + len(name_bytes).to_bytes(2, "big") + name_bytes
        self._broadcast_bytes_from_thread(header + jpeg)

    def _encode_jpeg(self, msg: Image) -> bytes | None:
        if self._turbo_jpeg is None:
            from turbojpeg import TurboJPEG

            self._turbo_jpeg = TurboJPEG()

        from turbojpeg import TJPF_BGR, TJPF_GRAY, TJPF_RGB

        data = msg.data
        if data is None:
            return None
        match msg.format:
            case ImageFormat.RGB:
                pixel_format = TJPF_RGB
            case ImageFormat.BGR:
                pixel_format = TJPF_BGR
            case ImageFormat.GRAY:
                pixel_format = TJPF_GRAY
            case _:
                # RGBA/BGRA: drop alpha to keep encode cheap.
                if data.ndim == 3 and data.shape[2] == 4:
                    data = data[:, :, :3]
                    pixel_format = TJPF_BGR if msg.format == ImageFormat.BGRA else TJPF_RGB
                else:
                    return None

        encoded: bytes = self._turbo_jpeg.encode(
            np.ascontiguousarray(data),
            quality=self._camera_jpeg_quality,
            pixel_format=pixel_format,
        )
        return encoded

    def _broadcast_bytes_from_thread(self, payload: bytes) -> None:
        loop = self._server_loop
        if loop is None or not self._clients:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_bytes(payload), loop)

    def _broadcast_pointcloud_from_thread(self, payload: bytes) -> None:
        loop = self._server_loop
        if loop is None or not self._clients:
            return
        with self._pointcloud_pending_lock:
            if self._pointcloud_send_pending:
                return
            self._pointcloud_send_pending = True
        future = asyncio.run_coroutine_threadsafe(self._broadcast_bytes(payload), loop)
        future.add_done_callback(lambda _: self._clear_pointcloud_send_pending())

    def _clear_pointcloud_send_pending(self) -> None:
        with self._pointcloud_pending_lock:
            self._pointcloud_send_pending = False

    async def _broadcast_bytes(self, payload: bytes) -> None:
        dead: list[WebSocket] = []
        for websocket in tuple(self._clients):
            try:
                await websocket.send_bytes(payload)
            except Exception:
                dead.append(websocket)
        for websocket in dead:
            self._clients.discard(websocket)

    def _make_pointcloud_payload(self, msg: PointCloud2) -> bytes | None:
        points = msg.points_f32()
        if points.size == 0:
            return None

        points = points[np.isfinite(points).all(axis=1)]
        if len(points) == 0:
            return None

        if len(points) > self._pointcloud_max_points:
            indices = np.linspace(0, len(points) - 1, self._pointcloud_max_points).astype(np.int64)
            points = points[indices]

        z_values = points[:, 2]
        if len(z_values) >= 20:
            z_min, z_max = np.quantile(z_values, [0.02, 0.98])
        else:
            z_min, z_max = float(z_values.min()), float(z_values.max())
        normalized = np.clip((z_values - z_min) / (z_max - z_min + 1e-6), 0.0, 1.0)
        colors = _get_colormap_lut("turbo")[(normalized * 255).astype(np.uint8)]

        ambient = np.array([34, 42, 50], dtype=np.float32)
        colors = np.clip(colors.astype(np.float32) * 0.82 + ambient * 0.18, 0, 255).astype(np.uint8)

        count = len(points)
        header = bytearray(_WS_POINTCLOUD_HEADER_BYTES)
        header[0] = _WS_MSG_POINTCLOUD
        header[4:8] = count.to_bytes(4, "big")
        positions = np.ascontiguousarray(points.astype(np.float32, copy=False))
        colors = np.ascontiguousarray(colors)
        return bytes(header) + positions.reshape(-1).tobytes() + colors.reshape(-1).tobytes()

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
        self.entity_descriptors.publish(descriptor)
        self._broadcast_json_from_thread(
            {"type": "entity_spawn", "descriptor": descriptor.to_wire(), "pose": pose_to_wire(pose)}
        )
        return True

    @rpc
    def despawn_entity(self, entity_id: str) -> bool:
        with self._entity_lock:
            existed = self._entities.pop(entity_id, None) is not None
            self._entity_poses.pop(entity_id, None)
        if not existed:
            return False
        self._broadcast_json_from_thread({"type": "entity_despawn", "entity_id": entity_id})
        return True

    @rpc
    def set_entity_pose(self, entity_id: str, pose: Pose) -> bool:
        """Teleport the entity. Browser applies as a kinematic pose write."""
        with self._entity_lock:
            if entity_id not in self._entities:
                return False
        self._broadcast_json_from_thread(
            {"type": "entity_set_pose", "entity_id": entity_id, "pose": pose_to_wire(pose)}
        )
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
        """HUD-driven smoke spawn: drops a 40cm dynamic cube at ``point``.

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
            extents=(0.4, 0.4, 0.4),
            mass=2.0,
        )
        pose = Pose(x, y, z)
        self.spawn_entity(descriptor, pose)

    def _handle_entity_clear(self) -> None:
        with self._entity_lock:
            ids = list(self._entities.keys())
        for entity_id in ids:
            self.despawn_entity(entity_id)

    def _publish_entity_states(self, states_wire: list[dict[str, Any]]) -> None:
        """Browser → python entity state batch. Republish:

        * per-entity ``entity_states`` (for in-process Python consumers)
        * aggregated ``entity_state_batch`` (for cross-process LCM
          subscribers like the rust scene_lidar — single message per
          browser tick instead of N).
        """
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
                    # Browser sent state for an entity we don't know — most
                    # likely a despawn race. Drop silently.
                    continue
                self._entity_poses[state.entity_id] = state.pose
            self.entity_states.publish(state)
            batch_entries.append((desc, state.pose))
            ts = max(ts, state.ts)
        if batch_entries:
            self.entity_state_batch.publish(EntityStateBatch(entries=batch_entries, ts=ts))

    def _entity_spawn_messages(self) -> list[dict[str, Any]]:
        """Replay payload: a fresh tab gets spawn commands for every entity.

        Initial pose is the identity (browser physics will integrate from
        there once the upstream caller next pushes set_entity_pose). For
        kinematic-only entities this is fine; dynamic entities accept
        whatever the next state tick reflects.
        """
        with self._entity_lock:
            entities = list(self._entities.values())
        identity = Pose()
        return [
            {"type": "entity_spawn", "descriptor": d.to_wire(), "pose": pose_to_wire(identity)}
            for d in entities
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
                await websocket.send_json(payload)
            except Exception:
                dead.append(websocket)
        for websocket in dead:
            self._clients.discard(websocket)
