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

"""VR World module — live single-robot visualization + driving in VR.

Subclasses :class:`QuestTeleopModule` to reuse its embedded HTTPS web server,
then:

* Subscribes to the robot's live ``odom`` / ``lidar`` / ``color_image`` streams.
* Accumulates lidar into an Open3D voxel grid (live) and pushes the full map to
  the headset on a throttle (the map is too big to resend every frame).
* Pushes the latest robot pose (fast) and camera frame (throttled).
* Publishes ``cmd_vel`` (Twist) from the headset's left thumbstick so the same
  headset drives the robot.

All view navigation (god-view yaw / scale / teleport) is client-side. The server
is a throttled relay + voxel accumulator + Twist publisher.

See PLAN.md for the architecture overview.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from pathlib import Path
import threading
import time
from typing import Any

import cv2
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.experimental.vr_world.messages import (
    MSG_CAMERA,
    MSG_VOXEL_MAP,
    decode_text,
    encode_binary,
    encode_text,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.teleop.quest.quest_teleop_module import QuestTeleopConfig, QuestTeleopModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "web" / "static"


@dataclass(eq=False)
class _ClientConn:
    """One connected vr-world client. Mirrors memory_world._ClientConn."""

    ws: WebSocket
    loop: asyncio.AbstractEventLoop
    queue: asyncio.Queue[bytes | str] = field(default_factory=lambda: asyncio.Queue(maxsize=64))

    def send_threadsafe(self, msg: bytes | str) -> None:
        try:
            self.loop.call_soon_threadsafe(self._enqueue, msg)
        except RuntimeError:
            pass

    def _enqueue(self, msg: bytes | str) -> None:
        try:
            self.queue.put_nowait(msg)
        except asyncio.QueueFull:
            # Backpressure — drop oldest, keep newest (live data; stale is useless).
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            self.queue.put_nowait(msg)


class VrWorldConfig(QuestTeleopConfig):
    """Config for the live VR World."""

    # ---- voxel map ----
    voxel_size: float = 0.08
    max_points: int = 150_000
    map_z_min: float = -0.2
    map_z_max: float = 2.4
    # If the robot already publishes lidar in the odom/world frame, skip the
    # per-scan pose transform (otherwise we'd double-transform). Default False:
    # raw sensor scans get transformed by the latest odom pose.
    lidar_world_frame: bool = False

    # ---- throttle rates (Hz) ----
    map_resend_hz: float = 1.5  # full voxel-map push to headset
    lidar_process_hz: float = 4.0  # how often we add a scan to the grid
    pose_push_hz: float = 15.0  # robot pose push (cheap)
    image_push_hz: float = 3.0  # camera frame push

    # ---- camera thumbnail ----
    image_max_size: int = 320
    image_jpeg_quality: int = 60

    # ---- driving (left thumbstick -> Twist) ----
    linear_speed: float = 0.8  # m/s at full deflection
    angular_speed: float = 1.2  # rad/s at full deflection

    client_route: str = "/vr_world"
    ws_route: str = "/ws_vr_world"
    listen_host: str = "0.0.0.0"


class VrWorldModule(QuestTeleopModule):
    """Live VR visualization + driving for one robot.

    See :mod:`dimos.experimental.vr_world` for the overview.
    """

    config: VrWorldConfig

    # Live inputs from the robot.
    odom: In[PoseStamped]
    lidar: In[PointCloud2]
    color_image: In[Image]
    # Drive command out to the robot.
    cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        self._world_clients: set[_ClientConn] = set()
        self._clients_lock = threading.RLock()

        # Voxel grid is created lazily on the first lidar scan (needs Open3D).
        self._grid: Any = None
        self._grid_lock = threading.RLock()
        self._latest_pose: PoseStamped | None = None

        # Throttle bookkeeping: last-emit wall-clock per stream.
        self._last_lidar_add = 0.0
        self._last_map_send = 0.0
        self._last_pose_send = 0.0
        self._last_image_send = 0.0

        super().__init__(**kwargs)

        self._web_server.host = self.config.listen_host

    # ---- routes ------------------------------------------------------------

    def _setup_routes(self) -> None:
        super()._setup_routes()
        app = self._web_server.app

        @app.get(self.config.client_route, response_class=HTMLResponse)
        async def vr_world_index() -> HTMLResponse:
            return HTMLResponse(content=(STATIC_DIR / "index.html").read_text())

        if STATIC_DIR.is_dir():
            app.mount("/static_vw", StaticFiles(directory=str(STATIC_DIR)), name="vr_world_static")

        @app.websocket(self.config.ws_route)
        async def ws_world(ws: WebSocket) -> None:
            await self._handle_ws(ws)

    # ---- lifecycle ---------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        # Wire the live robot streams once the module is running.
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))
        logger.info("VR World module started; subscribed to odom/lidar/color_image")

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            with self._grid_lock:
                if self._grid is not None:
                    try:
                        self._grid.dispose()
                    except Exception:
                        logger.exception("error disposing voxel grid")
                    self._grid = None

    # ---- websocket handling ------------------------------------------------

    async def _handle_ws(self, ws: WebSocket) -> None:
        await ws.accept()
        loop = asyncio.get_running_loop()
        conn = _ClientConn(ws=ws, loop=loop)
        with self._clients_lock:
            self._world_clients.add(conn)
        logger.info("vr-world client connected (now %d)", len(self._world_clients))

        sender = asyncio.create_task(self._sender_loop(conn))
        # Push whatever map we already have so a late joiner isn't staring at nothing.
        threading.Thread(target=self._send_current_map, args=(conn,), daemon=True).start()

        try:
            while True:
                raw = await ws.receive_text()
                msg = decode_text(raw)
                if msg:
                    self._on_client_message(conn, msg)
        except WebSocketDisconnect:
            logger.info("vr-world client disconnected")
        except Exception:
            logger.exception("vr-world ws error")
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

    def _broadcast(self, msg: bytes | str) -> None:
        with self._clients_lock:
            clients = list(self._world_clients)
        for conn in clients:
            conn.send_threadsafe(msg)

    def _has_clients(self) -> bool:
        with self._clients_lock:
            return bool(self._world_clients)

    # ---- live stream callbacks --------------------------------------------

    def _on_odom(self, pose: PoseStamped) -> None:
        self._latest_pose = pose
        now = time.monotonic()
        if now - self._last_pose_send < 1.0 / max(self.config.pose_push_hz, 1e-3):
            return
        self._last_pose_send = now
        if not self._has_clients():
            return
        self._broadcast(
            encode_text(
                "robot_pose",
                pose=[
                    float(pose.x),
                    float(pose.y),
                    float(pose.z),
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                ],
            )
        )

    def _on_lidar(self, cloud: PointCloud2) -> None:
        if not self._has_clients():
            return
        now = time.monotonic()
        # Throttle how often we fold a scan into the grid (accumulation cost).
        if now - self._last_lidar_add >= 1.0 / max(self.config.lidar_process_hz, 1e-3):
            self._last_lidar_add = now
            self._accumulate(cloud)
        # Throttle the (heavy) full-map resend independently.
        if now - self._last_map_send >= 1.0 / max(self.config.map_resend_hz, 1e-3):
            self._last_map_send = now
            self._send_current_map()

    def _accumulate(self, cloud: PointCloud2) -> None:
        try:
            world_cloud = cloud
            if not self.config.lidar_world_frame:
                pose = self._latest_pose
                if pose is None:
                    return  # no pose yet; can't place the scan
                tf = Transform(
                    translation=Vector3(float(pose.x), float(pose.y), float(pose.z)),
                    rotation=Quaternion(
                        float(pose.orientation.x),
                        float(pose.orientation.y),
                        float(pose.orientation.z),
                        float(pose.orientation.w),
                    ),
                )
                world_cloud = cloud.transform(tf)
            with self._grid_lock:
                if self._grid is None:
                    from dimos.mapping.voxels import VoxelGrid

                    self._grid = VoxelGrid(voxel_size=self.config.voxel_size)
                self._grid.add_frame(world_cloud)
        except Exception:
            logger.exception("lidar accumulate failed")

    def _send_current_map(self, conn: _ClientConn | None = None) -> None:
        try:
            with self._grid_lock:
                if self._grid is None:
                    return
                global_cloud = self._grid.get_global_pointcloud2()
            built = self._pack_cloud(global_cloud)
            if built is None:
                return
            header, payload = built
            frame = encode_binary(MSG_VOXEL_MAP, header, payload)
            if conn is not None:
                conn.send_threadsafe(frame)
            else:
                self._broadcast(frame)
        except Exception:
            logger.exception("send map failed")

    def _pack_cloud(self, cloud: PointCloud2) -> tuple[dict[str, Any], bytes] | None:
        xyz, _colors = cloud.as_numpy()
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
        rgb = self._height_colors(positions)
        header = {
            "n": int(positions.shape[0]),
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
        return header, positions.tobytes() + rgb.tobytes()

    def _height_colors(self, positions: np.ndarray) -> np.ndarray:
        """Bright violet→red rainbow by Z, anchored to the fixed height slab."""
        zc = positions[:, 2]
        lo, hi = float(self.config.map_z_min), float(self.config.map_z_max)
        if hi - lo < 1e-3:
            lo, hi = float(zc.min()), float(zc.max()) + 1e-3
        t = np.clip((zc - lo) / (hi - lo), 0.0, 1.0)
        h = ((1.0 - t) * 140.0).astype(np.uint8).reshape(-1, 1)
        full = np.full_like(h, 255)
        hsv = np.concatenate([h, full, full], axis=1).reshape(-1, 1, 3)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR).reshape(-1, 3)
        return np.ascontiguousarray(bgr[:, ::-1])

    def _on_image(self, img: Image) -> None:
        if not self._has_clients():
            return
        now = time.monotonic()
        if now - self._last_image_send < 1.0 / max(self.config.image_push_hz, 1e-3):
            return
        self._last_image_send = now
        try:
            im = img
            if hasattr(im, "resize_to_fit"):
                im, _ = im.resize_to_fit(self.config.image_max_size, self.config.image_max_size)
            bgr = im.to_bgr().to_opencv() if hasattr(im, "to_bgr") else im.to_opencv()
            ok, buf = cv2.imencode(
                ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.image_jpeg_quality)]
            )
            if not ok:
                return
            self._broadcast(encode_binary(MSG_CAMERA, {}, buf.tobytes()))
        except Exception:
            logger.exception("camera encode failed")

    # ---- client -> server --------------------------------------------------

    def _on_client_message(self, conn: _ClientConn, msg: dict[str, Any]) -> None:
        kind = msg.get("type")
        if kind == "drive":
            self._publish_drive(msg)
        elif kind == "ping":
            conn.send_threadsafe(encode_text("pong"))
        elif kind == "diag":
            logger.info(
                "[client/diag] %s %s",
                msg.get("event", "?"),
                {k: v for k, v in msg.items() if k not in ("type", "event")},
            )
        elif kind in (
            "yaw",
            "scale_delta",
            "teleport_commit",
            "teleport_aim",
            "teleport_cancel",
            "reset_view",
            "toggle_render",
        ):
            logger.debug("[client] %s", kind)
        else:
            logger.warning("[client] unknown msg kind=%r", kind)

    def _publish_drive(self, msg: dict[str, Any]) -> None:
        try:
            x = float(msg.get("x", 0.0))  # forward/back, [-1, 1]
            yaw = float(msg.get("yaw", 0.0))  # turn rate, [-1, 1]
        except (TypeError, ValueError):
            return
        twist = Twist(
            Vector3(x * self.config.linear_speed, 0.0, 0.0),
            Vector3(0.0, 0.0, yaw * self.config.angular_speed),
        )
        self.cmd_vel.publish(twist)
