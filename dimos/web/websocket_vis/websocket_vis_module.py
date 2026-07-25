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

"""
WebSocket Visualization Module for Dimos navigation and mapping.

This module provides a WebSocket data server for real-time visualization.
The frontend is served from a separate HTML file.
"""

import asyncio
import json
from pathlib import Path as FilePath
import threading
import time
from typing import Any
from uuid import uuid4
import webbrowser

from dimos_lcm.std_msgs import Bool, String
from reactivex.disposable import Disposable
import socketio  # type: ignore[import-untyped]
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import FileResponse, RedirectResponse, Response
from starlette.routing import Route
import uvicorn

from dimos.utils.data import get_data

# Path to the frontend HTML templates and command-center build
_TEMPLATES_DIR = FilePath(__file__).parent.parent / "templates"
_DASHBOARD_HTML = _TEMPLATES_DIR / "rerun_dashboard.html"
_COMMAND_CENTER_DIR = (
    FilePath(__file__).parent.parent / "command-center-extension" / "dist-standalone"
)
_MAP_MARKING_EXTENSION_JS = FilePath(__file__).parent / "map_marking_extension.js"
_MAP_MARKING_SCRIPT_TAG = (
    '<script defer src="/map-marking-extension.js"></script>'
)
_MAX_MARKABLE_COST = 49

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.models import LatLon
from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

from .optimized_costmap import OptimizedCostmapEncoder

logger = setup_logger()

_browser_open_lock = threading.Lock()
_browser_opened = False


class WebsocketConfig(ModuleConfig):
    port: int = 7779


class WebsocketVisModule(Module):
    """
    WebSocket-based visualization module for real-time navigation data.

    This module provides a web interface for visualizing:
    - Robot position and orientation
    - Navigation paths
    - Costmaps
    - Interactive goal setting via mouse clicks

    Inputs:
        - robot_pose: Current robot position
        - path: Navigation path
        - global_costmap: Global costmap for visualization
        - semantic_place_confirmation: Result of a semantic-place request

    Outputs:
        - click_goal: Goal position from user clicks
        - semantic_place_candidate: Operator-named map point that must be
          confirmed by the canonical SemanticWorld
    """

    config: WebsocketConfig

    # LCM inputs
    odom: In[PoseStamped]
    gps_location: In[LatLon]
    path: In[Path]
    global_costmap: In[OccupancyGrid]
    semantic_place_confirmation: In[String]

    # LCM outputs
    goal_request: Out[PoseStamped]
    semantic_place_candidate: Out[String]
    gps_goal: Out[LatLon]
    explore_cmd: Out[Bool]
    stop_explore_cmd: Out[Bool]
    tele_cmd_vel: Out[Twist]
    movecmd_stamped: Out[TwistStamped]

    def __init__(self, **kwargs: Any) -> None:
        """Initialize the WebSocket visualization module.

        Args:
            port: Port to run the web server on
            cfg: Optional global config for viewer settings
        """
        super().__init__(**kwargs)
        self._uvicorn_server_thread: threading.Thread | None = None
        self.sio: socketio.AsyncServer | None = None
        self.app = None
        self._broadcast_loop = None
        self._broadcast_thread = None
        self._uvicorn_server: uvicorn.Server | None = None

        self.vis_state = {}  # type: ignore[var-annotated]
        self.state_lock = threading.Lock()
        self.costmap_encoder = OptimizedCostmapEncoder(chunk_size=64)
        self._map_marking_lock = threading.Lock()
        self._pending_map_marker_name: str | None = None
        self._map_markers: dict[str, dict[str, Any]] = {}
        self._map_marking_error: str | None = None

        # Track GPS goal points for visualization
        self.gps_goal_points: list[dict[str, float]] = []
        logger.info(
            f"WebSocket visualization module initialized on port {self.config.port}, GPS goal tracking enabled"
        )

    def _start_broadcast_loop(self) -> None:
        def websocket_vis_loop() -> None:
            self._broadcast_loop = asyncio.new_event_loop()  # type: ignore[assignment]
            asyncio.set_event_loop(self._broadcast_loop)
            try:
                self._broadcast_loop.run_forever()  # type: ignore[attr-defined]
            except Exception as e:
                logger.error(f"Broadcast loop error: {e}")
            finally:
                self._broadcast_loop.close()  # type: ignore[attr-defined]

        self._broadcast_thread = threading.Thread(target=websocket_vis_loop, daemon=True)  # type: ignore[assignment]
        self._broadcast_thread.start()  # type: ignore[attr-defined]

    @rpc
    def start(self) -> None:
        super().start()

        self._create_server()

        self._start_broadcast_loop()

        self._uvicorn_server_thread = threading.Thread(target=self._run_uvicorn_server, daemon=True)
        self._uvicorn_server_thread.start()

        # Only auto-open when the user chose web-based viewing.
        if self.config.g.viewer == "rerun" and self.config.g.rerun_open in ("web", "both"):
            url = f"http://localhost:{self.config.port}/"
            logger.info(f"Dimensional Command Center: {url}")

            global _browser_opened
            with _browser_open_lock:
                if not _browser_opened:
                    try:
                        webbrowser.open_new_tab(url)
                        _browser_opened = True
                    except Exception as e:
                        logger.debug(f"Failed to open browser: {e}")

        try:
            unsub = self.odom.subscribe(self._on_robot_pose)
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.gps_location.subscribe(self._on_gps_location)
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.path.subscribe(self._on_path)
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.global_costmap.subscribe(self._on_global_costmap)
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.semantic_place_confirmation.subscribe(
                self._on_semantic_place_confirmation
            )
            self.register_disposable(Disposable(unsub))
        except Exception:
            ...

    @rpc
    def stop(self) -> None:
        if getattr(self, "_ws_stopped", False):
            return
        self._ws_stopped = True

        if self._uvicorn_server:
            self._uvicorn_server.should_exit = True

        if self.sio and self._broadcast_loop and not self._broadcast_loop.is_closed():

            async def _disconnect_all() -> None:
                await self.sio.disconnect()

            asyncio.run_coroutine_threadsafe(_disconnect_all(), self._broadcast_loop)

        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            self._broadcast_loop.call_soon_threadsafe(self._broadcast_loop.stop)

        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        if self._uvicorn_server_thread and self._uvicorn_server_thread.is_alive():
            self._uvicorn_server_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        super().stop()

    @rpc
    def set_gps_travel_goal_points(self, points: list[LatLon]) -> None:
        json_points = [{"lat": x.lat, "lon": x.lon} for x in points]
        self.vis_state["gps_travel_goal_points"] = json_points
        self._emit("gps_travel_goal_points", json_points)

    def _create_server(self) -> None:
        # Create SocketIO server
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

        async def serve_index(request):  # type: ignore[no-untyped-def]
            """Serve appropriate HTML based on viewer mode."""
            if not (
                self.config.g.viewer == "rerun" and self.config.g.rerun_open in ("web", "both")
            ):
                return RedirectResponse(url="/command-center")
            return FileResponse(_DASHBOARD_HTML, media_type="text/html")

        async def serve_command_center(request):  # type: ignore[no-untyped-def]
            """Serve the command center 2D visualization (built React app)."""
            index_file = get_data("command_center.html")
            if index_file.exists():
                html = index_file.read_text(encoding="utf-8")
                return Response(
                    content=self._inject_map_marking_extension(html),
                    media_type="text/html",
                )
            else:
                return Response(
                    content="Command center not built. Run: cd dimos/web/command-center-extension && npm install && npm run build:standalone",
                    status_code=503,
                    media_type="text/plain",
                )

        async def serve_map_marking_extension(request):  # type: ignore[no-untyped-def]
            """Serve the thin semantic marker overlay for Command Center."""
            return FileResponse(
                _MAP_MARKING_EXTENSION_JS,
                media_type="text/javascript",
            )

        async def map_marking_state(request):  # type: ignore[no-untyped-def]
            """Return pending and confirmed map markers without robot control."""
            return Response(
                content=json.dumps(
                    self._map_marking_state(),
                    ensure_ascii=False,
                    separators=(",", ":"),
                ),
                media_type="application/json",
            )

        async def arm_map_marking(request: Request) -> Response:
            """Arm exactly one non-motion map click with an operator label."""
            try:
                payload = await request.json()
                name = payload.get("name") if isinstance(payload, dict) else None
                self._arm_map_marking(name)
            except (TypeError, ValueError) as exc:
                return Response(
                    content=json.dumps(
                        {"accepted": False, "reason": str(exc)},
                        ensure_ascii=False,
                    ),
                    status_code=400,
                    media_type="application/json",
                )
            return Response(
                content=json.dumps(
                    {"accepted": True, "name": self._pending_map_marker_name},
                    ensure_ascii=False,
                ),
                media_type="application/json",
            )

        async def cancel_map_marking(request):  # type: ignore[no-untyped-def]
            """Cancel a pending map marker without changing robot state."""
            self._cancel_map_marking()
            return Response(
                content='{"accepted":true}',
                media_type="application/json",
            )

        routes = [
            Route("/", serve_index),
            Route("/command-center", serve_command_center),
            Route("/map-marking-extension.js", serve_map_marking_extension),
            Route("/api/map-marking/state", map_marking_state),
            Route("/api/map-marking/arm", arm_map_marking, methods=["POST"]),
            Route(
                "/api/map-marking/cancel",
                cancel_map_marking,
                methods=["POST"],
            ),
        ]

        starlette_app = Starlette(routes=routes)

        self.app = socketio.ASGIApp(self.sio, starlette_app)

        # Register SocketIO event handlers
        @self.sio.event  # type: ignore[untyped-decorator]
        async def connect(sid, environ) -> None:  # type: ignore[no-untyped-def]
            with self.state_lock:
                current_state = dict(self.vis_state)

            # Include GPS goal points in the initial state
            if self.gps_goal_points:
                current_state["gps_travel_goal_points"] = self.gps_goal_points

            # Force full costmap update on new connection
            self.costmap_encoder.last_full_grid = None

            await self.sio.emit("full_state", current_state, room=sid)  # type: ignore[union-attr]
            logger.info(
                f"Client {sid} connected, sent state with {len(self.gps_goal_points)} GPS goal points"
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def click(sid, position) -> None:  # type: ignore[no-untyped-def]
            handled, candidate = self._consume_map_marking_click(
                float(position[0]),
                float(position[1]),
            )
            if handled:
                if candidate is not None:
                    self.semantic_place_candidate.publish(
                        String(
                            json.dumps(
                                candidate,
                                ensure_ascii=False,
                                separators=(",", ":"),
                            )
                        )
                    )
                    logger.info(
                        "Semantic map marker requested",
                        name=candidate["place"]["name"],
                        x=round(candidate["place"]["pose"]["x"], 3),
                        y=round(candidate["place"]["pose"]["y"], 3),
                    )
                if self.sio is not None:
                    await self.sio.emit(
                        "semantic_map_markers",
                        self._map_marking_state(),
                    )
                return
            goal = PoseStamped(
                position=(position[0], position[1], 0),
                orientation=(0, 0, 0, 1),  # Default orientation
                frame_id="world",
            )
            self.goal_request.publish(goal)
            logger.info(
                "Click goal published", x=round(goal.position.x, 3), y=round(goal.position.y, 3)
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def gps_goal(sid: str, goal: dict[str, float]) -> None:
            logger.info(f"Received GPS goal: {goal}")

            # Publish the goal to LCM
            self.gps_goal.publish(LatLon(lat=goal["lat"], lon=goal["lon"]))

            # Add to goal points list for visualization
            self.gps_goal_points.append(goal)
            logger.info(f"Added GPS goal to list. Total goals: {len(self.gps_goal_points)}")

            # Emit updated goal points back to all connected clients
            if self.sio is not None:
                await self.sio.emit("gps_travel_goal_points", self.gps_goal_points)
            logger.debug(
                f"Emitted gps_travel_goal_points with {len(self.gps_goal_points)} points: {self.gps_goal_points}"
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def start_explore(sid: str) -> None:
            logger.info("Starting exploration")
            self.explore_cmd.publish(Bool(data=True))

        @self.sio.event  # type: ignore[untyped-decorator]
        async def stop_explore(sid) -> None:  # type: ignore[no-untyped-def]
            logger.info("Stopping exploration")
            self.stop_explore_cmd.publish(Bool(data=True))

        @self.sio.event  # type: ignore[untyped-decorator]
        async def clear_gps_goals(sid: str) -> None:
            logger.info("Clearing all GPS goal points")
            self.gps_goal_points.clear()
            if self.sio is not None:
                await self.sio.emit("gps_travel_goal_points", self.gps_goal_points)
            logger.info("GPS goal points cleared and updated clients")

        @self.sio.event  # type: ignore[untyped-decorator]
        async def move_command(sid: str, data: dict[str, Any]) -> None:
            # Publish Twist if transport is configured
            if self.tele_cmd_vel and self.tele_cmd_vel.transport:
                twist = Twist(
                    linear=Vector3(data["linear"]["x"], data["linear"]["y"], data["linear"]["z"]),
                    angular=Vector3(
                        data["angular"]["x"], data["angular"]["y"], data["angular"]["z"]
                    ),
                )
                self.tele_cmd_vel.publish(twist)

            # Publish TwistStamped if transport is configured
            if self.movecmd_stamped and self.movecmd_stamped.transport:
                twist_stamped = TwistStamped(
                    ts=time.time(),
                    frame_id="base_link",
                    linear=Vector3(data["linear"]["x"], data["linear"]["y"], data["linear"]["z"]),
                    angular=Vector3(
                        data["angular"]["x"], data["angular"]["y"], data["angular"]["z"]
                    ),
                )
                self.movecmd_stamped.publish(twist_stamped)

    def _run_uvicorn_server(self) -> None:
        config = uvicorn.Config(
            self.app,  # type: ignore[arg-type]
            host=global_config.listen_host,
            port=self.config.port,
            log_level="error",  # Reduce verbosity
        )
        self._uvicorn_server = uvicorn.Server(config)
        self._uvicorn_server.run()

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        pose_data = {"type": "vector", "c": [msg.position.x, msg.position.y, msg.position.z]}
        self.vis_state["robot_pose"] = pose_data
        self._emit("robot_pose", pose_data)

    def _on_gps_location(self, msg: LatLon) -> None:
        pose_data = {"lat": msg.lat, "lon": msg.lon}
        self.vis_state["gps_location"] = pose_data
        self._emit("gps_location", pose_data)

    def _on_path(self, msg: Path) -> None:
        points = [[pose.position.x, pose.position.y] for pose in msg.poses]
        path_data = {"type": "path", "points": points}
        self.vis_state["path"] = path_data
        self._emit("path", path_data)

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        costmap_data = self._process_costmap(msg)
        self.vis_state["costmap"] = costmap_data
        self._emit("costmap", costmap_data)

    def _inject_map_marking_extension(self, html: str) -> str:
        """Inject the semantic marker overlay without rebuilding Command Center."""

        if _MAP_MARKING_SCRIPT_TAG in html:
            return html
        if "</body>" not in html:
            return f"{html}{_MAP_MARKING_SCRIPT_TAG}"
        return html.replace(
            "</body>",
            f"{_MAP_MARKING_SCRIPT_TAG}</body>",
            1,
        )

    def _arm_map_marking(self, raw_name: Any) -> None:
        if not isinstance(raw_name, str):
            raise ValueError("地点名称不能为空")
        name = " ".join(raw_name.strip().split())
        if not name:
            raise ValueError("地点名称不能为空")
        if len(name) > 200:
            raise ValueError("地点名称不能超过 200 个字符")
        with self._map_marking_lock:
            self._pending_map_marker_name = name
            self._map_marking_error = None

    def _cancel_map_marking(self) -> None:
        with self._map_marking_lock:
            self._pending_map_marker_name = None
            self._map_marking_error = None

    def _consume_map_marking_click(
        self,
        x: float,
        y: float,
    ) -> tuple[bool, dict[str, Any] | None]:
        """Consume an armed click without publishing a navigation goal."""

        with self._map_marking_lock:
            name = self._pending_map_marker_name
            if name is None:
                return False, None
            if not self._is_markable_world_point(x, y):
                self._map_marking_error = "该位置不是当前地图中的可通行空地"
                return True, None

            request_id = f"map-marker-{uuid4().hex}"
            marker = {
                "request_id": request_id,
                "name": name,
                "x": x,
                "y": y,
                "frame_id": "world",
                "status": "pending",
                "reason": None,
            }
            self._map_markers[request_id] = marker
            self._pending_map_marker_name = None
            self._map_marking_error = None

        candidate = {
            "request_id": request_id,
            "place": {
                "name": name,
                "aliases": [],
                "pose": {
                    "frame_id": "world",
                    "ts": time.time(),
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                    "qw": 1.0,
                },
            },
        }
        return True, candidate

    def _is_markable_world_point(self, x: float, y: float) -> bool:
        grid = self.costmap_encoder.last_full_grid
        costmap = self.vis_state.get("costmap")
        if grid is None or not isinstance(costmap, dict):
            return False
        origin = costmap.get("origin")
        if not isinstance(origin, dict):
            return False
        coords = origin.get("c")
        resolution = costmap.get("resolution")
        if (
            not isinstance(coords, list)
            or len(coords) < 2
            or not isinstance(resolution, (int, float))
            or resolution <= 0
        ):
            return False
        col = int((x - float(coords[0])) // float(resolution))
        row = int((y - float(coords[1])) // float(resolution))
        if row < 0 or col < 0 or row >= grid.shape[0] or col >= grid.shape[1]:
            return False
        cost = float(grid[row, col])
        return 0 <= cost <= _MAX_MARKABLE_COST

    def _map_marking_state(self) -> dict[str, Any]:
        with self._map_marking_lock:
            pending_name = self._pending_map_marker_name
            markers = list(self._map_markers.values())
            error = self._map_marking_error

        map_meta: dict[str, Any] | None = None
        costmap = self.vis_state.get("costmap")
        if isinstance(costmap, dict):
            grid = costmap.get("grid")
            origin = costmap.get("origin")
            shape = grid.get("shape") if isinstance(grid, dict) else None
            coords = origin.get("c") if isinstance(origin, dict) else None
            resolution = costmap.get("resolution")
            if (
                isinstance(shape, list)
                and len(shape) == 2
                and isinstance(coords, list)
                and len(coords) >= 2
                and isinstance(resolution, (int, float))
            ):
                map_meta = {
                    "rows": int(shape[0]),
                    "cols": int(shape[1]),
                    "origin_x": float(coords[0]),
                    "origin_y": float(coords[1]),
                    "resolution": float(resolution),
                }
        return {
            "pending_name": pending_name,
            "markers": markers,
            "error": error,
            "map": map_meta,
        }

    def _on_semantic_place_confirmation(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            logger.warning("Ignored invalid semantic place confirmation")
            return
        if not isinstance(payload, dict):
            return
        request_id = payload.get("request_id")
        if not isinstance(request_id, str):
            return
        with self._map_marking_lock:
            marker = self._map_markers.get(request_id)
            if marker is None:
                return
            marker["status"] = (
                "confirmed" if payload.get("accepted") is True else "rejected"
            )
            marker["reason"] = payload.get("reason")
        self._emit("semantic_map_markers", self._map_marking_state())

    def _process_costmap(self, costmap: OccupancyGrid) -> dict[str, Any]:
        """Convert OccupancyGrid to visualization format."""
        costmap = gradient(simple_inflate(costmap, 0.1), max_distance=1.0)
        grid_data = self.costmap_encoder.encode_costmap(costmap.grid)

        return {
            "type": "costmap",
            "grid": grid_data,
            "origin": {
                "type": "vector",
                "c": [costmap.origin.position.x, costmap.origin.position.y, 0],
            },
            "resolution": costmap.resolution,
            "origin_theta": 0,  # Assuming no rotation for now
        }

    def _emit(self, event: str, data: Any) -> None:
        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.sio.emit(event, data), self._broadcast_loop)
