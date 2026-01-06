#!/usr/bin/env python3

# Copyright 2025 Dimensional Inc.
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
"""

import asyncio
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from reactivex.disposable import Disposable
import socketio  # type: ignore[import-untyped]
from starlette.applications import Starlette
from starlette.responses import HTMLResponse
from starlette.routing import Route
import uvicorn

from dimos.core import In, Module, Out, rpc
from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.types import LatLon
from dimos.msgs.geometry_msgs import PoseStamped, Twist, TwistStamped, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.utils.logging_config import setup_logger

from .optimized_costmap import OptimizedCostmapEncoder

logger = setup_logger()


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

    Outputs:
        - click_goal: Goal position from user clicks
    """

    # LCM inputs
    odom: In[PoseStamped]
    gps_location: In[LatLon]
    path: In[Path]
    global_costmap: In[OccupancyGrid]

    # LCM outputs
    goal_request: Out[PoseStamped]
    gps_goal: Out[LatLon]
    explore_cmd: Out[Bool]
    stop_explore_cmd: Out[Bool]
    cmd_vel: Out[Twist]
    movecmd_stamped: Out[TwistStamped]

    def __init__(self, port: int = 7779, **kwargs) -> None:  # type: ignore[no-untyped-def]
        """Initialize the WebSocket visualization module.

        Args:
            port: Port to run the web server on
        """
        super().__init__(**kwargs)

        self.port = port
        self._uvicorn_server_thread: threading.Thread | None = None
        self.sio: socketio.AsyncServer | None = None
        self.app = None
        self._broadcast_loop = None
        self._broadcast_thread = None
        self._uvicorn_server: uvicorn.Server | None = None

        self.vis_state = {}  # type: ignore[var-annotated]
        self.state_lock = threading.Lock()
        self.costmap_encoder = OptimizedCostmapEncoder(chunk_size=64)

        # Track GPS goal points for visualization
        self.gps_goal_points: list[dict[str, float]] = []
        logger.info(
            f"WebSocket visualization module initialized on port {port}, GPS goal tracking enabled"
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

        try:
            unsub = self.odom.subscribe(self._on_robot_pose)
            self._disposables.add(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.gps_location.subscribe(self._on_gps_location)
            self._disposables.add(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.path.subscribe(self._on_path)
            self._disposables.add(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.global_costmap.subscribe(self._on_global_costmap)
            self._disposables.add(Disposable(unsub))
        except Exception:
            ...

    @rpc
    def stop(self) -> None:
        if self._uvicorn_server:
            self._uvicorn_server.should_exit = True

        if self.sio and self._broadcast_loop and not self._broadcast_loop.is_closed():

            async def _disconnect_all() -> None:
                await self.sio.disconnect()

            asyncio.run_coroutine_threadsafe(_disconnect_all(), self._broadcast_loop)

        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            self._broadcast_loop.call_soon_threadsafe(self._broadcast_loop.stop)

        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=1.0)

        if self._uvicorn_server_thread and self._uvicorn_server_thread.is_alive():
            self._uvicorn_server_thread.join(timeout=2.0)

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
            return HTMLResponse("<html><body>Use the extension.</body></html>")

        routes = [Route("/", serve_index)]
        starlette_app = Starlette(routes=routes)

        self.app = socketio.ASGIApp(self.sio, starlette_app)

        # Register SocketIO event handlers
        @self.sio.event  # type: ignore[misc, untyped-decorator]
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

        @self.sio.event  # type: ignore[misc, untyped-decorator]
        async def click(sid, position) -> None:  # type: ignore[no-untyped-def]
            goal = PoseStamped(
                position=(position[0], position[1], 0),
                orientation=(0, 0, 0, 1),  # Default orientation
                frame_id="world",
            )
            self.goal_request.publish(goal)
            logger.info(
                "Click goal published", x=round(goal.position.x, 3), y=round(goal.position.y, 3)
            )

        @self.sio.event  # type: ignore[misc, untyped-decorator]
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

        @self.sio.event  # type: ignore[misc, untyped-decorator]
        async def start_explore(sid: str) -> None:
            logger.info("Starting exploration")
            self.explore_cmd.publish(Bool(data=True))

        @self.sio.event  # type: ignore[misc, untyped-decorator]
        async def stop_explore(sid) -> None:  # type: ignore[no-untyped-def]
            logger.info("Stopping exploration")
            self.stop_explore_cmd.publish(Bool(data=True))

        @self.sio.event  # type: ignore[misc, untyped-decorator]
        async def clear_gps_goals(sid: str) -> None:
            logger.info("Clearing all GPS goal points")
            self.gps_goal_points.clear()
            if self.sio is not None:
                await self.sio.emit("gps_travel_goal_points", self.gps_goal_points)
            logger.info("GPS goal points cleared and updated clients")

        @self.sio.event  # type: ignore[misc, untyped-decorator]
        async def move_command(sid: str, data: dict[str, Any]) -> None:
            # Publish Twist if transport is configured
            if self.cmd_vel and self.cmd_vel.transport:
                twist = Twist(
                    linear=Vector3(data["linear"]["x"], data["linear"]["y"], data["linear"]["z"]),
                    angular=Vector3(
                        data["angular"]["x"], data["angular"]["y"], data["angular"]["z"]
                    ),
                )
                self.cmd_vel.publish(twist)

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
            host="0.0.0.0",
            port=self.port,
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


websocket_vis = WebsocketVisModule.blueprint

__all__ = ["WebsocketVisModule", "websocket_vis"]
