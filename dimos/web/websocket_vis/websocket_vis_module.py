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
from typing import Any, Dict, Optional
import base64
import numpy as np

import socketio
import uvicorn
from starlette.applications import Starlette
from starlette.responses import HTMLResponse
from starlette.routing import Route

from dimos.core import Module, In, Out, rpc
from dimos_lcm.std_msgs import Bool
from dimos.msgs.geometry_msgs import PoseStamped, Twist, TwistStamped, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.web.websocket_vis")


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
    robot_pose: In[PoseStamped] = None
    path: In[Path] = None
    global_costmap: In[OccupancyGrid] = None

    # LCM outputs
    click_goal: Out[PoseStamped] = None
    explore_cmd: Out[Bool] = None
    stop_explore_cmd: Out[Bool] = None
    movecmd: Out[Twist] = None
    movecmd_stamped: Out[TwistStamped] = None

    def __init__(self, port: int = 7779, **kwargs):
        """Initialize the WebSocket visualization module.

        Args:
            port: Port to run the web server on
        """
        super().__init__(**kwargs)

        self.port = port
        self.server_thread: Optional[threading.Thread] = None
        self.sio: Optional[socketio.AsyncServer] = None
        self.app = None
        self._broadcast_loop = None
        self._broadcast_thread = None

        self.vis_state = {}
        self.state_lock = threading.Lock()

        logger.info(f"WebSocket visualization module initialized on port {port}")

    def _start_broadcast_loop(self):
        def run_loop():
            self._broadcast_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._broadcast_loop)
            try:
                self._broadcast_loop.run_forever()
            except Exception as e:
                logger.error(f"Broadcast loop error: {e}")
            finally:
                self._broadcast_loop.close()

        self._broadcast_thread = threading.Thread(target=run_loop, daemon=True)
        self._broadcast_thread.start()

    @rpc
    def start(self):
        self._create_server()
        self._start_broadcast_loop()

        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()

        # Only subscribe to connected topics
        if self.robot_pose.connection is not None:
            self.robot_pose.subscribe(self._on_robot_pose)
        if self.path.connection is not None:
            self.path.subscribe(self._on_path)
        if self.global_costmap.connection is not None:
            self.global_costmap.subscribe(self._on_global_costmap)

        logger.info(f"WebSocket server started on http://localhost:{self.port}")

    @rpc
    def stop(self):
        """Stop the WebSocket server."""
        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            self._broadcast_loop.call_soon_threadsafe(self._broadcast_loop.stop)
        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=1.0)
        logger.info("WebSocket visualization module stopped")

    def _create_server(self):
        # Create SocketIO server
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

        async def serve_index(request):
            return HTMLResponse("<html><body>Use the extension.</body></html>")

        routes = [Route("/", serve_index)]
        starlette_app = Starlette(routes=routes)

        self.app = socketio.ASGIApp(self.sio, starlette_app)

        # Register SocketIO event handlers
        @self.sio.event
        async def connect(sid, environ):
            with self.state_lock:
                current_state = dict(self.vis_state)
            await self.sio.emit("full_state", current_state, room=sid)

        @self.sio.event
        async def click(sid, position):
            goal = PoseStamped(
                position=(position[0], position[1], 0),
                orientation=(0, 0, 0, 1),  # Default orientation
                frame_id="world",
            )
            self.click_goal.publish(goal)
            logger.info(f"Click goal published: ({goal.position.x:.2f}, {goal.position.y:.2f})")

        @self.sio.event
        async def start_explore(sid):
            logger.info("Starting exploration")
            self.explore_cmd.publish(Bool(data=True))

        @self.sio.event
        async def stop_explore(sid):
            logger.info("Stopping exploration")
            self.stop_explore_cmd.publish(Bool(data=True))

        @self.sio.event
        async def move_command(sid, data):
            # Publish Twist if transport is configured
            if self.movecmd and self.movecmd.transport:
                twist = Twist(
                    linear=Vector3(data["linear"]["x"], data["linear"]["y"], data["linear"]["z"]),
                    angular=Vector3(
                        data["angular"]["x"], data["angular"]["y"], data["angular"]["z"]
                    ),
                )
                self.movecmd.publish(twist)

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

    def _run_server(self):
        uvicorn.run(
            self.app,
            host="0.0.0.0",
            port=self.port,
            log_level="error",  # Reduce verbosity
        )

    def _on_robot_pose(self, msg: PoseStamped):
        pose_data = {"type": "vector", "c": [msg.position.x, msg.position.y, msg.position.z]}
        self.vis_state["robot_pose"] = pose_data
        self._emit("robot_pose", pose_data)

    def _on_path(self, msg: Path):
        points = [[pose.position.x, pose.position.y] for pose in msg.poses]
        path_data = {"type": "path", "points": points}
        self.vis_state["path"] = path_data
        self._emit("path", path_data)

    def _on_global_costmap(self, msg: OccupancyGrid):
        costmap_data = self._process_costmap(msg)
        self.vis_state["costmap"] = costmap_data
        self._emit("costmap", costmap_data)

    def _process_costmap(self, costmap: OccupancyGrid) -> Dict[str, Any]:
        """Convert OccupancyGrid to visualization format."""
        costmap = costmap.inflate(0.1).gradient(max_distance=1.0)

        # Convert grid data to base64 encoded string
        grid_bytes = costmap.grid.astype(np.float32).tobytes()
        grid_base64 = base64.b64encode(grid_bytes).decode("ascii")

        return {
            "type": "costmap",
            "grid": {
                "type": "grid",
                "shape": [costmap.height, costmap.width],
                "dtype": "f32",
                "compressed": False,
                "data": grid_base64,
            },
            "origin": {
                "type": "vector",
                "c": [costmap.origin.position.x, costmap.origin.position.y, 0],
            },
            "resolution": costmap.resolution,
            "origin_theta": 0,  # Assuming no rotation for now
        }

    def _emit(self, event: str, data: Any):
        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.sio.emit(event, data), self._broadcast_loop)
