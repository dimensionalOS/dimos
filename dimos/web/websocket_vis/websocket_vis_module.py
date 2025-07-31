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
import concurrent.futures
import os
import threading
from typing import Any, Dict, Optional

import socketio
import uvicorn
from starlette.applications import Starlette
from starlette.responses import HTMLResponse
from starlette.routing import Route
from starlette.staticfiles import StaticFiles

from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped
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
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

        # Visualization state
        self.vis_state = {
            "draw": {},  # Client expects visualization data under 'draw' key
            "connected_clients": 0,
            "status": "running",
        }
        self.state_lock = threading.Lock()

        logger.info(f"WebSocket visualization module initialized on port {port}")

    @rpc
    def start(self):
        """Start the WebSocket server and subscribe to inputs."""
        # Create the server
        self._create_server()

        # Start the server in a background thread
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()

        # Subscribe to inputs
        self.robot_pose.subscribe(self._on_robot_pose)
        self.path.subscribe(self._on_path)
        self.global_costmap.subscribe(self._on_global_costmap)

        logger.info(f"WebSocket server started on http://localhost:{self.port}")

    @rpc
    def stop(self):
        """Stop the WebSocket server."""
        if self._executor:
            self._executor.shutdown(wait=True)
        logger.info("WebSocket visualization module stopped")

    def _create_server(self):
        """Create the SocketIO server and Starlette app."""
        # Create SocketIO server
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

        # Create Starlette app
        async def serve_index(request):
            index_path = os.path.join(os.path.dirname(__file__), "static", "index.html")
            with open(index_path, "r") as f:
                content = f.read()
            return HTMLResponse(content)

        routes = [Route("/", serve_index)]
        starlette_app = Starlette(routes=routes)

        # Mount static files
        static_dir = os.path.join(os.path.dirname(__file__), "static")
        starlette_app.mount("/", StaticFiles(directory=static_dir), name="static")

        # Create ASGI app
        self.app = socketio.ASGIApp(self.sio, starlette_app)

        # Register SocketIO event handlers
        @self.sio.event
        async def connect(sid, environ):
            logger.info(f"Client connected: {sid}")
            with self.state_lock:
                self.vis_state["connected_clients"] += 1
                current_state = dict(self.vis_state)
            # Send current state to new client
            await self.sio.emit("full_state", current_state, room=sid)

        @self.sio.event
        async def disconnect(sid):
            logger.info(f"Client disconnected: {sid}")
            with self.state_lock:
                self.vis_state["connected_clients"] -= 1

        @self.sio.event
        async def message(sid, data):
            """Handle messages from the client."""
            msg_type = data.get("type")

            if msg_type == "click":
                # Convert click to navigation goal
                position = data.get("position", [])
                if isinstance(position, list) and len(position) >= 2:
                    goal = PoseStamped(
                        position=(position[0], position[1], 0),
                        orientation=(0, 0, 0, 1),  # Default orientation
                        frame_id="world",
                    )
                    self.click_goal.publish(goal)
                    logger.info(
                        f"Click goal published: ({goal.position.x:.2f}, {goal.position.y:.2f})"
                    )

    def _run_server(self):
        """Run the uvicorn server."""
        uvicorn.run(
            self.app,
            host="0.0.0.0",
            port=self.port,
            log_level="error",  # Reduce verbosity
        )

    def _on_robot_pose(self, msg: PoseStamped):
        """Handle robot pose updates."""
        pose_data = {"type": "vector", "c": [msg.position.x, msg.position.y, msg.position.z]}
        self._update_state({"draw": {"robot_pos": pose_data}})

    def _on_path(self, msg: Path):
        """Handle path updates."""
        points = []
        for pose in msg.poses:
            points.append([pose.position.x, pose.position.y])
        path_data = {"type": "path", "points": points}
        self._update_state({"draw": {"path": path_data}})

    def _on_global_costmap(self, msg: OccupancyGrid):
        """Handle global costmap updates."""
        costmap_data = self._process_costmap(msg)
        self._update_state({"draw": {"costmap": costmap_data}})

    def _process_costmap(self, costmap: OccupancyGrid) -> Dict[str, Any]:
        """Convert OccupancyGrid to visualization format."""
        import base64
        import numpy as np

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

    def _update_state(self, new_data: Dict[str, Any]):
        """Update visualization state and broadcast to clients."""
        with self.state_lock:
            # If updating draw data, merge it properly
            if "draw" in new_data:
                if "draw" not in self.vis_state:
                    self.vis_state["draw"] = {}
                self.vis_state["draw"].update(new_data["draw"])
            else:
                self.vis_state.update(new_data)

        # Broadcast update asynchronously
        def broadcast():
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.sio.emit("state_update", new_data))
            except Exception as e:
                logger.error(f"Failed to broadcast state update: {e}")

        self._executor.submit(broadcast)
