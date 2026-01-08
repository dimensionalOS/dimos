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
FastAPI WebSocket server for Quest3 VR teleoperation.

Handles WebSocket connections and processes VR controller tracking data
using FastAPI for better performance and features.

Now includes HTTPS support and serves the standalone HTML VR client.
"""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
import subprocess
from typing import TYPE_CHECKING, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from dimos.teleop.quest3.control.tracking_processor import TrackingProcessor
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    import numpy as np
    from numpy.typing import NDArray

logger = setup_logger()


class ConnectionManager:
    """Manages active WebSocket connections for teleoperation."""

    def __init__(self):
        self.active_connections: list[WebSocket] = []
        self.data_callback: Callable | None = None
        self.command_callback: Callable | None = None
        self.connection_count = 0

    async def connect(self, websocket: WebSocket) -> None:
        """Accept and register a new WebSocket connection.

        Args:
            websocket: The WebSocket connection to register
        """
        await websocket.accept()
        self.active_connections.append(websocket)
        self.connection_count += 1
        logger.info(f"Client connected (total: {len(self.active_connections)})")

    def disconnect(self, websocket: WebSocket) -> None:
        """Remove a WebSocket connection from active connections.

        Args:
            websocket: The WebSocket connection to remove
        """
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(f"Client disconnected (remaining: {len(self.active_connections)})")

    def set_callback(self, callback: Callable) -> None:
        """Set callback function to send controller data to teleop module.

        Args:
            callback: Function that takes (left_pose, right_pose, left_gripper, right_gripper)
        """
        self.data_callback = callback
        logger.info("Controller data callback registered")

    def set_command_callback(self, callback: Callable) -> None:
        """Set callback function to handle teleop commands (start_teleop/stop_teleop).

        Args:
            callback: Async function that takes (command_type: str, websocket: WebSocket) -> dict
        """
        self.command_callback = callback
        logger.info("Command callback registered")

    async def broadcast(self, message: dict) -> None:
        """Broadcast a message to all connected clients.

        Args:
            message: Dictionary to send as JSON to all clients
        """
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                logger.error(f"Failed to broadcast to client: {e}")
                disconnected.append(connection)

        # Clean up disconnected clients
        for conn in disconnected:
            self.disconnect(conn)

    def get_connection_count(self) -> int:
        """Get the number of active connections.

        Returns:
            Number of active WebSocket connections
        """
        return len(self.active_connections)


class TeleopFastAPIServer:
    """FastAPI-based WebSocket server for Quest3 teleoperation."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8443,  # Changed default to 8443 for HTTPS
        use_https: bool = True,
    ):
        """Initialize the FastAPI server.

        Args:
            host: Host address to bind to (default: 0.0.0.0)
            port: Port to bind to (default: 8443 for HTTPS)
            use_https: Whether to use HTTPS (required for WebXR on Quest 3)
        """
        self.host = host
        self.port = port
        self.use_https = use_https
        self.app = FastAPI(title="Quest3 Teleoperation Server")
        self.manager = ConnectionManager()
        self.server: uvicorn.Server | None = None

        # SSL certificate paths
        self.cert_dir = Path(__file__).parent.parent / "certs"
        self.cert_file = self.cert_dir / "cert.pem"
        self.key_file = self.cert_dir / "key.pem"

        # Static files directory
        self.static_dir = Path(__file__).parent.parent / "static"

        # Enable CORS for Quest 3 browser
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # Allow all origins for VR headset
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # Register routes
        self._setup_routes()

        # Setup SSL certificates if needed
        if self.use_https:
            self._ensure_ssl_certificates()

    def _ensure_ssl_certificates(self) -> None:
        """Generate self-signed SSL certificates if they don't exist."""
        self.cert_dir.mkdir(parents=True, exist_ok=True)

        if self.cert_file.exists() and self.key_file.exists():
            logger.info("SSL certificates found")
            return

        logger.info("Generating self-signed SSL certificates...")
        try:
            subprocess.run(
                [
                    "openssl",
                    "req",
                    "-x509",
                    "-newkey",
                    "rsa:2048",
                    "-nodes",
                    "-sha256",
                    "-subj",
                    "/CN=localhost",
                    "-keyout",
                    str(self.key_file),
                    "-out",
                    str(self.cert_file),
                    "-days",
                    "365",
                ],
                check=True,
                capture_output=True,
            )
            logger.info("✓ SSL certificates generated successfully")
            logger.warning(
                "⚠ These are self-signed certificates. Quest 3 will show a security warning."
            )
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to generate SSL certificates: {e}")
            logger.error("Make sure openssl is installed: sudo apt-get install openssl")
            raise
        except FileNotFoundError:
            logger.error("openssl command not found. Please install: sudo apt-get install openssl")
            raise

    def _setup_routes(self) -> None:
        """Setup FastAPI routes and WebSocket endpoints."""

        @self.app.get("/")
        async def root():
            """Serve the VR client HTML page."""
            html_file = self.static_dir / "index.html"
            if html_file.exists():
                return FileResponse(html_file)
            else:
                return {
                    "service": "Quest3 Teleoperation Server",
                    "status": "running",
                    "active_connections": self.manager.get_connection_count(),
                    "error": "VR client HTML not found. Check static/index.html",
                }

        @self.app.get("/health")
        async def health():
            """Health check endpoint."""
            return {
                "status": "healthy",
                "connections": self.manager.get_connection_count(),
            }

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket endpoint for VR teleoperation tracking data.

            Args:
                websocket: The WebSocket connection from Quest 3 client
            """
            await self.manager.connect(websocket)
            processor = TrackingProcessor()
            message_count = 0

            try:
                # Wait for initial handshake message
                initial_data = await websocket.receive_json()
                role = initial_data.get("role")
                logger.info(f"Client handshake: {initial_data}")

                if role != "teleop":
                    await websocket.send_json(
                        {"type": "error", "error": f"Invalid role: {role}. Expected 'teleop'."}
                    )
                    await websocket.close()
                    return

                # Send acknowledgment
                await websocket.send_json({"type": "handshake_ack", "status": "connected"})

                # Main tracking loop
                while True:
                    # Receive message
                    message = await websocket.receive_json()
                    message_count += 1

                    # Check if this is a command message (start_teleop/stop_teleop)
                    message_type = message.get("type")
                    if message_type in ("start_teleop", "stop_teleop"):
                        logger.info(f"Received command: {message_type}")
                        # Route to command handler
                        if self.manager.command_callback is not None:
                            try:
                                response = await self.manager.command_callback(
                                    message_type, websocket
                                )
                                # Send response back to client
                                await websocket.send_json(response)
                            except Exception as e:
                                logger.error(
                                    f"Error handling command {message_type}: {e}", exc_info=True
                                )
                                await websocket.send_json(
                                    {"type": "error", "error": f"Command failed: {e!s}"}
                                )
                        else:
                            logger.warning(f"Command callback not set, ignoring {message_type}")
                            await websocket.send_json(
                                {"type": "error", "error": "Command handler not available"}
                            )
                        continue

                    # Otherwise, process as tracking data
                    result = processor.process_tracking_message(message)

                    if result is not None:
                        left_pose, right_pose, left_gripper, right_gripper = result

                        # Send to callback (teleop module)
                        if self.manager.data_callback is not None:
                            self.manager.data_callback(
                                left_pose, right_pose, left_gripper, right_gripper
                            )

                        # Log periodically
                        if message_count % 100 == 0:
                            logger.debug(f"Processed {message_count} tracking messages")

            except WebSocketDisconnect:
                logger.info("Client disconnected normally")
            except Exception as e:
                logger.error(f"Error in WebSocket handler: {e}", exc_info=True)
            finally:
                self.manager.disconnect(websocket)
                logger.info(f"Connection closed after {message_count} messages")

    def set_callback(self, callback: Callable) -> None:
        """Set callback function for controller data.

        Args:
            callback: Function that takes (left_pose, right_pose, left_gripper, right_gripper)
        """
        self.manager.set_callback(callback)

    def set_command_callback(self, callback: Callable) -> None:
        """Set callback function for teleop commands.

        Args:
            callback: Async function that takes (command_type: str, websocket: WebSocket) -> dict
        """
        self.manager.set_command_callback(callback)

    async def start_async(self) -> None:
        """Start the FastAPI server asynchronously."""
        config_kwargs = {
            "app": self.app,
            "host": self.host,
            "port": self.port,
            "log_level": "info",
            "access_log": False,  # Disable access logs for performance
        }

        # Add SSL if enabled
        if self.use_https:
            config_kwargs["ssl_keyfile"] = str(self.key_file)
            config_kwargs["ssl_certfile"] = str(self.cert_file)
            protocol = "https"
            ws_protocol = "wss"
        else:
            protocol = "http"
            ws_protocol = "ws"

        config = uvicorn.Config(**config_kwargs)
        self.server = uvicorn.Server(config)

        logger.info(f"FastAPI server starting on {protocol}://{self.host}:{self.port}")
        logger.info(f"VR Client: {protocol}://{self.host}:{self.port}/")
        logger.info(f"WebSocket: {ws_protocol}://{self.host}:{self.port}/ws")

        await self.server.serve()

    def run(self) -> None:
        """Run the FastAPI server (blocking).

        This is for standalone testing only.
        """
        run_kwargs = {
            "app": self.app,
            "host": self.host,
            "port": self.port,
            "log_level": "info",
        }

        if self.use_https:
            run_kwargs["ssl_keyfile"] = str(self.key_file)
            run_kwargs["ssl_certfile"] = str(self.cert_file)

        uvicorn.run(**run_kwargs)

    async def stop_async(self) -> None:
        """Stop the FastAPI server asynchronously."""
        if self.server:
            logger.info("Stopping FastAPI server...")
            self.server.should_exit = True
            await asyncio.sleep(0.1)  # Give time for graceful shutdown

    async def broadcast_robot_state(self, state: dict) -> None:
        """Broadcast robot state to all connected clients.

        Args:
            state: Dictionary containing robot state information
        """
        await self.manager.broadcast(state)


# Standalone testing
if __name__ == "__main__":
    import logging

    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    def test_callback(left_pose, right_pose, left_gripper, right_gripper):
        """Test callback for tracking data."""
        print(f"Left pose: {left_pose[:3, 3]}, gripper: {left_gripper}")
        print(f"Right pose: {right_pose[:3, 3]}, gripper: {right_gripper}")

    server = TeleopFastAPIServer()
    server.set_callback(test_callback)
    server.run()
