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
Quest3 Teleoperation Module

A dimos Module that runs the WebSocket signaling server for Quest3 VR teleoperation,
receives controller tracking data, and streams DELTA poses to TeleopRobotController.

This module inherits from BaseTeleopModule which handles:
- Calibration (capture initial controller poses)
- Delta computation (current - initial)
- Publishing delta poses to LCM
- Rerun visualization
- Standard RPC interface

This module implements Quest3-specific:
- FastAPI/WebSocket server for Quest3 VR headset
- X button handler for calibration trigger
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, Any

from dimos.core import rpc
from dimos.teleop.base import BaseTeleopConfig, BaseTeleopModule
from dimos.teleop.quest3.control.fastapi_server import TeleopFastAPIServer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from dimos.core.global_config import GlobalConfig

logger = setup_logger()


@dataclass
class Quest3TeleopConfig(BaseTeleopConfig):
    """Configuration for Quest3 Teleoperation Module."""

    # Quest3-specific WebSocket server settings
    signaling_host: str = "0.0.0.0"
    signaling_port: int = 8443  # HTTPS port for WebXR
    use_https: bool = True  # Enable HTTPS for WebXR (required by Quest 3)
    # Exposed from BaseTeleopConfig for blueprint configuration
    enable_left_arm: bool = True  # Enable left arm teleoperation
    enable_right_arm: bool = True  # Enable right arm teleoperation
    visualize_in_rerun: bool = True  # Visualize VR controller poses in Rerun
    safety_limits: bool = True  # Enable safety limits

    # Other inherited from BaseTeleopConfig (with defaults):
    # - position_scale: float = 1.0
    # - max_velocity: float = 0.5
    # - workspace_limits: dict[str, tuple[float, float]]


class Quest3TeleopModule(BaseTeleopModule):
    """Quest3 VR Teleoperation Module.

    Inherits calibration, delta computation, and visualization from BaseTeleopModule.
    Implements Quest3-specific WebSocket server and VR connection logic.

    ## Device-Specific Features:
    - FastAPI/WebSocket server for Quest3 VR headset communication
    - HTTPS support for WebXR (required by Quest 3)
    - X button handler for calibration trigger

    ## LCM Topics (inherited from base):
    - left_controller_delta: Out[PoseStamped] - Left controller delta pose
    - right_controller_delta: Out[PoseStamped] - Right controller delta pose
    - left_trigger: Out[Bool] - Left trigger button state
    - right_trigger: Out[Bool] - Right trigger button state

    ## RPC Methods (inherited from base):
    - start() -> None: Start the module and signaling server
    - stop() -> None: Stop the module and signaling server
    - calibrate_vr() -> dict: Calibrate VR (capture initial controller poses)
    - reset_calibration() -> dict: Reset calibration
    - is_vr_calibrated() -> bool: Check if VR is calibrated
    - get_status() -> dict: Get current teleoperation status
    """

    default_config = Quest3TeleopConfig
    config: Quest3TeleopConfig

    def __init__(
        self, global_config: GlobalConfig | None = None, *args: Any, **kwargs: Any
    ) -> None:
        # Remove global_config from kwargs to avoid passing it twice
        kwargs.pop("global_config", None)
        # Pass global_config as positional argument to match base class signature
        super().__init__(global_config, *args, **kwargs)

        # Quest3-specific: FastAPI WebSocket server
        self._fastapi_server: TeleopFastAPIServer | None = None
        self._server_thread: threading.Thread | None = None
        self._event_loop: asyncio.AbstractEventLoop | None = None

        logger.info("Quest3TeleopModule initialized")

    # =========================================================================
    # Module Lifecycle (Quest3-specific)
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the Quest3 teleoperation module and signaling server."""
        logger.info("Starting Quest3 Teleoperation Module...")

        # Call base class start (handles Rerun connection, etc.)
        super().start()

        # Start Quest3-specific signaling server
        self._start_signaling_server()

        protocol = "https" if self.config.use_https else "http"
        logger.info(
            f"Quest3 Teleoperation Module started on {protocol}://{self.config.signaling_host}:{self.config.signaling_port}"
        )
        logger.info(
            f"Open this URL on Quest 3: {protocol}://<your-ip>:{self.config.signaling_port}/"
        )
        logger.info("Press X button in VR to calibrate and start teleoperation")

    @rpc
    def stop(self) -> None:
        """Stop the Quest3 teleoperation module and signaling server."""
        logger.info("Stopping Quest3 Teleoperation Module...")

        # Stop signaling server
        self._stop_signaling_server()

        super().stop()

    def _start_signaling_server(self) -> None:
        """Start the FastAPI WebSocket server in a background thread."""

        def run_server() -> None:
            # Create new event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._event_loop = loop

            # Create FastAPI server instance
            self._fastapi_server = TeleopFastAPIServer(
                host=self.config.signaling_host,
                port=self.config.signaling_port,
                use_https=self.config.use_https,
            )

            # Register callbacks
            self._fastapi_server.set_callback(self._on_tracking_data)
            self._fastapi_server.set_command_callback(self._handle_x_button)
            logger.info("FastAPI server initialized with callbacks")

            try:
                # Run FastAPI server
                loop.run_until_complete(self._fastapi_server.start_async())
            except Exception as e:
                logger.error(f"FastAPI server error: {e}", exc_info=True)
            finally:
                loop.close()

        # Start server thread
        self._server_thread = threading.Thread(target=run_server, daemon=True, name="FastAPIServer")
        self._server_thread.start()
        logger.info("FastAPI server thread started")

    def _stop_signaling_server(self) -> None:
        """Stop the FastAPI WebSocket server."""
        if self._fastapi_server and self._event_loop:
            # Schedule stop on the event loop
            asyncio.run_coroutine_threadsafe(self._fastapi_server.stop_async(), self._event_loop)

        if self._event_loop and self._event_loop.is_running():
            # Stop the event loop
            self._event_loop.call_soon_threadsafe(self._event_loop.stop)

        # Wait for thread to finish
        if self._server_thread and self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)

        logger.info("FastAPI server stopped")

    # =========================================================================
    # Quest3-Specific: X Button Handler
    # =========================================================================

    async def _handle_x_button(
        self, command_type: str, websocket: Any
    ) -> dict[str, Any]:
        """Handle X button press from VR client.

        X button toggles calibration:
        - If not calibrated: calibrate VR
        - If calibrated: reset calibration (stop streaming)

        Args:
            command_type: 'start_teleop' or 'stop_teleop'
            websocket: WebSocket connection to send responses

        Returns:
            Response dictionary to send back to client
        """
        try:
            if command_type == "start_teleop":
                logger.info("X button pressed - calibrating VR...")
                logger.info(
                    f"Current state: left_pose={self._left_pose is not None}, right_pose={self._right_pose is not None}"
                )
                result = self.calibrate_vr()
                logger.info(f"Calibration result: {result}")

                if result.get("success"):
                    return {
                        "type": "teleop_started",
                        "message": result.get("message", "VR calibrated"),
                    }
                else:
                    return {
                        "type": "calibration_failed",
                        "error": result.get("error", "Calibration failed"),
                    }

            elif command_type == "stop_teleop":
                logger.info("X button pressed - stopping teleop...")
                result = self.reset_calibration()

                return {
                    "type": "teleop_stopped",
                    "message": result.get("message", "Teleop stopped"),
                }

        except Exception as e:
            logger.error(f"Error handling X button: {e}", exc_info=True)
            return {"type": "error", "error": f"Command failed: {e!s}"}

        return {"type": "error", "error": f"Unknown command: {command_type}"}

    # =========================================================================
    # Quest3-Specific: Controller Data Processing
    # =========================================================================

    def _on_tracking_data(
        self,
        left_pose: NDArray[np.float32],
        right_pose: NDArray[np.float32],
        left_gripper: float,
        right_gripper: float,
    ) -> None:
        """Receive tracking data from Quest3 VR.

        Called by the FastAPI server when new tracking data arrives from Quest3.
        Delegates to base class method which handles calibration, delta computation,
        and publishing.

        Args:
            left_pose: 4x4 transformation matrix for left controller
            right_pose: 4x4 transformation matrix for right controller
            left_gripper: Left gripper value (0.0-1.0)
            right_gripper: Right gripper value (0.0-1.0)
        """
        # Call base class method to handle everything
        self.update_controller_poses(left_pose, right_pose, left_gripper, right_gripper)


# Expose blueprint for declarative composition
quest3_teleop_module = Quest3TeleopModule.blueprint
