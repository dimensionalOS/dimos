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
VR Teleoperation Module

A dimos Module that runs the WebSocket signaling server for VR teleoperation,
receives controller tracking data, and streams DELTA poses to TeleopRobotController.

This module inherits from BaseTeleopModule which handles:
- Calibration (capture initial controller poses)
- Delta computation (current - initial)
- Publishing delta poses to LCM
- Rerun visualization
- Standard RPC interface

This module implements VR-specific:
- FastAPI/WebSocket server for VR headset
- X button handler for calibration trigger
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.core import Out, rpc
from dimos.teleop.devices.base import BaseTeleopConfig, BaseTeleopModule
from dimos.teleop.devices.vr_headset.control.fastapi_server import TeleopFastAPIServer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from dimos.core.global_config import GlobalConfig
    from dimos.msgs.geometry_msgs import PoseStamped
    from dimos.msgs.std_msgs import Float32

logger = setup_logger()


@dataclass
class VRTeleopConfig(BaseTeleopConfig):
    """Configuration for VR Teleoperation Module."""

    # VR-specific WebSocket server settings
    signaling_host: str = "0.0.0.0"
    signaling_port: int = 8443  # HTTPS port for WebXR
    use_https: bool = True  # Enable HTTPS for WebXR (required by Quest 3)

    # Control settings
    num_inputs: int = 2  # Number of inputs (controllers)
    enable_inputs: list[bool] = field(default_factory=lambda: [True, True])
    input_labels: list[str] = field(default_factory=lambda: ["left_vr", "right_vr"])

    # Visualization settings
    visualize_in_rerun: bool = True  # Visualize controller poses in Rerun
    log_input_data: bool = False  # Log input pose/gripper data periodically
    log_input_data_interval: int = 100  # Log every N publishes when enabled


class VRTeleopModule(BaseTeleopModule):
    """VR Teleoperation Module.

    ## LCM Topics (inherited from base):
    - controller_delta_0: Out[PoseStamped] - Controller 0 delta pose
    - trigger_value_0: Out[Float32] - Controller 0 trigger/gripper value (0.0-1.0)

    ## Additional LCM Topics:
    - controller_delta_1: Out[PoseStamped] - Controller 1 delta pose
    - trigger_value_1: Out[Float32] - Controller 1 trigger/gripper value (0.0-1.0)
    """

    default_config = VRTeleopConfig
    config: VRTeleopConfig

    # LCM output topics for VR (controller_0 is inherited from base)
    controller_delta_1: Out[PoseStamped] = None  # type: ignore[assignment]
    trigger_value_1: Out[Float32] = None  # type: ignore[assignment]

    def __init__(
        self, global_config: GlobalConfig | None = None, *args: Any, **kwargs: Any
    ) -> None:
        # Remove global_config from kwargs to avoid passing it twice
        kwargs.pop("global_config", None)
        # Pass global_config as positional argument to match base class signature
        super().__init__(global_config, *args, **kwargs)

        # VR-specific: FastAPI WebSocket server
        self._fastapi_server: TeleopFastAPIServer | None = None
        self._server_thread: threading.Thread | None = None
        self._event_loop: asyncio.AbstractEventLoop | None = None

        logger.info("VRTeleopModule initialized")

    # =========================================================================
    # Module Lifecycle (VR-specific)
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the VR teleoperation module and signaling server."""
        super().start()

        # Start VR-specific signaling server
        self._start_signaling_server()

        protocol = "https" if self.config.use_https else "http"
        logger.info(
            f"VR Teleoperation Module started: Open this URL on Quest 3: {protocol}://<your-ip>:{self.config.signaling_port}/"
        )

    @rpc
    def stop(self) -> None:
        """Stop the VR teleoperation module and signaling server."""
        logger.info("Stopping VR Teleoperation Module...")

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
    # VR-Specific: X Button Handler
    # =========================================================================

    async def _handle_x_button(self, command_type: str, websocket: Any) -> dict[str, Any]:
        """Handle X button press from VR client.
        X button toggles calibration and Starts/stops teleoperation
        """
        try:
            if command_type == "start_teleop":
                logger.info("X button pressed - calibrating VR...")
                result = self.calibrate()
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
    # Data Processing
    # =========================================================================

    def _on_tracking_data(
        self,
        controller_poses: list[NDArray[np.float32]],
        controller_gripper_values: list[float],
    ) -> None:
        """Receive tracking data from VR.

        Called by the FastAPI server when new tracking data arrives from VR.
        Delegates to base class method which handles calibration, delta computation,
        and publishing.

        Args:
            controller_poses: List of 4x4 transformation matrices for controllers
            controller_gripper_values: List of gripper values (0.0-1.0)
        """
        # Call base class method to handle everything
        self.update_controller_poses(controller_poses, controller_gripper_values)


# Expose blueprint for declarative composition
vr_teleop_module = VRTeleopModule.blueprint
