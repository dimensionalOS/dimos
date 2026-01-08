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
receives controller tracking data, calibrates VR poses, and streams DELTA poses to
TeleopArmController.

Architecture:
- X button pressed → calibrate VR (capture initial controller poses)
- Computes: delta = current_controller - initial_controller
- Publishes: delta poses (Pose) to TeleopArmController
- TeleopArmController auto-calibrates robot on first delta received
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.core import Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Vector3
from dimos.msgs.std_msgs import Bool
from dimos.teleop.quest3.control.fastapi_server import TeleopFastAPIServer
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray
    from websockets.server import WebSocketServerProtocol

logger = setup_logger()


@dataclass
class Quest3TeleopConfig(ModuleConfig):
    """Configuration for Quest3 Teleoperation Module."""

    # WebSocket server settings
    signaling_host: str = "0.0.0.0"
    signaling_port: int = 8443  # HTTPS port (was 8013 for old setup)
    use_https: bool = True  # Enable HTTPS for WebXR (required by Quest 3)

    # Driver module settings
    driver_module_name: str = "XArmDriver"  # Can be "XArmDriver", "PiperDriver", etc.

    # Control settings
    position_scale: float = 1.0  # Scale factor for positions
    enable_left_arm: bool = True
    enable_right_arm: bool = False

    # Safety limits
    max_velocity: float = 0.5  # m/s
    workspace_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "x": (0.1, 1),
            "y": (-0.8, 0.8),
            "z": (0.1, 0.7),
        }
    )


class Quest3TeleopModule(Module):
    """Quest3 VR Teleoperation Module.

    This module:
    1. Runs a WebSocket signaling server for VR connections
    2. Receives controller tracking data from Quest3
    3. Calibrates VR poses when X button is pressed
    4. Computes and streams DELTA poses (current - initial) to TeleopArmController
    5. TeleopArmController receives deltas and auto-calibrates robot on first delta

    ## Output Topics:
    - left_controller_delta: PoseStamped - Left controller delta pose (position + orientation delta)
    - right_controller_delta: PoseStamped - Right controller delta pose
    - left_trigger: Bool - Left trigger button state
    - right_trigger: Bool - Right trigger button state

    ## RPC Methods:
    - start() -> None: Start the module and signaling server
    - stop() -> None: Stop the module and signaling server
    - calibrate_vr() -> dict: Calibrate VR (capture initial controller poses)
    - is_vr_calibrated() -> bool: Check if VR is calibrated
    - get_status() -> dict: Get current teleoperation status
    """

    default_config = Quest3TeleopConfig

    # Output topics - publishing DELTA poses as PoseStamped
    left_controller_delta: Out[PoseStamped] = None  # type: ignore[assignment]
    right_controller_delta: Out[PoseStamped] = None  # type: ignore[assignment]
    left_trigger: Out[Bool] = None  # type: ignore[assignment]
    right_trigger: Out[Bool] = None  # type: ignore[assignment]

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        # No RPC dependencies - data-driven activation
        self.rpc_calls = []

        # FastAPI WebSocket server
        self._fastapi_server: TeleopFastAPIServer | None = None
        self._server_thread: threading.Thread | None = None
        self._event_loop: asyncio.AbstractEventLoop | None = None

        # VR Calibration state
        self._vr_calibrated = False
        self._left_controller_initial: Pose | None = None
        self._right_controller_initial: Pose | None = None

        # Latest controller data (absolute poses from VR)
        self._left_pose: NDArray[np.float32] | None = None
        self._right_pose: NDArray[np.float32] | None = None
        self._left_trigger_pressed: bool = False
        self._right_trigger_pressed: bool = False

        # Connection state
        self._connected_clients = 0

        # Rate limiting: streaming frequency
        self._last_stream_time: float = 0.0
        self._stream_frequency = 20  # Hz - stream controller data at 20Hz
        self._stream_period = 1.0 / self._stream_frequency

        logger.info("Quest3TeleopModule initialized")

    # =========================================================================
    # Module Lifecycle
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the Quest3 teleoperation module and signaling server."""
        logger.info("🚀 Starting Quest3TeleopModule...")
        super().start()

        # Start signaling server in background thread
        self._start_signaling_server()

        protocol = "https" if self.config.use_https else "http"
        logger.info(
            f"✅ Quest3 Teleoperation Module started on {protocol}://{self.config.signaling_host}:{self.config.signaling_port}"
        )
        logger.info(
            f"📱 Open this URL on Quest 3: {protocol}://<your-ip>:{self.config.signaling_port}/"
        )
        logger.info("⏸️ Press X button in VR to calibrate and start teleoperation")

    @rpc
    def stop(self) -> None:
        """Stop the Quest3 teleoperation module and signaling server."""
        logger.info("Stopping Quest3TeleopModule")

        # Stop signaling server
        self._stop_signaling_server()

        super().stop()

    def _start_signaling_server(self) -> None:
        """Start the FastAPI WebSocket server in a background thread."""

        def run_server():
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
    # VR Calibration
    # =========================================================================

    @rpc
    def calibrate_vr(self) -> dict[str, Any]:
        """Calibrate VR by capturing initial controller poses.

        This is called when X button is pressed in VR.
        Captures the current controller poses as the "zero" reference.
        After calibration, delta poses are published.

        Returns:
            Dict with 'success' and optional 'message' or 'error'
        """
        logger.info("📐 Calibrating VR controllers...")

        try:
            # Check if we have controller data
            if self._left_pose is None and self._right_pose is None:
                return {
                    "success": False,
                    "error": "No controller data received yet. Move controllers and try again.",
                }

            # Capture left controller initial pose
            if self.config.enable_left_arm and self._left_pose is not None:
                left_pose_obj = matrix_to_pose(self._left_pose)

                # Check if pose is valid (not all zeros)
                pose_magnitude = (
                    left_pose_obj.x**2 + left_pose_obj.y**2 + left_pose_obj.z**2
                ) ** 0.5
                if pose_magnitude < 0.001:
                    return {
                        "success": False,
                        "error": "Left controller pose is invalid (all zeros)",
                    }

                self._left_controller_initial = left_pose_obj
                logger.info(
                    f"✅ Captured left controller initial: "
                    f"pos=[{left_pose_obj.x:.3f}, {left_pose_obj.y:.3f}, {left_pose_obj.z:.3f}], "
                    f"rpy=[{left_pose_obj.roll:.3f}, {left_pose_obj.pitch:.3f}, {left_pose_obj.yaw:.3f}]"
                )

            # Capture right controller initial pose
            if self.config.enable_right_arm and self._right_pose is not None:
                right_pose_obj = matrix_to_pose(self._right_pose)

                # Check if pose is valid
                pose_magnitude = (
                    right_pose_obj.x**2 + right_pose_obj.y**2 + right_pose_obj.z**2
                ) ** 0.5
                if pose_magnitude < 0.001:
                    return {
                        "success": False,
                        "error": "Right controller pose is invalid (all zeros)",
                    }

                self._right_controller_initial = right_pose_obj
                logger.info(
                    f"✅ Captured right controller initial: "
                    f"pos=[{right_pose_obj.x:.3f}, {right_pose_obj.y:.3f}, {right_pose_obj.z:.3f}], "
                    f"rpy=[{right_pose_obj.roll:.3f}, {right_pose_obj.pitch:.3f}, {right_pose_obj.yaw:.3f}]"
                )

            self._vr_calibrated = True
            self._last_stream_time = 0.0  # Reset to start streaming immediately

            logger.info("✅ VR calibration complete - now streaming delta poses")
            return {"success": True, "message": "VR calibrated - move controllers to control robot"}

        except Exception as e:
            logger.error(f"VR calibration failed: {e}", exc_info=True)
            return {"success": False, "error": str(e)}

    @rpc
    def reset_calibration(self) -> dict[str, Any]:
        """Reset VR calibration. Stops streaming until recalibrated.

        Returns:
            Dict with 'success' and 'message'
        """
        self._vr_calibrated = False
        self._left_controller_initial = None
        self._right_controller_initial = None

        logger.info("⏸️ VR calibration reset - press X to recalibrate")
        return {"success": True, "message": "Calibration reset - press X to recalibrate"}

    @rpc
    def is_vr_calibrated(self) -> bool:
        """Check if VR is calibrated.

        Returns:
            True if VR is calibrated and streaming deltas
        """
        return self._vr_calibrated

    @rpc
    def get_status(self) -> dict:
        """Get current teleoperation status.

        Returns:
            Dictionary with status information
        """
        return {
            "vr_calibrated": self._vr_calibrated,
            "connected_clients": self._connected_clients,
            "server_running": self._server_thread is not None and self._server_thread.is_alive(),
            "left_arm_enabled": self.config.enable_left_arm,
            "right_arm_enabled": self.config.enable_right_arm,
            "has_left_data": self._left_pose is not None,
            "has_right_data": self._right_pose is not None,
            "left_trigger_pressed": self._left_trigger_pressed,
            "right_trigger_pressed": self._right_trigger_pressed,
        }

    # =========================================================================
    # X Button Handler
    # =========================================================================

    async def _handle_x_button(self, command_type: str, websocket) -> dict:
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
                logger.info("🎮 X button pressed - calibrating VR...")
                result = self.calibrate_vr()

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
                logger.info("⏸️ X button pressed - stopping teleop...")
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
    # Controller Data Processing
    # =========================================================================

    def _on_tracking_data(
        self,
        left_pose: NDArray[np.float32],
        right_pose: NDArray[np.float32],
        left_gripper: float,
        right_gripper: float,
    ) -> None:
        """Receive tracking data from VR.

        Called by the signaling server when new tracking data arrives.
        Stores absolute poses and computes/streams deltas if calibrated.

        Args:
            left_pose: 4x4 transformation matrix for left controller
            right_pose: 4x4 transformation matrix for right controller
            left_gripper: Left gripper value (0.0-1.0)
            right_gripper: Right gripper value (0.0-1.0)
        """
        # Store absolute poses
        self._left_pose = left_pose
        self._right_pose = right_pose

        # Convert gripper values to trigger booleans (threshold at 0.5)
        self._left_trigger_pressed = left_gripper > 0.5
        self._right_trigger_pressed = right_gripper > 0.5

        # Only stream deltas if VR is calibrated
        if not self._vr_calibrated:
            return

        # Rate limit streaming
        current_time = time.time()
        time_since_last_stream = current_time - self._last_stream_time
        if time_since_last_stream < self._stream_period:
            return

        self._last_stream_time = current_time
        self._stream_delta_poses(left_pose, right_pose)

    def _stream_delta_poses(
        self,
        left_pose: NDArray[np.float32],
        right_pose: NDArray[np.float32],
    ) -> None:
        """Compute and stream delta poses (current - initial).

        Args:
            left_pose: 4x4 transformation matrix for left controller (absolute)
            right_pose: 4x4 transformation matrix for right controller (absolute)
        """
        # Track publish count for logging
        if not hasattr(self, "_publish_count"):
            self._publish_count = 0
        self._publish_count += 1

        try:
            current_time = time.time()

            # Left controller delta
            if self.config.enable_left_arm and self._left_controller_initial is not None:
                if self.left_controller_delta and hasattr(self.left_controller_delta, "publish"):
                    left_pose_obj = matrix_to_pose(left_pose)
                    delta_pose_stamped = self._compute_delta(
                        left_pose_obj,
                        self._left_controller_initial,
                        current_time,
                        "quest3_left_controller_delta",
                    )

                    try:
                        self.left_controller_delta.publish(delta_pose_stamped)

                        # Log periodically
                        if self._publish_count <= 5 or self._publish_count % 100 == 0:
                            logger.info(
                                f"📤 Published left delta #{self._publish_count}: "
                                f"pos=[{delta_pose_stamped.position.x:.3f}, {delta_pose_stamped.position.y:.3f}, {delta_pose_stamped.position.z:.3f}], "
                                f"rpy=[{delta_pose_stamped.roll:.3f}, {delta_pose_stamped.pitch:.3f}, {delta_pose_stamped.yaw:.3f}], "
                                f"frame_id={delta_pose_stamped.frame_id}"
                            )
                    except Exception as e:
                        logger.error(f"Failed to publish left delta: {e}")

            # Right controller delta
            if self.config.enable_right_arm and self._right_controller_initial is not None:
                if self.right_controller_delta and hasattr(self.right_controller_delta, "publish"):
                    right_pose_obj = matrix_to_pose(right_pose)
                    delta_pose_stamped = self._compute_delta(
                        right_pose_obj,
                        self._right_controller_initial,
                        current_time,
                        "quest3_right_controller_delta",
                    )

                    try:
                        self.right_controller_delta.publish(delta_pose_stamped)

                        if self._publish_count <= 5 or self._publish_count % 100 == 0:
                            logger.info(
                                f"📤 Published right delta #{self._publish_count}: "
                                f"pos=[{delta_pose_stamped.position.x:.3f}, {delta_pose_stamped.position.y:.3f}, {delta_pose_stamped.position.z:.3f}], "
                                f"frame_id={delta_pose_stamped.frame_id}"
                            )
                    except Exception as e:
                        logger.error(f"Failed to publish right delta: {e}")

            # Publish trigger states
            if self.left_trigger and hasattr(self.left_trigger, "publish"):
                try:
                    self.left_trigger.publish(Bool(data=self._left_trigger_pressed))
                except Exception as e:
                    logger.debug(f"Failed to publish left trigger: {e}")

            if self.right_trigger and hasattr(self.right_trigger, "publish"):
                try:
                    self.right_trigger.publish(Bool(data=self._right_trigger_pressed))
                except Exception as e:
                    logger.debug(f"Failed to publish right trigger: {e}")

        except Exception as e:
            logger.error(f"Error streaming delta poses: {e}")

    def _compute_delta(
        self, current: Pose, initial: Pose, timestamp: float, frame_id: str
    ) -> PoseStamped:
        """Compute delta pose: current - initial.

        For position: simple subtraction
        For orientation: delta_quat = current * inverse(initial)

        Args:
            current: Current controller pose
            initial: Initial controller pose (reference)
            timestamp: Timestamp for the delta pose
            frame_id: Frame ID for the delta pose

        Returns:
            Delta pose as PoseStamped (position delta + orientation delta)
        """
        # Position delta
        delta_x = current.x - initial.x
        delta_y = current.y - initial.y
        delta_z = current.z - initial.z

        # Orientation delta: delta_quat = current * inverse(initial)
        delta_quat = current.orientation * initial.orientation.inverse()

        delta_pose = Pose(
            position=Vector3(delta_x, delta_y, delta_z),
            orientation=delta_quat,
        )

        return PoseStamped(
            ts=timestamp,
            frame_id=frame_id,
            position=delta_pose.position,
            orientation=delta_pose.orientation,
        )


# Expose blueprint for declarative composition
quest3_teleop_module = Quest3TeleopModule.blueprint
