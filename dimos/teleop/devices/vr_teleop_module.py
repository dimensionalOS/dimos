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

from dataclasses import dataclass, field
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from scipy.spatial.transform import Rotation as R

from dimos.core import In, rpc
from dimos.teleop.devices.base_teleop_module import BaseTeleopConfig, BaseTeleopModule
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos_lcm.geometry_msgs import Transform as LCMTransform
    from numpy.typing import NDArray

    from dimos.core.global_config import GlobalConfig
    from dimos.msgs.std_msgs import Bool, Float32

logger = setup_logger()

# Coordinate frame transformation from VR (WebXR) to robot frame
# WebXR: X=right, Y=up, Z=back (towards user)
# Robot: X=forward, Y=left, Z=up
VR_TO_ROBOT_FRAME = np.array(
    [
        [0, 0, -1, 0],  # Robot X = -VR Z (forward)
        [-1, 0, 0, 0],  # Robot Y = -VR X (left)
        [0, 1, 0, 0],  # Robot Z = +VR Y (up)
        [0, 0, 0, 1],
    ],
    dtype=np.float64,
)


@dataclass
class VRTeleopConfig(BaseTeleopConfig):
    """Configuration for VR Teleoperation Module."""

    # Control settings
    num_inputs: int = 2  # Number of inputs (controllers)
    enable_inputs: list[bool] = field(default_factory=lambda: [True, True])
    input_labels: list[str] = field(default_factory=lambda: ["left_vr", "right_vr"])

    # Visualization settings
    visualize_in_rerun: bool = True  # Visualize controller poses in Rerun
    log_input_data: bool = False  # Log input pose/gripper data periodically
    log_input_data_interval: int = 100  # Log every N publishes when enabled

    # Injectable connectors (one per input, None for raw delta output)
    # Use ArmConnector, QuadrupedConnector, etc. from dimos.teleop.connectors
    connectors: list[Any] = field(default_factory=list)

    # Control loop frequency
    control_loop_hz: float = 50.0  # Hz


class VRTeleopModule(BaseTeleopModule):
    """VR Teleoperation Module"""

    default_config = VRTeleopConfig
    config: VRTeleopConfig

    # LCM input topics for raw Transform from web clients (uses dimos_lcm type for correct msg_name)
    vr_left_transform: In[LCMTransform] = None  # type: ignore[assignment]
    vr_right_transform: In[LCMTransform] = None  # type: ignore[assignment]
    vr_trigger_0: In[Float32] = None  # type: ignore[assignment]
    vr_trigger_1: In[Float32] = None  # type: ignore[assignment]
    teleop_enable: In[Bool] = None  # type: ignore[assignment]

    # RPC dependencies (dynamically set based on connector configs)
    rpc_calls: list[str] = []

    def __init__(
        self, global_config: GlobalConfig | None = None, *args: Any, **kwargs: Any
    ) -> None:
        # Remove global_config from kwargs to avoid passing it twice
        kwargs.pop("global_config", None)
        # Pass global_config as positional argument to match base class signature
        super().__init__(global_config, *args, **kwargs)

        self._lcm_lock = threading.Lock()
        self._lcm_controller_poses: list[NDArray[np.float32] | None] = [
            None
        ] * self.config.num_inputs
        self._lcm_gripper_values: list[float] = [0.0] * self.config.num_inputs

        # Initialize connectors list (pad to num_inputs)
        self._connectors: list[Any] = list(self.config.connectors)
        while len(self._connectors) < self.config.num_inputs:
            self._connectors.append(None)

        # Set RPC calls dynamically based on connector configs
        self.rpc_calls = []
        for connector in self._connectors:
            if connector is not None and hasattr(connector, "config"):
                if not getattr(connector.config, "dummy_driver", False):
                    driver_name = getattr(connector.config, "driver_module_name", "")
                    method_name = getattr(connector.config, "driver_method_name", "get_state")
                    if driver_name:
                        self.rpc_calls.append(f"{driver_name}.{method_name}")

        # Control loop
        self._control_loop_thread: threading.Thread | None = None
        self._control_loop_running = False

        logger.info("VRTeleopModule initialized")

    # =========================================================================
    # Module Lifecycle (VR-specific)
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the VR teleoperation module and signaling server."""
        super().start()

        # Determine enabled indices from config
        enabled_indices = [i for i, enabled in enumerate(self.config.enable_inputs) if enabled]

        # Map left VR controller to first enabled index, right to second enabled index
        left_index = enabled_indices[0] if len(enabled_indices) > 0 else 0
        right_index = enabled_indices[1] if len(enabled_indices) > 1 else 1

        logger.info(
            f"VR controller mapping: left→index {left_index} {self.config.input_labels[left_index]}, right→index {right_index} {self.config.input_labels[right_index]}"
        )

        # Subscribe to LCM inputs with dynamically determined indices
        if self.vr_left_transform and self.vr_left_transform.transport:
            self.vr_left_transform.subscribe(
                lambda msg, idx=left_index: self._on_lcm_transform(idx, msg)
            )

        if self.vr_right_transform and self.vr_right_transform.transport:
            self.vr_right_transform.subscribe(
                lambda msg, idx=right_index: self._on_lcm_transform(idx, msg)
            )

        if self.vr_trigger_0 and self.vr_trigger_0.transport:
            self.vr_trigger_0.subscribe(lambda msg, idx=left_index: self._on_lcm_trigger(idx, msg))

        if self.vr_trigger_1 and self.vr_trigger_1.transport:
            self.vr_trigger_1.subscribe(lambda msg, idx=right_index: self._on_lcm_trigger(idx, msg))

        if self.teleop_enable and self.teleop_enable.transport:
            self.teleop_enable.subscribe(self._on_lcm_teleop_enable)

        logger.info("VR Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        """Stop the VR teleoperation module and signaling server."""
        logger.info("Stopping VR Teleoperation Module...")
        super().stop()

    @rpc
    def start_teleop(self) -> dict[str, Any]:
        """RPC helper to trigger calibration for LCM-only clients."""
        logger.info("RPC start_teleop called - calibrating VR...")

        # Initialize connectors with robot poses via RPC
        self._init_connector_poses()
        res = self.calibrate()
        return res

    @rpc
    def stop_teleop(self) -> dict[str, Any]:
        """RPC helper to stop teleop for LCM-only clients."""
        logger.info("RPC stop_teleop called - resetting calibration...")
        self._stop_control_loop()
        return self.reset_calibration()

    def _on_lcm_teleop_enable(self, msg: Bool) -> None:
        """Handle teleop enable/disable from LCM."""
        logger.info(f"Received teleop_enable: {msg.data}")
        if bool(msg.data):
            self.start_teleop()
        else:
            self.stop_teleop()

    def _init_connector_poses(self) -> None:
        """Get initial poses from robot drivers and set on connectors."""
        for connector in self._connectors:
            if connector is None:
                continue

            # Get state via RPC if connector has a real driver
            result = None
            if not connector.config.dummy_driver:
                rpc_method_name = (
                    f"{connector.config.driver_module_name}.{connector.config.driver_method_name}"
                )
                if rpc_method_name in self.rpc_calls:
                    try:
                        get_state = self.get_rpc_calls(rpc_method_name)
                        if get_state is not None:
                            result = get_state()
                    except Exception as e:
                        logger.error(f"Failed to get robot state via RPC: {e}")

            connector.set_initial_pos(result)

    def _on_lcm_transform(self, index: int, transform: LCMTransform) -> None:
        """Handle raw Transform input from LCM and forward to base processing.

        The transform comes directly from WebXR. We:
        1. Convert to 4x4 matrix
        2. Apply controller alignment rotation (in VR space)
        3. Apply VR→robot coordinate frame transformation

        WebXR coordinate system: X=right, Y=up, Z=back (towards user)
        Robot coordinate system: X=forward, Y=left, Z=up
        """
        # start control loop
        self._start_control_loop()

        # Convert LCM Transform to 4x4 matrix
        t = transform.translation
        q = transform.rotation
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        # Build VR-space matrix
        vr_matrix = np.eye(4, dtype=np.float64)
        vr_matrix[:3, :3] = rot_matrix
        vr_matrix[:3, 3] = [t.x, t.y, t.z]

        # Apply controller alignment rotation in VR space (from tracking_processor.py)
        # Right controller (index 1) rotates -90° around Z, left (index 0) rotates +90°
        # This aligns the controller orientation with gripper orientation
        direction = 1 if index == 0 else -1
        z_rotation = R.from_euler("z", 90 * direction, degrees=True).as_matrix()
        vr_matrix[:3, :3] = vr_matrix[:3, :3] @ z_rotation

        # Apply VR to robot frame transformation
        transform_matrix = VR_TO_ROBOT_FRAME @ vr_matrix

        with self._lcm_lock:
            if index < 0 or index >= self.config.num_inputs:
                logger.warning(
                    "Ignoring transform index %s (num_inputs=%s)", index, self.config.num_inputs
                )
                return
            self._lcm_controller_poses[index] = transform_matrix

    def _on_lcm_trigger(self, index: int, msg: Float32) -> None:
        """Handle trigger/gripper value from LCM."""

        # start control loop
        self._start_control_loop()

        with self._lcm_lock:
            if index < 0 or index >= self.config.num_inputs:
                logger.warning(
                    "Ignoring trigger index %s (num_inputs=%s)", index, self.config.num_inputs
                )
                return
            self._lcm_gripper_values[index] = float(msg.data)

    # =========================================================================
    # Control Loop
    # =========================================================================

    def _start_control_loop(self) -> None:
        """Start the control loop thread."""
        if self._control_loop_running:
            return

        self._control_loop_running = True
        self._control_loop_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="VRTeleopControlLoop",
        )
        self._control_loop_thread.start()
        logger.info(f"Control loop started at {self.config.control_loop_hz} Hz")

    def _stop_control_loop(self) -> None:
        """Stop the control loop thread."""
        self._control_loop_running = False
        if self._control_loop_thread is not None:
            self._control_loop_thread.join(timeout=1.0)
            self._control_loop_thread = None
        logger.info("Control loop stopped")

    def _control_loop(self) -> None:
        """Main control loop running at fixed frequency."""
        period = 1.0 / self.config.control_loop_hz

        while self._control_loop_running:
            loop_start = time.perf_counter()

            # Get current poses and trigger values
            with self._lcm_lock:
                controller_poses = list(self._lcm_controller_poses)
                controller_trigger_values = list(self._lcm_gripper_values)

            # Compute deltas
            deltas = self.compute_deltas(controller_poses, controller_trigger_values)

            # Route each delta through its connector
            for i in range(self.config.num_inputs):
                delta = deltas[i] if i < len(deltas) else None
                if delta is None:
                    continue

                connector = self._connectors[i] if i < len(self._connectors) else None
                if connector is None:
                    continue

                # Transform through connector and publish
                command, aux_command = connector.transform_delta(delta, self._all_trigger_values[i])
                self.publish_command(i, command, aux_command)

            # Sleep for remaining time
            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


# Expose blueprint for declarative composition
vr_teleop_module = VRTeleopModule.blueprint
