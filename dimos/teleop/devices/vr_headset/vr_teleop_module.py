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
from typing import TYPE_CHECKING, Any

import numpy as np
from dimos_lcm.geometry_msgs import Transform as LCMTransform
from scipy.spatial.transform import Rotation as R

from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.std_msgs import Bool, Float32
from dimos.teleop.devices.base import BaseTeleopConfig, BaseTeleopModule
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from dimos.core.global_config import GlobalConfig

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
    # LCM input topics for raw Transform from web clients (uses dimos_lcm type for correct msg_name)
    vr_left_transform: In[LCMTransform] = None  # type: ignore[assignment]
    vr_right_transform: In[LCMTransform] = None  # type: ignore[assignment]
    vr_trigger_0: In[Float32] = None  # type: ignore[assignment]
    vr_trigger_1: In[Float32] = None  # type: ignore[assignment]
    teleop_enable: In[Bool] = None  # type: ignore[assignment]

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

        logger.info("VRTeleopModule initialized")

    # =========================================================================
    # Module Lifecycle (VR-specific)
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the VR teleoperation module and signaling server."""
        super().start()

        if (
            self.vr_left_transform
            and self.vr_left_transform.transport
            and hasattr(self.vr_left_transform, "subscribe")
        ):
            self.vr_left_transform.subscribe(lambda msg: self._on_lcm_transform(0, msg))
        else:
            logger.debug("vr_left_transform not wired; skipping LCM transform subscription")

        if (
            self.vr_right_transform
            and self.vr_right_transform.transport
            and hasattr(self.vr_right_transform, "subscribe")
        ):
            self.vr_right_transform.subscribe(lambda msg: self._on_lcm_transform(1, msg))
        else:
            logger.debug("vr_right_transform not wired; skipping LCM transform subscription")

        if (
            self.vr_trigger_0
            and self.vr_trigger_0.transport
            and hasattr(self.vr_trigger_0, "subscribe")
        ):
            self.vr_trigger_0.subscribe(lambda msg: self._on_lcm_trigger(0, msg))
        else:
            logger.debug("vr_trigger_0 not wired; skipping LCM trigger subscription")

        if (
            self.vr_trigger_1
            and self.vr_trigger_1.transport
            and hasattr(self.vr_trigger_1, "subscribe")
        ):
            self.vr_trigger_1.subscribe(lambda msg: self._on_lcm_trigger(1, msg))
        else:
            logger.debug("vr_trigger_1 not wired; skipping LCM trigger subscription")

        if (
            self.teleop_enable
            and self.teleop_enable.transport
            and hasattr(self.teleop_enable, "subscribe")
        ):
            self.teleop_enable.subscribe(self._on_lcm_teleop_enable)
            logger.info("Subscribed to teleop_enable LCM topic")
        else:
            logger.warning("teleop_enable not wired; skipping LCM teleop subscription")

        logger.info("VR Teleoperation Module started (LCM-only web client mode)")

    @rpc
    def stop(self) -> None:
        """Stop the VR teleoperation module and signaling server."""
        logger.info("Stopping VR Teleoperation Module...")

        super().stop()

    @rpc
    def start_teleop(self) -> dict[str, Any]:
        """RPC helper to trigger calibration for LCM-only clients."""
        logger.info("RPC start_teleop called - calibrating VR...")
        return self.calibrate()

    @rpc
    def stop_teleop(self) -> dict[str, Any]:
        """RPC helper to stop teleop for LCM-only clients."""
        logger.info("RPC stop_teleop called - resetting calibration...")
        return self.reset_calibration()

    def _on_lcm_transform(self, index: int, transform: LCMTransform) -> None:
        """Handle raw Transform input from LCM and forward to base processing.

        The transform comes directly from WebXR. We:
        1. Convert to 4x4 matrix
        2. Apply controller alignment rotation (in VR space)
        3. Apply VR→robot coordinate frame transformation

        WebXR coordinate system: X=right, Y=up, Z=back (towards user)
        Robot coordinate system: X=forward, Y=left, Z=up
        """
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
            controller_poses = list(self._lcm_controller_poses)
            controller_gripper_values = list(self._lcm_gripper_values)

        self.update_controller_poses(controller_poses, controller_gripper_values)

    def _on_lcm_trigger(self, index: int, msg: Float32) -> None:
        """Handle trigger/gripper value from LCM."""
        with self._lcm_lock:
            if index < 0 or index >= self.config.num_inputs:
                logger.warning(
                    "Ignoring trigger index %s (num_inputs=%s)", index, self.config.num_inputs
                )
                return
            self._lcm_gripper_values[index] = float(msg.data)

    def _on_lcm_teleop_enable(self, msg: Bool) -> None:
        """Handle teleop enable/disable from LCM."""
        logger.info(f"Received teleop_enable: {msg.data}")
        if bool(msg.data):
            self.start_teleop()
        else:
            self.stop_teleop()


# Expose blueprint for declarative composition
vr_teleop_module = VRTeleopModule.blueprint
