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
VR Teleoperation Module for Quest 3 controllers.

Receives VR controller tracking data via LCM from Deno bridge,
transforms from WebXR to robot frame, computes deltas, and publishes commands.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.core import In, rpc
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.teleop.devices.base_teleop_module import BaseTeleopConfig, BaseTeleopModule
from dimos.utils.logging_config import setup_logger
from dimos.utils.teleop_transforms import transform_delta, transform_vr_to_robot

if TYPE_CHECKING:
    from dimos_lcm.geometry_msgs import Transform as LCMTransform
    import numpy as np
    from numpy.typing import NDArray

    from dimos.msgs.std_msgs import Bool, Float32


logger = setup_logger()


@dataclass
class VRTeleopConfig(BaseTeleopConfig):
    """Configuration for VR Teleoperation Module."""

    output_types: list[type] = field(default_factory=lambda: [PoseStamped, PoseStamped])
    input_labels: list[str] = field(default_factory=lambda: ["left_vr", "right_vr"])
    control_loop_hz: float = 50.0


class VRTeleopModule(BaseTeleopModule):
    """VR Teleoperation Module for Quest 3 controllers.

    Subscribes to controller data from Deno bridge, transforms WebXR→robot frame,
    computes deltas from calibration point, and publishes commands.
    """

    default_config = VRTeleopConfig
    config: VRTeleopConfig

    # LCM inputs from Deno bridge
    vr_left_transform: In[LCMTransform] = None  # type: ignore[assignment]
    vr_right_transform: In[LCMTransform] = None  # type: ignore[assignment]
    vr_trigger_0: In[Float32] = None  # type: ignore[assignment]
    vr_trigger_1: In[Float32] = None  # type: ignore[assignment]
    teleop_enable: In[Bool] = None  # type: ignore[assignment]  # X button calibration toggle

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        self._lcm_lock = threading.Lock()
        self._lcm_controller_poses: dict[int, NDArray[np.float64] | None] = {
            i: None for i in self._active_indices
        }
        self._lcm_gripper_values: dict[int, float] = {i: 0.0 for i in self._active_indices}
        self._control_loop_thread: threading.Thread | None = None
        self._control_loop_running = False

    @rpc
    def start(self) -> None:
        """Start the VR teleoperation module."""
        super().start()

        left_index = self._active_indices[0] if len(self._active_indices) > 0 else 0
        right_index = self._active_indices[1] if len(self._active_indices) > 1 else 1

        logger.info(
            f"VR controller mapping: left→{self._get_label(left_index)}(idx {left_index}), "
            f"right→{self._get_label(right_index)}(idx {right_index})"
        )

        if self.vr_left_transform and self.vr_left_transform.transport:
            self.vr_left_transform.subscribe(
                lambda msg, idx=left_index: self._on_lcm_transform(idx, msg)  # type: ignore[misc]
            )

        if self.vr_right_transform and self.vr_right_transform.transport:
            self.vr_right_transform.subscribe(
                lambda msg, idx=right_index: self._on_lcm_transform(idx, msg)  # type: ignore[misc]
            )

        if self.vr_trigger_0 and self.vr_trigger_0.transport:
            self.vr_trigger_0.subscribe(
                lambda msg, idx=left_index: self._on_lcm_trigger(idx, msg)  # type: ignore[misc]
            )

        if self.vr_trigger_1 and self.vr_trigger_1.transport:
            self.vr_trigger_1.subscribe(
                lambda msg, idx=right_index: self._on_lcm_trigger(idx, msg)  # type: ignore[misc]
            )

        if self.teleop_enable and self.teleop_enable.transport:
            self.teleop_enable.subscribe(self._on_lcm_teleop_enable)

        logger.info("VR Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        """Stop the VR teleoperation module."""
        logger.info("Stopping VR Teleoperation Module...")
        self._stop_control_loop()
        super().stop()

    @rpc
    def start_teleop(self) -> dict[str, Any]:
        """Calibrate and start teleoperation (called via X button)."""
        logger.info("Starting teleop - calibrating VR...")
        result: dict[str, Any] = self.calibrate()
        if not result.get("success"):
            logger.error(f"Calibration failed: {result.get('error')}")
        return result

    @rpc
    def stop_teleop(self) -> dict[str, Any]:
        """Stop teleoperation and reset calibration."""
        logger.info("Stopping teleop - resetting calibration...")
        self._stop_control_loop()
        result: dict[str, Any] = self.reset_calibration()
        return result

    def _on_lcm_teleop_enable(self, msg: Bool) -> None:
        """Handle teleop enable/disable from X button."""
        logger.info(f"Received teleop_enable: {msg.data}")
        if bool(msg.data):
            self.start_teleop()
        else:
            self.stop_teleop()

    def _on_lcm_transform(self, index: int, transform: LCMTransform) -> None:
        """Handle controller transform, converting WebXR to robot frame."""
        self._start_control_loop()
        is_left = index == self._active_indices[0]
        transform_matrix = transform_vr_to_robot(transform, is_left_controller=is_left)
        with self._lcm_lock:
            self._lcm_controller_poses[index] = transform_matrix  # type: ignore[assignment]

    def _on_lcm_trigger(self, index: int, msg: Float32) -> None:
        """Handle trigger value for gripper control."""
        self._start_control_loop()
        with self._lcm_lock:
            self._lcm_gripper_values[index] = float(msg.data)

    def _start_control_loop(self) -> None:
        """Start the control loop thread if not running."""
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
        self._control_loop_running = False
        if self._control_loop_thread is not None:
            self._control_loop_thread.join(timeout=1.0)
            self._control_loop_thread = None
        logger.info("Control loop stopped")

    def _control_loop(self) -> None:
        """Main control loop: compute deltas and publish commands at fixed rate."""
        period = 1.0 / self.config.control_loop_hz

        while self._control_loop_running:
            loop_start = time.perf_counter()

            with self._lcm_lock:
                controller_poses = dict(self._lcm_controller_poses)
                controller_trigger_values = dict(self._lcm_gripper_values)

            deltas = self.compute_deltas(controller_poses, controller_trigger_values)

            for idx, i in enumerate(self._active_indices):
                delta = deltas.get(i)
                if delta is None:
                    continue

                output_type = self.config.output_types[idx]
                command, aux_command = transform_delta(
                    delta_pose=delta,
                    trigger_value=self._all_trigger_values.get(i, 0.0),
                    output_type=output_type,
                    initial_robot_pose=self._initial_robot_poses.get(i),
                    linear_scale=self.config.linear_scale,
                    angular_scale=self.config.angular_scale,
                    max_linear_velocity=self.config.max_linear_velocity,
                    max_angular_velocity=self.config.max_angular_velocity,
                    gripper_threshold=self.config.gripper_threshold,
                )
                self.publish_command(i, command, aux_command)

            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


vr_teleop_module = VRTeleopModule.blueprint
