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

from dataclasses import dataclass, field
import threading
import time
from typing import Any

from dimos_lcm.geometry_msgs import Transform as LCMTransform

from dimos.core import In, rpc
from dimos.msgs.geometry_msgs import ControllerPose, PoseStamped, Twist
from dimos.msgs.std_msgs import Bool, Float32
from dimos.teleop.devices.base_teleop_module import BaseTeleopConfig, BaseTeleopModule
from dimos.utils.logging_config import setup_logger
from dimos.utils.teleop_transforms import transform_vr_to_robot
from dimos.utils.transform_utils import matrix_to_pose

logger = setup_logger()


@dataclass
class VRTeleopConfig(BaseTeleopConfig):
    """Configuration for VR Teleoperation Module."""

    output_types: list[type] = field(default_factory=lambda: [PoseStamped, PoseStamped])
    input_labels: list[str] = field(default_factory=lambda: ["left_vr", "right_vr"])
    robot_pose_rpc_methods: list[str | None] = field(default_factory=lambda: [None, None])
    control_loop_hz: float = 50.0


class VRTeleopModule(BaseTeleopModule[VRTeleopConfig]):
    """VR Teleoperation Module for Quest 3 controllers.

    Subscribes to controller data from Deno bridge, transforms WebXR→robot frame,
    computes deltas from calibration point, and publishes commands.
    """

    default_config = VRTeleopConfig

    # LCM inputs from Deno bridge
    vr_left_transform: In[LCMTransform] = None  # type: ignore[assignment]
    vr_right_transform: In[LCMTransform] = None  # type: ignore[assignment]
    vr_trigger_0: In[Float32] = None  # type: ignore[assignment]
    vr_trigger_1: In[Float32] = None  # type: ignore[assignment]
    teleop_enable: In[Bool] = None  # type: ignore[assignment]  # X button calibration toggle

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        self._lcm_lock = threading.Lock()
        self._lcm_controller_poses: dict[int, ControllerPose | None] = {
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

        # Subscribe to LCM inputs
        subscriptions = [
            (self.vr_left_transform, lambda msg, idx=left_index: self._on_lcm_transform(idx, msg)),
            (
                self.vr_right_transform,
                lambda msg, idx=right_index: self._on_lcm_transform(idx, msg),
            ),
            (self.vr_trigger_0, lambda msg, idx=left_index: self._on_lcm_trigger(idx, msg)),
            (self.vr_trigger_1, lambda msg, idx=right_index: self._on_lcm_trigger(idx, msg)),
            (self.teleop_enable, self._on_lcm_teleop_enable),
        ]
        for stream, handler in subscriptions:
            if stream and stream.transport:
                stream.subscribe(handler)  # type: ignore[misc, arg-type]

        logger.info("VR Teleoperation Module started")

    @rpc
    def stop(self) -> None:
        """Stop the VR teleoperation module."""
        logger.info("Stopping VR Teleoperation Module...")
        self._stop_control_loop()
        super().stop()

    @rpc
    def start_teleop(self) -> None:
        """Calibrate and start teleoperation (called via X button)."""
        logger.info("Starting teleop - calibrating VR...")
        self.calibrate()

    @rpc
    def stop_teleop(self) -> None:
        """Stop teleoperation and reset calibration."""
        logger.info("Stopping teleop - resetting calibration...")
        self._stop_control_loop()
        self.reset_calibration()

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
        pose = ControllerPose.from_pose(matrix_to_pose(transform_matrix))
        with self._lcm_lock:
            self._lcm_controller_poses[index] = pose

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

                trigger_value = self._all_trigger_values.get(i, 0.0)
                trigger_bool = Bool(data=trigger_value > self.config.gripper_threshold)

                output_type = self.config.output_types[idx]
                if output_type == PoseStamped:
                    command = delta.to_pose_stamped(
                        initial_robot_pose=self._initial_robot_poses.get(i),
                    )
                elif output_type == Twist:
                    command = delta.to_twist(
                        linear_scale=self.config.linear_scale,
                        angular_scale=self.config.angular_scale,
                        max_linear=self.config.max_linear_velocity,
                        max_angular=self.config.max_angular_velocity,
                    )
                else:
                    continue

                self.publish_command(i, command, trigger_bool)

            elapsed = time.perf_counter() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


vr_teleop_module = VRTeleopModule.blueprint
