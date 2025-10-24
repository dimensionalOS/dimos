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
Sample Trajectory Generator for xArm Manipulator.

This module demonstrates how to:
- Subscribe to joint_state and robot_state from the xArm driver
- Publish joint position or velocity commands
- Implement a simple control loop

Usage:
    cluster = core.start(1)

    # Deploy trajectory generator
    traj_gen = cluster.deploy(
        SampleTrajectoryGenerator,
        num_joints=6,
        control_mode="position",  # or "velocity"
        publish_rate=10.0,
    )

    # Set up transports
    traj_gen.joint_state_input.transport = core.LCMTransport("/xarm/joint_states", JointState)
    traj_gen.robot_state_input.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    traj_gen.joint_position_command.transport = core.LCMTransport("/xarm/joint_position_command", List[float])

    traj_gen.start()
"""

import time
import threading
import math
from typing import List, Optional
from dataclasses import dataclass

from dimos.core import Module, ModuleConfig, In, Out, rpc
from dimos.msgs.sensor_msgs import JointState, RobotState, JointCommand
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


@dataclass
class TrajectoryGeneratorConfig(ModuleConfig):
    """Configuration for trajectory generator."""

    num_joints: int = 6  # Number of joints (5, 6, or 7)
    control_mode: str = "position"  # "position" or "velocity"
    publish_rate: float = 10.0  # Command publishing rate in Hz
    enable_on_start: bool = False  # Start publishing commands immediately


class SampleTrajectoryGenerator(Module):
    """
    Sample trajectory generator for xArm manipulator.

    This module demonstrates command publishing and state monitoring.
    Currently sends zero commands, but can be extended for trajectory generation.

    Architecture:
    - Subscribes to joint_state and robot_state from xArm driver
    - Publishes either joint_position_command OR joint_velocity_command
    - Runs a control loop at publish_rate Hz
    """

    default_config = TrajectoryGeneratorConfig

    # Input topics (state feedback from robot)
    joint_state_input: In[JointState] = None  # Current joint state
    robot_state_input: In[RobotState] = None  # Current robot state

    # Output topics (commands to robot) - only use ONE at a time
    joint_position_command: Out[JointCommand] = None  # Position commands (radians)
    joint_velocity_command: Out[JointCommand] = None  # Velocity commands (rad/s)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # State tracking
        self._current_joint_state: Optional[JointState] = None
        self._current_robot_state: Optional[RobotState] = None
        self._state_lock = threading.Lock()

        # Control thread
        self._running = False
        self._stop_event = threading.Event()
        self._control_thread: Optional[threading.Thread] = None

        # Publishing enabled flag
        self._publishing_enabled = self.config.enable_on_start

        # Command publish counter (for logging)
        self._command_count = 0

        # Trajectory state
        self._trajectory_active = False
        self._trajectory_start_time = 0.0
        self._trajectory_duration = 0.0
        self._trajectory_start_positions = None
        self._trajectory_end_positions = None
        self._trajectory_is_velocity = False  # True for velocity trajectories

        logger.info(
            f"TrajectoryGenerator initialized: {self.config.num_joints} joints, "
            f"mode={self.config.control_mode}, rate={self.config.publish_rate}Hz"
        )

    @rpc
    def start(self):
        """Start the trajectory generator."""
        super().start()

        # Subscribe to state topics
        try:
            unsub_js = self.joint_state_input.subscribe(self._on_joint_state)
            self._disposables.add(lambda: unsub_js())
        except (AttributeError, ValueError) as e:
            logger.debug(f"joint_state_input transport not configured: {e}")

        try:
            unsub_rs = self.robot_state_input.subscribe(self._on_robot_state)
            self._disposables.add(lambda: unsub_rs())
        except (AttributeError, ValueError) as e:
            logger.debug(f"robot_state_input transport not configured: {e}")

        # Start control loop
        self._start_control_loop()

        logger.info("Trajectory generator started")

    @rpc
    def stop(self):
        """Stop the trajectory generator."""
        logger.info("Stopping trajectory generator...")

        # Stop control thread
        self._running = False
        self._stop_event.set()

        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)

        super().stop()
        logger.info("Trajectory generator stopped")

    @rpc
    def enable_publishing(self):
        """Enable command publishing."""
        self._publishing_enabled = True
        logger.info("Command publishing enabled")

    @rpc
    def disable_publishing(self):
        """Disable command publishing."""
        self._publishing_enabled = False
        logger.info("Command publishing disabled")

    @rpc
    def get_current_state(self) -> dict:
        """Get current joint and robot state."""
        with self._state_lock:
            return {
                "joint_state": self._current_joint_state,
                "robot_state": self._current_robot_state,
                "publishing_enabled": self._publishing_enabled,
                "trajectory_active": self._trajectory_active,
            }

    @rpc
    def move_joint(self, joint_index: int, delta_degrees: float, duration: float) -> str:
        """
        Move a single joint by a relative amount over a duration.

        Args:
            joint_index: Index of joint to move (0-based, so joint 6 = index 5)
            delta_degrees: Amount to rotate in degrees (positive = counterclockwise)
            duration: Time to complete motion in seconds

        Returns:
            Status message
        """
        with self._state_lock:
            if self._current_joint_state is None:
                return "Error: No joint state received yet"

            if self._trajectory_active:
                return "Error: Trajectory already in progress"

            if joint_index < 0 or joint_index >= self.config.num_joints:
                return f"Error: Invalid joint index {joint_index} (must be 0-{self.config.num_joints - 1})"

            # Convert delta to radians (Note: positive degrees = clockwise for rotation)
            delta_rad = math.radians(delta_degrees)

            # Set up trajectory
            self._trajectory_start_positions = list(self._current_joint_state.position)
            self._trajectory_end_positions = list(self._current_joint_state.position)
            self._trajectory_end_positions[joint_index] += delta_rad
            self._trajectory_duration = duration
            self._trajectory_start_time = time.time()
            self._trajectory_active = True

            logger.info(
                f"Starting trajectory: joint{joint_index + 1} "
                f"from {math.degrees(self._trajectory_start_positions[joint_index]):.2f}° "
                f"to {math.degrees(self._trajectory_end_positions[joint_index]):.2f}° "
                f"over {duration}s"
            )

            return (
                f"Started moving joint {joint_index + 1} by {delta_degrees:.1f}° over {duration}s"
            )

    @rpc
    def move_joint_velocity(self, joint_index: int, velocity_deg_s: float, duration: float) -> str:
        """
        Move a single joint with velocity control (constant velocity).

        Sends constant velocity commands for the specified duration.

        Args:
            joint_index: Index of joint to move (0-based, so joint 6 = index 5)
            velocity_deg_s: Target velocity in degrees/second (positive = counterclockwise)
            duration: Time to send velocity commands in seconds

        Returns:
            Status message
        """
        with self._state_lock:
            if self._current_joint_state is None:
                return "Error: No joint state received yet"

            if self._trajectory_active:
                return "Error: Trajectory already in progress"

            if joint_index < 0 or joint_index >= self.config.num_joints:
                return f"Error: Invalid joint index {joint_index} (must be 0-{self.config.num_joints - 1})"

            # NOTE: xArm SDK vc_set_joint_velocity expects degrees/second, not radians!
            # So we keep velocity in degrees/second
            velocity_value = velocity_deg_s

            # Set up trajectory (using same state variables, but different generation logic)
            self._trajectory_start_positions = [joint_index]  # Store joint index
            self._trajectory_end_positions = [velocity_value]  # Store target velocity in deg/s
            self._trajectory_duration = duration
            self._trajectory_start_time = time.time()
            self._trajectory_active = True
            self._trajectory_is_velocity = True  # Flag to use velocity generation

            logger.info(
                f"Starting velocity trajectory: joint{joint_index + 1} "
                f"velocity={velocity_deg_s:.2f}°/s "
                f"duration={duration}s (constant velocity)"
            )

            return f"Started velocity control on joint {joint_index + 1}: {velocity_deg_s:.1f}°/s for {duration}s"

    # =========================================================================
    # Private Methods: Callbacks
    # =========================================================================

    def _on_joint_state(self, msg: JointState):
        """Callback for receiving joint state updates."""
        with self._state_lock:
            # Log first message with all joints
            if self._current_joint_state is None:
                logger.info("✓ Received first joint state:")
                logger.info(f"  Positions (rad): {[f'{p:.4f}' for p in msg.position]}")
                logger.info(
                    f"  Positions (deg): {[f'{math.degrees(p):.2f}' for p in msg.position]}"
                )
                logger.info(f"  Velocities (rad/s): {[f'{v:.4f}' for v in msg.velocity]}")
                logger.info(
                    f"  Velocities (deg/s): {[f'{math.degrees(v):.2f}' for v in msg.velocity]}"
                )
            self._current_joint_state = msg

    def _on_robot_state(self, msg: RobotState):
        """Callback for receiving robot state updates."""
        with self._state_lock:
            # Log first message or when state/error changes
            if self._current_robot_state is None:
                logger.info(
                    f"✓ Received first robot state: "
                    f"state={msg.state}, mode={msg.mode}, "
                    f"error={msg.error_code}, warn={msg.warn_code}"
                )
            elif (
                msg.state != self._current_robot_state.state
                or msg.error_code != self._current_robot_state.error_code
            ):
                # State definitions: 1=ready, 2=moving, 3=paused, 4=stopped, 5=stopped(?)
                logger.info(
                    f"⚠ Robot state changed: "
                    f"state={msg.state}, mode={msg.mode}, "
                    f"error={msg.error_code}, warn={msg.warn_code}"
                )
                if msg.error_code != 0:
                    logger.warning(
                        f"⚠ Robot has error code {msg.error_code} "
                        f"(9='not ready to move', check if servo is enabled)"
                    )
            self._current_robot_state = msg

    # =========================================================================
    # Private Methods: Control Loop
    # =========================================================================

    def _start_control_loop(self):
        """Start the control loop thread."""
        logger.info(f"Starting control loop at {self.config.publish_rate}Hz")

        self._running = True
        self._stop_event.clear()

        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="traj_gen_control_thread"
        )
        self._control_thread.start()

    def _control_loop(self):
        """
        Control loop for publishing commands.

        Runs at publish_rate Hz and publishes either position or velocity commands.
        """
        period = 1.0 / self.config.publish_rate
        next_time = time.time()
        loop_count = 0

        logger.info(
            f"Control loop started at {self.config.publish_rate}Hz "
            f"(mode={self.config.control_mode})"
        )

        while self._running:
            loop_count += 1
            try:
                # Only publish if enabled
                if self._publishing_enabled:
                    # Generate command (currently just zeros)
                    command = self._generate_command()

                    # Publish command based on control mode
                    if command is not None:
                        # Log current joint state periodically (every 50 loops)
                        # if loop_count % 50 == 0:
                        #     with self._state_lock:
                        #         if self._current_joint_state is not None:
                        #             js = self._current_joint_state
                        #             logger.info(
                        #                 f"Loop #{loop_count}: Current joint positions (deg): "
                        #                 f"{[f'{math.degrees(p):.2f}' for p in js.position]}"
                        #             )
                        #             logger.info(
                        #                 f"Loop #{loop_count}: Current joint velocities (deg/s): "
                        #                 f"{[f'{math.degrees(v):.2f}' for v in js.velocity]}"
                        #             )

                        # Publish to correct topic based on current trajectory type
                        if self._trajectory_is_velocity:
                            # Currently executing velocity trajectory - publish velocities
                            self._publish_velocity_command(command)
                        elif self.config.control_mode == "position":
                            self._publish_position_command(command)
                        elif self.config.control_mode == "velocity":
                            self._publish_velocity_command(command)
                        else:
                            logger.warning(f"Unknown control mode: {self.config.control_mode}")

                # Maintain loop frequency
                next_time += period
                sleep_time = next_time - time.time()

                if sleep_time > 0:
                    if self._stop_event.wait(timeout=sleep_time):
                        break
                else:
                    next_time = time.time()

            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                time.sleep(period)

        logger.info("Control loop stopped")

    def _generate_command(self) -> Optional[List[float]]:
        """
        Generate command for the robot.

        If trajectory is active: interpolate between start and end positions.
        Otherwise: hold current position (safe).

        Returns:
            List of joint commands (positions or velocities), or None if not ready.
        """
        with self._state_lock:
            # Wait until we have joint state feedback
            if self._current_joint_state is None:
                return None

            # Check if trajectory is active
            if self._trajectory_active and self._trajectory_start_positions is not None:
                # Calculate elapsed time
                elapsed = time.time() - self._trajectory_start_time

                # Check if trajectory is complete
                if elapsed >= self._trajectory_duration:
                    # Trajectory complete
                    self._trajectory_active = False
                    self._trajectory_is_velocity = False
                    logger.info(f"✓ Trajectory completed in {elapsed:.3f}s")

                    # For velocity mode, return zeros to stop
                    if self.config.control_mode == "velocity":
                        return [0.0] * self.config.num_joints
                    else:
                        # For position mode, return end position
                        return list(self._trajectory_end_positions)

                # Generate command based on trajectory type
                if self._trajectory_is_velocity:
                    # VELOCITY TRAJECTORY: Constant velocity (no ramping)
                    joint_index = self._trajectory_start_positions[0]
                    target_velocity = self._trajectory_end_positions[0]

                    # Just send constant velocity for the entire duration
                    velocity = target_velocity

                    # Create command: zero velocity for all joints except target
                    command = [0.0] * self.config.num_joints
                    command[joint_index] = velocity
                    return command

                else:
                    # POSITION TRAJECTORY: Linear interpolation
                    s = elapsed / self._trajectory_duration

                    command = []
                    for i in range(self.config.num_joints):
                        start = self._trajectory_start_positions[i]
                        end = self._trajectory_end_positions[i]
                        position = start + s * (end - start)
                        command.append(position)

                    return command

            # No active trajectory
            if self.config.control_mode == "position":
                # Position mode: hold current position (safe)
                return list(self._current_joint_state.position)
            else:
                # Velocity mode: zero velocities (no motion)
                return [0.0] * self.config.num_joints

    def _publish_position_command(self, command: List[float]):
        """Publish joint position command."""
        if self.joint_position_command._transport or (
            hasattr(self.joint_position_command, "connection")
            and self.joint_position_command.connection
        ):
            try:
                # Create JointCommand message with timestamp
                cmd_msg = JointCommand(positions=command)
                self.joint_position_command.publish(cmd_msg)
                self._command_count += 1

                # Log first few commands and periodically
                # if self._command_count <= 3 or self._command_count % 100 == 0:
                #     logger.info(
                #         f"✓ Published position command #{self._command_count}: "
                #         f"{[f'{c:.3f}' for c in command[:3]]}... "
                #         f"(timestamp={cmd_msg.timestamp:.6f})"
                #     )
            except Exception as e:
                logger.error(f"Failed to publish position command: {e}")
        else:
            if self._command_count == 0:
                logger.warning("joint_position_command transport not configured!")

    def _publish_velocity_command(self, command: List[float]):
        """Publish joint velocity command."""
        if self.joint_velocity_command._transport or (
            hasattr(self.joint_velocity_command, "connection")
            and self.joint_velocity_command.connection
        ):
            try:
                # Create JointCommand message with timestamp
                cmd_msg = JointCommand(positions=command)
                self.joint_velocity_command.publish(cmd_msg)
                self._command_count += 1

                # Log first few commands
                if self._command_count <= 3:
                    logger.info(
                        f"✓ Published velocity command #{self._command_count}: "
                        f"{[f'{c:.3f}' for c in command[:3]]}... "
                        f"(timestamp={cmd_msg.timestamp:.6f})"
                    )
            except Exception as e:
                logger.error(f"Failed to publish velocity command: {e}")
        else:
            if self._command_count == 0:
                logger.warning("joint_velocity_command transport not configured!")
