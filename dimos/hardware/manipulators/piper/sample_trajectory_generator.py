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
Sample Trajectory Generator for Piper Manipulator.

This module demonstrates how to:
- Subscribe to joint_state and robot_state from the Piper driver
- Publish joint position commands
- Implement a simple control loop
"""

from dataclasses import dataclass
import math
import threading
import time

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


@dataclass
class TrajectoryGeneratorConfig(ModuleConfig):
    """Configuration for trajectory generator."""

    num_joints: int = 6  # Number of joints
    control_mode: str = "position"  # "position" mode
    publish_rate: float = 100.0  # Command publishing rate in Hz
    enable_on_start: bool = False  # Start publishing commands immediately


class SampleTrajectoryGenerator(Module):
    """
    Sample trajectory generator for Piper manipulator.

    This module demonstrates command publishing and state monitoring.

    Architecture:
    - Subscribes to joint_state and robot_state from Piper driver
    - Publishes joint_position_command
    - Runs a control loop at publish_rate Hz
    """

    default_config = TrajectoryGeneratorConfig

    # Input topics (state feedback from robot)
    joint_state_input: In[JointState] = None  # Current joint state
    robot_state_input: In[RobotState] = None  # Current robot state

    # Output topics (commands to robot)
    joint_position_command: Out[JointCommand] = None  # Position commands (radians)
    joint_velocity_command: Out[JointCommand] = None  # Velocity commands (rad/s)

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        # State tracking
        self._current_joint_state: JointState | None = None
        self._current_robot_state: RobotState | None = None
        self._state_lock = threading.Lock()

        # Control thread
        self._running = False
        self._stop_event = threading.Event()
        self._control_thread: threading.Thread | None = None

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

        logger.info(
            f"TrajectoryGenerator initialized: {self.config.num_joints} joints, "
            f"mode={self.config.control_mode}, rate={self.config.publish_rate}Hz"
        )

    @rpc
    def start(self) -> None:
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
    def stop(self) -> None:
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
    def enable_publishing(self) -> None:
        """Enable command publishing."""
        self._publishing_enabled = True
        logger.info("Command publishing enabled")

    @rpc
    def disable_publishing(self) -> None:
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
            joint_index: Index of joint to move (0-based)
            delta_degrees: Amount to rotate in degrees (positive = counterclockwise)
            duration: Time to complete motion in seconds

        Returns:
            Status message
        """
        # Re-enable publishing if it was disabled
        if not self._publishing_enabled:
            logger.info("Re-enabling publishing for new trajectory")
            self._publishing_enabled = True

        with self._state_lock:
            if self._current_joint_state is None:
                return "Error: No joint state received yet"

            if self._trajectory_active:
                return "Error: Trajectory already in progress"

            if joint_index < 0 or joint_index >= self.config.num_joints:
                return f"Error: Invalid joint index {joint_index} (must be 0-{self.config.num_joints - 1})"

            # Convert delta to radians
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

    # =========================================================================
    # Private Methods: Callbacks
    # =========================================================================

    def _on_joint_state(self, msg: JointState) -> None:
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

    def _on_robot_state(self, msg: RobotState) -> None:
        """Callback for receiving robot state updates."""
        with self._state_lock:
            # Log first message or when state/error changes
            if self._current_robot_state is None:
                logger.info(
                    f"✓ Received first robot state: "
                    f"state={msg.state}, mode={msg.mode}, "
                    f"error={msg.error_code}"
                )
            self._current_robot_state = msg

    # =========================================================================
    # Private Methods: Control Loop
    # =========================================================================

    def _start_control_loop(self) -> None:
        """Start the control loop thread."""
        logger.info(f"Starting control loop at {self.config.publish_rate}Hz")

        self._running = True
        self._stop_event.clear()

        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="traj_gen_control_thread"
        )
        self._control_thread.start()

    def _control_loop(self) -> None:
        """
        Control loop for publishing commands.

        Runs at publish_rate Hz and publishes position commands.
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
                    # Generate command
                    command = self._generate_command()

                    # Publish command
                    if command is not None:
                        self._publish_position_command(command)

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

    def _generate_command(self) -> list[float] | None:
        """
        Generate command for the robot.

        If trajectory is active: interpolate between start and end positions.
        Otherwise: hold current position (safe).

        Returns:
            List of joint commands (positions), or None if not ready.
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
                    logger.info(f"✓ Trajectory completed in {elapsed:.3f}s")
                    self._trajectory_active = False

                    # Stop publishing (robot holds last position)
                    logger.info("  Trajectory complete - stopping command publishing")
                    self._publishing_enabled = False
                    return None

                # POSITION TRAJECTORY: Linear interpolation
                s = elapsed / self._trajectory_duration

                command = []
                for i in range(self.config.num_joints):
                    start = self._trajectory_start_positions[i]
                    end = self._trajectory_end_positions[i]
                    position = start + s * (end - start)
                    command.append(position)

                return command

            # No active trajectory: hold current position (safe)
            return list(self._current_joint_state.position)

    def _publish_position_command(self, command: list[float]) -> None:
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
            except Exception as e:
                logger.error(f"Failed to publish position command: {e}")
        else:
            if self._command_count == 0:
                logger.warning("joint_position_command transport not configured!")
