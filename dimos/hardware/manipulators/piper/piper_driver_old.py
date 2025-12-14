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
Piper Real-time Driver Module

This module provides a real-time controller for the Piper manipulator
compatible with the Piper Python SDK (CAN bus communication).

Architecture Overview (mirrors xArm driver):
- Main thread: Handles RPC calls and manages lifecycle
- Joint State Thread: Reads and publishes joint_state at joint_state_rate Hz
- Control Thread: Sends joint commands at control_frequency Hz
- Robot State Thread: Publishes robot_state at robot_state_rate Hz

Units:
- All external interfaces use RADIANS for joint positions/velocities
- Internally converts to/from Piper's 0.001 degree units
"""

from dataclasses import dataclass
import sys
import threading
import time
from typing import Optional

from piper_sdk import C_PiperInterface_V2
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.utils.logging_config import setup_logger

from .components import (
    ConfigurationComponent,
    GripperControlComponent,
    KinematicsComponent,
    MotionControlComponent,
    StateQueryComponent,
    SystemControlComponent,
)

logger = setup_logger()

# Unit conversion constants
# Piper uses 0.001 degrees, we use radians
# 1 radian = 1000 * 180 / π * 0.001 degrees = 57295.7795
RAD_TO_PIPER = 57295.7795  # radians to Piper units
PIPER_TO_RAD = 1.0 / RAD_TO_PIPER  # Piper units to radians

# Velocity control uses integration-based approach:
# position_target += velocity * dt


@dataclass
class PiperDriverConfig(ModuleConfig):
    """Configuration for Piper driver."""

    can_name: str = "can0"  # CAN interface name
    control_frequency: float = 100.0  # Control loop frequency in Hz
    joint_state_rate: float = 100.0  # Joint state publishing rate in Hz
    robot_state_rate: float = 10.0  # Robot state publishing rate in Hz
    enable_on_start: bool = True  # Enable servo mode on start
    judge_flag: bool = True  # Enable SDK safety checks
    can_auto_init: bool = True  # Auto-initialize CAN bus
    dh_is_offset: int = 0  # DH parameter mode

    # Velocity control settings
    velocity_control: bool = (
        False  # Use velocity control mode (integration-based) instead of position
    )

    # Auto-recovery settings
    auto_recovery: bool = True  # Automatically recover from errors
    recovery_cooldown: float = 3.0  # Minimum seconds between recovery attempts

    @property
    def num_joints(self) -> int:
        """Piper has 6 joints (fixed)."""
        return 6


class PiperDriver(
    MotionControlComponent,
    StateQueryComponent,
    SystemControlComponent,
    KinematicsComponent,
    GripperControlComponent,
    ConfigurationComponent,
    Module,
):
    """
    Real-time driver for Piper manipulator.

    This driver implements a real-time control architecture with component-based design:
    - Core driver: Handles threads, callbacks, and connection management
    - MotionControlComponent: Motion control RPC methods
    - StateQueryComponent: State query RPC methods
    - SystemControlComponent: System control RPC methods
    - KinematicsComponent: Kinematics RPC methods
    - GripperControlComponent: Gripper control RPC methods
    - ConfigurationComponent: Configuration RPC methods

    Architecture:
    - Subscribes to joint commands and publishes joint states
    - Runs a 100Hz control loop for joint position control
    - Provides RPC methods for Piper SDK API access via components
    """

    default_config = PiperDriverConfig

    # Input topics (commands from controllers)
    joint_position_command: In[JointCommand] = None  # Target joint positions (radians)
    joint_velocity_command: In[JointCommand] = None  # Target joint velocities (rad/s)

    # Output topics (state publishing)
    joint_state: Out[JointState] = None  # Joint state (position, velocity, effort)
    robot_state: Out[RobotState] = None  # Robot state (mode, errors, etc.)

    def __init__(self, *args, **kwargs) -> None:
        """Initialize the Piper driver."""
        super().__init__(*args, **kwargs)

        # Piper SDK instance
        self.piper: C_PiperInterface_V2 | None = None

        # State tracking variables
        self.curr_state: int = 0  # Robot state
        self.curr_err: int = 0  # Current error code
        self.curr_mode: int = 0  # Current control mode
        self.curr_warn: int = 0  # Warning code

        # Shared state (protected by locks)
        self._joint_cmd_lock = threading.Lock()
        self._joint_state_lock = threading.Lock()
        self._joint_cmd_: list[float] | None = None  # Latest joint command (radians)
        self._vel_cmd_: list[float] | None = None  # Latest velocity command (rad/s)
        self._position_target_: list[float] | None = (
            None  # Position target for velocity integration. Required if sdk does not support direct velocity control.
        )
        self._joint_states_: JointState | None = None  # Latest joint state
        self._robot_state_: RobotState | None = None  # Latest robot state
        self._last_cmd_time: float = 0.0  # Timestamp of last command

        # Thread management
        self._state_thread: threading.Thread | None = None  # Joint state publishing
        self._control_thread: threading.Thread | None = None  # Command sending
        self._robot_state_thread: threading.Thread | None = None  # Robot state publishing
        self._stop_event = threading.Event()  # Thread-safe stop signal

        # Auto-recovery tracking
        self._last_recovery_attempt: float = 0.0

        # Joint names
        self._joint_names = [f"joint{i + 1}" for i in range(self.config.num_joints)]

        logger.info(
            f"PiperDriver initialized for {self.config.num_joints}-joint arm on "
            f"{self.config.can_name}"
        )

    @rpc
    def start(self):
        """
        Start the Piper driver.

        Initializes the Piper connection and starts control threads.
        """
        super().start()

        # Initialize state variables
        self.curr_err = 0
        self.curr_state = 0
        self.curr_mode = 0
        self.curr_warn = 0
        self.piper = None

        logger.info(
            f"can_name={self.config.can_name}, "
            f"dh_is_offset={self.config.dh_is_offset}, "
            f"dof={self.config.num_joints}"
        )

        # Create Piper SDK instance
        logger.info("Creating Piper SDK instance...")
        try:
            self.piper = C_PiperInterface_V2(
                can_name=self.config.can_name,
                judge_flag=self.config.judge_flag,
                can_auto_init=self.config.can_auto_init,
                dh_is_offset=self.config.dh_is_offset,
            )
            logger.info("Piper SDK instance created")
        except Exception as e:
            logger.error(f"Failed to create Piper SDK instance: {e}")
            raise

        # Connect to CAN port
        logger.info(f"Connecting to Piper via CAN port {self.config.can_name}...")
        # Use piper_init=True and start_thread=True to enable firmware version reading
        self.piper.ConnectPort(piper_init=True, start_thread=True)
        # Note: ConnectPort() may return False even on success
        # The SDK prints "can0 bus opened successfully" on success
        logger.info("Connected to Piper via CAN bus")

        # Wait for firmware feedback to be received
        time.sleep(0.025)

        # Get firmware version
        try:
            version = self.piper.GetPiperFirmwareVersion()
            logger.info(f"Piper firmware version: {version}")
        except Exception as e:
            logger.warning(f"Could not read firmware version: {e}")

        # Enable the arm if configured
        if self.config.enable_on_start:
            logger.info("Enabling Piper arm...")
            attempts = 0
            max_attempts = 100
            while not self.piper.EnablePiper() and attempts < max_attempts:
                time.sleep(0.01)
                attempts += 1
            if attempts >= max_attempts:
                logger.error("Failed to enable Piper arm")
            else:
                logger.info(f"Piper enabled (attempt {attempts}/{max_attempts})")

            # Set control mode to CAN control (0x01)
            logger.info("Setting Piper control mode...")
            self.piper.MotionCtrl_2(
                ctrl_mode=0x01,  # CAN control mode
                move_mode=0x01,  # Move mode
                move_spd_rate_ctrl=30,  # Speed rate
                is_mit_mode=0x00,  # Not MIT mode
            )
            logger.info("Control mode set successfully")

        # Initialize joint state message
        self._init_publishers()

        # Start threads
        self._start_threads()

        # Subscribe to command topics
        try:
            unsub = self.joint_position_command.subscribe(self._on_joint_position_command)
            self._disposables.add(Disposable(unsub))
        except (AttributeError, ValueError) as e:
            logger.debug(f"joint_position_command not configured: {e}")

        try:
            unsub = self.joint_velocity_command.subscribe(self._on_joint_velocity_command)
            self._disposables.add(Disposable(unsub))
        except (AttributeError, ValueError) as e:
            logger.debug(f"joint_velocity_command not configured: {e}")

        logger.info("PiperDriver started successfully")

    @rpc
    def stop(self):
        """Stop the Piper driver."""
        logger.info("Stopping PiperDriver...")

        # Signal threads to stop
        self._stop_event.set()

        # Wait for threads to finish
        for thread in [self._state_thread, self._control_thread, self._robot_state_thread]:
            if thread and thread.is_alive():
                thread.join(timeout=2.0)

        # Disable the arm
        if self.piper:
            logger.info("Disabling Piper arm...")
            self.piper.DisablePiper()
            logger.info("Disconnecting from Piper...")
            self.piper.DisconnectPort()
            logger.info("Disconnected from Piper")

        super().stop()
        logger.info("PiperDriver stopped")

    @rpc
    def get_joint_state(self) -> JointState | None:
        """Get the latest joint state."""
        with self._joint_state_lock:
            return self._joint_states_

    @rpc
    def get_robot_state(self) -> RobotState | None:
        """Get the latest robot state."""
        with self._joint_state_lock:
            return self._robot_state_

    @rpc
    def enable_servo_mode(self) -> bool:
        """Enable Piper arm (equivalent to xArm servo mode)."""
        if not self.piper:
            return False
        return self.piper.EnablePiper()

    @rpc
    def disable_servo_mode(self) -> bool:
        """Disable Piper arm."""
        if not self.piper:
            return False
        return self.piper.DisablePiper()

    @rpc
    def clear_errors(self) -> bool:
        """
        Clear Piper errors.

        Piper doesn't have a direct error clearing method,
        so we disable and re-enable the arm.
        """
        if not self.piper:
            return False
        logger.info("Attempting to clear Piper errors...")
        self.piper.DisablePiper()
        time.sleep(0.1)
        success = self.piper.EnablePiper()
        if success:
            logger.info("Errors cleared (arm disabled and re-enabled)")
        return success

    # =========================================================================
    # Internal Methods
    # =========================================================================

    def _init_publishers(self) -> None:
        """Initialize publisher messages."""
        # Initialize joint state message
        self._joint_state_msg = JointState(
            ts=time.time(),
            frame_id="piper_joint_state",
            name=self._joint_names,
            position=[0.0] * self.config.num_joints,
            velocity=[0.0] * self.config.num_joints,
            effort=[0.0] * self.config.num_joints,
        )

    def _start_threads(self) -> None:
        """Start all background threads."""
        logger.info("Starting background threads...")

        # Joint state thread
        logger.info(f"Starting joint state thread at {self.config.joint_state_rate}Hz")
        self._state_thread = threading.Thread(
            target=self._joint_state_loop, daemon=True, name="piper_state_thread"
        )
        self._state_thread.start()
        logger.info("Joint state loop started")

        # Control thread
        logger.info(f"Starting control thread at {self.config.control_frequency}Hz")
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="piper_control_thread"
        )
        self._control_thread.start()
        logger.info("Control loop started")

        # Robot state thread
        logger.info(f"Starting robot state thread at {self.config.robot_state_rate}Hz")
        self._robot_state_thread = threading.Thread(
            target=self._robot_state_loop, daemon=True, name="piper_robot_state_thread"
        )
        self._robot_state_thread.start()
        logger.info("Robot state loop started")

    def _joint_state_loop(self) -> None:
        """Joint state publishing loop (runs at joint_state_rate Hz)."""
        period = 1.0 / self.config.joint_state_rate
        next_time = time.time()

        while not self._stop_event.is_set():
            try:
                # Read joint state from Piper
                if self.piper:
                    arm_msg = self.piper.GetArmJointMsgs()
                    if arm_msg and arm_msg.joint_state:
                        # Convert from Piper units (0.001 degrees) to radians
                        # arm_msg.joint_state has fields: joint_1, joint_2, ..., joint_6
                        positions = [
                            arm_msg.joint_state.joint_1 * PIPER_TO_RAD,
                            arm_msg.joint_state.joint_2 * PIPER_TO_RAD,
                            arm_msg.joint_state.joint_3 * PIPER_TO_RAD,
                            arm_msg.joint_state.joint_4 * PIPER_TO_RAD,
                            arm_msg.joint_state.joint_5 * PIPER_TO_RAD,
                            arm_msg.joint_state.joint_6 * PIPER_TO_RAD,
                        ]

                        # Update joint state message
                        self._joint_state_msg.ts = time.time()
                        self._joint_state_msg.position = positions
                        # Velocity not available from Piper SDK
                        self._joint_state_msg.velocity = [0.0] * self.config.num_joints
                        self._joint_state_msg.effort = [0.0] * self.config.num_joints

                        # Store in shared state
                        with self._joint_state_lock:
                            self._joint_states_ = self._joint_state_msg

                        # Publish (only if transport is configured)
                        if self.joint_state._transport:
                            try:
                                self.joint_state.publish(self._joint_state_msg)
                            except Exception:
                                pass  # Transport error, skip publishing

                # Maintain loop frequency
                next_time += period
                sleep_time = next_time - time.time()
                if sleep_time > 0:
                    if self._stop_event.wait(timeout=sleep_time):
                        break
                else:
                    next_time = time.time()

            except Exception as e:
                logger.error(f"Error in joint state loop: {e}")
                time.sleep(period)

        logger.info("Joint state loop stopped")

    def _control_loop(self) -> None:
        """
        Control loop (runs at control_frequency Hz).

        Supports two modes:
        - Position control: Uses JointCtrl with position commands
        - Velocity control: Uses JointMitCtrl with MIT gains for velocity-based control
        """
        period = 1.0 / self.config.control_frequency
        next_time = time.time()
        timeout_logged = False

        while not self._stop_event.is_set():
            try:
                current_time = time.time()

                # Get command from shared state
                with self._joint_cmd_lock:
                    if self.config.velocity_control:
                        joint_cmd = self._vel_cmd_  # Velocity commands (rad/s)
                    else:
                        joint_cmd = self._joint_cmd_  # Position commands (radians)
                    last_cmd_time = self._last_cmd_time

                # Check for timeout (0.1 second without new commands)
                time_since_last_cmd = current_time - last_cmd_time if last_cmd_time > 0 else 0

                if time_since_last_cmd > 0.1 and joint_cmd is not None:
                    if not timeout_logged:
                        logger.warning(
                            f"Command timeout: no commands received for {time_since_last_cmd:.2f}s. "
                            f"Stopping robot."
                        )
                        timeout_logged = True
                    # Send zero velocity to stop in velocity mode
                    if self.config.velocity_control:
                        joint_cmd = [0.0] * 6
                    else:
                        # In position mode, just skip sending (robot holds position)
                        joint_cmd = None
                else:
                    timeout_logged = False

                # Send command to Piper
                if self.piper and joint_cmd:
                    if self.config.velocity_control:
                        # VELOCITY CONTROL MODE: Integration-based approach
                        # Initialize position target from current joint states on first command
                        if self._position_target_ is None and self._joint_states_:
                            self._position_target_ = list(self._joint_states_.position)
                            logger.info(
                                f"Velocity control: Initialized position target from current state: {self._position_target_}"
                            )

                        # Integrate velocity to get position: pos_target += vel * dt
                        if self._position_target_ is not None:
                            for i in range(6):
                                self._position_target_[i] += joint_cmd[i] * period

                            # Convert from radians to Piper units (0.001 degrees)
                            piper_joints = [
                                round(rad * RAD_TO_PIPER) for rad in self._position_target_
                            ]

                            # Send joint control command with integrated position
                            self.piper.JointCtrl(
                                piper_joints[0],
                                piper_joints[1],
                                piper_joints[2],
                                piper_joints[3],
                                piper_joints[4],
                                piper_joints[5],
                            )
                    else:
                        # POSITION CONTROL MODE: Use standard joint control
                        # Convert from radians to Piper units (0.001 degrees)
                        piper_joints = [round(rad * RAD_TO_PIPER) for rad in joint_cmd]

                        # Send joint control command
                        self.piper.JointCtrl(
                            piper_joints[0],
                            piper_joints[1],
                            piper_joints[2],
                            piper_joints[3],
                            piper_joints[4],
                            piper_joints[5],
                        )

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

    def _robot_state_loop(self) -> None:
        """Robot state publishing loop (runs at robot_state_rate Hz)."""
        period = 1.0 / self.config.robot_state_rate
        next_time = time.time()

        while not self._stop_event.is_set():
            try:
                # Create robot state message
                robot_state = RobotState()
                robot_state.state = self.curr_state
                robot_state.mode = self.curr_mode
                robot_state.error_code = self.curr_err
                robot_state.warn_code = self.curr_warn

                # Store in shared state
                with self._joint_state_lock:
                    self._robot_state_ = robot_state

                # Publish (only if transport is configured)
                if self.robot_state._transport:
                    try:
                        self.robot_state.publish(robot_state)
                    except Exception:
                        pass  # Transport error, skip publishing

                # Maintain loop frequency
                next_time += period
                sleep_time = next_time - time.time()
                if sleep_time > 0:
                    if self._stop_event.wait(timeout=sleep_time):
                        break
                else:
                    next_time = time.time()

            except Exception as e:
                logger.error(f"Error in robot state loop: {e}")
                time.sleep(period)

        logger.info("Robot state loop stopped")

    def _on_joint_position_command(self, cmd_msg: JointCommand) -> None:
        """Callback when joint position command is received."""
        with self._joint_cmd_lock:
            self._joint_cmd_ = list(cmd_msg.positions)
            self._last_cmd_time = time.time()

    def _on_joint_velocity_command(self, cmd_msg: JointCommand) -> None:
        """Callback when joint velocity command is received."""
        with self._joint_cmd_lock:
            self._vel_cmd_ = list(cmd_msg.positions)
            self._last_cmd_time = time.time()


# Expose blueprint for declarative composition
piper_driver = PiperDriver.blueprint
