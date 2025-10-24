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
xArm Real-time Driver Module

This module provides a real-time controller for the xArm manipulator family
(xArm5, xArm6, xArm7) compatible with the xArm Python SDK.

Architecture (mirrors C++ xarm_driver.cpp):
- Main thread: Handles RPC calls and manages lifecycle
- Joint State Thread: Reads and publishes joint_state at joint_state_rate Hz
  (default: 100Hz for dev, 5Hz for normal report_type)
- Control Thread: Sends joint commands at control_frequency Hz (100Hz)
- SDK Report Callback: Updates robot_state at report_type frequency
  ('dev'=100Hz, 'rich'=5Hz, 'normal'=5Hz) - SEPARATE from joint state!

Key Insight:
- joint_state_rate: How often we READ positions (get_joint_states/get_servo_angle)
- report_type: How often SDK pushes robot state updates (state, mode, errors)
- These are INDEPENDENT! You can read joint states at 100Hz while getting
  robot state updates at 5Hz (normal mode).
"""

import time
import threading
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass

from xarm.wrapper import XArmAPI

from dimos.core import Module, In, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.sensor_msgs import JointState, JointCommand, RobotState
from dimos.msgs.geometry_msgs import WrenchStamped
from dimos.utils.logging_config import setup_logger
from reactivex.disposable import Disposable, CompositeDisposable

logger = setup_logger(__file__)


@dataclass
class XArmDriverConfig(ModuleConfig):
    """Configuration for xArm driver."""

    ip_address: str = "192.168.1.185"  # xArm IP address
    is_radian: bool = True  # Use radians (True) or degrees (False)
    control_frequency: float = 100.0  # Control loop frequency in Hz (for sending commands)
    joint_state_rate: float = (
        -1.0
    )  # Joint state publishing rate (-1 = auto: 100Hz for dev, 5Hz for normal)
    robot_state_rate: float = 10.0  # Robot state publishing rate in Hz
    report_type: str = "normal"  # SDK report type: 'dev'=100Hz, 'rich'=5Hz+torque, 'normal'=5Hz
    enable_on_start: bool = True  # Enable servo mode on start
    num_joints: int = 7  # Number of joints (5, 6, or 7)
    check_joint_limit: bool = True  # Check joint limits
    check_cmdnum_limit: bool = True  # Check command queue limit
    max_cmdnum: int = 512  # Maximum command queue size
    velocity_control: bool = False  # Use velocity control mode instead of position
    velocity_duration: float = 0.1  # Duration for velocity commands (seconds)


class XArmDriver(Module):
    """
    Real-time driver for xArm manipulators (xArm5/6/7).

    This driver implements a real-time control architecture:
    - Subscribes to joint commands and publishes joint states
    - Runs a 100Hz control loop for servo angle control
    - Provides RPC methods for xArm SDK API access
    """

    default_config = XArmDriverConfig

    # Input topics (commands from controllers)
    joint_position_command: In[JointCommand] = None  # Target joint positions (radians)
    joint_velocity_command: In[JointCommand] = None  # Target joint velocities (rad/s)

    # Output topics (state publishing)
    joint_state: Out[JointState] = None  # Joint state (position, velocity, effort)
    robot_state: Out[RobotState] = None  # Robot state (mode, errors, etc.)
    ft_ext: Out[WrenchStamped] = None  # External force/torque (compensated)
    ft_raw: Out[WrenchStamped] = None  # Raw force/torque sensor data

    def __init__(self, *args, **kwargs):
        """Initialize the xArm driver."""
        super().__init__(*args, **kwargs)

        # xArm SDK instance
        self.arm: Optional[XArmAPI] = None

        # State tracking variables (updated by SDK callback)
        self.curr_state: int = 4  # Robot state (4 = stopped initially)
        self.curr_err: int = 0  # Current error code
        self.curr_mode: int = 0  # Current control mode
        self.curr_cmdnum: int = 0  # Command queue length
        self.curr_warn: int = 0  # Warning code

        # Shared state (protected by locks)
        self._joint_cmd_lock = threading.Lock()
        self._joint_state_lock = threading.Lock()
        self._joint_cmd_: Optional[List[float]] = None  # Latest joint command
        self._vel_cmd_: Optional[List[float]] = None  # Latest velocity command
        self._joint_states_: Optional[JointState] = None  # Latest joint state
        self._robot_state_: Optional[RobotState] = None  # Latest robot state
        self._last_cmd_time: float = 0.0  # Timestamp of last command received (for timeout)

        # Thread management
        self._running = False
        self._state_thread: Optional[threading.Thread] = None  # Joint state publishing
        self._control_thread: Optional[threading.Thread] = None  # Command sending
        self._stop_event = threading.Event()

        # Subscription management
        self._disposables = CompositeDisposable()

        # Joint names based on number of joints
        self._joint_names = [f"joint{i + 1}" for i in range(self.config.num_joints)]

        # Joint state message (initialized in _init_publisher)
        self._joint_state_msg: Optional[JointState] = None

        # Firmware version cache
        self._firmware_version: Optional[Tuple[int, int, int]] = None

        logger.info(
            f"XArmDriver initialized for {self.config.num_joints}-joint arm at "
            f"{self.config.ip_address}"
        )

    @rpc
    def start(self):
        """
        Start the xArm driver (mirrors C++ XArmDriver::init).

        Initializes the xArm connection, registers callbacks, and starts
        the joint state publishing thread.
        """
        super().start()

        # Initialize state variables (like C++)
        self.curr_err = 0
        self.curr_state = 4  # Stopped initially
        self.curr_mode = 0
        self.curr_cmdnum = 0
        self.curr_warn = 0
        self.arm = None

        # Joint names based on configuration
        self._joint_names = [f"joint{i + 1}" for i in range(self.config.num_joints)]

        logger.info(
            f"robot_ip={self.config.ip_address}, "
            f"report_type={self.config.report_type}, "
            f"dof={self.config.num_joints}"
        )

        # Create XArmAPI instance (matching C++ constructor parameters)
        logger.info("Creating XArmAPI instance...")
        try:
            self.arm = XArmAPI(
                port=self.config.ip_address,
                is_radian=self.config.is_radian,
                do_not_open=True,  # Don't auto-connect (we'll call connect())
                check_tcp_limit=True,  # Check TCP limits
                check_joint_limit=self.config.check_joint_limit,
                check_cmdnum_limit=self.config.check_cmdnum_limit,
                check_robot_sn=False,  # Don't check serial number
                check_is_ready=True,  # Check if ready before commands
                check_is_pause=True,  # Check if paused
                max_cmdnum=self.config.max_cmdnum,
                init_axis=self.config.num_joints,  # Initialize with specified DOF
                debug=False,  # Disable debug mode
                report_type=self.config.report_type,
            )
            logger.info("XArmAPI instance created")
        except Exception as e:
            logger.error(f"Failed to create XArmAPI: {e}")
            raise

        # Release and register callbacks (like C++ pattern)
        self.arm.release_connect_changed_callback(True)
        self.arm.release_report_callback(True)
        self.arm.register_connect_changed_callback(self._report_connect_changed_callback)
        self.arm.register_report_callback(self._report_data_callback)

        # Connect to the robot
        logger.info(f"Connecting to xArm at {self.config.ip_address}...")
        self.arm.connect()
        logger.info("Connected to xArm")

        # Check for errors and warnings (like C++ code)
        err_warn = [0, 0]
        self.arm.get_err_warn_code(err_warn)
        if err_warn[0] != 0:
            logger.warning(f"UFACTORY ErrorCode: C{err_warn[0]}")

        # Check and clear servo errors (like C++ dbmsg handling)
        self._check_and_clear_servo_errors()

        # Initialize publishers (joint state message structure)
        self._init_publisher()

        # Start joint state publishing thread (read-only)
        self._start_joint_state_thread()

        # Start robot state publishing thread
        self._start_robot_state_thread()

        # Subscribe to input topics (only if they have connections/transports)
        try:
            unsub_joint = self.joint_position_command.subscribe(self._on_joint_position_command)
            self._disposables.add(Disposable(unsub_joint))
        except (AttributeError, ValueError) as e:
            logger.debug(
                f"joint_position_command transport not configured, skipping subscription: {e}"
            )

        try:
            unsub_vel = self.joint_velocity_command.subscribe(self._on_joint_velocity_command)
            self._disposables.add(Disposable(unsub_vel))
        except (AttributeError, ValueError) as e:
            logger.debug(
                f"joint_velocity_command transport not configured, skipping subscription: {e}"
            )

        # Start control thread (command sending)
        self._start_control_thread()

        # Enable servo mode if configured
        if self.config.enable_on_start:
            self._initialize_arm()

        logger.info("xArm driver started successfully")

    @rpc
    def stop(self):
        """Stop the xArm driver and disable servo mode."""
        logger.info("Stopping xArm driver...")

        # Stop both threads
        self._running = False
        self._stop_event.set()

        # Wait for state thread to finish
        if self._state_thread and self._state_thread.is_alive():
            self._state_thread.join(timeout=2.0)

        # Wait for robot state thread to finish
        if (
            hasattr(self, "_robot_state_thread")
            and self._robot_state_thread
            and self._robot_state_thread.is_alive()
        ):
            self._robot_state_thread.join(timeout=2.0)

        # Wait for control thread to finish
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)

        # Disable servo mode
        if self.arm:
            try:
                self.arm.set_mode(0)  # Position mode
                self.arm.set_state(0)  # Stop state
                logger.info("Servo mode disabled")
            except Exception as e:
                logger.error(f"Error disabling servo mode: {e}")

        # Disconnect from arm
        if self.arm:
            try:
                self.arm.disconnect()
                logger.info("Disconnected from xArm")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")

        # Clean up subscriptions
        self._disposables.dispose()

        super().stop()
        logger.info("xArm driver stopped")

    # =========================================================================
    # Private Methods: Initialization
    # =========================================================================

    def _init_publisher(self):
        """
        Initialize publisher message structures.
        Mirrors C++ XArmDriver::_init_publisher().
        """
        # Initialize joint state message structure
        self._joint_state_msg = JointState(
            ts=time.time(),
            frame_id="joint-state data",
            name=self._joint_names.copy(),
            position=[0.0] * self.config.num_joints,
            velocity=[0.0] * self.config.num_joints,
            effort=[0.0] * self.config.num_joints,
        )

        logger.info("Publishers initialized")

    def _report_connect_changed_callback(self, connected: bool, reported: bool = True):
        """
        Callback invoked when connection state changes.

        Args:
            connected: True if connected, False if disconnected
            reported: True if this is a reported change (unused but required by SDK)
        """
        if connected:
            logger.info("xArm connected")
        else:
            logger.error("xArm disconnected! Please reconnect...")

    def _check_and_clear_servo_errors(self):
        """
        Check servo debug messages and clear low-voltage or other errors.
        Mirrors the C++ dbmsg handling logic.
        """
        try:
            # Get servo debug messages (similar to C++ servo_get_dbmsg)
            dbg_msg = [0] * (self.config.num_joints * 2)

            # Check if the core API has this method
            if hasattr(self.arm, "core") and hasattr(self.arm.core, "servo_get_dbmsg"):
                self.arm.core.servo_get_dbmsg(dbg_msg)

                for i in range(self.config.num_joints):
                    error_type = dbg_msg[i * 2]
                    error_code = dbg_msg[i * 2 + 1]

                    if error_type == 1 and error_code == 40:
                        # Low-voltage error
                        self.arm.clean_error()
                        logger.warning(f"Cleared low-voltage error of joint {i + 1}")
                    elif error_type == 1:
                        # Other servo error
                        self.arm.clean_error()
                        logger.warning(
                            f"There is servo error code:(0x{error_code:x}) in joint {i + 1}, "
                            f"trying to clear it.."
                        )
            else:
                logger.debug("servo_get_dbmsg not available in SDK")

        except Exception as e:
            logger.debug(f"Could not check servo errors: {e}")

    def _firmware_version_is_ge(self, major: int, minor: int, patch: int) -> bool:
        """
        Check if firmware version is greater than or equal to specified version.

        Args:
            major: Major version number
            minor: Minor version number
            patch: Patch version number

        Returns:
            True if firmware >= specified version
        """
        if self._firmware_version is None:
            try:
                code, version_str = self.arm.get_version()
                if code == 0 and version_str:
                    # Parse version string
                    # Can be "v1.8.103" or "1.8.103" or "6,6,XI1303,DC1305,v2.6.0"
                    version_str = version_str.strip()

                    # Try to find version pattern (v2.6.0 or similar)
                    import re

                    match = re.search(r"v?(\d+)\.(\d+)\.(\d+)", version_str)
                    if match:
                        self._firmware_version = (
                            int(match.group(1)),
                            int(match.group(2)),
                            int(match.group(3)),
                        )
                    else:
                        logger.warning(f"Could not parse firmware version: {version_str}")
                        return False
                else:
                    logger.warning("Could not retrieve firmware version")
                    return False
            except Exception as e:
                logger.warning(f"Error getting firmware version: {e}")
                return False

        if self._firmware_version:
            fw_maj, fw_min, fw_pat = self._firmware_version
            if fw_maj > major:
                return True
            elif fw_maj == major:
                if fw_min > minor:
                    return True
                elif fw_min == minor:
                    return fw_pat >= patch
        return False

    def _start_joint_state_thread(self):
        """
        Start the joint state publishing thread.
        Mirrors the C++ joint state publishing thread logic.
        This thread ONLY reads and publishes joint states.
        """
        # Determine joint state rate based on report type
        if self.config.control_frequency < 0:
            joint_state_rate = 100 if self.config.report_type == "dev" else 5
        else:
            joint_state_rate = self.config.control_frequency

        logger.info(f"Starting joint state thread at {joint_state_rate}Hz")

        # Start state publishing thread
        self._running = True
        self._stop_event.clear()

        self._state_thread = threading.Thread(
            target=self._joint_state_loop, daemon=True, name="xarm_state_thread"
        )
        self._state_thread.start()

    def _start_control_thread(self):
        """
        Start the control thread for sending commands.
        This thread ONLY sends joint commands to the robot.
        """
        logger.info(f"Starting control thread at {self.config.control_frequency}Hz")

        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="xarm_control_thread"
        )
        self._control_thread.start()

    def _start_robot_state_thread(self):
        """
        Start the robot state publishing thread.
        This thread publishes robot state at a fixed rate (default 10Hz),
        using data from _report_data_callback when available, or current state variables.
        """
        logger.info(f"Starting robot state thread at {self.config.robot_state_rate}Hz")

        self._robot_state_thread = threading.Thread(
            target=self._robot_state_loop, daemon=True, name="xarm_robot_state_thread"
        )
        self._robot_state_thread.start()

    def _initialize_arm(self):
        """Initialize the arm: clear errors, set mode, enable motion."""
        try:
            # Clear any existing errors
            self.arm.clean_error()
            self.arm.clean_warn()

            # Enable motion
            self.arm.motion_enable(enable=True)

            # Enable servo mode if configured
            if self.config.enable_on_start:
                logger.info("Enabling servo mode (mode=1)...")
                code = self.arm.set_mode(1)  # Servo mode
                if code != 0:
                    logger.error(f"Failed to enable servo mode: code={code}")
                else:
                    logger.info("✓ Servo mode enabled")

                # Verify mode was set
                time.sleep(0.1)  # Give SDK time to update
                logger.info(f"Verifying mode: curr_mode={self.curr_mode} (should be 1 for servo)")

            # Set state to ready (0) - MUST be after setting mode
            logger.info("Setting robot state to ready (state=0)...")
            code = self.arm.set_state(state=0)
            if code != 0:
                logger.error(f"Failed to set state to ready: code={code}")
            else:
                logger.info("✓ Robot state set to ready")

            # Verify state
            time.sleep(0.1)  # Give SDK time to update
            logger.info(
                f"Robot initialized: state={self.curr_state} (0=ready), "
                f"mode={self.curr_mode} (1=servo), error={self.curr_err}"
            )

            logger.info("Arm initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize arm: {e}")
            raise

    # =========================================================================
    # Private Methods: Callbacks (Non-blocking)
    # =========================================================================

    def _on_joint_position_command(self, cmd_msg: JointCommand):
        """
        Callback when joint position command is received.
        Non-blocking: just store the latest command.

        Args:
            cmd_msg: JointCommand message containing positions and timestamp
        """
        with self._joint_cmd_lock:
            self._joint_cmd_ = list(cmd_msg.positions)
            self._last_cmd_time = time.time()  # Update timestamp for timeout detection

        # DEBUG: Log first few commands received
        # if not hasattr(self, '_cmd_recv_count'):
        #     self._cmd_recv_count = 0
        # self._cmd_recv_count += 1
        #
        # if self._cmd_recv_count <= 3 or self._cmd_recv_count % 100 == 0:
        #     logger.info(
        #         f"✓ Received position command #{self._cmd_recv_count}: "
        #         f"joint6={math.degrees(cmd_msg.positions[5]):.2f}°, "
        #         f"timestamp={cmd_msg.timestamp:.3f}"
        #     )

    def _on_joint_velocity_command(self, cmd_msg: JointCommand):
        """
        Callback when joint velocity command is received.
        Non-blocking: just store the latest command.

        Args:
            cmd_msg: JointCommand message containing velocities and timestamp
        """
        with self._joint_cmd_lock:
            self._vel_cmd_ = list(cmd_msg.positions)
            self._last_cmd_time = time.time()  # Update timestamp for timeout detection

    # =========================================================================
    # Private Methods: Thread Loops
    # =========================================================================

    def _joint_state_loop(self):
        """
        Joint state publishing loop.
        Mirrors the C++ lambda thread in XArmDriver::init (line 234-256).

        This thread ONLY reads joint states and publishes them.
        Runs at joint_state_rate Hz (independent of report_type).
        """
        # Determine rate (C++ line 237-239)
        # If joint_state_rate < 0, use default based on report_type
        # Otherwise use the configured rate
        if self.config.joint_state_rate < 0:
            joint_state_rate = 100 if self.config.report_type == "dev" else 5
        else:
            joint_state_rate = self.config.joint_state_rate

        period = 1.0 / joint_state_rate

        # Check firmware version to determine which API to use (C++ line 238)
        use_new = self._firmware_version_is_ge(1, 8, 103)
        logger.info(f"Joint state loop started at {joint_state_rate}Hz (use_new={use_new})")

        # For velocity calculation (old firmware)
        prev_time = time.time()
        prev_position = [0.0] * self.config.num_joints
        initialized = False

        next_time = time.time()

        while self._running and self.arm.connected:
            try:
                curr_time = time.time()

                # Read joint states
                if use_new:
                    # Newer firmware: get_joint_states returns (code, data)
                    # where data might be [[pos], [vel], [eff]] nested arrays
                    code, data = self.arm.get_joint_states()
                    if code != 0:
                        logger.warning(f"get_joint_states failed with code: {code}")
                        continue

                    # Check if data is nested (list of lists) or flat
                    if isinstance(data[0], (list, tuple)):
                        # Nested: [[positions], [velocities], [efforts]]
                        position = (
                            list(data[0]) if len(data) > 0 else [0.0] * self.config.num_joints
                        )
                        velocity = list(data[1]) if len(data) > 1 else [0.0] * len(position)
                        effort = list(data[2]) if len(data) > 2 else [0.0] * len(position)
                    else:
                        # Flat: just positions
                        position = list(data)
                        velocity = [0.0] * len(position)
                        effort = [0.0] * len(position)
                else:
                    # Older firmware: only get_servo_angle available
                    code, position = self.arm.get_servo_angle(is_radian=self.config.is_radian)
                    if code != 0:
                        logger.warning(f"get_servo_angle failed with code: {code}")
                        continue

                    # Calculate velocity from position difference
                    velocity = [0.0] * len(position)
                    if initialized:
                        dt = curr_time - prev_time
                        if dt > 0:
                            velocity = [
                                (position[i] - prev_position[i]) / dt for i in range(len(position))
                            ]

                    effort = [0.0] * len(position)

                # Update joint state message
                self._joint_state_msg.ts = curr_time
                self._joint_state_msg.position = list(position)
                self._joint_state_msg.velocity = list(velocity)
                self._joint_state_msg.effort = list(effort)

                # Update shared state
                with self._joint_state_lock:
                    self._joint_states_ = self._joint_state_msg

                # Publish (only if transport is configured)
                if self.joint_state._transport or (
                    hasattr(self.joint_state, "connection") and self.joint_state.connection
                ):
                    try:
                        self.joint_state.publish(self._joint_state_msg)
                    except Exception:
                        pass  # Transport error, skip publishing

                # Save for next iteration (velocity calculation)
                prev_position = list(position)
                prev_time = curr_time
                initialized = True

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

        if not self.arm.connected:
            logger.error("xArm Control Connection Failed! Please Shut Down and Retry...")

        logger.info("Joint state loop stopped")

    def _control_loop(self):
        """
        Control loop for sending joint commands.

        This thread ONLY sends commands to the robot.
        Runs at control_frequency Hz.
        """
        period = 1.0 / self.config.control_frequency
        next_time = time.time()

        logger.info(f"Control loop started at {self.config.control_frequency}Hz")

        command_count = 0
        last_log_time = time.time()
        timeout_logged = False

        while self._running:
            try:
                current_time = time.time()

                # Read latest command from shared state
                with self._joint_cmd_lock:
                    if self.config.velocity_control:
                        joint_cmd = self._vel_cmd_
                    else:
                        joint_cmd = self._joint_cmd_
                    last_cmd_time = self._last_cmd_time

                # Check for timeout (1 second without new commands)
                time_since_last_cmd = current_time - last_cmd_time if last_cmd_time > 0 else 0

                if time_since_last_cmd > 1.0 and joint_cmd is not None:
                    if not timeout_logged:
                        logger.warning(
                            f"Command timeout: no commands received for {time_since_last_cmd:.2f}s. "
                            f"Stopping robot."
                        )
                        timeout_logged = True
                    # Send zero velocity to stop
                    if self.config.velocity_control:
                        zero_vel = [0.0] * self.config.num_joints
                        self.arm.vc_set_joint_velocity(
                            zero_vel, True, self.config.velocity_duration
                        )
                    continue
                else:
                    timeout_logged = False

                # Send command if available
                if joint_cmd is not None and len(joint_cmd) == self.config.num_joints:
                    if self.config.velocity_control:
                        # Velocity control mode (mode 4)
                        # NOTE: velocities are in degrees/second, not radians!
                        code = self.arm.vc_set_joint_velocity(
                            joint_cmd, False, self.config.velocity_duration
                        )
                    else:
                        # Position control mode (mode 1)
                        code = self.arm.set_servo_angle_j(
                            angles=joint_cmd, is_radian=self.config.is_radian
                        )

                    command_count += 1

                    # Log every second with detailed info
                    if current_time - last_log_time >= 1.0:
                        mode_str = "velocity" if self.config.velocity_control else "position"
                        logger.info(
                            f"Control loop ({mode_str}): sent {command_count} cmds in last second, "
                            f"state={self.curr_state}, mode={self.curr_mode}, "
                            f"joint6={math.degrees(joint_cmd[5]):.2f}°"
                        )
                        command_count = 0
                        last_log_time = current_time

                    if code != 0:
                        if code == 9:
                            logger.warning(
                                f"Command failed: not ready to move "
                                f"(code=9, state={self.curr_state}, mode={self.curr_mode}). "
                                f"Check robot mode and state."
                            )
                        else:
                            logger.warning(f"Command failed with code: {code}")

                # Maintain loop frequency
                next_time += period
                sleep_time = next_time - time.time()

                if sleep_time > 0:
                    if self._stop_event.wait(timeout=sleep_time):
                        break
                else:
                    logger.debug(f"Control loop overrun: {-sleep_time * 1000:.2f}ms")
                    next_time = time.time()

            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                time.sleep(period)

        logger.info("Control loop stopped")

    def _robot_state_loop(self):
        """
        Robot state publishing loop.

        Publishes robot state at robot_state_rate Hz (default 10Hz).
        Uses latest data from _report_data_callback (when available) or
        current state variables (curr_state, curr_mode, curr_err, etc.).
        """
        period = 1.0 / self.config.robot_state_rate
        next_time = time.time()

        logger.info(f"Robot state loop started at {self.config.robot_state_rate}Hz")

        while self._running:
            try:
                # Create robot state message from current state variables
                # These are updated by _report_data_callback when SDK pushes updates
                robot_state = RobotState(
                    state=self.curr_state,
                    mode=self.curr_mode,
                    error_code=self.curr_err,
                    warn_code=self.curr_warn,
                    cmdnum=self.curr_cmdnum,
                    mt_brake=0,  # Not always available, updated by callback
                    mt_able=0,  # Not always available, updated by callback
                )

                # Update shared state
                with self._joint_state_lock:
                    self._robot_state_ = robot_state

                # Publish robot state (only if transport is configured)
                if self.robot_state._transport or (
                    hasattr(self.robot_state, "connection") and self.robot_state.connection
                ):
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

    # =========================================================================
    # Private Methods: SDK Report Callback (Event-Driven)
    # =========================================================================

    def _report_data_callback(self, data: dict):
        """
        Callback invoked by xArm SDK when new report data is available.

        This runs periodically based on report_type:
        - 'dev': ~100Hz (high frequency for development)
        - 'rich': ~5Hz (includes extra data like torques)
        - 'normal': ~5Hz (basic state only)

        Data dictionary contains:
        - state: Robot state (0=ready, 3=pause, 4=stop)
        - mode: Control mode (0=position, 1=servo, etc.)
        - error_code: Error code
        - warn_code: Warning code
        - cmdnum: Command queue length
        - cartesian: TCP pose [x, y, z, roll, pitch, yaw]
        - mtbrake: Motor brake state
        - mtable: Motor enable state
        """
        # Debug: track callback invocations
        if not hasattr(self, "_report_callback_count"):
            self._report_callback_count = 0
        self._report_callback_count += 1

        if self._report_callback_count <= 3:
            logger.debug(f"_report_data_callback called #{self._report_callback_count}")

        try:
            # Update state tracking variables
            self.curr_state = data.get("state", self.curr_state)
            self.curr_err = data.get("error_code", 0)
            self.curr_mode = data.get("mode", self.curr_mode)
            self.curr_cmdnum = data.get("cmdnum", 0)
            self.curr_warn = data.get("warn_code", 0)

            # Create and publish RobotState
            robot_state = RobotState(
                state=self.curr_state,
                mode=self.curr_mode,
                error_code=self.curr_err,
                warn_code=self.curr_warn,
                cmdnum=self.curr_cmdnum,
                mt_brake=data.get("mtbrake", 0),
                mt_able=data.get("mtable", 0),
            )

            # Update shared state
            with self._joint_state_lock:
                self._robot_state_ = robot_state

            # Publish robot state (only if transport is configured)
            if self.robot_state._transport or (
                hasattr(self.robot_state, "connection") and self.robot_state.connection
            ):
                try:
                    self.robot_state.publish(robot_state)
                except Exception:
                    pass  # Transport error, skip publishing

            # Publish force/torque sensor data if available
            self._publish_ft_sensor_data()

        except Exception as e:
            logger.error(f"Error in report data callback: {e}")

    def _publish_ft_sensor_data(self):
        """Publish force/torque sensor data from SDK properties."""
        try:
            # External force (compensated) - ft_ext_force is a list property
            if hasattr(self.arm, "ft_ext_force") and len(self.arm.ft_ext_force) == 6:
                ft_ext_msg = WrenchStamped.from_force_torque_array(
                    ft_data=self.arm.ft_ext_force, frame_id="ft_sensor_ext", ts=time.time()
                )
                if self.ft_ext._transport or (
                    hasattr(self.ft_ext, "connection") and self.ft_ext.connection
                ):
                    try:
                        self.ft_ext.publish(ft_ext_msg)
                    except Exception:
                        pass

            # Raw force sensor data
            if hasattr(self.arm, "ft_raw_force") and len(self.arm.ft_raw_force) == 6:
                ft_raw_msg = WrenchStamped.from_force_torque_array(
                    ft_data=self.arm.ft_raw_force, frame_id="ft_sensor_raw", ts=time.time()
                )
                if self.ft_raw._transport or (
                    hasattr(self.ft_raw, "connection") and self.ft_raw.connection
                ):
                    try:
                        self.ft_raw.publish(ft_raw_msg)
                    except Exception:
                        pass

        except Exception as e:
            logger.debug(f"FT sensor data not available: {e}")

    # =========================================================================
    # RPC Methods: Control Commands
    # =========================================================================

    @rpc
    def set_joint_angles(self, angles: List[float]) -> Tuple[int, str]:
        """
        Set joint angles (RPC method).

        Args:
            angles: List of joint angles (in radians if is_radian=True)

        Returns:
            Tuple of (code, message)
        """
        try:
            code = self.arm.set_servo_angle_j(angles=angles, is_radian=self.config.is_radian)
            msg = "Success" if code == 0 else f"Error code: {code}"
            return (code, msg)
        except Exception as e:
            logger.error(f"set_joint_angles failed: {e}")
            return (-1, str(e))

    @rpc
    def set_joint_velocities(self, velocities: List[float]) -> Tuple[int, str]:
        """
        Set joint velocities (RPC method).
        Note: Requires velocity control mode.

        Args:
            velocities: List of joint velocities (rad/s)

        Returns:
            Tuple of (code, message)
        """
        try:
            # For velocity control, you would use vc_set_joint_velocity
            # This requires mode 4 (joint velocity control)
            code = self.arm.vc_set_joint_velocity(
                speeds=velocities, is_radian=self.config.is_radian
            )
            msg = "Success" if code == 0 else f"Error code: {code}"
            return (code, msg)
        except Exception as e:
            logger.error(f"set_joint_velocities failed: {e}")
            return (-1, str(e))

    @rpc
    def get_joint_state(self) -> Optional[JointState]:
        """
        Get the current joint state (RPC method).

        Returns:
            Current JointState or None
        """
        with self._joint_state_lock:
            return self._joint_states_

    @rpc
    def get_robot_state(self) -> Optional[RobotState]:
        """
        Get the current robot state (RPC method).

        Returns:
            Current RobotState or None
        """
        with self._joint_state_lock:
            return self._robot_state_

    @rpc
    def enable_servo_mode(self) -> Tuple[int, str]:
        """
        Enable servo mode (mode 1).
        Required for set_servo_angle_j to work.

        Returns:
            Tuple of (code, message)
        """
        try:
            code = self.arm.set_mode(1)
            if code == 0:
                logger.info("Servo mode enabled")
                return (code, "Servo mode enabled")
            else:
                logger.warning(f"Failed to enable servo mode: code={code}")
                return (code, f"Error code: {code}")
        except Exception as e:
            logger.error(f"enable_servo_mode failed: {e}")
            return (-1, str(e))

    @rpc
    def disable_servo_mode(self) -> Tuple[int, str]:
        """
        Disable servo mode (set to position mode).

        Returns:
            Tuple of (code, message)
        """
        try:
            code = self.arm.set_mode(0)
            if code == 0:
                logger.info("Servo mode disabled (position mode)")
                return (code, "Position mode enabled")
            else:
                logger.warning(f"Failed to disable servo mode: code={code}")
                return (code, f"Error code: {code}")
        except Exception as e:
            logger.error(f"disable_servo_mode failed: {e}")
            return (-1, str(e))

    @rpc
    def enable_velocity_control_mode(self) -> Tuple[int, str]:
        """
        Enable velocity control mode (mode 4).
        Required for vc_set_joint_velocity to work.

        Returns:
            Tuple of (code, message)
        """
        try:
            # Step 1: Set mode to 4 (velocity control)
            code = self.arm.set_mode(4)
            if code != 0:
                logger.warning(f"Failed to set mode to 4: code={code}")
                return (code, f"Failed to set mode: code={code}")

            # Step 2: Set state to 0 (ready/sport mode) - this activates the mode!
            code = self.arm.set_state(0)
            if code == 0:
                # Switch driver to velocity control
                self.config.velocity_control = True
                logger.info("Velocity control mode enabled (mode=4, state=0)")
                return (code, "Velocity control mode enabled")
            else:
                logger.warning(f"Failed to set state to 0: code={code}")
                return (code, f"Failed to set state: code={code}")
        except Exception as e:
            logger.error(f"enable_velocity_control_mode failed: {e}")
            return (-1, str(e))

    @rpc
    def disable_velocity_control_mode(self) -> Tuple[int, str]:
        """
        Disable velocity control mode and return to position control (mode 1).

        Returns:
            Tuple of (code, message)
        """
        try:
            # Step 1: Set state to 0 (sport mode) to allow mode change
            code = self.arm.set_state(0)
            if code != 0:
                logger.warning(f"Failed to set state to 0: code={code}")
                # Continue anyway, try to set mode

            # Step 2: Set mode to 1 (servo/position control)
            code = self.arm.set_mode(1)
            if code == 0:
                # Switch driver to position control
                self.config.velocity_control = False
                logger.info("Position control mode enabled (state=0, mode=1)")
                return (code, "Position control mode enabled")
            else:
                logger.warning(f"Failed to disable velocity control mode: code={code}")
                return (code, f"Error code: {code}")
        except Exception as e:
            logger.error(f"disable_velocity_control_mode failed: {e}")
            return (-1, str(e))

    # =========================================================================
    # RPC Methods: Additional xArm SDK API Functions
    # =========================================================================

    @rpc
    def motion_enable(self, enable: bool = True) -> Tuple[int, str]:
        """Enable or disable arm motion."""
        try:
            code = self.arm.motion_enable(enable=enable)
            msg = f"Motion {'enabled' if enable else 'disabled'}"
            return (code, msg if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_state(self, state: int) -> Tuple[int, str]:
        """
        Set robot state.

        Args:
            state: 0=ready, 3=pause, 4=stop
        """
        try:
            code = self.arm.set_state(state=state)
            return (code, "Success" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_joint_command(self, positions: List[float]) -> Tuple[int, str]:
        """
        Manually set the joint command (for testing).
        This updates the shared joint_cmd that the control loop reads.

        Args:
            positions: List of joint positions in radians

        Returns:
            Tuple of (code, message)
        """
        try:
            if len(positions) != self.config.num_joints:
                return (-1, f"Expected {self.config.num_joints} positions, got {len(positions)}")

            with self._joint_cmd_lock:
                self._joint_cmd_ = list(positions)

            logger.info(f"✓ Joint command set: {[f'{math.degrees(p):.2f}°' for p in positions]}")
            return (0, f"Joint command updated")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def clean_error(self) -> Tuple[int, str]:
        """Clear error codes."""
        try:
            code = self.arm.clean_error()
            return (code, "Errors cleared" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def clean_warn(self) -> Tuple[int, str]:
        """Clear warning codes."""
        try:
            code = self.arm.clean_warn()
            return (code, "Warnings cleared" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def get_position(self) -> Tuple[int, Optional[List[float]]]:
        """
        Get TCP position [x, y, z, roll, pitch, yaw].

        Returns:
            Tuple of (code, position)
        """
        try:
            code, position = self.arm.get_position(is_radian=self.config.is_radian)
            return (code, list(position) if code == 0 else None)
        except Exception as e:
            logger.error(f"get_position failed: {e}")
            return (-1, None)

    @rpc
    def set_position(self, position: List[float], wait: bool = False) -> Tuple[int, str]:
        """
        Set TCP position [x, y, z, roll, pitch, yaw].

        Args:
            position: Target position
            wait: Wait for motion to complete

        Returns:
            Tuple of (code, message)
        """
        try:
            code = self.arm.set_position(*position, is_radian=self.config.is_radian, wait=wait)
            return (code, "Success" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def move_gohome(self, wait: bool = False) -> Tuple[int, str]:
        """Move to home position."""
        try:
            code = self.arm.move_gohome(wait=wait, is_radian=self.config.is_radian)
            return (code, "Moving home" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def emergency_stop(self) -> Tuple[int, str]:
        """Emergency stop the arm."""
        try:
            code = self.arm.emergency_stop()
            return (code, "Emergency stop" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def get_version(self) -> Tuple[int, Optional[str]]:
        """Get firmware version."""
        try:
            code, version = self.arm.get_version()
            return (code, version if code == 0 else None)
        except Exception as e:
            return (-1, None)

    @rpc
    def get_inverse_kinematics(self, pose: List[float]) -> Tuple[int, Optional[List[float]]]:
        """
        Compute inverse kinematics.

        Args:
            pose: [x, y, z, roll, pitch, yaw]

        Returns:
            Tuple of (code, joint_angles)
        """
        try:
            code, angles = self.arm.get_inverse_kinematics(
                pose, input_is_radian=self.config.is_radian, return_is_radian=self.config.is_radian
            )
            return (code, list(angles) if code == 0 else None)
        except Exception as e:
            return (-1, None)

    @rpc
    def get_forward_kinematics(self, angles: List[float]) -> Tuple[int, Optional[List[float]]]:
        """
        Compute forward kinematics.

        Args:
            angles: Joint angles

        Returns:
            Tuple of (code, pose)
        """
        try:
            code, pose = self.arm.get_forward_kinematics(
                angles,
                input_is_radian=self.config.is_radian,
                return_is_radian=self.config.is_radian,
            )
            return (code, list(pose) if code == 0 else None)
        except Exception as e:
            return (-1, None)
