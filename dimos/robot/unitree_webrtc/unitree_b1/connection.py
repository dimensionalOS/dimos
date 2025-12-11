#!/usr/bin/env python3
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

# Copyright 2025 Dimensional Inc.

"""B1 Connection Module that accepts standard Twist commands and converts to UDP packets."""

import logging
import socket
import threading
import time

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Twist, TwistStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.std_msgs import Int32
from dimos.utils.logging_config import setup_logger

from .b1_command import B1Command

# Setup logger with DEBUG level for troubleshooting
logger = setup_logger(level=logging.DEBUG)


class RobotMode:
    """Constants for B1 robot modes."""

    IDLE = 0
    STAND = 1
    WALK = 2
    RECOVERY = 6


class B1ConnectionModule(Module):
    """UDP connection module for B1 robot with standard Twist interface.

    Accepts standard ROS Twist messages on /cmd_vel and mode changes on /b1/mode,
    internally converts to B1Command format, and sends UDP packets at 50Hz.
    """

    cmd_vel: In[TwistStamped]  # Timestamped velocity commands from ROS
    mode_cmd: In[Int32]  # Mode changes
    odom_in: In[Odometry]  # External odometry from ROS SLAM/lidar

    odom_pose: Out[PoseStamped]  # Converted pose for internal use

    def __init__(  # type: ignore[no-untyped-def]
        self, ip: str = "192.168.12.1", port: int = 9090, test_mode: bool = False, *args, **kwargs
    ) -> None:
        """Initialize B1 connection module.

        Args:
            ip: Robot IP address
            port: UDP port for joystick server
            test_mode: If True, print commands instead of sending UDP
        """
        Module.__init__(self, *args, **kwargs)

        self.ip = ip
        self.port = port
        self.test_mode = test_mode
        self.current_mode = RobotMode.IDLE  # Start in IDLE mode
        self._current_cmd = B1Command(mode=RobotMode.IDLE)
        self.cmd_lock = threading.Lock()  # Thread lock for _current_cmd access
        # Thread control
        self.running = False
        self.send_thread = None
        self.socket = None
        self.packet_count = 0
        self.last_command_time = time.time()
        self.command_timeout = 0.2  # 200ms safety timeout
        self.watchdog_thread = None
        self.watchdog_running = False
        self.timeout_active = False

    @rpc
    def start(self) -> None:
        """Start the connection and subscribe to command streams."""

        super().start()

        # Setup UDP socket (unless in test mode)
        if not self.test_mode:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # type: ignore[assignment]
            logger.info(f"B1 Connection started - UDP to {self.ip}:{self.port} at 50Hz")
        else:
            logger.info(f"[TEST MODE] B1 Connection started - would send to {self.ip}:{self.port}")

        # Subscribe to input streams
        if self.cmd_vel:
            unsub = self.cmd_vel.subscribe(self.handle_twist_stamped)
            self._disposables.add(Disposable(unsub))
        if self.mode_cmd:
            unsub = self.mode_cmd.subscribe(self.handle_mode)
            self._disposables.add(Disposable(unsub))
        if self.odom_in:
            unsub = self.odom_in.subscribe(self._publish_odom_pose)
            self._disposables.add(Disposable(unsub))

        # Start threads
        self.running = True
        self.watchdog_running = True

        # Start 50Hz sending thread
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)  # type: ignore[assignment]
        self.send_thread.start()  # type: ignore[attr-defined]

        # Start watchdog thread
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)  # type: ignore[assignment]
        self.watchdog_thread.start()  # type: ignore[attr-defined]

    @rpc
    def stop(self) -> None:
        """Stop the connection and send stop commands."""

        self.set_mode(RobotMode.IDLE)  # IDLE
        with self.cmd_lock:
            self._current_cmd = B1Command(mode=RobotMode.IDLE)  # Zero all velocities

        # Send multiple stop packets
        if not self.test_mode and self.socket:
            stop_cmd = B1Command(mode=RobotMode.IDLE)
            for _ in range(5):
                data = stop_cmd.to_bytes()
                self.socket.sendto(data, (self.ip, self.port))
                time.sleep(0.02)

        self.running = False
        self.watchdog_running = False

        if self.send_thread:
            self.send_thread.join(timeout=0.5)
        if self.watchdog_thread:
            self.watchdog_thread.join(timeout=0.5)

        if self.socket:
            self.socket.close()
            self.socket = None

        super().stop()

    def handle_twist_stamped(self, twist_stamped: TwistStamped) -> None:
        """Handle timestamped Twist message and convert to B1Command.

        This is called automatically when messages arrive on cmd_vel input.
        """
        # Extract Twist from TwistStamped
        twist = Twist(linear=twist_stamped.linear, angular=twist_stamped.angular)

        logger.debug(
            f"Received cmd_vel: linear=({twist.linear.x:.3f}, {twist.linear.y:.3f}, {twist.linear.z:.3f}), angular=({twist.angular.x:.3f}, {twist.angular.y:.3f}, {twist.angular.z:.3f})"
        )

        # In STAND mode, all twist values control body pose, not movement
        # W/S: height (linear.z), A/D: yaw (angular.z), J/L: roll (angular.x), I/K: pitch (angular.y)
        if self.current_mode == RobotMode.STAND:
            # In STAND mode, don't auto-switch since all inputs are valid body pose controls
            has_movement = False
        else:
            # In other modes, consider linear x/y and angular.z as movement
            has_movement = (
                abs(twist.linear.x) > 0.01
                or abs(twist.linear.y) > 0.01
                or abs(twist.angular.z) > 0.01
            )

        if has_movement and self.current_mode not in (RobotMode.STAND, RobotMode.WALK):
            logger.info("Auto-switching to WALK mode for ROS control")
            self.set_mode(RobotMode.WALK)
        elif not has_movement and self.current_mode == RobotMode.WALK:
            logger.info("Auto-switching to IDLE mode (zero velocities)")
            self.set_mode(RobotMode.IDLE)

        if self.test_mode:
            logger.info(
                f"[TEST] Received TwistStamped: linear=({twist.linear.x:.2f}, {twist.linear.y:.2f}), angular.z={twist.angular.z:.2f}"
            )

        with self.cmd_lock:
            self._current_cmd = B1Command.from_twist(twist, self.current_mode)

        logger.debug(f"Converted to B1Command: {self._current_cmd}")

        self.last_command_time = time.time()
        self.timeout_active = False  # Reset timeout state since we got a new command

    def handle_mode(self, mode_msg: Int32) -> None:
        """Handle mode change message.

        This is called automatically when messages arrive on mode_cmd input.
        """
        logger.debug(f"Received mode change: {mode_msg.data}")
        if self.test_mode:
            logger.info(f"[TEST] Received mode change: {mode_msg.data}")
        self.set_mode(mode_msg.data)

    @rpc
    def set_mode(self, mode: int) -> bool:
        """Set robot mode (0=idle, 1=stand, 2=walk, 6=recovery)."""
        self.current_mode = mode
        with self.cmd_lock:
            self._current_cmd.mode = mode

            # Clear velocities when not in walk mode
            if mode != RobotMode.WALK:
                self._current_cmd.lx = 0.0
                self._current_cmd.ly = 0.0
                self._current_cmd.rx = 0.0
                self._current_cmd.ry = 0.0

        mode_names = {
            RobotMode.IDLE: "IDLE",
            RobotMode.STAND: "STAND",
            RobotMode.WALK: "WALK",
            RobotMode.RECOVERY: "RECOVERY",
        }
        logger.info(f"Mode changed to: {mode_names.get(mode, mode)}")
        if self.test_mode:
            logger.info(f"[TEST] Mode changed to: {mode_names.get(mode, mode)}")

        return True

    def _send_loop(self) -> None:
        """Continuously send current command at 50Hz.

        The watchdog thread handles timeout and zeroing commands, so this loop
        just sends whatever is in self._current_cmd at 50Hz.
        """
        while self.running:
            try:
                # Watchdog handles timeout, we just send current command
                with self.cmd_lock:
                    cmd_to_send = self._current_cmd

                # Log status every second (50 packets)
                if self.packet_count % 50 == 0:
                    logger.info(
                        f"Sending B1 commands at 50Hz | Mode: {self.current_mode} | Count: {self.packet_count}"
                    )
                    if not self.test_mode:
                        logger.debug(f"Current B1Command: {self._current_cmd}")
                        data = cmd_to_send.to_bytes()
                        hex_str = " ".join(f"{b:02x}" for b in data)
                        logger.debug(f"UDP packet ({len(data)} bytes): {hex_str}")

                if self.socket:
                    data = cmd_to_send.to_bytes()
                    self.socket.sendto(data, (self.ip, self.port))

                self.packet_count += 1

                # 50Hz rate (20ms between packets)
                time.sleep(0.020)

            except Exception as e:
                if self.running:
                    logger.error(f"Send error: {e}")

    def _publish_odom_pose(self, msg: Odometry) -> None:
        """Convert and publish odometry as PoseStamped.

        This matches G1's approach of receiving external odometry.
        """
        if self.odom_pose:
            pose_stamped = PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.pose.pose.position,
                orientation=msg.pose.pose.orientation,
            )
            self.odom_pose.publish(pose_stamped)

    def _watchdog_loop(self) -> None:
        """Single watchdog thread that monitors command freshness."""
        while self.watchdog_running:
            try:
                time_since_last_cmd = time.time() - self.last_command_time

                if time_since_last_cmd > self.command_timeout:
                    if not self.timeout_active:
                        # First time detecting timeout
                        logger.warning(
                            f"Watchdog timeout ({time_since_last_cmd:.1f}s) - zeroing commands"
                        )
                        if self.test_mode:
                            logger.info("[TEST] Watchdog timeout - zeroing commands")

                        with self.cmd_lock:
                            self._current_cmd.lx = 0.0
                            self._current_cmd.ly = 0.0
                            self._current_cmd.rx = 0.0
                            self._current_cmd.ry = 0.0

                        self.timeout_active = True
                else:
                    if self.timeout_active:
                        logger.info("Watchdog: Commands resumed - control restored")
                        if self.test_mode:
                            logger.info("[TEST] Watchdog: Commands resumed")
                        self.timeout_active = False

                # Check every 50ms
                time.sleep(0.05)

            except Exception as e:
                if self.watchdog_running:
                    logger.error(f"Watchdog error: {e}")

    @rpc
    def idle(self) -> bool:
        """Set robot to idle mode."""
        self.set_mode(RobotMode.IDLE)
        return True

    @rpc
    def pose(self) -> bool:
        """Set robot to stand/pose mode for reaching ground objects with manipulator."""
        self.set_mode(RobotMode.STAND)
        return True

    @rpc
    def walk(self) -> bool:
        """Set robot to walk mode."""
        self.set_mode(RobotMode.WALK)
        return True

    @rpc
    def recovery(self) -> bool:
        """Set robot to recovery mode."""
        self.set_mode(RobotMode.RECOVERY)
        return True

    @rpc
    def move(self, twist_stamped: TwistStamped, duration: float = 0.0) -> bool:
        """Direct RPC method for sending TwistStamped commands.

        Args:
            twist_stamped: Timestamped velocity command
            duration: Not used, kept for compatibility
        """
        self.handle_twist_stamped(twist_stamped)
        return True


class MockB1ConnectionModule(B1ConnectionModule):
    """Test connection module that prints commands instead of sending UDP."""

    def __init__(self, ip: str = "127.0.0.1", port: int = 9090, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        """Initialize test connection without creating socket."""
        super().__init__(ip, port, test_mode=True, *args, **kwargs)  # type: ignore[misc]

    def _send_loop(self) -> None:
        """Override to provide better test output with timeout detection."""
        timeout_warned = False

        while self.running:
            time_since_last_cmd = time.time() - self.last_command_time
            is_timeout = time_since_last_cmd > self.command_timeout

            # Show timeout transitions
            if is_timeout and not timeout_warned:
                logger.info(
                    f"[TEST] Command timeout! Sending zeros after {time_since_last_cmd:.1f}s"
                )
                timeout_warned = True
            elif not is_timeout and timeout_warned:
                logger.info("[TEST] Commands resumed - control restored")
                timeout_warned = False

            # Print current state every 0.5 seconds
            if self.packet_count % 25 == 0:
                if is_timeout:
                    logger.info(f"[TEST] B1Cmd[ZEROS] (timeout) | Count: {self.packet_count}")
                else:
                    logger.info(f"[TEST] {self._current_cmd} | Count: {self.packet_count}")

            self.packet_count += 1
            time.sleep(0.020)

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()
