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

import socket
import threading
import time
from typing import Optional

from dimos.core import In, Module, rpc
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.std_msgs import Int32
from .b1_command import B1Command


class B1ConnectionModule(Module):
    """UDP connection module for B1 robot with standard Twist interface.

    Accepts standard ROS Twist messages on /cmd_vel and mode changes on /b1/mode,
    internally converts to B1Command format, and sends UDP packets at 50Hz.
    """

    # Module inputs
    cmd_vel: In[Twist] = None  # Standard velocity commands
    mode_cmd: In[Int32] = None  # Mode changes

    def __init__(
        self, ip: str = "192.168.12.1", port: int = 9090, test_mode: bool = False, *args, **kwargs
    ):
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
        self.current_mode = 0  # Start in IDLE mode for safety

        # Internal state as B1Command
        self._current_cmd = B1Command(mode=0)

        # Thread control
        self.running = False
        self.send_thread = None

        # UDP socket (created in start)
        self.socket = None

        # Packet counter for debugging
        self.packet_count = 0

    @rpc
    def start(self):
        """Start the connection and subscribe to command streams."""

        # Setup UDP socket (unless in test mode)
        if not self.test_mode:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"B1 Connection started - UDP to {self.ip}:{self.port} at 50Hz")
        else:
            print(f"[TEST MODE] B1 Connection started - would send to {self.ip}:{self.port}")

        # Subscribe to input streams
        if self.cmd_vel:
            self.cmd_vel.subscribe(self.handle_twist)
        if self.mode_cmd:
            self.mode_cmd.subscribe(self.handle_mode)

        # Start 50Hz sending thread
        self.running = True
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

        return True

    @rpc
    def stop(self):
        """Stop the connection and send stop commands."""
        # Send stop command multiple times for safety
        self.set_mode(0)  # IDLE mode
        self._current_cmd = B1Command(mode=0)  # Zero all velocities

        # Send multiple stop packets to ensure receipt
        if not self.test_mode and self.socket:
            stop_cmd = B1Command(mode=0)
            for _ in range(5):
                data = stop_cmd.to_bytes()
                self.socket.sendto(data, (self.ip, self.port))
                time.sleep(0.02)

        # Stop thread
        self.running = False
        if self.send_thread:
            self.send_thread.join(timeout=0.5)

        # Close socket
        if self.socket:
            self.socket.close()
            self.socket = None

        return True

    def handle_twist(self, twist: Twist):
        """Handle standard Twist message and convert to B1Command.

        This is called automatically when messages arrive on cmd_vel input.
        """
        if self.test_mode:
            print(
                f"[TEST] Received Twist: linear=({twist.linear.x:.2f}, {twist.linear.y:.2f}), angular.z={twist.angular.z:.2f}"
            )
        # Convert Twist to B1Command using current mode
        self._current_cmd = B1Command.from_twist(twist, self.current_mode)

    def handle_mode(self, mode_msg: Int32):
        """Handle mode change message.

        This is called automatically when messages arrive on mode_cmd input.
        """
        if self.test_mode:
            print(f"[TEST] Received mode change: {mode_msg.data}")
        self.set_mode(mode_msg.data)

    @rpc
    def set_mode(self, mode: int):
        """Set robot mode (0=idle, 1=stand, 2=walk, 6=recovery)."""
        self.current_mode = mode
        self._current_cmd.mode = mode

        # Clear velocities when not in walk mode
        if mode != 2:
            self._current_cmd.lx = 0.0
            self._current_cmd.ly = 0.0
            self._current_cmd.rx = 0.0
            self._current_cmd.ry = 0.0

        if self.test_mode:
            mode_names = {0: "IDLE", 1: "STAND", 2: "WALK", 6: "RECOVERY"}
            print(f"[TEST] Mode changed to: {mode_names.get(mode, mode)}")

        return True

    def _send_loop(self):
        """Continuously send current command at 50Hz."""
        while self.running:
            try:
                if self.test_mode:
                    # Test mode - print status periodically
                    if self.packet_count % 50 == 0:  # Every second
                        print(f"[TEST] {self._current_cmd} | Packets: {self.packet_count}")
                else:
                    # Real mode - send UDP packet
                    if self.socket:
                        data = self._current_cmd.to_bytes()
                        self.socket.sendto(data, (self.ip, self.port))

                self.packet_count += 1

                # Maintain 50Hz rate (20ms between packets)
                time.sleep(0.020)

            except Exception as e:
                if self.running:
                    print(f"Send error: {e}")

    # Direct RPC methods for mode control
    @rpc
    def idle(self):
        """Set robot to idle mode."""
        self.set_mode(0)
        return True

    @rpc
    def stand(self):
        """Set robot to stand mode."""
        self.set_mode(1)
        return True

    @rpc
    def walk(self):
        """Set robot to walk mode."""
        self.set_mode(2)
        return True

    @rpc
    def recovery(self):
        """Set robot to recovery mode."""
        self.set_mode(6)
        return True

    @rpc
    def move(self, twist: Twist, duration: float = 0.0):
        """Direct RPC method for sending Twist commands.

        Args:
            twist: Velocity command
            duration: Not used, kept for compatibility
        """
        self.handle_twist(twist)
        return True

    def cleanup(self):
        """Clean up resources when module is destroyed."""
        self.stop()


class TestB1ConnectionModule(B1ConnectionModule):
    """Test connection module that prints commands instead of sending UDP."""

    def __init__(self, ip: str = "127.0.0.1", port: int = 9090, *args, **kwargs):
        """Initialize test connection without creating socket."""
        super().__init__(ip, port, test_mode=True, *args, **kwargs)

    def _send_loop(self):
        """Override to provide better test output."""
        while self.running:
            # Print current state every 0.5 seconds (not every packet)
            if self.packet_count % 25 == 0:
                print(f"[TEST] {self._current_cmd} | Count: {self.packet_count}")

            self.packet_count += 1
            time.sleep(0.020)
