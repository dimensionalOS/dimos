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

"""
UDP Joystick Control Client for Unitree B1 Robot
Sends joystick commands via UDP datagrams - no fragmentation issues!
"""

import socket
import struct
import sys
import time
import termios
import tty
import select
import signal


class NetworkJoystickCmd:
    """Structure matching the C++ NetworkJoystickCmd"""

    # Format: 4 floats (lx, ly, rx, ry), 1 uint16 (buttons), 1 uint8 (mode)
    # Little-endian byte order to match x86/ARM
    FORMAT = "<ffffHB"  # 19 bytes total
    SIZE = struct.calcsize(FORMAT)

    def __init__(self):
        self.lx = 0.0  # left stick x (-1 to 1)
        self.ly = 0.0  # left stick y (-1 to 1)
        self.rx = 0.0  # right stick x (-1 to 1)
        self.ry = 0.0  # right stick y (-1 to 1)
        self.buttons = 0  # button states (uint16)
        self.mode = 0  # control mode (uint8)

    def pack(self):
        """Pack the structure into bytes matching C++ memory layout"""
        return struct.pack(self.FORMAT, self.lx, self.ly, self.rx, self.ry, self.buttons, self.mode)

    def reset_sticks(self):
        """Reset stick values (they don't hold position)"""
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0


class JoystickClient:
    """UDP client for sending joystick commands to the robot server"""

    def __init__(self, server_ip, server_port, test_mode=False, verbose=False):
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.old_settings = None
        self.running = False
        self.active_keys = set()  # Track all currently active keys
        self.test_mode = test_mode
        self.verbose = verbose  # Show packet details when running real
        self.packet_count = 0

    def connect(self):
        """Create UDP socket (no actual connection needed)"""
        try:
            # Create UDP socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # Set socket to non-blocking for potential future enhancements
            # self.socket.setblocking(0)

            if self.test_mode:
                print(f"TEST MODE: Created UDP socket")
                print(f"Will send to: {self.server_ip}:{self.server_port}")
                print(f"Packet size: {NetworkJoystickCmd.SIZE} bytes")
            else:
                print(f"UDP client ready")
                print(f"Target server: {self.server_ip}:{self.server_port}")
                print(f"Packet size: {NetworkJoystickCmd.SIZE} bytes")
                print(f"Protocol: UDP (fire-and-forget)")

            return True
        except Exception as e:
            print(f"Failed to create UDP socket: {e}")
            if self.socket:
                self.socket.close()
                self.socket = None
            return False

    def disconnect(self):
        """Close the UDP socket"""
        if self.socket:
            self.socket.close()
            self.socket = None

    def send_command(self, cmd):
        """Send a joystick command via UDP datagram"""
        if not self.socket and not self.test_mode:
            return False

        try:
            data = cmd.pack()
            self.packet_count += 1

            # Verify packet size
            if len(data) != 19:
                print(f"ERROR: Packet size is {len(data)}, expected 19")
                return False

            if self.test_mode:
                # Test mode - clean output
                if cmd.lx != 0 or cmd.ly != 0 or cmd.rx != 0 or cmd.ry != 0:
                    # Movement active
                    sys.stdout.write(
                        f"\r[Moving] Mode: {cmd.mode} | LX: {cmd.lx:+.1f} LY: {cmd.ly:+.1f} | RX: {cmd.rx:+.1f} RY: {cmd.ry:+.1f}     "
                    )
                    sys.stdout.flush()
                elif self.packet_count % 50 == 0:
                    # Idle status update
                    sys.stdout.write(
                        f"\r[Idle] Mode: {cmd.mode} | Packets: {self.packet_count}     "
                    )
                    sys.stdout.flush()
                return True

            # Send UDP datagram (fire and forget)
            bytes_sent = self.socket.sendto(data, (self.server_ip, self.server_port))

            # Show real-time status when verbose mode is enabled (same format as test mode)
            if self.verbose:
                if cmd.lx != 0 or cmd.ly != 0 or cmd.rx != 0 or cmd.ry != 0:
                    # Movement active
                    sys.stdout.write(
                        f"\r[Moving] Mode: {cmd.mode} | LX: {cmd.lx:+.1f} LY: {cmd.ly:+.1f} | RX: {cmd.rx:+.1f} RY: {cmd.ry:+.1f}     "
                    )
                    sys.stdout.flush()
                elif self.packet_count % 50 == 0:
                    # Idle status update
                    sys.stdout.write(
                        f"\r[Idle] Mode: {cmd.mode} | Packets: {self.packet_count}     "
                    )
                    sys.stdout.flush()
            elif self.packet_count % 50 == 0:
                # Non-verbose: just show packet count every second
                print(f"Sent {self.packet_count} packets ({self.packet_count * 0.02:.1f}s)")

            return bytes_sent == NetworkJoystickCmd.SIZE

        except Exception as e:
            print(f"\nFailed to send UDP packet: {e}")
            return False

    def set_nonblocking_input(self):
        """Set terminal to non-blocking input mode"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    def restore_input(self):
        """Restore terminal settings"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self, timeout=0):
        """Get a single keypress without blocking"""
        if select.select([sys.stdin], [], [], timeout)[0]:
            key = sys.stdin.read(1)
            # Check for Ctrl+C (ASCII code 3)
            if ord(key) == 3:
                print("\nCtrl+C pressed - exiting...")
                self.running = False
                return None
            return key
        return None

    def run_keyboard_control(self):
        """Main control loop with keyboard input"""
        self.set_nonblocking_input()
        self.running = True

        cmd = NetworkJoystickCmd()

        print("\n" + "=" * 50)
        print("UDP Keyboard Control (No TCP fragmentation!)")
        print("=" * 50)
        print("Modes: 0=Idle, 1=Stand, 2=Walk, 6=Recovery")
        print("WASD = Movement/Turning (left stick)")
        print("IJKL = Strafing/Pitch (right stick)")
        print("Space = Emergency Stop")
        print("Q = Quit")
        print("\nHOLD keys for continuous movement")
        print("Multiple keys work together (e.g., W+A = forward + turn)")
        print(f"\nCurrent mode: {cmd.mode}")

        try:
            while self.running:
                # Check for any key presses
                key = self.get_key()
                if not self.running:  # Ctrl+C was pressed in get_key()
                    break
                key_lower = key.lower() if key else None

                if key:
                    # Mode selection (immediate, not held)
                    if key == "0":
                        cmd.mode = 0
                        print(f"\rMode: Idle     ", end="", flush=True)
                    elif key == "1":
                        cmd.mode = 1
                        print(f"\rMode: Stand    ", end="", flush=True)
                    elif key == "2":
                        cmd.mode = 2
                        print(f"\rMode: Walk     ", end="", flush=True)
                    elif key == "6":
                        cmd.mode = 6
                        print(f"\rMode: Recovery ", end="", flush=True)
                    elif key_lower == "q":
                        print("\nQuitting...")
                        self.running = False
                        break  # Break immediately
                    elif key == " ":  # Space bar = emergency stop
                        cmd.mode = 0
                        print(f"\rEMERGENCY STOP ", end="", flush=True)

                # Reset stick values EVERY frame
                cmd.reset_sticks()

                # Only apply movement for CURRENT key press (no toggle, no persistence)
                if key_lower == "w":
                    cmd.ly = 1.0  # Forward
                if key_lower == "s":
                    cmd.ly = -1.0  # Backward
                if key_lower == "a":
                    cmd.lx = -1.0  # Turn/yaw left
                if key_lower == "d":
                    cmd.lx = 1.0  # Turn/yaw right
                if key_lower == "i":
                    cmd.ry = 1.0  # Pitch up
                if key_lower == "k":
                    cmd.ry = -1.0  # Pitch down
                if key_lower == "j":
                    cmd.rx = -1.0  # Roll/strafe left
                if key_lower == "l":
                    cmd.rx = 1.0  # Roll/strafe right

                # Send UDP packet at 50Hz
                if not self.send_command(cmd):
                    if not self.test_mode:
                        print("\nFailed to send UDP packet")
                        break

                # Maintain 50Hz update rate (20ms between packets)
                time.sleep(0.020)

        except KeyboardInterrupt:
            print("\nCtrl+C received, shutting down...")
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            # Send final idle command for safety
            print("\nSending idle command...")
            final_cmd = NetworkJoystickCmd()
            final_cmd.mode = 0
            for _ in range(5):  # Send multiple times to ensure receipt
                self.send_command(final_cmd)
                time.sleep(0.02)

            self.restore_input()
            print("Terminal restored")
            print(f"Total packets sent: {self.packet_count}")

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        self.running = False


def main():
    # Check for test mode
    test_mode = "--test" in sys.argv
    if test_mode:
        sys.argv.remove("--test")
        # Use dummy values if not provided
        if len(sys.argv) < 3:
            sys.argv.extend(["127.0.0.1", "9090"])

    # Check for verbose mode
    verbose = "--verbose" in sys.argv or "-v" in sys.argv
    if "--verbose" in sys.argv:
        sys.argv.remove("--verbose")
    if "-v" in sys.argv:
        sys.argv.remove("-v")

    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} [--test] [--verbose|-v] <server_ip> <server_port>")
        print(f"Example: {sys.argv[0]} 192.168.123.220 9090")
        print(f"Verbose: {sys.argv[0]} 192.168.123.220 9090 --verbose")
        print(f"Test mode: {sys.argv[0]} --test")
        return 1

    server_ip = sys.argv[1]
    server_port = int(sys.argv[2])

    print(f"\n{'=' * 50}")
    print(f"UDP Joystick Client for Unitree B1")
    if verbose:
        print(f"VERBOSE MODE: Showing all packet details")
    print(f"{'=' * 50}")

    client = JoystickClient(server_ip, server_port, test_mode=test_mode, verbose=verbose)

    # Set up signal handler for clean shutdown
    signal.signal(signal.SIGINT, client.signal_handler)
    signal.signal(signal.SIGTERM, client.signal_handler)

    if not client.connect():
        return 1

    try:
        client.run_keyboard_control()
    finally:
        client.disconnect()
        if not test_mode:
            print("UDP socket closed")

    return 0


if __name__ == "__main__":
    sys.exit(main())
