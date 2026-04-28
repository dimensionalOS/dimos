#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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
Arrow key control for G1 robot.
Use arrow keys and WASD for real-time robot control.
"""

import curses
import time
from typing import Any

from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk


def draw_ui(stdscr: Any, state_text: str = "Not connected") -> None:
    """Draw the control UI."""
    stdscr.clear()
    height, width = stdscr.getmaxyx()

    # Title
    title = "🤖 G1 Arrow Key Control"
    stdscr.addstr(0, (width - len(title)) // 2, title, curses.A_BOLD)

    # Controls
    controls = [
        "",
        "Movement Controls:",
        "  ↑/W     - Move forward",
        "  ↓/S     - Move backward",
        "  ←/A     - Rotate left",
        "  →/D     - Rotate right",
        "  Q       - Strafe left",
        "  E       - Strafe right",
        "  SPACE   - Stop",
        "",
        "Robot Controls:",
        "  1       - Stand up",
        "  2       - Lie down",
        "  R       - Show robot state",
        "",
        "  ESC/Ctrl+C - Quit",
        "",
        f"Status: {state_text}",
    ]

    start_row = 2
    for i, line in enumerate(controls):
        if i < height - 1:
            stdscr.addstr(start_row + i, 2, line)

    stdscr.refresh()


def main(stdscr: Any) -> None:
    # Setup curses
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(1)  # Non-blocking input
    stdscr.timeout(100)  # 100ms timeout for getch()

    draw_ui(stdscr, "Initializing...")

    # Initialize connection
    conn = G1HighLevelDdsSdk(network_interface="eth0")
    conn.start()
    time.sleep(1)

    draw_ui(stdscr, "✓ Connected - Ready for commands")

    # Movement parameters
    linear_speed = 0.3  # m/s for forward/backward/strafe
    angular_speed = 0.5  # rad/s for rotation
    move_duration = 0.2  # Duration of each movement pulse

    try:
        last_cmd_time = 0.0
        cmd_cooldown = 0.15  # Minimum time between commands

        while True:
            key = stdscr.getch()
            current_time = time.time()

            # Skip if in cooldown period
            if current_time - last_cmd_time < cmd_cooldown:
                continue

            if key == -1:  # No key pressed
                continue

            # Handle quit
            if key == 27 or key == 3:  # ESC or Ctrl+C
                break

            # Convert key to character
            try:
                key_char = chr(key).lower() if key < 256 else None
            except ValueError:
                key_char = None

            # Movement commands
            twist = None
            action = None

            # Arrow keys
            if key == curses.KEY_UP or key_char == "w":
                twist = Twist(linear=Vector3(linear_speed, 0, 0), angular=Vector3(0, 0, 0))
                action = "Moving forward..."
            elif key == curses.KEY_DOWN or key_char == "s":
                twist = Twist(linear=Vector3(-linear_speed, 0, 0), angular=Vector3(0, 0, 0))
                action = "Moving backward..."
            elif key == curses.KEY_LEFT or key_char == "a":
                twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, angular_speed))
                action = "Rotating left..."
            elif key == curses.KEY_RIGHT or key_char == "d":
                twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -angular_speed))
                action = "Rotating right..."
            elif key_char == "q":
                twist = Twist(linear=Vector3(0, linear_speed, 0), angular=Vector3(0, 0, 0))
                action = "Strafing left..."
            elif key_char == "e":
                twist = Twist(linear=Vector3(0, -linear_speed, 0), angular=Vector3(0, 0, 0))
                action = "Strafing right..."
            elif key_char == " ":
                conn.move(
                    Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)), duration=move_duration
                )
                action = "🛑 Stopped"
                last_cmd_time = current_time

            # Robot state commands
            elif key_char == "1":
                draw_ui(stdscr, "Standing up...")
                conn.stand_up()
                action = "✓ Standup complete"
                last_cmd_time = current_time
            elif key_char == "2":
                draw_ui(stdscr, "Lying down...")
                conn.lie_down()
                action = "✓ Liedown complete"
                last_cmd_time = current_time
            elif key_char == "r":
                state = conn.get_state()
                action = f"State: {state}"
                last_cmd_time = current_time

            # Execute movement
            if twist is not None:
                conn.move(twist, duration=move_duration)
                last_cmd_time = current_time

            # Update UI with action
            if action:
                draw_ui(stdscr, action)

    except KeyboardInterrupt:
        pass
    finally:
        draw_ui(stdscr, "Stopping and disconnecting...")
        conn.disconnect()
        draw_ui(stdscr, "✓ Disconnected")
        time.sleep(1)


if __name__ == "__main__":
    print("\n⚠️  WARNING: Ensure area is clear around robot!")
    print("Starting in 3 seconds...")
    time.sleep(3)

    try:
        curses.wrapper(main)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback

        traceback.print_exc()

    print("\n✓ Done")
