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

"""Arrow key + WASD real-time control for G1, publishing Twist on /cmd_vel."""

import curses
import time
from typing import Any

import lcm

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk

CMD_VEL_CHANNEL = "/cmd_vel#geometry_msgs.Twist"


def publish_twist(lc: lcm.LCM, twist: Twist) -> None:
    lc.publish(CMD_VEL_CHANNEL, twist.lcm_encode())


def draw_ui(stdscr: Any, state_text: str = "Not connected") -> None:
    stdscr.clear()
    height, width = stdscr.getmaxyx()

    title = "G1 Arrow Key Control (cmd_vel)"
    stdscr.addstr(0, (width - len(title)) // 2, title, curses.A_BOLD)

    controls = [
        "",
        "Movement Controls:",
        "  UP/W    - Move forward",
        "  DOWN/S  - Move backward",
        "  LEFT/A  - Rotate left",
        "  RIGHT/D - Rotate right",
        "  Q       - Strafe left",
        "  E       - Strafe right",
        "  SPACE   - Stop",
        "",
        "Robot Controls:",
        "  1       - Stand up",
        "  2       - Lie down",
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
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.timeout(100)

    draw_ui(stdscr, "Initializing...")

    # Set up G1HighLevelDdsSdk with cmd_vel LCM transport so it subscribes
    conn = G1HighLevelDdsSdk(network_interface="eth0")
    conn.cmd_vel.transport = LCMTransport("/cmd_vel", Twist)
    conn.start()
    time.sleep(1)

    # Raw LCM publisher — messages go to the transport above
    lc = lcm.LCM()

    draw_ui(stdscr, "Connected - publishing on " + CMD_VEL_CHANNEL)

    linear_speed = 0.3  # m/s
    angular_speed = 0.5  # rad/s
    cmd_cooldown = 0.15

    try:
        last_cmd_time = 0.0

        while True:
            key = stdscr.getch()
            current_time = time.time()

            if current_time - last_cmd_time < cmd_cooldown:
                continue

            if key == -1:
                continue

            if key == 27 or key == 3:  # ESC or Ctrl+C
                break

            try:
                key_char = chr(key).lower() if key < 256 else None
            except ValueError:
                key_char = None

            twist = None
            action = None

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
                stop = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                publish_twist(lc, stop)
                action = "Stopped"
                last_cmd_time = current_time
            elif key_char == "1":
                draw_ui(stdscr, "Standing up...")
                conn.stand_up()
                action = "Standup complete"
                last_cmd_time = current_time
            elif key_char == "2":
                draw_ui(stdscr, "Lying down...")
                conn.lie_down()
                action = "Liedown complete"
                last_cmd_time = current_time

            if twist is not None:
                publish_twist(lc, twist)
                last_cmd_time = current_time

            if action:
                draw_ui(stdscr, action)

    except KeyboardInterrupt:
        pass
    finally:
        draw_ui(stdscr, "Stopping...")
        stop = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
        publish_twist(lc, stop)
        time.sleep(0.5)
        conn.disconnect()
        draw_ui(stdscr, "Done")
        time.sleep(1)


if __name__ == "__main__":
    print("\nWARNING: Ensure area is clear around robot!")
    print("Starting in 3 seconds...")
    time.sleep(3)

    try:
        curses.wrapper(main)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback

        traceback.print_exc()

    print("\nDone")
