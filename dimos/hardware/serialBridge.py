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
import random
import threading
import time
import serial
import argparse

import pytest

import dimos.core as core
from dimos.core import Module, In, Out, rpc
from dimos.protocol.service.lcmservice import autoconf
from dimos.msgs.geometry_msgs import Pose, Vector3, Twist
import dimos.protocol.service.lcmservice as lcmservice
from dimos.msgs.sensor_msgs.JointState import JointState

dev_serial = "/dev/ttyUSB0"


class SerialBrige(Module):
    joint_state: In[JointState] = None
    pose_state: Out[Pose] = None
    joint_feedback: Out[JointState] = None

    def __init__(
        self,
        port: str = dev_serial,
        baud: int = 115200,
        timeout: float = 0.1,
        joint_names: list = None,
        *args,
        **kwargs,
    ):
        print(f"Initializing SerialBrige with port {port} and baud {baud} for joints {joint_names}")
        super().__init__(*args, **kwargs)
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.joint_names = joint_names or []
        self.last_positions = {}
        self.reader_thread = None
        self.latest_joint_state = None

    @rpc
    def start(self):
        # subscribe to incoming LCM Twist messages
        self.joint_state.subscribe(self._on_joint_state)
        print(f"Subscribed to {self.joint_state}")

        # start reader thread for bidirectional communication
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()
        print("Started serial reader thread")

    def _on_joint_state(self, msg: JointState):
        # print(f"[SerialBrige] Received joint state: {msg}")

        # Store the latest joint state for merging with device feedback
        self.latest_joint_state = msg

        for joint_name in self.joint_names:
            if joint_name not in msg.name:
                print(
                    f"[SerialBrige] Joint name {joint_name} not found in message names: {msg.name}"
                )
                continue

            idx = msg.name.index(joint_name)
            position = msg.position[idx]

            # Check if the position has changed and if so, send the command
            if position != self.last_positions.get(joint_name):
                # Format: <sec,joint_name,position>
                cmd = f"<{msg.header.stamp.sec},{joint_name},{position}>"
                self.ser.write(cmd.encode("ascii"))
                print(f"[SerialBrige] Sent to serial: {cmd}")
                self.last_positions[joint_name] = position

        # mirror your pc_echo.py's sleep to let Arduino respond
        time.sleep(0.05)

    def _merge_joint_feedback(self, joint_name, actual_position, timestamp):
        """Merge device feedback with latest joint state and publish"""
        if self.latest_joint_state is None:
            print(f"[SerialBrige] No joint state available for merging feedback")
            return

        # Create a fresh JointState message
        feedback_msg = JointState()
        feedback_msg.header = self.latest_joint_state.header
        feedback_msg.header.stamp.sec = timestamp

        # Copy data from latest joint state
        feedback_msg.name = list(self.latest_joint_state.name)
        feedback_msg.position = list(self.latest_joint_state.position)
        feedback_msg.velocity = (
            list(self.latest_joint_state.velocity) if self.latest_joint_state.velocity else []
        )
        feedback_msg.effort = (
            list(self.latest_joint_state.effort) if self.latest_joint_state.effort else []
        )

        # Update the specific joint position
        if joint_name in feedback_msg.name:
            idx = feedback_msg.name.index(joint_name)
            old_position = feedback_msg.position[idx]
            feedback_msg.position[idx] = actual_position
            print(
                f"[SerialBrige] Updated {joint_name} position from {old_position} to {actual_position}"
            )
        else:
            print(f"[SerialBrige] Joint {joint_name} not found in latest joint state")
            return

        # Publish the merged feedback
        self.joint_feedback.publish(feedback_msg)
        print(f"[SerialBrige] Published merged joint feedback")

    def _parse_and_merge_feedback(self, text):
        """Parse device feedback and merge with joint state"""
        try:
            # Remove < and > brackets
            content = text[1:-1]
            parts = content.split(",")

            if len(parts) != 3:
                print(f"[SerialBrige] Invalid joint feedback format: {text}")
                return

            timestamp_str, joint_name, angle_str = parts
            timestamp = int(timestamp_str)
            angle = float(angle_str)

            # Only process feedback for joints we're tracking
            if joint_name in self.joint_names:
                # self._merge_joint_feedback(joint_name, angle, timestamp)
                print(f"[SerialBrige] Received feedback for {joint_name}: {angle} at {timestamp}")
            else:
                print(f"[SerialBrige] Ignoring feedback for untracked joint: {joint_name}")

        except (ValueError, IndexError) as e:
            print(f"[SerialBrige] Failed to parse joint feedback '{text}': {e}")

    def _reader(self):
        while True:
            line = self.ser.readline()
            if not line:
                continue
            text = line.decode("ascii", "ignore").strip()
            print(f"[SerialBrige] Received from serial: {text}")

            # Handle joint feedback messages: <timestamp,joint_name,angle>
            if text.startswith("<") and text.endswith(">"):
                print(f"[SerialBrige] Joint feedback: {text}")
                self._parse_and_merge_feedback(text)
                continue

            # Handle pose messages: "POSE,0.100,0.200,…"
            if text.startswith("POSE"):
                parts = text.split(",")
                if len(parts) == 7:
                    try:
                        # parse six floats
                        lx, ly, lz, ax, ay, az = map(float, parts[1:])
                        cp = Pose()
                        cp.position.x, cp.position.y, cp.position.z = lx, ly, lz
                        cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w = (
                            ax,
                            ay,
                            az,
                            1.0,
                        )
                        self.pose_state.publish(cp)
                    except ValueError:
                        print(f"[SerialBrige] Failed to parse pose: {text}")


def TestSerialBridge(port, joint_names):
    lcmservice.autoconf()
    dimos = core.start(2)

    torso = dimos.deploy(SerialBrige, port=port, baud=115200, joint_names=joint_names)

    torso.pose_state.transport = core.LCMTransport("/pose", Pose)
    torso.joint_state.transport = core.LCMTransport("/joint_states", JointState)
    torso.joint_feedback.transport = core.LCMTransport("/joint_feedback", JointState)

    torso.start()
    print("SerialBridge started")

    while True:
        time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial bridge for robot joint control.")
    parser.add_argument(
        "--port", type=str, default=dev_serial, help="Serial device path (e.g. /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--joint",
        type=str,
        action="append",
        required=True,
        help="Joint name to control (must match JointState name). Can be specified multiple times.",
    )
    args = parser.parse_args()
    TestSerialBridge(args.port, args.joint)
