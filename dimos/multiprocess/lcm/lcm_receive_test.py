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

import lcm
import signal
import sys
import threading
import time
from lcm_msgs.geometry_msgs import Twist, Vector3


class LcmReceiver:
    def __init__(self):
        self.lc = lcm.LCM()
        self.running = True

        # Subscribe to the Vector3 channel
        self.lc.subscribe("thing1_vector3#geometry_msgs.Vector3", self.handle_vector3)

        # Subscribe to the Twist channel
        self.lc.subscribe("thing1_twist#geometry_msgs.Twist", self.handle_twist)

        # Setup signal handling for graceful exit
        signal.signal(signal.SIGINT, self.signal_handler)

    def handle_vector3(self, channel, data):
        """Handler for Vector3 messages"""
        msg = Vector3.decode(data)
        print(f"\n[RECEIVED] Channel: {channel}")
        print(f"Vector3: x={msg.x}, y={msg.y}, z={msg.z}")

    def handle_twist(self, channel, data):
        """Handler for Twist messages"""
        msg = Twist.decode(data)
        print(f"\n[RECEIVED] Channel: {channel}")
        print(f"Twist: linear(x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z})")
        if hasattr(msg, "angular"):
            print(f"       angular(x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z})")

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C for clean exit"""
        print("\nExiting gracefully...")
        self.running = False

    def start(self):
        """Start listening for messages"""
        print("LCM Receiver started. Listening for messages...")
        print("Press Ctrl+C to exit")

        # Create a separate thread for LCM handling
        thread = threading.Thread(target=self._loop)
        thread.daemon = True
        thread.start()

        # Keep the main thread running
        while self.running:
            time.sleep(0.1)

    def _loop(self):
        """LCM message handling loop"""
        while self.running:
            try:
                self.lc.handle()
            except Exception as e:
                print(f"Error in LCM handling: {e}")


if __name__ == "__main__":
    receiver = LcmReceiver()
    receiver.start()
