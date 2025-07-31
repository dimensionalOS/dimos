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

"""Demo of Minecraft virtual robot with fake lidar."""

import time
from dimos.robot.sim.minecraft.connection import Minecraft
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def main():
    # Create Minecraft robot connection
    mc = Minecraft()

    # Connect to environment
    mc.connect()
    print("Connected to Minecraft!")

    # Get lidar stream
    lidar_stream = mc.lidar_stream()

    # Subscribe to lidar data
    def on_lidar(pc2):
        points = pc2.pointcloud.points
        print(f"Received PointCloud2: {len(points)} points at time {pc2.ts:.3f}")

    subscription = lidar_stream.subscribe(on_lidar)

    # Move the robot around
    print("Moving robot...")
    for i in range(10):
        # Move forward
        mc.move(Vector3(x=0.5, y=0, z=0))
        time.sleep(0.5)

        # Strafe right
        mc.move(Vector3(x=0, y=0.3, z=0))
        time.sleep(0.5)

        # Stop
        mc.move(Vector3(x=0, y=0, z=0))
        time.sleep(0.5)

    # Cleanup
    subscription.dispose()
    mc.close()
    print("Demo complete!")


if __name__ == "__main__":
    main()
