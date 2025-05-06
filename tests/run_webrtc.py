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
import cv2
import os
import asyncio
from dotenv import load_dotenv
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream
from dimos.types.vector import Vector
import logging
import open3d as o3d
import reactivex.operators as ops
import numpy as np

# logging.basicConfig(level=logging.DEBUG)


async def main():
    load_dotenv()
    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="normal")

    print("standing up")
    # await robot.standup()
    print("robot is up")

    # show3d_stream(robot.lidar_stream())

    # def parse_frame(frame):
    #    return o3d.geometry.Image(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    # show3d_stream(robot.video_stream().pipe(ops.map(parse_frame)), clearframe=True)

    robot.odom_stream().subscribe(print)

    try:
        while True:
            robot.move_vel(Vector(0.1, 0.1, 0.1))
            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopping robot")
        await robot.liedown()


if __name__ == "__main__":
    asyncio.run(main())
