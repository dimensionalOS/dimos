# Copyright 2025-2026 Dimensional Inc.
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

"""Headless screenshot harness for lcmflow with synthetic traffic.

Usage: python demo_lcmflow.py [out.svg] [seconds]
Renders the app off-screen, injects fake robot-ish packet streams,
and writes an SVG screenshot (convert with cairosvg for PNG).
"""

from __future__ import annotations

import asyncio
import random
import sys

from dimos.utils.cli.lcmflow.run_lcmflow import LCMFlowApp

STREAMS = [
    # channel, avg bytes, hz
    ("/color_image#sensor_msgs.Image", 2_764_800, 12.8),
    ("/lidar#sensor_msgs.PointCloud2", 320_000, 7.0),
    ("/global_map#sensor_msgs.PointCloud2", 800_000, 1.4),
    ("/global_costmap#nav_msgs.OccupancyGrid", 60_000, 1.4),
    ("/tf#tf2_msgs.TFMessage", 300, 17.0),
    ("/odom#geometry_msgs.PoseStamped", 88, 17.0),
    ("/cmd_vel#geometry_msgs.Twist", 60, 50.0),
    ("/camera_info#sensor_msgs.CameraInfo", 360, 1.0),
]


async def screenshot(path: str, seconds: float) -> None:
    app = LCMFlowApp()
    async with app.run_test(size=(170, 34)) as pilot:
        elapsed = 0.0
        step = 0.05
        while elapsed < seconds:
            for channel, size, hz in STREAMS:
                if random.random() < hz * step:
                    jitter = max(20, int(random.gauss(size, size * 0.1)))
                    app.highway.spy.msg(channel, b"\0" * jitter)
            await pilot.pause(step)
            elapsed += step
        app.save_screenshot(path)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "lcmflow.svg"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
    asyncio.run(screenshot(out, secs))
    print(f"wrote {out}")
