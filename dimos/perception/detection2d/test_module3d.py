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
import time

import pytest
from dimos_lcm.foxglove_msgs import ImageAnnotations, SceneUpdate
from dimos_lcm.sensor_msgs import Image, PointCloud2

from dimos.core import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2 as PointCloud2Msg
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d import testing
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.moduleDB import ObjectDBModule
from dimos.perception.detection2d.type import (
    Detection2D,
    Detection3D,
    ImageDetections2D,
    ImageDetections3D,
)
from dimos.robot.unitree_webrtc.modular import deploy_connection, deploy_navigation
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map


def test_module3d():
    import os
    import subprocess
    import traceback

    from dimos.protocol.service import lcmservice as lcm

    def count_open_files():
        """Count open file descriptors for current process"""
        pid = os.getpid()
        try:
            # Count files in /proc/PID/fd/
            fd_dir = f"/proc/{pid}/fd"
            if os.path.exists(fd_dir):
                return len(os.listdir(fd_dir))
            else:
                # Fallback to lsof
                result = subprocess.run(
                    f"lsof -p {pid} | wc -l", shell=True, capture_output=True, text=True
                )
                return int(result.stdout.strip()) - 1  # Subtract header line
        except Exception as e:
            print(f"Error counting open files: {e}")
            return -1

    try:
        print(f"Starting test_module3d, PID: {os.getpid()}")
        initial_files = count_open_files()
        print(f"Initial open files: {initial_files}")

        lcm.autoconf()
        print(f"LCM autoconf completed, open files: {count_open_files()}")

        for i in range(100):
            try:
                seek_value = 30.0 + i
                moment = testing.detections3d(seek=seek_value)
                print(f"detections3d returned, open files: {count_open_files()}")

                testing.publish_moment(moment)
                testing.publish_moment(moment)

                time.sleep(0.2)

                # Check if we're approaching the limit
                current_files = count_open_files()
                if current_files > 900:
                    print(f"WARNING: Approaching file descriptor limit! Current: {current_files}")
                    # List what files are open
                    subprocess.run(f"lsof -p {os.getpid()} | tail -20", shell=True)

            except OSError as e:
                if e.errno == 24:  # Too many open files
                    print(f"Too many open files at iteration {i}")
                    subprocess.run(f"lsof -p {os.getpid()} | tail -50", shell=True)
                raise

    except Exception as e:
        print(f"Error in test_module3d: {type(e).__name__}: {e}")
        traceback.print_exc()
        raise
