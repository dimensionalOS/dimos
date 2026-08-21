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

"""Alfred recording every sensor at full rate, for offline SLAM work.

    dimos run alfred-455-rec
    dimos run alfred-455-rec --alfredrecorder.db_path=/data/office_loop.db

Teleop is included, so this is the blueprint to run for a hand-driven recording.
It comes from the viewer's websocket rather than a local pygame window, so drive
it from dimos-viewer connected to this host -- no display needed here.

Drives the robot and records rather than localising: the D455's colour, depth
and both IR imagers, its IMU, Point-LIO's cloud and odometry, wheel odometry,
and the tf tree. Images are written uncompressed -- see `AlfredRecorder`.

The emitter is left ON. This recording feeds depth work as well as tracking, and
a recording is not a live tracking run; a consumer that wants clean IR for
feature tracking should record a second pass with it off.
"""

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.recorder import AlfredRecorder
from dimos.visualization.vis_module import vis_module

alfred_455_rec = (
    autoconnect(
        RealSenseCamera.blueprint(
            width=848,
            height=480,
            fps=30,
            enable_color=True,
            enable_depth=True,
            enable_infrared=True,
            enable_imu=True,
            # Depth stays in its own optical frame: alignment resamples it against
            # the colour intrinsics, which is a lossy choice to bake into a
            # recording. Consumers can align afterwards.
            align_depth_to_color=False,
            # A point cloud is derivable from the recorded depth frames; recording
            # it as well only competes for disk bandwidth.
            enable_pointcloud=False,
        ),
        PointLio.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.1.5"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.1.189"),
        ),
        AlfredRecorder.blueprint(),
        MovementManager.blueprint(),
        AlfredHighLevel.blueprint(),
        # Teleop rides in with the viewer: vis_module bundles
        # RerunWebSocketServer, whose tele_cmd_vel is already the manager's
        # teleop inlet, so its stop gate applies to a hand-driven run.
        vis_module(global_config.viewer),
    )
    .remappings(
        [
            # Point-LIO owns "odometry"; the wheel encoders record alongside it.
            (MovementManager, "way_point", "_mgr_way_point_unused"),
        ]
    )
    .global_config(n_workers=10)
)
