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

"""TRON1 full visualization stack for video, lidar, IMU, and odometry."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.limx.tron1.connection import TRON1Connection
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


tron1_full_vis = autoconnect(
    TRON1Connection.blueprint(),
    RealSenseCamera.blueprint(
        camera_name="realsense",
        enable_depth=True,
        enable_pointcloud=False,
    ),
    PointLio.blueprint(frame_id="world"),
    RerunBridgeModule.blueprint(
        pubsubs=[LCM()],
        rerun_open=global_config.rerun_open,
        rerun_web=global_config.rerun_web,
    ),
    RerunWebSocketServer.blueprint(),
).global_config(
    n_workers=6,
    robot_model="tron1_full_vis",
)
