#!/usr/bin/env python3
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

from datetime import datetime
import os
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.logging_config import set_run_log_dir, setup_logger

logger = setup_logger()

_LIDAR_IP = os.getenv("LIDAR_IP", "192.168.1.171")
_LIDAR_HOST_IP = os.getenv("LIDAR_HOST_IP", "192.168.1.100")


def _default_recording_dir() -> Path:
    now = datetime.now()
    stamp = now.strftime("%Y-%m-%d") + "_" + now.strftime("%I-%M%p").lower() + "-PST"
    return Path("recordings") / stamp


class Go2Recorder(PointlioRecorder):
    """Records Point-LIO odom + lidar plus the Go2's companion streams.

    Point-LIO stamps each ``pointlio_lidar`` frame with the latest odometry pose
    (inherited ``@pose_setter_for``), so the trajectory is baked into the cloud at
    record time — no static-transform pose-fill needed here. The companion streams
    (camera, raw livox, go2 odom/lidar) are recorded as-is; the offline
    post-process aligns them.
    """

    pointlio_odometry: In[Odometry]
    pointlio_lidar: In[PointCloud2]
    go2_lidar: In[PointCloud2]
    go2_odom: In[PoseStamped]
    color_image: In[Image]
    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]


unitree_go2_record = autoconnect(
    MovementManager.blueprint(),
    GO2Connection.blueprint().remappings(
        [
            (GO2Connection, "lidar", "go2_lidar"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    ),
    Mid360.blueprint(
        lidar_ip=_LIDAR_IP,
        host_ip=_LIDAR_HOST_IP,
    ).remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    PointLio.blueprint(
        frame_id="world",
        lidar_ip=_LIDAR_IP,
    ).remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    Go2Recorder.blueprint(),
    # Pygame keyboard teleop (WASD drive + Q/E strafe). Its cmd_vel feeds
    # MovementManager's tele_cmd_vel.
    KeyboardTeleop.blueprint(linear_speed=0.3, angular_speed=0.6).remappings(
        [
            (KeyboardTeleop, "cmd_vel", "tele_cmd_vel"),
        ]
    ),
).global_config(n_workers=10, robot_model="unitree_go2")


if __name__ == "__main__":
    recording_dir = _default_recording_dir().resolve()
    recording_dir.mkdir(parents=True, exist_ok=True)
    set_run_log_dir(recording_dir)
    global_config.obstacle_avoidance = False
    coordinator = ModuleCoordinator.build(
        unitree_go2_record,
        {Go2Recorder.name: {"db_path": str(recording_dir / "mem2.db")}},
    )
    coordinator.loop()
