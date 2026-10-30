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

"""Drive-and-record blueprint for the G1.

Desktop teleop walks the robot while Point-LIO odom+lidar, the
RealSense color+depth streams, and tf are recorded into a timestamped folder
under recordings/.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.memory.module import default_recording_dir
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.g1_recorder import G1Recorder
from dimos.robot.unitree.g1.g1_tf_publisher import G1TfPublisher
from dimos.robot.unitree.g1.skill_container import UnitreeG1SkillContainer

_RECORDING_DIR = default_recording_dir()


unitree_g1_record = autoconnect(
    MovementManager.blueprint(),
    G1Connection.blueprint(stock_transport="dds"),
    UnitreeG1SkillContainer.blueprint(),
    mid360_for_pointlio(host_ip="192.168.123.164", lidar_ip="192.168.123.120"),
    PointLio.blueprint(frame_id="world").remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    RealSenseCamera.blueprint(frame_id="d435_link").remappings(
        [
            (RealSenseCamera, "depth_image", "realsense_depth_image"),
            (RealSenseCamera, "camera_info", "realsense_camera_info"),
            (RealSenseCamera, "depth_camera_info", "realsense_depth_camera_info"),
        ]
    ),
    G1Recorder.blueprint(db_path=str(_RECORDING_DIR / "mem2.db")),
    # Mount frames onto tf, base_link edge live from the waist joints.
    G1TfPublisher.blueprint(),
).global_config(n_workers=13, robot_model="unitree_g1", viewer="none")


if __name__ == "__main__":
    coordinator = ModuleCoordinator.build(unitree_g1_record)
    coordinator.loop()
