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

import rclpy  # type: ignore[import-untyped]

from dimos import core
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Twist, Vector3
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.navigation.rosnav import ROSNav
from dimos.protocol import pubsub
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def main() -> None:
    pubsub.lcm.autoconf()  # type: ignore[attr-defined]
    dimos = core.start(2)

    ros_nav = dimos.deploy(ROSNav)  # type: ignore[attr-defined]

    ros_nav.goal_req.transport = core.LCMTransport("/goal", PoseStamped)
    ros_nav.pointcloud.transport = core.LCMTransport("/pointcloud_map", PointCloud2)
    ros_nav.global_pointcloud.transport = core.LCMTransport("/global_pointcloud", PointCloud2)
    ros_nav.goal_active.transport = core.LCMTransport("/goal_active", PoseStamped)
    ros_nav.path_active.transport = core.LCMTransport("/path_active", Path)
    ros_nav.cmd_vel.transport = core.LCMTransport("/cmd_vel", Twist)

    ros_nav.start()

    logger.info("\nTesting navigation in 2 seconds...")
    time.sleep(2)

    test_pose = PoseStamped(
        ts=time.time(),
        frame_id="map",
        position=Vector3(2.0, 2.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    logger.info("Sending navigation goal to: (2.0, 2.0, 0.0)")
    success = ros_nav.navigate_to(test_pose, timeout=30.0)
    logger.info(f"Navigated successfully: {success}")

    try:
        logger.info("\nNavBot running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
        ros_nav.stop()

        if rclpy.ok():  # type: ignore[attr-defined]
            rclpy.shutdown()


if __name__ == "__main__":
    main()
