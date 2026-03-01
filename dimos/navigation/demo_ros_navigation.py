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

import time

from dimos.core.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.navigation import rosnav
from dimos.protocol.service.lcmservice import autoconf
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def main() -> None:
    autoconf()
    dimos = ModuleCoordinator()
    dimos.start()

    ros_nav = rosnav.deploy(dimos)

    logger.info("\nTesting navigation in 2 seconds...")
    time.sleep(2)

    test_pose = PoseStamped(
        ts=time.time(),
        frame_id="map",
        position=Vector3(10.0, 10.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    logger.info("Sending navigation goal to: (10.0, 10.0, 0.0)")
    ros_nav.set_goal(test_pose)
    time.sleep(5)

    logger.info("Cancelling goal after 5 seconds...")
    cancelled = ros_nav.cancel_goal()
    logger.info(f"Goal cancelled: {cancelled}")

    try:
        logger.info("\nNavBot running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        dimos.stop()


if __name__ == "__main__":
    main()
