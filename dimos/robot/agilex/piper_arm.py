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

import asyncio

# Import LCM message types
from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]

from dimos import core
from dimos.hardware.sensors.camera.zed import ZEDModule
from dimos.manipulation.visual_servoing.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.robot.robot import Robot
from dimos.skills.skills import SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PiperArmRobot(Robot):
    """Piper Arm robot with ZED camera and manipulation capabilities."""

    def __init__(self, robot_capabilities: list[RobotCapability] | None = None) -> None:
        super().__init__()
        self.dimos = None
        self.stereo_camera = None
        self.manipulation_interface = None
        self.skill_library = SkillLibrary()  # type: ignore[assignment]

        # Initialize capabilities
        self.capabilities = robot_capabilities or [
            RobotCapability.VISION,
            RobotCapability.MANIPULATION,
        ]

    async def start(self) -> None:
        """Start the robot modules."""
        # Start Dimos
        self.dimos = core.start(2)  # type: ignore[assignment]  # Need 2 workers for ZED and manipulation modules
        self.foxglove_bridge = FoxgloveBridge()

        # Enable LCM auto-configuration
        pubsub.lcm.autoconf()  # type: ignore[attr-defined]

        # Deploy ZED module
        logger.info("Deploying ZED module...")
        self.stereo_camera = self.dimos.deploy(  # type: ignore[attr-defined]
            ZEDModule,
            camera_id=0,
            resolution="HD720",
            depth_mode="NEURAL",
            fps=30,
            enable_tracking=False,  # We don't need tracking for manipulation
            publish_rate=30.0,
            frame_id="zed_camera",
        )

        # Configure ZED LCM transports
        self.stereo_camera.color_image.transport = core.LCMTransport("/zed/color_image", Image)  # type: ignore[attr-defined]
        self.stereo_camera.depth_image.transport = core.LCMTransport("/zed/depth_image", Image)  # type: ignore[attr-defined]
        self.stereo_camera.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)  # type: ignore[attr-defined]

        # Deploy manipulation module
        logger.info("Deploying manipulation module...")
        self.manipulation_interface = self.dimos.deploy(ManipulationModule)  # type: ignore[attr-defined]

        # Connect manipulation inputs to ZED outputs
        self.manipulation_interface.rgb_image.connect(self.stereo_camera.color_image)  # type: ignore[attr-defined]
        self.manipulation_interface.depth_image.connect(self.stereo_camera.depth_image)  # type: ignore[attr-defined]
        self.manipulation_interface.camera_info.connect(self.stereo_camera.camera_info)  # type: ignore[attr-defined]

        # Configure manipulation output
        self.manipulation_interface.viz_image.transport = core.LCMTransport(  # type: ignore[attr-defined]
            "/manipulation/viz", Image
        )

        # Print module info
        logger.info("Modules configured:")
        print("\nZED Module:")
        print(self.stereo_camera.io())  # type: ignore[attr-defined]
        print("\nManipulation Module:")
        print(self.manipulation_interface.io())  # type: ignore[attr-defined]

        # Start modules
        logger.info("Starting modules...")
        self.foxglove_bridge.start()
        self.stereo_camera.start()  # type: ignore[attr-defined]
        self.manipulation_interface.start()  # type: ignore[attr-defined]

        # Give modules time to initialize
        await asyncio.sleep(2)

        logger.info("PiperArmRobot initialized and started")

    def pick_and_place(  # type: ignore[no-untyped-def]
        self, pick_x: int, pick_y: int, place_x: int | None = None, place_y: int | None = None
    ):
        """Execute pick and place task.

        Args:
            pick_x: X coordinate for pick location
            pick_y: Y coordinate for pick location
            place_x: X coordinate for place location (optional)
            place_y: Y coordinate for place location (optional)

        Returns:
            Result of the pick and place operation
        """
        if self.manipulation_interface:
            return self.manipulation_interface.pick_and_place(pick_x, pick_y, place_x, place_y)
        else:
            logger.error("Manipulation module not initialized")
            return False

    def handle_keyboard_command(self, key: str):  # type: ignore[no-untyped-def]
        """Pass keyboard commands to manipulation module.

        Args:
            key: Keyboard key pressed

        Returns:
            Action taken or None
        """
        if self.manipulation_interface:
            return self.manipulation_interface.handle_keyboard_command(key)
        else:
            logger.error("Manipulation module not initialized")
            return None

    def stop(self) -> None:
        """Stop all modules and clean up."""
        logger.info("Stopping PiperArmRobot...")

        try:
            if self.manipulation_interface:
                self.manipulation_interface.stop()

            if self.stereo_camera:
                self.stereo_camera.stop()
        except Exception as e:
            logger.warning(f"Error stopping modules: {e}")

        # Close dimos last to ensure workers are available for cleanup
        if self.dimos:
            self.dimos.close()

        logger.info("PiperArmRobot stopped")


async def run_piper_arm() -> None:
    """Run the Piper Arm robot."""
    robot = PiperArmRobot()  # type: ignore[abstract]

    await robot.start()

    # Keep the robot running
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        await robot.stop()  # type: ignore[func-returns-value]


if __name__ == "__main__":
    asyncio.run(run_piper_arm())
