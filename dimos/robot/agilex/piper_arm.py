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

import logging
import time
from typing import Optional, List, Dict, Any

from dimos import core
from dimos.hardware.zed_camera import ZEDModule
from dimos.hardware.piper_arm import PiperArmModule
from dimos.manipulation.visual_servoing.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.protocol import pubsub
from dimos.skills.skills import SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.utils.logging_config import setup_logger
from dimos.robot.robot import Robot

# Import LCM message types
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.vision_msgs import Detection3DArray, Detection2DArray
from dimos_lcm.std_msgs import String
from dimos_lcm.geometry_msgs import Twist

logger = setup_logger("dimos.robot.agilex.piper_arm")


class PiperArmRobot(Robot):
    """Piper Arm robot with ZED camera, detection, and manipulation capabilities."""

    def __init__(
        self,
        robot_capabilities: Optional[List[RobotCapability]] = None,
        enable_mobile_base_control: bool = False,
    ):
        super().__init__()
        self.dimos = None
        self.stereo_camera = None
        self.piper_arm = None
        self.manipulation_interface = None
        self.skill_library = SkillLibrary()
        self.enable_mobile_base_control = enable_mobile_base_control

        # Initialize capabilities
        self.capabilities = robot_capabilities or [
            RobotCapability.VISION,
            RobotCapability.MANIPULATION,
        ]

    def start(self):
        """Start the robot modules."""
        # Start Dimos
        self.dimos = core.start(4)  # Need 4 workers for ZED, Piper, Detection, and Manipulation
        self.foxglove_bridge = FoxgloveBridge()

        # Enable LCM auto-configuration
        pubsub.lcm.autoconf()

        # Deploy ZED module
        logger.info("Deploying ZED module...")
        self.stereo_camera = self.dimos.deploy(
            ZEDModule,
            camera_id=0,
            resolution="HD720",
            enable_tracking=False,  # Disable tracking - using piper_arm transforms
            frame_id="zed_camera_link",
            optical_frame_id="zed_camera_link_optical",
        )

        # Configure ZED LCM transports
        self.stereo_camera.color_image.transport = core.LCMTransport("/zed/color_image", Image)
        self.stereo_camera.depth_image.transport = core.LCMTransport("/zed/depth_image", Image)
        self.stereo_camera.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)
        self.stereo_camera.pose.transport = core.LCMTransport("/zed/pose", PoseStamped)

        # Deploy Piper Arm module
        logger.info("Deploying Piper Arm module...")
        self.piper_arm = self.dimos.deploy(
            PiperArmModule,
            publish_rate=30.0,
            base_frame_id="base_link",
            ee_frame_id="ee_link",
            camera_frame_id="zed_camera_link",
            ee_to_camera_6dof=[-0.065, 0.03, -0.095, 0.0, -1.57, 0.0],  # EE to camera transform
        )

        # Configure Piper Arm output
        self.piper_arm.ee_pose.transport = core.LCMTransport("/piper/ee_pose", PoseStamped)

        # Deploy manipulation module with integrated detection
        logger.info("Deploying manipulation module with integrated detection...")
        # Use world frame for tracking if mobile base is enabled
        track_frame = "world" if self.enable_mobile_base_control else "base_link"
        self.manipulation_interface = self.dimos.deploy(
            ManipulationModule,
            piper_arm_module=self.piper_arm,  # Pass the arm module reference
            min_confidence=0.3,
            max_depth=1.0,
            max_object_size=0.15,
            camera_frame_id="zed_camera_link_optical",  # Use ZED optical frame
            base_frame_id="base_link",
            track_frame_id=track_frame,  # Use world frame if mobile base enabled
            reach_timeout=10.0,  # Simple timeout for reaching poses
            enable_mobile_base=self.enable_mobile_base_control,  # Pass mobile base flag
        )

        # Connect manipulation inputs
        self.manipulation_interface.rgb_image.connect(self.stereo_camera.color_image)
        self.manipulation_interface.depth_image.connect(self.stereo_camera.depth_image)
        self.manipulation_interface.camera_info.connect(self.stereo_camera.camera_info)

        # Configure manipulation outputs
        self.manipulation_interface.viz_image.transport = core.LCMTransport(
            "/manipulation/viz", Image
        )
        self.manipulation_interface.grasp_state.transport = core.LCMTransport(
            "/manipulation/grasp_state", String
        )
        self.manipulation_interface.detection3d_array.transport = core.LCMTransport(
            "/detection/3d_array", Detection3DArray
        )
        self.manipulation_interface.detection2d_array.transport = core.LCMTransport(
            "/detection/2d_array", Detection2DArray
        )

        # Configure cmd_vel output if mobile base is enabled
        if self.enable_mobile_base_control:
            self.manipulation_interface.cmd_vel.transport = core.LCMTransport("/cmd_vel", Twist)
            logger.info("Mobile base control enabled - publishing to /cmd_vel")

        # Print module info
        logger.info("Modules configured:")
        print("\nZED Module:")
        print(self.stereo_camera.io())
        print("\nPiper Arm Module:")
        print(self.piper_arm.io())
        print("\nManipulation Module:")
        print(self.manipulation_interface.io())

        # Start modules
        logger.info("Starting modules...")
        self.foxglove_bridge.start()
        self.stereo_camera.start()
        self.piper_arm.start()
        self.manipulation_interface.start()

        # Give modules time to initialize
        time.sleep(2)

        logger.info("PiperArmRobot initialized and started with modular architecture")

    def pick_and_place(
        self,
        pick_x: int,
        pick_y: int,
        place_x: Optional[int] = None,
        place_y: Optional[int] = None,
        rpc_timeout: float = 60.0,
    ) -> Dict[str, Any]:
        """Execute pick and place task (blocking).

        Args:
            pick_x: X coordinate for pick location
            pick_y: Y coordinate for pick location
            place_x: X coordinate for place location (optional)
            place_y: Y coordinate for place location (optional)
            rpc_timeout: RPC timeout in seconds (default 60.0 for long-running operations)

        Returns:
            Dict with success status and details
        """
        if self.manipulation_interface:
            pick_target = (pick_x, pick_y)
            place_target = (place_x, place_y) if place_x is not None and place_y is not None else None
            return self.manipulation_interface.rpc.call_sync(
                f"{self.manipulation_interface.remote_name}/pick_and_place",
                ([pick_target, place_target], {}),
                rpc_timeout=rpc_timeout,
                max_retries=1,  # Don't retry for long operations
            )
        else:
            logger.error("Manipulation module not initialized")
            return {"success": False, "error": "Manipulation module not initialized"}

    def handle_keyboard_command(self, key: str):
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

    def stop(self):
        """Stop all modules and clean up."""
        logger.info("Stopping PiperArmRobot...")

        try:
            if self.manipulation_interface:
                self.manipulation_interface.stop()
                self.manipulation_interface.cleanup()

            if self.piper_arm:
                self.piper_arm.goto_zero()
                time.sleep(0.5)
                self.piper_arm.stop()

            if self.stereo_camera:
                self.stereo_camera.stop()
        except Exception as e:
            logger.warning(f"Error stopping modules: {e}")

        # Close dimos last to ensure workers are available for cleanup
        if self.dimos:
            self.dimos.close()

        logger.info("PiperArmRobot stopped")


def main():
    """Main entry point."""
    robot = PiperArmRobot(enable_mobile_base_control=True)
    robot.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
