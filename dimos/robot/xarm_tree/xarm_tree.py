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
import logging
from typing import Optional, List, Dict, Any

from dimos import core
from dimos.robot.robot import Robot
from dimos.types.robot_capabilities import RobotCapability
from dimos.manipulation.visual_servoing.mobile_base_pbvs import MobileBasePBVS
from dimos.protocol.rpc.lcmrpc import LCMRPC
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.protocol import pubsub
from dimos.msgs.geometry_msgs import Twist, PoseStamped
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.std_msgs import Int32
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import String
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.vision_msgs import Detection2DArray, Detection3DArray

logger = setup_logger("dimos.robot.xarm_tree")

# Suppress verbose loggers
logging.getLogger("lcm").setLevel(logging.WARNING)


class XArmTree(Robot):
    """XArmTree: Robot interface for XArm on Unitree B1 with visual servoing."""

    # RPC service names for external robots
    MANIPULATION_MODULE = "ManipulationModule"
    XARM_MODULE = "XArmModule"
    UNITREE_B1_CONNECTION = "B1ConnectionModule"

    # LCM topic names
    ZED_COLOR_TOPIC = "/zed/color_image"
    ZED_DEPTH_TOPIC = "/zed/depth_image"
    ZED_CAMERA_INFO_TOPIC = "/zed/camera_info"
    SERVOING_STATE_TOPIC = "/servoing/state"
    SERVOING_VIZ_TOPIC = "/servoing/viz"
    SERVOING_CMD_VEL_TOPIC = "/cmd_vel"
    SERVOING_DETECTION3D_TOPIC = "/servoing/detection3d"
    SERVOING_DETECTION2D_TOPIC = "/servoing/detection2d"
    ODOM_TOPIC = "/odom"
    # B1 control topics
    CMD_VEL_TOPIC = "/cmd_vel"
    B1_MODE_TOPIC = "/b1/mode"

    def __init__(self, robot_capabilities: Optional[List[RobotCapability]] = None):
        """Initialize the XArmTree robot."""
        super().__init__()

        self.dimos = None
        self.rpc_client = None
        self.lcm = None
        self.mobile_base_servoing = None

        # LCM publishers for B1 control
        self.cmd_vel_topic = None
        self.mode_topic = None

        # Set capabilities
        self.capabilities = robot_capabilities or [
            RobotCapability.VISION,
            RobotCapability.MANIPULATION,
            RobotCapability.LOCOMOTION,
        ]

    def start(self):
        """Start the robot with visual servoing module and RPC connections."""
        # Enable LCM auto-configuration
        pubsub.lcm.autoconf()

        # Start Dimos for deploying servoing module
        self.dimos = core.start(2)

        # Initialize RPC client for external robot communication
        self.rpc_client = LCMRPC()
        self.rpc_client.start()

        # Initialize LCM for topic subscriptions and publishing
        self.lcm = LCM()
        self.lcm.start()

        # Set up LCM publishers for B1 control
        self.cmd_vel_topic = Topic(self.CMD_VEL_TOPIC, Twist)
        self.mode_topic = Topic(self.B1_MODE_TOPIC, Int32)

        # Deploy Mobile Base PBVS module using ZED camera with PID controllers
        logger.info("Deploying Mobile Base PBVS module with ZED camera and PID control...")
        self.mobile_base_servoing = self.dimos.deploy(
            MobileBasePBVS,
            # PID parameters for linear X (forward/backward) - tuned for B1
            linear_x_kp=0.8,
            linear_x_ki=0.25,
            linear_x_kd=0.08,
            # PID parameters for linear Y (left/right) - tuned for B1
            linear_y_kp=0.7,
            linear_y_ki=0.15,
            linear_y_kd=0.12,
            # PID parameters for angular Z (rotation) - tuned for B1
            angular_z_kp=0.6,
            angular_z_ki=0.12,
            angular_z_kd=0.08,
            # Velocity limits - B1 is larger and more powerful than Go2
            max_linear_velocity=0.8,
            max_angular_velocity=0.6,
            target_tolerance=0.12,
            min_confidence=0.5,
            camera_frame_id="zed_camera_link_optical",
            track_frame_id="world",
            base_frame_id="base_link",
            tracking_loss_timeout=6.0,
        )

        # Configure input transports using ZED camera topics
        self.mobile_base_servoing.rgb_image.transport = core.LCMTransport(
            self.ZED_COLOR_TOPIC, Image
        )
        self.mobile_base_servoing.depth_image.transport = core.LCMTransport(
            self.ZED_DEPTH_TOPIC, Image
        )
        self.mobile_base_servoing.camera_info.transport = core.LCMTransport(
            self.ZED_CAMERA_INFO_TOPIC, CameraInfo
        )
        self.mobile_base_servoing.odom.transport = core.LCMTransport(self.ODOM_TOPIC, PoseStamped)

        # Configure output transports
        self.mobile_base_servoing.viz_image.transport = core.LCMTransport(
            self.SERVOING_VIZ_TOPIC, Image
        )
        self.mobile_base_servoing.cmd_vel.transport = core.LCMTransport(
            self.SERVOING_CMD_VEL_TOPIC, Twist
        )
        self.mobile_base_servoing.tracking_state.transport = core.LCMTransport(
            self.SERVOING_STATE_TOPIC, String
        )
        self.mobile_base_servoing.detection3d_array.transport = core.LCMTransport(
            self.SERVOING_DETECTION3D_TOPIC, Detection3DArray
        )
        self.mobile_base_servoing.detection2d_array.transport = core.LCMTransport(
            self.SERVOING_DETECTION2D_TOPIC, Detection2DArray
        )

        # Start the PBVS module
        self.mobile_base_servoing.start()
        logger.info("Mobile Base PBVS module started with ZED camera")

        logger.info("XArmTree robot started")
        logger.info("Expecting XArmRobot and UnitreeB1 to be running separately")

    def pick_and_place(
        self,
        pick_x: int,
        pick_y: int,
        place_x: Optional[int] = None,
        place_y: Optional[int] = None,
        timeout: float = 60.0,
    ) -> Dict[str, Any]:
        """Execute pick and place via RPC to XArmRobot.

        Args:
            pick_x: X coordinate for pick location
            pick_y: Y coordinate for pick location
            place_x: X coordinate for place location (optional)
            place_y: Y coordinate for place location (optional)
            timeout: RPC timeout in seconds

        Returns:
            Dict with success status and details
        """
        try:
            # Create tuples for the new signature
            pick_target = (pick_x, pick_y) if pick_x is not None and pick_y is not None else None
            place_target = (
                (place_x, place_y) if place_x is not None and place_y is not None else None
            )

            result = self.rpc_client.call_sync(
                f"{self.MANIPULATION_MODULE}/pick_and_place",
                ([pick_target, place_target], {}),
                rpc_timeout=timeout,
                max_retries=1,
            )

            self.pose()
            return result
        except Exception as e:
            logger.error(f"Pick and place RPC failed: {e}")
            self.pose()
            return {"success": False, "error": str(e)}

    def mobile_pick_and_place(
        self,
        target_x: int,
        target_y: int,
        servo_distance: float = 0.5,
        servo_timeout: float = 35.0,
        pick_timeout: float = 60.0,
    ) -> Dict[str, Any]:
        """Execute mobile pick and place: servo to object then pick it.

        This function combines visual servoing with object picking:
        1. Servo to the object at the specified distance
        2. Get the tracked Detection3D object
        3. Pass it directly to the manipulation module for picking

        Args:
            target_x: X pixel coordinate of target
            target_y: Y pixel coordinate of target
            servo_distance: Distance to maintain from object during servoing (meters)
            servo_timeout: Timeout for servoing operation (seconds)
            pick_timeout: Timeout for pick operation (seconds)

        Returns:
            Dict with success status and details
        """
        # Step 1: Servo to the object
        logger.info(f"Starting mobile pick: servoing to object at ({target_x}, {target_y})")
        if not self.servo_to_object(target_x, target_y, servo_distance, servo_timeout):
            return {"success": False, "error": "Failed to servo to object"}

        # Step 2: Get the tracked Detection3D object
        if not self.mobile_base_servoing:
            return {"success": False, "error": "Mobile base servoing module not initialized"}

        tracked_detection = self.mobile_base_servoing.get_latest_detection3d()
        if not tracked_detection:
            return {"success": False, "error": "No tracked object available after servoing"}

        logger.info("Successfully tracked object, now executing pick")

        # Put robot in pose/stand mode for manipulation
        self.pose()

        # Step 3: Execute pick using the Detection3D object directly in pick_and_place
        try:
            # Call pick_and_place directly with the Detection3D object
            result = self.rpc_client.call_sync(
                f"{self.MANIPULATION_MODULE}/pick_and_place",
                ([tracked_detection, None], {}),  # Pass Detection3D as pick_target, no place target
                rpc_timeout=pick_timeout,
                max_retries=1,
            )

            return result

        except Exception as e:
            logger.error(f"Mobile pick and place failed: {e}")
            self.pose()
            return {"success": False, "error": str(e)}

    def servo_to_object(
        self,
        target_x: int,
        target_y: int,
        target_distance: float = 0.5,
        timeout: float = 35.0,
    ) -> bool:
        """Servo to object using local visual servoing module.

        Args:
            target_x: X pixel coordinate of target
            target_y: Y pixel coordinate of target
            target_distance: Distance to maintain from object (meters)
            timeout: Timeout for servoing operation (seconds)

        Returns:
            True if servoing completed successfully
        """
        if not self.mobile_base_servoing:
            logger.error("Mobile base servoing module not initialized")
            return False

        try:
            # Start tracking using local module
            result = self.mobile_base_servoing.track(
                target_x=target_x, target_y=target_y, target_distance=target_distance
            )

            logger.info(
                f"Started servoing to object at ({target_x}, {target_y}) with distance {target_distance}"
            )

            if result.get("status") != "success":
                logger.error(f"Failed to start tracking: {result.get('message')}")
                return False

            # Monitor state using get_state() RPC like navigator
            start_time = time.time()

            while time.time() - start_time < timeout:
                # Get current state via RPC
                current_state = self.mobile_base_servoing.get_state()

                if current_state == "reached":
                    logger.info("Target reached successfully")
                    # Stop servoing after reaching
                    self.mobile_base_servoing.stop_track()
                    self.pose()
                    return True
                elif current_state == "idle":
                    logger.warning("Servoing stopped without reaching target")
                    self.pose()
                    return False

                time.sleep(0.1)

            logger.warning("Servoing timeout reached")
            self.mobile_base_servoing.stop_track()
            self.pose()
            return False

        except Exception as e:
            logger.error(f"Servo to object failed: {e}")
            self.pose()
            return False

    def stop_servoing(self) -> Dict[str, Any]:
        """Stop visual servoing."""
        if self.mobile_base_servoing:
            return self.mobile_base_servoing.stop_track()
        else:
            return {"status": "error", "message": "Servoing module not initialized"}

    def move(self, twist: Twist, duration: float = 0.0):
        """Send movement command to Unitree B1 base via LCM.

        Args:
            twist: Velocity command
            duration: Duration to execute command (not used for LCM)
        """
        if self.lcm and self.cmd_vel_topic:
            logger.info(f"Publishing cmd_vel to topic: {self.CMD_VEL_TOPIC}")
            logger.info(
                f"Twist: linear=({twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f}), angular=({twist.angular.x:.2f}, {twist.angular.y:.2f}, {twist.angular.z:.2f})"
            )
            self.lcm.publish(self.cmd_vel_topic, twist)
            logger.info("Published cmd_vel successfully")
        else:
            logger.error(
                f"LCM initialized: {self.lcm is not None}, cmd_vel_topic initialized: {self.cmd_vel_topic is not None}"
            )

    def idle(self) -> bool:
        """Make robot go to idle mode via LCM."""
        if self.lcm and self.mode_topic:
            mode_msg = Int32()
            mode_msg.data = 0  # Idle mode
            self.lcm.publish(self.mode_topic, mode_msg)
            # Small delay to ensure message is processed
            import time

            time.sleep(0.1)
            logger.info("B1 robot set to idle mode")
            return True
        else:
            logger.error("LCM or mode topic not initialized")
            return False

    def pose(self) -> bool:
        """Make robot go to pose/stand mode via LCM."""
        if self.lcm and self.mode_topic:
            mode_msg = Int32()
            mode_msg.data = 1  # Stand/pose mode
            logger.info(f"Publishing mode 1 to topic: {self.B1_MODE_TOPIC}")
            self.lcm.publish(self.mode_topic, mode_msg)
            # Small delay to ensure message is processed
            import time

            time.sleep(0.1)
            logger.info("B1 robot set to pose mode")
            return True
        else:
            logger.error(
                f"LCM initialized: {self.lcm is not None}, mode_topic initialized: {self.mode_topic is not None}"
            )
            return False

    def walk(self) -> bool:
        """Make robot go to walk mode via LCM."""
        if self.lcm and self.mode_topic:
            mode_msg = Int32()
            mode_msg.data = 2  # Walk mode
            self.lcm.publish(self.mode_topic, mode_msg)
            # Small delay to ensure message is processed
            import time

            time.sleep(0.1)
            logger.info("B1 robot set to walk mode")
            return True
        else:
            logger.error("LCM or mode topic not initialized")
            return False

    def recovery(self) -> bool:
        """Make robot go to recovery mode via LCM."""
        if self.lcm and self.mode_topic:
            mode_msg = Int32()
            mode_msg.data = 6  # Recovery mode
            self.lcm.publish(self.mode_topic, mode_msg)
            logger.info("B1 robot set to recovery mode")
            return True
        else:
            logger.error("LCM or mode topic not initialized")
            return False

    def reset_arm(self) -> bool:
        """Reset arm to observe position via RPC."""
        try:
            self.rpc_client.call_sync(
                f"{self.XARM_MODULE}/goto_observe",
                ([], {}),
                rpc_timeout=10.0,
            )
            logger.info("XArm reset to observe position")
            return True
        except Exception as e:
            logger.error(f"Reset arm RPC failed: {e}")
            return False

    def open_gripper(self) -> bool:
        """Open gripper via RPC."""
        try:
            self.rpc_client.call_sync(
                f"{self.XARM_MODULE}/release_gripper",
                ([], {}),
                rpc_timeout=5.0,
            )
            return True
        except Exception as e:
            logger.error(f"Open gripper RPC failed: {e}")
            return False

    def close_gripper(self) -> bool:
        """Close gripper via RPC."""
        try:
            self.rpc_client.call_sync(
                f"{self.XARM_MODULE}/close_gripper",
                ([], {}),
                rpc_timeout=5.0,
            )
            return True
        except Exception as e:
            logger.error(f"Close gripper RPC failed: {e}")
            return False

    def handle_keyboard_command(self, key: str) -> Optional[str]:
        """Handle keyboard command via RPC to manipulation module.

        Args:
            key: Keyboard key pressed

        Returns:
            Action taken or None
        """
        try:
            return self.rpc_client.call_sync(
                f"{self.MANIPULATION_MODULE}/handle_keyboard_command",
                ([key], {}),
                rpc_timeout=2.0,
            )
        except Exception as e:
            logger.error(f"Handle keyboard command RPC failed: {e}")
            return None

    def stop(self):
        """Stop the robot and clean up connections."""
        logger.info("Stopping XArmTree robot...")

        # Stop servoing module
        if self.mobile_base_servoing:
            self.mobile_base_servoing.stop()
            self.mobile_base_servoing.cleanup()

        # Clean up RPC client
        if self.rpc_client:
            self.rpc_client.stop()

        # Clean up LCM
        if self.lcm:
            self.lcm.stop()

        # Close dimos
        if self.dimos:
            self.dimos.close()

        logger.info("XArmTree robot stopped")


def main():
    """Example usage of XArmTree robot."""
    # Create and start XArmTree robot
    robot = XArmTree()
    robot.start()

    logger.info("=" * 60)
    logger.info("XArmTree Robot Interface")
    logger.info("=" * 60)
    logger.info("This robot:")
    logger.info("  - Deploys MobileBasePBVS using ZED camera topics")
    logger.info("  - Communicates via RPC with:")
    logger.info("    1. XArmRobot (for manipulation)")
    logger.info("    2. UnitreeB1 (for base movement)")
    logger.info("=" * 60)

    try:
        # Example commands
        robot.pose()
        time.sleep(2)

        # Keep running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
