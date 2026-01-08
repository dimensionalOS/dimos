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

"""
Manipulation Skill Container

High-level manipulation skills for LLM agents, built on top of the
MoveAxisModule, ManipulationModule, and RobotCapabilities.
"""

import math

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.core.skill_module import SkillModule
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# ============================================================================
# Manipulation Skill Container
# ============================================================================


class ManipulationSkillContainer(SkillModule):
    """High-level manipulation skills for LLM agents.

    This skill container provides directional movement skills and other
    manipulation capabilities by using RPC methods from MoveAxisModule
    and ManipulationModule.
    """

    # Declare RPC dependencies from other modules
    rpc_calls: list[str] = [
        # From MoveAxisModule
        "MoveAxisModule.move_up",
        "MoveAxisModule.move_down",
        "MoveAxisModule.move_left",
        "MoveAxisModule.move_right",
        "MoveAxisModule.move_forward",
        "MoveAxisModule.move_backward",
        # From ManipulationModule
        "ManipulationModule.get_ee_pose",
        "ManipulationModule.move_to_pose",
        "ManipulationModule.get_current_joints",
        "ManipulationModule.move_to_joints",
        "ManipulationModule.get_state",
        "ManipulationModule.cancel",
        "ManipulationModule.reset",
        # From PerceptionModule
        "PerceptionModule.get_images",
        "PerceptionModule.get_grasp_pose",
        # From RobotCapabilities (for greet skill)
        "RobotCapabilities.speak",
    ]

    # =========================================================================
    # Directional Movement Skills
    # =========================================================================

    @skill()
    def move_arm_up(self, distance: float = 0.05) -> str:
        """Move the robot arm upward.

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            Status message describing the result
        """
        try:
            move_up = self.get_rpc_calls("MoveAxisModule.move_up")
            success = move_up(distance)
            if success:
                return f"Successfully moved arm up by {distance:.3f} meters"
            else:
                return f"Failed to move arm up by {distance:.3f} meters"
        except Exception as e:
            logger.error(f"move_arm_up failed: {e}")
            return f"Error: {e}"

    @skill()
    def move_arm_down(self, distance: float = 0.05) -> str:
        """Move the robot arm downward.

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            Status message describing the result
        """
        try:
            move_down = self.get_rpc_calls("MoveAxisModule.move_down")
            success = move_down(distance)
            if success:
                return f"Successfully moved arm down by {distance:.3f} meters"
            else:
                return f"Failed to move arm down by {distance:.3f} meters"
        except Exception as e:
            logger.error(f"move_arm_down failed: {e}")
            return f"Error: {e}"

    @skill()
    def move_arm_left(self, distance: float = 0.05) -> str:
        """Move the robot arm to the left.

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            Status message describing the result
        """
        try:
            move_left = self.get_rpc_calls("MoveAxisModule.move_left")
            success = move_left(distance)
            if success:
                return f"Successfully moved arm left by {distance:.3f} meters"
            else:
                return f"Failed to move arm left by {distance:.3f} meters"
        except Exception as e:
            logger.error(f"move_arm_left failed: {e}")
            return f"Error: {e}"

    @skill()
    def move_arm_right(self, distance: float = 0.05) -> str:
        """Move the robot arm to the right.

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            Status message describing the result
        """
        try:
            move_right = self.get_rpc_calls("MoveAxisModule.move_right")
            success = move_right(distance)
            if success:
                return f"Successfully moved arm right by {distance:.3f} meters"
            else:
                return f"Failed to move arm right by {distance:.3f} meters"
        except Exception as e:
            logger.error(f"move_arm_right failed: {e}")
            return f"Error: {e}"

    @skill()
    def move_arm_forward(self, distance: float = 0.05) -> str:
        """Move the robot arm forward.

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            Status message describing the result
        """
        try:
            move_forward = self.get_rpc_calls("MoveAxisModule.move_forward")
            success = move_forward(distance)
            if success:
                return f"Successfully moved arm forward by {distance:.3f} meters"
            else:
                return f"Failed to move arm forward by {distance:.3f} meters"
        except Exception as e:
            logger.error(f"move_arm_forward failed: {e}")
            return f"Error: {e}"

    @skill()
    def move_arm_backward(self, distance: float = 0.05) -> str:
        """Move the robot arm backward.

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            Status message describing the result
        """
        try:
            move_backward = self.get_rpc_calls("MoveAxisModule.move_backward")
            success = move_backward(distance)
            if success:
                return f"Successfully moved arm backward by {distance:.3f} meters"
            else:
                return f"Failed to move arm backward by {distance:.3f} meters"
        except Exception as e:
            logger.error(f"move_arm_backward failed: {e}")
            return f"Error: {e}"

    # =========================================================================
    # Additional Manipulation Skills
    # =========================================================================

    @skill()
    def move_arm_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> str:
        """Move the robot arm end-effector to a specific pose.

        IMPORTANT: If roll, pitch, or yaw are not provided (None), the current
        end-effector orientation values will be used. This means you can move
        to a new position while keeping the current orientation by only
        specifying x, y, z.

        Args:
            x: X position in meters (forward direction, front from base)
            y: Y position in meters (left direction, towards left from base)
            z: Z position in meters (up direction, vertical from base)
            roll: Rotation about X axis in radians (None to use current roll)
            pitch: Rotation about Y axis in radians (None to use current pitch)
            yaw: Rotation about Z axis in radians (None to use current yaw)

        Returns:
            Status message describing the result
        """
        try:
            get_ee_pose = self.get_rpc_calls("ManipulationModule.get_ee_pose")
            move_to_pose = self.get_rpc_calls("ManipulationModule.move_to_pose")

            # Get current pose to use current RPY if not provided
            pose = get_ee_pose()
            if pose is None:
                return "End-effector pose not available (no feedback received)"

            current_x, current_y, current_z, current_roll, current_pitch, current_yaw = pose

            # Use provided values or current values
            final_roll = roll if roll is not None else current_roll
            final_pitch = pitch if pitch is not None else current_pitch
            final_yaw = yaw if yaw is not None else current_yaw

            success = move_to_pose(x, y, z, final_roll, final_pitch, final_yaw)
            if success:
                return (
                    f"Successfully moved arm to pose: "
                    f"position=({x:.3f}, {y:.3f}, {z:.3f})m, "
                    f"orientation=({final_roll:.2f}, {final_pitch:.2f}, {final_yaw:.2f})rad"
                )
            else:
                return "Failed to move arm to target pose"
        except Exception as e:
            logger.error(f"move_arm_to_pose failed: {e}")
            return f"Error: {e}"

    @skill()
    def change_arm_orientation(
        self,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> str:
        """Change the end-effector orientation while keeping the current position.

        This skill changes only the orientation (roll, pitch, yaw) of the end-effector
        while maintaining its current position. If any orientation parameter is None,
        that axis will keep its current orientation.

        ORIENTATION EXPLANATION (XArm6 coordinate frame):
        XArm6 base coordinate frame:
        - X-axis: Forward (front direction, away from base)
        - Y-axis: Left (towards left from base)
        - Z-axis: Up (towards up, vertical from base)

        Euler angles (roll, pitch, yaw) in ZYX convention:
        - Roll: Rotation around X-axis (forward/front axis)
        - Pitch: Rotation around Y-axis (left axis)
        - Yaw: Rotation around Z-axis (up axis)

        IMPORTANT: When user requests "face up", "face down", "face right", etc.:
        - First call get_arm_position() to get current end-effector pose [x, y, z, roll, pitch, yaw]
        - Then use change_arm_orientation() to modify only the RPY values while keeping xyz position
        - Do NOT use predefined orientation values - always read current pose first

        Reference XArm6 orientations (for understanding, not for direct use):
        - Facing front: roll=2.55, pitch=-1.52, yaw=0.53
        - Facing down: roll=3.14, pitch=0.0, yaw=0.0

        Args:
            roll: Rotation about X-axis in radians (None to keep current)
            pitch: Rotation about Y-axis in radians (None to keep current)
            yaw: Rotation about Z-axis in radians (None to keep current)

        Returns:
            Status message describing the result
        """
        try:
            get_ee_pose = self.get_rpc_calls("ManipulationModule.get_ee_pose")
            move_to_pose = self.get_rpc_calls("ManipulationModule.move_to_pose")

            # Get current pose
            pose = get_ee_pose()
            if pose is None:
                return "End-effector pose not available (no feedback received)"

            x, y, z, current_roll, current_pitch, current_yaw = pose

            # Use provided values or keep current
            new_roll = roll if roll is not None else current_roll
            new_pitch = pitch if pitch is not None else current_pitch
            new_yaw = yaw if yaw is not None else current_yaw

            success = move_to_pose(x, y, z, new_roll, new_pitch, new_yaw)
            if success:
                return (
                    f"Successfully changed orientation to "
                    f"roll={new_roll:.3f}rad, pitch={new_pitch:.3f}rad, yaw={new_yaw:.3f}rad "
                    f"at position ({x:.3f}, {y:.3f}, {z:.3f})m"
                )
            else:
                return "Failed to change arm orientation"
        except Exception as e:
            logger.error(f"change_arm_orientation failed: {e}")
            return f"Error: {e}"

    @skill()
    def move_to_home_pose(self) -> str:
        """Move the robot arm to the home position.

        Home pose for XArm6:
        - Position: x=0.2m, y=0.0m, z=0.062m
        - Orientation: roll=3.14 rad, pitch=0.0 rad, yaw=0.0 rad

        Returns:
            Status message describing the result
        """
        try:
            move_to_pose = self.get_rpc_calls("ManipulationModule.move_to_pose")
            success = move_to_pose(0.2, 0.0, 0.062, 3.14, 0.0, 0.0)
            if success:
                return "Successfully moved arm to home pose: position=(0.200, 0.000, 0.062)m, orientation=(3.14, 0.00, 0.00)rad"
            else:
                return "Failed to move arm to home pose"
        except Exception as e:
            logger.error(f"move_to_home_pose failed: {e}")
            return f"Error: {e}"

    @skill()
    def get_arm_position(self) -> str:
        """Get the current position of the robot arm end-effector.

        Returns:
            Current end-effector pose as a formatted string
        """
        try:
            get_ee_pose = self.get_rpc_calls("ManipulationModule.get_ee_pose")
            pose = get_ee_pose()
            if pose is None:
                return "End-effector pose not available (no feedback received)"
            x, y, z, roll, pitch, yaw = pose
            return (
                f"End-effector pose: position=({x:.4f}, {y:.4f}, {z:.4f})m, "
                f"orientation=({roll:.3f}, {pitch:.3f}, {yaw:.3f})rad"
            )
        except Exception as e:
            logger.error(f"get_arm_position failed: {e}")
            return f"Error: {e}"

    @skill()
    def get_arm_state(self) -> str:
        """Get the current state of the robot arm controller.

        Returns the current manipulation state which can be:
        - IDLE (0): Ready for new commands
        - PLANNING (1): Currently planning a path
        - EXECUTING (2): Currently executing a trajectory
        - COMPLETED (3): Last motion completed successfully
        - FAULT (4): Error occurred, requires reset before new commands

        Use this to check if the arm is ready for commands, currently moving,
        or in a fault state that needs resetting.

        Returns:
            Current arm state as a formatted string
        """
        try:
            get_state = self.get_rpc_calls("ManipulationModule.get_state")
            state_value = get_state()

            state_names = {
                0: "IDLE",
                1: "PLANNING",
                2: "EXECUTING",
                3: "COMPLETED",
                4: "FAULT",
            }

            state_name = state_names.get(state_value, f"UNKNOWN({state_value})")

            if state_value == 4:  # FAULT
                return f"Arm state: {state_name} (value: {state_value}) - System is in fault state, call reset_arm() before attempting new commands"
            elif state_value == 2:  # EXECUTING
                return f"Arm state: {state_name} (value: {state_value}) - Arm is currently executing a movement"
            elif state_value == 1:  # PLANNING
                return f"Arm state: {state_name} (value: {state_value}) - System is currently planning a path"
            elif state_value == 3:  # COMPLETED
                return f"Arm state: {state_name} (value: {state_value}) - Last motion completed successfully, ready for new commands"
            else:  # IDLE
                return f"Arm state: {state_name} (value: {state_value}) - Ready for new commands"
        except Exception as e:
            logger.error(f"get_arm_state failed: {e}")
            return f"Error: {e}"

    @skill()
    def stop_arm_movement(self) -> str:
        """Stop the current arm movement immediately.

        Returns:
            Status message
        """
        try:
            cancel = self.get_rpc_calls("ManipulationModule.cancel")
            success = cancel()
            if success:
                return "Successfully stopped arm movement"
            else:
                return "Failed to stop arm movement"
        except Exception as e:
            logger.error(f"stop_arm_movement failed: {e}")
            return f"Error: {e}"

    @skill()
    def reset_arm(self) -> str:
        """Reset the manipulation system from FAULT or COMPLETED state to IDLE.

        This skill resets the arm controller state, clearing any errors and
        preparing the system for new commands. Required before executing new
        motions after a fault or completion.

        IMPORTANT: When inverse kinematics (IK) fails, the system enters FAULT
        state. You MUST call reset_arm() before attempting IK again. Without
        resetting, any subsequent IK attempts will fail because the system
        remains in FAULT state. Always reset after an IK failure before retrying
        any movement commands.

        Returns:
            Status message describing the result
        """
        try:
            reset = self.get_rpc_calls("ManipulationModule.reset")
            success = reset()
            if success:
                return "Successfully reset arm controller to IDLE state - ready for new commands"
            else:
                return "Cannot reset while executing (cancel first)"
        except Exception as e:
            logger.error(f"reset_arm failed: {e}")
            return f"Error: {e}"

    # =========================================================================
    # Perception Skills
    # =========================================================================

    @skill()
    def get_images(self) -> str:
        """Get the latest camera images (RGB and depth) from the perception system.

        This skill retrieves the most recent camera data captured by the perception
        module, including RGB color images and depth information. Use this to see
        what the robot's camera is currently viewing.

        Returns:
            Status message describing the retrieved images, including dimensions if available
        """
        try:
            get_images = self.get_rpc_calls("PerceptionModule.get_images")
            result = get_images()

            if "error" in result:
                return f"Error getting images: {result['error']}"

            has_rgb = result.get("has_rgb", False)
            result.get("has_depth", False)
            rgb_shape = result.get("rgb_shape")
            depth_shape = result.get("depth_shape")
            K = result.get("K")

            if not has_rgb:
                return "No RGB image available"

            msg = "Successfully retrieved images"
            if rgb_shape:
                # rgb_shape is a tuple like (height, width, channels) or (height, width)
                if len(rgb_shape) >= 2:
                    msg += f" - RGB: {rgb_shape[1]}x{rgb_shape[0]} pixels"
                else:
                    msg += f" - RGB: shape {rgb_shape}"
            if depth_shape:
                if len(depth_shape) >= 2:
                    msg += f" - Depth: {depth_shape[1]}x{depth_shape[0]} pixels"
                else:
                    msg += f" - Depth: shape {depth_shape}"
            if K:
                msg += " - Camera matrix available"

            return msg
        except Exception as e:
            logger.error(f"get_images failed: {e}")
            return f"Error: {e}"

    @skill()
    def get_grasp_pose(self, object_name: str) -> str:
        """Get grasp pose for a specific object.

        This skill requests grasp poses for a specified object from the perception
        system. The system will analyze the current camera view and return the best
        grasp pose (position and orientation) for grasping the specified object.

        Args:
            object_name: Name or label of the object to get grasp poses for (e.g., "cup", "bottle", "apple")

        Returns:
            Status message with the best grasp pose information, including position (x, y, z)
            and orientation (roll, pitch, yaw), or an error message if failed
        """
        try:
            get_grasp_pose = self.get_rpc_calls("PerceptionModule.get_grasp_pose")
            result = get_grasp_pose(label=object_name)

            if "error" in result:
                return f"Error getting grasp pose for '{object_name}': {result['error']}"

            transform = result.get("transform")
            score = result.get("score")
            gripper_type = result.get("gripper_type", "unknown")

            if transform is None or len(transform) != 6:
                return f"Invalid grasp pose data returned for '{object_name}'"

            x, y, z, roll, pitch, yaw = transform

            return (
                f"Best grasp pose for '{object_name}': "
                f"position=({x:.4f}, {y:.4f}, {z:.4f})m, "
                f"orientation=({roll:.3f}, {pitch:.3f}, {yaw:.3f})rad, "
                f"score={score:.3f}, gripper_type={gripper_type}"
            )
        except Exception as e:
            logger.error(f"get_grasp_pose failed: {e}")
            return f"Error: {e}"

    # =========================================================================
    # Greet Skill (responds with user's name)
    # =========================================================================

    @skill()
    def greet(self, name: str = "friend") -> str:
        """Greet someone by name using the robot's voice.

        This skill makes the robot say hello to a person. If you provide their name,
        the robot will greet them personally. For example, if the user says "My name
        is Alice", you should call greet("Alice").

        Args:
            name: Name of person to greet (default: "friend")

        Returns:
            Status message with greeting details
        """
        try:
            speak = self.get_rpc_calls("RobotCapabilities.speak")

            # Create personalized greeting
            if name and name.lower() != "friend":
                greeting_text = f"Hello, {name}! It's nice to meet you!"
            else:
                greeting_text = "Hello! Nice to meet you!"

            logger.info(f"[Skill] Greeting: {name}")
            speak(greeting_text)

            return f"Successfully greeted {name}"
        except Exception as e:
            logger.error(f"greet failed: {e}")
            return f"Error: {e}"

    # =========================================================================
    # Agent Registration
    # =========================================================================

    @rpc
    def set_LlmAgent_register_skills(self, register_skills: RpcCall) -> None:
        """Called by framework when composing with llm_agent().

        This method is discovered by convention during blueprint.build().
        It receives a callback to register this module's skills with the agent.

        Args:
            register_skills: Callback to register skills with the agent
        """
        register_skills.set_rpc(self.rpc)
        register_skills(RPCClient(self, self.__class__))


# Expose blueprint for declarative composition
manipulation_skill_container = ManipulationSkillContainer.blueprint

__all__ = [
    "ManipulationSkillContainer",
    "manipulation_skill_container",
]
