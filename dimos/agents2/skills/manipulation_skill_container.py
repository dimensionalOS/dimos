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
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> str:
        """Move the robot arm end-effector to a specific pose.

        Args:
            x: X position in meters (forward/backward from base)
            y: Y position in meters (left/right from base)
            z: Z position in meters (up/down from base)
            roll: Rotation about X axis in radians (default: 0.0)
            pitch: Rotation about Y axis in radians (default: 0.0)
            yaw: Rotation about Z axis in radians (default: 0.0)

        Returns:
            Status message describing the result
        """
        try:
            move_to_pose = self.get_rpc_calls("ManipulationModule.move_to_pose")
            success = move_to_pose(x, y, z, roll, pitch, yaw)
            if success:
                return (
                    f"Successfully moved arm to pose: "
                    f"position=({x:.3f}, {y:.3f}, {z:.3f})m, "
                    f"orientation=({roll:.2f}, {pitch:.2f}, {yaw:.2f})rad"
                )
            else:
                return "Failed to move arm to target pose"
        except Exception as e:
            logger.error(f"move_arm_to_pose failed: {e}")
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
