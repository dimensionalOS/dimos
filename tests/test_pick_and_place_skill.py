#!/usr/bin/env python3
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
Run script for Piper Arm robot with pick and place functionality.
Uses hardcoded points and the PickAndPlace skill.
"""

import sys
import asyncio

try:
    import pyzed.sl as sl  # Required for ZED camera
except ImportError:
    print("Error: ZED SDK not installed.")
    sys.exit(1)

from dimos.robot.agilex.piper_arm import PiperArmRobot
from dimos.skills.manipulation.pick_and_place import PickAndPlace
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.agilex.run_robot")


async def run_piper_arm():
    """Run the Piper Arm robot with pick and place skill."""
    logger.info("Starting Piper Arm Robot")

    # Create robot instance
    robot = PiperArmRobot()

    try:
        # Start the robot
        await robot.start()

        # Give modules time to fully initialize
        await asyncio.sleep(3)

        # Add the PickAndPlace skill to the robot's skill library
        robot.skill_library.add(PickAndPlace)

        logger.info("Robot initialized successfully")
        print("\n=== Piper Arm Robot - Pick and Place Demo ===")
        print("This demo uses hardcoded pick and place points.")
        print("\nCommands:")
        print("  1. Run pick and place with hardcoded points")
        print("  2. Run pick-only with hardcoded point")
        print("  r. Reset robot to idle")
        print("  q. Quit")
        print("")

        running = True
        while running:
            try:
                # Get user input
                command = input("\nEnter command: ").strip().lower()

                if command == "q":
                    logger.info("Quit requested")
                    running = False
                    break

                elif command == "r" or command == "s":
                    logger.info("Resetting robot")
                    robot.handle_keyboard_command(command)

                elif command == "1":
                    # Hardcoded pick and place points
                    # These should be adjusted based on your camera view
                    print("\nExecuting pick and place with hardcoded points...")

                    # Create and execute the skill
                    skill = PickAndPlace(
                        robot=robot,
                        object_query="labubu doll",  # Will use visual detection
                        target_query="on the keyboard",  # Will use visual detection
                    )

                    result = skill()

                    if result["success"]:
                        print(f"✓ {result['message']}")
                    else:
                        print(f"✗ Failed: {result.get('error', 'Unknown error')}")

                elif command == "2":
                    # Pick-only with hardcoded point
                    print("\nExecuting pick-only with hardcoded point...")

                    # Create and execute the skill for pick-only
                    skill = PickAndPlace(
                        robot=robot,
                        object_query="labubu doll",  # Will use visual detection
                        target_query=None,  # No place target - pick only
                    )

                    result = skill()

                    if result["success"]:
                        print(f"✓ {result['message']}")
                    else:
                        print(f"✗ Failed: {result.get('error', 'Unknown error')}")

                else:
                    print("Invalid command. Please try again.")

                # Small delay to prevent CPU spinning
                await asyncio.sleep(0.1)

            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received")
                running = False
                break
            except Exception as e:
                logger.error(f"Error in command loop: {e}")
                print(f"Error: {e}")

    except Exception as e:
        logger.error(f"Error running robot: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        logger.info("Shutting down robot...")
        await robot.stop()
        logger.info("Robot stopped")


def main():
    """Main entry point."""
    print("Starting Piper Arm Robot...")
    print("Note: The robot will use Qwen VLM to identify objects and locations")
    print("based on the queries specified in the code.")

    # Run the robot
    asyncio.run(run_piper_arm())


if __name__ == "__main__":
    main()
