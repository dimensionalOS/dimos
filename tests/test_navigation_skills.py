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
Simple test script for semantic map skills.

This script is a simplified version that focuses only on making the workflow work.

Usage:
  # Build and query in one run:
  python simple_navigation_test.py --query "kitchen"

  # Skip build and just query:
  python simple_navigation_test.py --skip-build --query "kitchen"
"""

import os
import sys
import time
import logging
import argparse

import tests.test_header

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.skills.navigation import BuildSemanticMap, Navigate
from dimos.utils.logging_config import setup_logger

# Setup logging
logger = setup_logger("simple_navigation_test")

def parse_args():
    parser = argparse.ArgumentParser(description="Simple test for semantic map skills.")
    parser.add_argument("--skip-build", action="store_true", 
                      help="Skip building the map and only run navigation")
    parser.add_argument("--query", type=str, default="kitchen",
                      help="Text query for navigation (default: kitchen)")
    parser.add_argument("--db-path", type=str, 
                      default="/home/stash/dimensional/dimos/assets/semantic_map/chromadb_data",
                      help="Path to ChromaDB database")
    parser.add_argument("--visual-memory-dir", type=str, 
                      default="/home/stash/dimensional/dimos/assets/semantic_map",
                      help="Directory for visual memory")
    parser.add_argument("--visual-memory-file", type=str, 
                      default="visual_memory.pkl",
                      help="Filename for visual memory")
    return parser.parse_args()

def build_map(robot, args):
    logger.info("Starting to build semantic map...")
    
    # Create the BuildSemanticMap skill
    build_skill = BuildSemanticMap(
        robot=robot,
        db_path=args.db_path,
        visual_memory_dir=args.visual_memory_dir,
        visual_memory_file=args.visual_memory_file
    )
    
    # Start the skill
    build_skill()
    
    # Wait for user to press Ctrl+C
    logger.info("Press Ctrl+C to stop mapping and proceed to navigation...")
    
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info("Stopping map building...")
        
    # Stop the skill
    build_skill.stop()
    logger.info("Map building complete.")

def query_map(robot, args):
    logger.info(f"Querying semantic map for: '{args.query}'")
    
    # Create the Navigate skill
    nav_skill = Navigate(
        robot=robot,
        query=args.query,
        db_path=args.db_path,
        visual_memory_path=os.path.join(args.visual_memory_dir, args.visual_memory_file)
    )
    
    # Query the map
    result = nav_skill()
    
    # Display the result
    if isinstance(result, dict) and result.get('success', False):
        position = result.get('position', (0, 0, 0))
        similarity = result.get('similarity', 0)
        logger.info(f"Found '{args.query}' at position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
        logger.info(f"Similarity score: {similarity:.4f}")
        return position

    else:
        logger.error(f"Navigation query failed: {result}")
        return False

def main():
    args = parse_args()
    
    # Ensure directories exist
    os.makedirs(args.db_path, exist_ok=True)
    os.makedirs(args.visual_memory_dir, exist_ok=True)
    
    # Initialize robot
    logger.info("Initializing robot...")
    ros_control = UnitreeROSControl(node_name="simple_nav_test", mock_connection=False)
    robot = UnitreeGo2(ros_control=ros_control, ip=os.getenv('ROBOT_IP'))
    
    try:
        # Build map if not skipped
        if not args.skip_build:
            build_map(robot, args)
        
        # Query the map
        target = query_map(robot, args)

        if not target:
            logger.error("No target found for navigation.")
            return

        # Nav
        robot.global_planner.set_goal(target)

        
    finally:
        # Clean up
        logger.info("Cleaning up resources...")
        try:
            robot.cleanup()
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
    
    logger.info("Test completed successfully")

if __name__ == "__main__":
    main()
