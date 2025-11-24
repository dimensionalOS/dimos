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
Simple test script for semantic / spatial memory skills.

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
import threading
from reactivex import Subject, operators as RxOps
import os

import tests.test_header

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.skills.navigation import BuildSemanticMap, Navigate
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

# Setup logging
logger = setup_logger("simple_navigation_test")


def parse_args():

    spatial_memory_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../assets/spatial_memory_vegas"))
    
    parser = argparse.ArgumentParser(description="Simple test for semantic map skills.")
    parser.add_argument("--skip-build", action="store_true", help="Skip building the map and run navigation with existing semantic and visual memory")
    parser.add_argument("--query", type=str, default="kitchen", help="Text query for navigation (default: kitchen)")
    parser.add_argument(
        "--db-path",
        type=str,
        default=os.path.join(spatial_memory_dir, "chromadb_data"),
        help="Path to ChromaDB database",
    )
    parser.add_argument("--justgo", type=str, help="Globally navigate to location")
    parser.add_argument(
        "--visual-memory-dir",
        type=str,
        default=spatial_memory_dir,
        help="Directory for visual memory",
    )
    parser.add_argument(
        "--visual-memory-file", type=str, default="visual_memory.pkl", help="Filename for visual memory"
    )
    parser.add_argument(
        "--port", type=int, default=5555, help="Port for web visualization interface"
    )
    return parser.parse_args()


def build_map(robot, args):
    logger.info("Starting to build spatial memory...")

    # Create the BuildSemanticMap skill
    build_skill = BuildSemanticMap(
        robot=robot,
        db_path=args.db_path,
        visual_memory_dir=args.visual_memory_dir,
        visual_memory_file=args.visual_memory_file,
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
    logger.info(f"Querying spatial memory for: '{args.query}'")

    # Create the Navigate skill
    nav_skill = Navigate(
        robot=robot,
        query=args.query,
        db_path=args.db_path,
        visual_memory_path=os.path.join(args.visual_memory_dir, args.visual_memory_file),
    )

    # Query the map
    result = nav_skill()

    # Display the result
    if isinstance(result, dict) and result.get("success", False):
        position = result.get("position", (0, 0, 0))
        similarity = result.get("similarity", 0)
        logger.info(f"Found '{args.query}' at position: {position}")
        logger.info(f"Similarity score: {similarity:.4f}")
        return position

    else:
        logger.error(f"Navigation query failed: {result}")
        return False


def setup_visualization(robot, port=5555):
    """Set up visualization streams for the web interface"""
    logger.info(f"Setting up visualization streams on port {port}")
    
    # Get video stream from robot
    video_stream = robot.video_stream_ros.pipe(
        RxOps.share(),
        RxOps.map(lambda frame: frame),
        RxOps.filter(lambda frame: frame is not None),
    )
    
    # Get local planner visualization stream
    local_planner_stream = robot.local_planner_viz_stream.pipe(
        RxOps.share(),
        RxOps.map(lambda frame: frame),
        RxOps.filter(lambda frame: frame is not None),
    )
    
    # Create web interface with streams
    streams = {
        "robot_video": video_stream,
        "local_planner": local_planner_stream
    }
    
    web_interface = RobotWebInterface(
        port=port,
        **streams
    )
    
    return web_interface


def run_navigation(robot, target):
    """Run navigation in a separate thread"""
    logger.info(f"Starting navigation to target: {target}")
    return robot.global_planner.set_goal(target)


def main():
    args = parse_args()

    # Ensure directories exist
    if not args.justgo:
        os.makedirs(args.db_path, exist_ok=True)
        os.makedirs(args.visual_memory_dir, exist_ok=True)

    # Initialize robot
    logger.info("Initializing robot...")
    ros_control = UnitreeROSControl(node_name="simple_nav_test", mock_connection=False)
    robot = UnitreeGo2(ros_control=ros_control, ip=os.getenv("ROBOT_IP"), skills=MyUnitreeSkills())

    # Set up visualization
    web_interface = None
    try:
        # Set up visualization first if the robot has video capabilities
        if hasattr(robot, 'video_stream_ros') and robot.video_stream_ros is not None:
            web_interface = setup_visualization(robot, port=args.port)
            # Start web interface in a separate thread
            viz_thread = threading.Thread(target=web_interface.run, daemon=True)
            viz_thread.start()
            logger.info(f"Web visualization available at http://localhost:{args.port}")
            # Wait a moment for the web interface to initialize
            time.sleep(2)
        
        if args.justgo:
            # Just go to the specified location
            coords = list(map(float, args.justgo.split(",")))
            logger.info(f"Navigating to coordinates: {coords}")
            
            # Run navigation
            navigate_thread = threading.Thread(
                target=lambda: run_navigation(robot, coords),
                daemon=True
            )
            navigate_thread.start()
            
            # Wait for navigation to complete or user to interrupt
            try:
                while navigate_thread.is_alive():
                    time.sleep(0.5)
                logger.info("Navigation completed")
            except KeyboardInterrupt:
                logger.info("Navigation interrupted by user")
        else:
            # Build map if not skipped
            if not args.skip_build:
                build_map(robot, args)

            # Query the map
            target = query_map(robot, args)

            if not target:
                logger.error("No target found for navigation.")
                return
            
            # Run navigation
            navigate_thread = threading.Thread(
                target=lambda: run_navigation(robot, target),
                daemon=True
            )
            navigate_thread.start()
            
            # Wait for navigation to complete or user to interrupt
            try:
                while navigate_thread.is_alive():
                    time.sleep(0.5)
                logger.info("Navigation completed")
            except KeyboardInterrupt:
                logger.info("Navigation interrupted by user")

        # If web interface is running, keep the main thread alive
        if web_interface:
            logger.info("Navigation completed. Visualization still available. Press Ctrl+C to exit.")
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                logger.info("Exiting...")
                
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
