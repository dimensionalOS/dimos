#!/usr/bin/env python3
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

"""
Example demonstrating the new modular Unitree Go2 implementation.

This example shows how to:
1. Initialize a Unitree Go2 robot without hard-coded model dependencies
2. Configure optional stream processors (person tracking, object detection, etc.)
3. Run the robot on CPU without heavy model dependencies
4. Dynamically enable/disable processing capabilities

The robot will run with only basic functionality if model dependencies are not available,
but can enable advanced features when the dependencies are installed.
"""

import time
import logging
from typing import Dict, Any

from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.stream.processors import get_loaded_processors

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Main example function."""

    # Check which processors are available
    available_processors = get_loaded_processors()
    logger.info("Available stream processors:")
    for name, available in available_processors.items():
        status = "✓" if available else "✗"
        logger.info(f"  {status} {name}")

    # Example 1: Basic robot without any stream processing
    logger.info("\n=== Example 1: Basic Robot (CPU-only, no models) ===")

    try:
        robot_basic = UnitreeGo2(
            use_ros=True,
            use_webrtc=False,
            mock_connection=True,  # Use mock for demo
            disable_video_stream=True,  # No video processing
        )

        logger.info("✓ Basic robot initialized successfully")

        # Basic movement
        robot_basic.move_vel(x=0.1, y=0.0, yaw=0.0, duration=1.0)
        logger.info("✓ Basic movement command sent")

        robot_basic.cleanup()

    except Exception as e:
        logger.error(f"✗ Basic robot failed: {e}")

    # Example 2: Robot with optional stream processors
    logger.info("\n=== Example 2: Robot with Stream Processors ===")

    # Configure processors to enable (only if available)
    desired_processors = ["person_tracking", "object_detection", "depth_estimation"]
    enabled_processors = [p for p in desired_processors if available_processors.get(p, False)]

    if enabled_processors:
        logger.info(f"Enabling processors: {enabled_processors}")

        # Processor configurations
        processor_configs = {
            "person_tracking": {
                "model_path": "yolo11n.pt",
                "device": "cpu",  # Use CPU for compatibility
                "confidence_threshold": 0.5,
            },
            "object_detection": {
                "model_path": "yolo11n.pt",
                "device": "cpu",
                "confidence_threshold": 0.5,
                "class_filter": [0, 1, 2, 3, 5],  # person, bicycle, car, motorcycle, bus
            },
            "depth_estimation": {"device": "cpu", "gt_depth_scale": 1000.0},
        }

        try:
            robot_advanced = UnitreeGo2(
                use_ros=True,
                use_webrtc=False,
                mock_connection=True,
                stream_processors=enabled_processors,
                processor_configs=processor_configs,
            )

            logger.info("✓ Advanced robot with stream processors initialized")

            # Get processed stream
            processed_stream = robot_advanced.get_processed_stream("main")
            if processed_stream:
                logger.info("✓ Processed stream available")

                # Subscribe to processed stream for a few seconds
                def on_processed_frame(result):
                    targets = result.get("targets", [])
                    processor = result.get("processor", "unknown")
                    if targets:
                        logger.info(f"Processor {processor} detected {len(targets)} targets")

                subscription = processed_stream.subscribe(on_processed_frame)

                # Let it run for a few seconds
                time.sleep(3)
                subscription.dispose()

            robot_advanced.cleanup()

        except Exception as e:
            logger.error(f"✗ Advanced robot failed: {e}")

    else:
        logger.info("No stream processors available - install YOLO/Metric3D for advanced features")

    # Example 3: Custom processor configuration
    logger.info("\n=== Example 3: Custom Processor Configuration ===")

    if available_processors.get("person_tracking", False):
        try:
            # Custom configuration for person tracking only
            custom_config = {
                "person_tracking": {
                    "model_path": "yolo11s.pt",  # Larger model
                    "device": "cpu",
                    "camera_height": 0.5,  # Custom camera height
                    "camera_pitch": 0.1,  # Custom camera pitch
                }
            }

            robot_custom = UnitreeGo2(
                use_ros=True,
                use_webrtc=False,
                mock_connection=True,
                stream_processors=["person_tracking"],
                processor_configs=custom_config,
            )

            logger.info("✓ Robot with custom processor configuration initialized")

            # Check available processors on this robot instance
            robot_processors = robot_custom.get_available_processors()
            logger.info(f"Robot processors: {robot_processors}")

            robot_custom.cleanup()

        except Exception as e:
            logger.error(f"✗ Custom robot failed: {e}")

    # Example 4: Graceful degradation
    logger.info("\n=== Example 4: Graceful Degradation ===")

    # Try to enable all possible processors, but gracefully handle missing ones
    all_possible_processors = [
        "person_tracking",
        "object_tracking",
        "object_detection",
        "depth_estimation",
        "semantic_segmentation",
    ]

    try:
        robot_degraded = UnitreeGo2(
            use_ros=True,
            use_webrtc=False,
            mock_connection=True,
            stream_processors=all_possible_processors,  # Will only enable available ones
            processor_configs={
                # Default CPU configurations for all
                proc: {"device": "cpu"}
                for proc in all_possible_processors
            },
        )

        logger.info("✓ Robot with graceful degradation initialized")

        # Check what actually got enabled
        if robot_degraded.stream_pipeline:
            enabled_count = len(robot_degraded.stream_pipeline.processors)
            logger.info(
                f"Successfully enabled {enabled_count} out of {len(all_possible_processors)} processors"
            )
        else:
            logger.info("No processors enabled - running in basic mode")

        robot_degraded.cleanup()

    except Exception as e:
        logger.error(f"✗ Degraded robot failed: {e}")

    logger.info("\n=== Examples completed ===")


if __name__ == "__main__":
    main()
