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
Test script for PerceptionModule to verify camera connection and get_grasp_pose functionality.

This script:
1. Initializes PerceptionModule
2. Starts the camera
3. Waits for images to arrive
4. Tests get_images() method
5. Tests get_grasp_pose() method with a sample object
"""

import logging
from pathlib import Path
import sys
import time

# Add dimos to path
sys.path.insert(0, str(Path(__file__).parent))

from dimos.core import start
from dimos.manipulation.modules.perception import PerceptionModule, PerceptionModuleConfig

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger(__name__)


def test_perception_module():
    """Test PerceptionModule camera connection and grasp pose functionality."""
    logger.info("=" * 60)
    logger.info("Testing PerceptionModule")
    logger.info("=" * 60)

    # Start DimOS cluster
    logger.info("\n1. Starting DimOS cluster...")
    dimos = start(1)

    try:
        # Deploy PerceptionModule
        logger.info("\n2. Deploying PerceptionModule...")
        # Pass config fields as keyword arguments
        perception = dimos.deploy(
            PerceptionModule,
            grasp_service_url="http://localhost:8080/grasp",
            calibration_file="/home/ruthwik/Documents/dimos/calibration_result.json",
            fps=30,
            rgb_resolution="1080p",
            mono_resolution="720p",
        )
        logger.info("   PerceptionModule deployed successfully")

        # Start the module (this starts the camera)
        logger.info("\n3. Starting PerceptionModule (starting camera)...")
        perception.start()
        logger.info("   PerceptionModule started")

        # Wait for camera to initialize and start receiving images
        logger.info("\n4. Waiting for camera to initialize and receive images...")
        max_wait = 10  # seconds
        wait_interval = 0.5
        waited = 0

        while waited < max_wait:
            time.sleep(wait_interval)
            waited += wait_interval

            # Check if images are available
            result = perception.get_images()
            has_rgb = result.get("has_rgb", False)
            has_depth = result.get("has_depth", False)

            logger.info(f"   Waiting... (RGB: {has_rgb}, Depth: {has_depth})")

            if has_rgb and has_depth:
                logger.info("   ✓ Images are now available!")
                break

        # Test get_images
        logger.info("\n5. Testing get_images() method...")
        result = perception.get_images()

        logger.info("   Result:")
        logger.info(f"     RGB available: {result.get('has_rgb', False)}")
        logger.info(f"     Depth available: {result.get('has_depth', False)}")

        if result.get("rgb_shape"):
            logger.info(f"     RGB shape: {result['rgb_shape']}")
        if result.get("depth_shape"):
            logger.info(f"     Depth shape: {result['depth_shape']}")
        if result.get("K"):
            logger.info("     Camera matrix K: Available")

        if not result.get("has_rgb"):
            logger.error("   ✗ ERROR: No RGB image available!")
            logger.error("   This means the camera is not receiving images.")
            return False

        if not result.get("has_depth"):
            logger.warning("   ⚠ WARNING: No depth image available")

        # Test get_grasp_pose
        logger.info("\n6. Testing get_grasp_pose() method...")
        logger.info("   Calling get_grasp_pose('cokecan')...")

        # Get image info first to show what we're sending
        img_info = perception.get_images()
        logger.info("   Request details:")
        logger.info("     Object label: 'cokecan'")
        logger.info(f"     RGB image size: {img_info.get('rgb_shape')}")
        logger.info(f"     Depth image size: {img_info.get('depth_shape')}")
        logger.info(
            f"     Camera matrix K: {'Available' if img_info.get('K') else 'Not available'}"
        )
        logger.info("     Service URL: http://localhost:8080/grasp")

        grasp_result = perception.get_grasp_pose(label="cokecan", gripper_type="robotiq_2f_140")

        logger.info("\n   Service Response:")
        logger.info("   " + "-" * 50)
        if "error" in grasp_result:
            logger.error(f"     ✗ ERROR: {grasp_result['error']}")

            # Try to get more details if it's a service error
            if "Service request failed" in grasp_result["error"]:
                logger.error("\n   Service Error Details:")
                logger.error("     - The grasp service may not be running")
                logger.error("     - Check if service is available at http://localhost:8080/grasp")
                logger.error("     - Verify the service accepts the expected request format")
                logger.error(f"     - Full error: {grasp_result['error']}")

            # Provide helpful diagnostics
            if "No RGB image" in grasp_result["error"]:
                logger.error("\n   DIAGNOSTICS:")
                logger.error("     - Camera may not be connected")
                logger.error("     - Camera may not be started properly")
                logger.error("     - Check camera hardware initialization logs")
            elif "No Depth image" in grasp_result["error"]:
                logger.error("\n   DIAGNOSTICS:")
                logger.error("     - Depth stream may not be working")
                logger.error("     - Check depth camera initialization")
            elif "No camera matrix" in grasp_result["error"]:
                logger.error("\n   DIAGNOSTICS:")
                logger.error("     - Camera info not extracted properly")
                logger.error("     - Check camera_info extraction in start()")
            elif "Calibration" in grasp_result["error"]:
                logger.error("\n   DIAGNOSTICS:")
                logger.error("     - Calibration file not found or invalid")
                logger.error(
                    "     - Expected: /home/ruthwik/Documents/dimos/calibration_result.json"
                )

            logger.info("   " + "-" * 50)
            return False
        else:
            logger.info("     ✓ SUCCESS: Grasp pose retrieved!")
            logger.info("")
            if "transform" in grasp_result:
                transform = grasp_result["transform"]
                x, y, z, roll, pitch, yaw = transform
                logger.info(f"     Position (x, y, z): ({x:.4f}, {y:.4f}, {z:.4f})")
                logger.info(
                    f"     Orientation (roll, pitch, yaw): ({roll:.4f}, {pitch:.4f}, {yaw:.4f})"
                )
            if "score" in grasp_result:
                logger.info(f"     Score: {grasp_result['score']:.4f}")
            if "gripper_type" in grasp_result:
                logger.info(f"     Gripper type: {grasp_result['gripper_type']}")
            logger.info("   " + "-" * 50)
            return True

    except Exception as e:
        logger.error(f"\n✗ ERROR during test: {e}")
        import traceback

        traceback.print_exc()
        return False

    finally:
        # Clean up
        logger.info("\n7. Cleaning up...")
        try:
            if "perception" in locals():
                perception.stop()
                logger.info("   PerceptionModule stopped")
        except Exception as e:
            logger.warning(f"   Error stopping module: {e}")

        dimos.close()
        logger.info("   DimOS cluster closed")
        logger.info("\n" + "=" * 60)
        logger.info("Test completed")
        logger.info("=" * 60)


if __name__ == "__main__":
    success = test_perception_module()
    sys.exit(0 if success else 1)
