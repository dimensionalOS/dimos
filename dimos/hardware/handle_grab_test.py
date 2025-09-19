#!/usr/bin/env python3
"""
Handle Grab Test/Deployment Script

Deploys and connects the ZED camera module, handle grab module, and agent
for integrated handle detection and grasping.
"""

import time
import argparse
from pathlib import Path

from dimos.core import start, LCMTransport
from dimos.msgs.sensor_msgs import Image, CameraInfo
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.hardware.zed_camera import ZEDModule
from dimos.hardware.handle_grab_skill import HandleGrabModule
from dimos.agents2.agent import Agent
from dimos.agents2.cli.human import HumanInput
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


def main():
    """Main deployment function for handle grabbing system."""
    parser = argparse.ArgumentParser(
        description="Deploy handle detection and grasping system with ZED camera and agent interface",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with default settings (simulation only)
  python handle_grab_test.py

  # Run with real xARM robot
  python handle_grab_test.py --xarm 192.168.1.100

  # Run with custom FastSAM model
  python handle_grab_test.py --fastsam ./weights/FastSAM-x.pt --xarm 192.168.1.100

  # Run with custom approach distance
  python handle_grab_test.py --approach-distance 0.3 --iterations 2
        """,
    )

    # ZED Camera arguments
    parser.add_argument(
        "--zed-resolution",
        type=str,
        default="HD720",
        choices=["HD2K", "HD1080", "HD720", "VGA"],
        help="ZED camera resolution (default: HD720)",
    )
    parser.add_argument(
        "--zed-fps",
        type=int,
        default=30,
        help="ZED camera FPS (default: 30)",
    )
    parser.add_argument(
        "--zed-depth-mode",
        type=str,
        default="NEURAL",
        choices=["ULTRA", "QUALITY", "PERFORMANCE", "NEURAL"],
        help="ZED depth mode (default: NEURAL)",
    )

    # Handle Grab arguments
    parser.add_argument(
        "--fastsam",
        type=str,
        default="./weights/FastSAM-x.pt",
        help="Path to FastSAM model weights",
    )
    parser.add_argument(
        "--xarm",
        type=str,
        help="xARM IP address for robot control (omit for simulation only)",
    )
    parser.add_argument(
        "--approach-distance",
        type=float,
        default=0.25,
        help="Distance to approach from handle (meters, default: 0.25)",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=3,
        help="Maximum refinement iterations (default: 3)",
    )
    parser.add_argument(
        "--no-visualization",
        action="store_true",
        help="Disable Drake visualization",
    )

    # LCM transport arguments
    parser.add_argument(
        "--lcm-image-channel",
        default="/zed/image",
        help="LCM channel for color image (default: /zed/image)",
    )
    parser.add_argument(
        "--lcm-depth-channel",
        default="/zed/depth",
        help="LCM channel for depth image (default: /zed/depth)",
    )
    parser.add_argument(
        "--lcm-camera-info-channel",
        default="/zed/camera_info",
        help="LCM channel for camera info (default: /zed/camera_info)",
    )
    parser.add_argument(
        "--lcm-pose-channel",
        default="/zed/pose",
        help="LCM channel for camera pose (default: /zed/pose)",
    )

    # General arguments
    parser.add_argument(
        "--processes",
        type=int,
        default=3,
        help="Number of Dimos processes (default: 3)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output",
    )

    args = parser.parse_args()

    # Check FastSAM model
    if args.fastsam:
        fastsam_path = Path(args.fastsam)
        if not fastsam_path.exists():
            logger.warning(f"FastSAM model not found at {fastsam_path}")
            logger.warning("Segmentation features will be limited")

    # Start Dimos
    logger.info("=" * 60)
    logger.info("Handle Detection and Grasping System")
    logger.info("=" * 60)
    logger.info(f"Starting Dimos with {args.processes} processes...")
    dimos = start(args.processes)

    # Deploy ZED camera module
    logger.info("Deploying ZED camera module...")
    logger.info(f"  Resolution: {args.zed_resolution}")
    logger.info(f"  FPS: {args.zed_fps}")
    logger.info(f"  Depth mode: {args.zed_depth_mode}")

    # Pass string arguments directly - ZEDModule handles the conversion
    zed = dimos.deploy(
        ZEDModule,
        resolution=args.zed_resolution,
        fps=args.zed_fps,
        depth_mode=args.zed_depth_mode,
        enable_tracking=True,
    )

    # Set up LCM transport for ZED outputs
    logger.info("Setting up LCM transports for ZED...")
    zed.color_image.transport = LCMTransport(args.lcm_image_channel, Image)
    zed.depth_image.transport = LCMTransport(args.lcm_depth_channel, Image)
    zed.camera_info.transport = LCMTransport(args.lcm_camera_info_channel, CameraInfo)
    zed.pose.transport = LCMTransport(args.lcm_pose_channel, PoseStamped)
    logger.info(f"  Color image: {args.lcm_image_channel}")
    logger.info(f"  Depth image: {args.lcm_depth_channel}")
    logger.info(f"  Camera info: {args.lcm_camera_info_channel}")
    logger.info(f"  Pose: {args.lcm_pose_channel}")

    # Deploy handle grab module
    logger.info("Deploying handle grab module...")
    logger.info(f"  FastSAM model: {args.fastsam or 'None'}")
    logger.info(f"  xARM IP: {args.xarm or 'None (simulation only)'}")
    logger.info(f"  Approach distance: {args.approach_distance}m")
    logger.info(f"  Max iterations: {args.iterations}")
    logger.info(f"  Visualization: {'Disabled' if args.no_visualization else 'Enabled'}")

    handle_grab = dimos.deploy(
        HandleGrabModule,
        fastsam_model_path=args.fastsam,
        xarm_ip=args.xarm,
        approach_distance=args.approach_distance,
        max_iterations=args.iterations,
        enable_visualization=not args.no_visualization,
    )

    # Connect handle grab module to ZED outputs
    handle_grab.color_image.connect(zed.color_image)
    handle_grab.depth_image.connect(zed.depth_image)
    handle_grab.camera_info.connect(zed.camera_info)
    logger.info("  Connected to ZED camera streams")

    # Deploy human input module
    logger.info("Deploying human input module...")
    human_input = dimos.deploy(HumanInput)

    # Deploy agent
    logger.info("Deploying agent...")
    agent = dimos.deploy(
        Agent,
        system_prompt="""You are a helpful robotic assistant that can detect and approach handles using computer vision.

Available skills:
- approach_handle: Detect and iteratively approach a handle for grasping (can specify iterations)
- get_handle_detection: Get information about the last handle detection

The approach_handle skill will:
1. Capture the current view from the ZED camera
2. Detect a handle point in the image
3. Segment the handle using FastSAM
4. Estimate the surface normal
5. Move the robot to an approach position
6. Repeat 2-3 times to refine the approach

Example commands:
- "Approach the handle" - Will run the default 3 iterations
- "Approach the handle with 2 iterations" - Will run 2 iterations
- "What's the last handle detection?" - Gets detection info

The robot will get progressively closer and more aligned with each iteration.
"""
    )

    # Register skills
    agent.register_skills(handle_grab)
    agent.register_skills(human_input)
    logger.info("  Registered handle grab and human input skills")

    # Start all modules
    logger.info("=" * 60)
    logger.info("Starting modules...")
    logger.info("=" * 60)

    # Start ZED camera
    zed.start()
    logger.info("ZED camera started")

    # Start handle grab module
    handle_grab.start()
    logger.info("Handle grab module started")

    # Start agent
    agent.run_implicit_skill("human")
    agent.start()
    agent.loop_thread()
    logger.info("Agent started with human interface")

    logger.info("=" * 60)
    logger.info("All modules started successfully!")
    logger.info("")
    logger.info("You can now interact with the handle detection system through the agent.")
    logger.info("Example commands:")
    logger.info("  'Approach the handle'")
    logger.info("  'Approach the handle with 2 iterations'")
    logger.info("  'What's the last handle detection?'")
    logger.info("")
    if not args.no_visualization and hasattr(handle_grab, 'meshcat') and handle_grab.meshcat:
        try:
            meshcat_url = handle_grab.meshcat.web_url()
            logger.info(f"Drake visualization available at: {meshcat_url}")
        except:
            pass
    logger.info("")
    logger.info("Press Ctrl+C to stop...")
    logger.info("=" * 60)

    # Main loop - keep running
    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        logger.info("")
        logger.info("=" * 60)
        logger.info("Shutting down...")
        logger.info("=" * 60)

        # Stop modules
        zed.stop()
        handle_grab.stop()

        # Shutdown Dimos
        time.sleep(0.5)
        dimos.shutdown()

        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()