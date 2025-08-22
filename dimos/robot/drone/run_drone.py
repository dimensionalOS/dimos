#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.

"""Multiprocess drone example for DimOS."""

import os
import time
import logging

from dimos.protocol import pubsub
from dimos.robot.drone.drone import Drone
from dimos.utils.logging_config import setup_logger

# Configure logging
logger = setup_logger("dimos.robot.drone", level=logging.INFO)

# Suppress verbose loggers
logging.getLogger("distributed").setLevel(logging.WARNING)
logging.getLogger("asyncio").setLevel(logging.WARNING)


def main():
    """Main entry point for drone system."""
    # Get configuration from environment
    connection = os.getenv("DRONE_CONNECTION", "udp:0.0.0.0:14550")
    video_port = int(os.getenv("DRONE_VIDEO_PORT", "5600"))
    
    print(f"""
╔══════════════════════════════════════════╗
║         DimOS Drone System v1.0          ║
╠══════════════════════════════════════════╣
║  MAVLink: {connection:<30} ║
║  Video:   UDP port {video_port:<22} ║
║  Foxglove: http://localhost:8765        ║
╚══════════════════════════════════════════╝
    """)
    
    # Configure LCM
    pubsub.lcm.autoconf()
    
    # Create and start drone
    drone = Drone(
        connection_string=connection,
        video_port=video_port
    )
    
    drone.start()
    
    print("\n✓ Drone system started successfully!")
    print("\nLCM Topics:")
    print("  • /drone/odom           - Odometry (PoseStamped)")
    print("  • /drone/status         - Status (String/JSON)")
    print("  • /drone/color_image    - RGB Video (Image)")
    print("  • /drone/depth_image    - Depth estimation (Image)")
    print("  • /drone/depth_colorized - Colorized depth (Image)")
    print("  • /drone/camera_info    - Camera calibration")
    print("  • /drone/cmd_vel        - Movement commands (Vector3)")
    
    print("\nPress Ctrl+C to stop the system...")
    
    try:
        # Keep running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down drone system...")
        drone.stop()
        print("✓ Drone system stopped cleanly")


if __name__ == "__main__":
    main()