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
ZED Camera Neural Depth Demo - Simple Live Visualization

This script demonstrates live visualization of ZED camera RGB and depth data using matplotlib.
Much simpler than the previous PIL-based approach.
"""

import os
import sys
import time
import argparse
import logging
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import cv2

# Add the project root to Python path
sys.path.append(str(Path(__file__).parent.parent))

try:
    import pyzed.sl as sl
except ImportError:
    print("ERROR: ZED SDK not found. Please install the ZED SDK and pyzed Python package.")
    print("Download from: https://www.stereolabs.com/developers/release/")
    sys.exit(1)

from dimos.hardware.zed_camera import ZEDCamera

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class ZEDLiveVisualizer:
    """Live matplotlib visualization for ZED camera data."""

    def __init__(self, camera, max_depth=10.0):
        self.camera = camera
        self.max_depth = max_depth

        # Set up matplotlib
        plt.ion()  # Turn on interactive mode
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))

        # Initialize plots
        self.ax1.set_title("RGB Camera Feed")
        self.ax1.axis("off")
        self.ax2.set_title("Neural Depth Map")
        self.ax2.axis("off")

        # Initialize image plots
        self.rgb_plot = None
        self.depth_plot = None

        # Stats text
        self.stats_text = self.fig.suptitle("ZED Camera - Neural Depth Demo", fontsize=14)

        plt.tight_layout()

    def normalize_depth(self, depth_map):
        """Normalize depth map for visualization."""
        # Handle invalid values (optimized)
        valid_mask = (depth_map > 0) & np.isfinite(depth_map)

        if not np.any(valid_mask):
            return np.zeros_like(depth_map, dtype=np.float32)

        # Fast normalization without modifying original data
        depth_norm = np.zeros_like(depth_map, dtype=np.float32)
        depth_clipped = np.clip(depth_map[valid_mask], 0, self.max_depth)
        depth_norm[valid_mask] = depth_clipped / self.max_depth

        return depth_norm

    def update_display(self):
        """Update the live display with new frames."""
        # Capture frame
        left_img, right_img, depth_map = self.camera.capture_frame()

        if left_img is None or depth_map is None:
            return False, None

        # Convert BGR to RGB for matplotlib (faster slicing)
        rgb_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2RGB)

        # Normalize depth
        depth_norm = self.normalize_depth(depth_map)

        # Update RGB image
        if self.rgb_plot is None:
            self.rgb_plot = self.ax1.imshow(rgb_img)
        else:
            self.rgb_plot.set_array(rgb_img)

        # Update depth image with colormap
        if self.depth_plot is None:
            self.depth_plot = self.ax2.imshow(depth_norm, cmap="jet", vmin=0, vmax=1)
            # Add colorbar
            cbar = plt.colorbar(self.depth_plot, ax=self.ax2, fraction=0.046, pad=0.04)
            cbar.set_label(f"Depth (0-{self.max_depth}m)")
        else:
            self.depth_plot.set_array(depth_norm)

        return True, depth_map

    def update_stats(self, depth_map, frame_count):
        """Update stats (call less frequently for performance)."""
        valid_depth = depth_map[depth_map > 0]
        if len(valid_depth) > 0:
            stats = (
                f"Frame {frame_count} | Valid pixels: {len(valid_depth):,} | "
                f"Min: {valid_depth.min():.2f}m | "
                f"Max: {valid_depth.max():.2f}m | "
                f"Mean: {valid_depth.mean():.2f}m"
            )
            self.stats_text.set_text(f"ZED Camera - Neural Depth Demo | {stats}")

    def refresh_display(self):
        """Efficiently refresh the display."""
        # Only redraw what's needed
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    parser = argparse.ArgumentParser(description="ZED Camera Neural Depth Demo - Live Matplotlib")
    parser.add_argument("--camera-id", type=int, default=0, help="ZED camera ID (default: 0)")
    parser.add_argument(
        "--resolution",
        type=str,
        default="HD720",
        choices=["HD2K", "HD1080", "HD720", "VGA"],
        help="Camera resolution (default: HD720)",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=10.0,
        help="Maximum depth for visualization in meters (default: 10.0)",
    )
    parser.add_argument(
        "--fps-limit", type=float, default=30.0, help="Display FPS limit (default: 30.0)"
    )
    parser.add_argument(
        "--camera-fps", type=int, default=30, help="Camera capture FPS (default: 30)"
    )
    parser.add_argument(
        "--depth-mode",
        type=str,
        default="NEURAL",
        choices=["NEURAL", "NEURAL_PLUS"],
        help="Depth mode (NEURAL=faster, NEURAL_PLUS=more accurate)",
    )

    args = parser.parse_args()

    # Map resolution string to ZED enum
    resolution_map = {
        "HD2K": sl.RESOLUTION.HD2K,
        "HD1080": sl.RESOLUTION.HD1080,
        "HD720": sl.RESOLUTION.HD720,
        "VGA": sl.RESOLUTION.VGA,
    }

    depth_mode_map = {"NEURAL": sl.DEPTH_MODE.NEURAL, "NEURAL_PLUS": sl.DEPTH_MODE.NEURAL_PLUS}

    try:
        # Initialize ZED camera with neural depth
        logger.info(
            f"Initializing ZED camera with {args.depth_mode} depth processing at {args.camera_fps} FPS..."
        )
        camera = ZEDCamera(
            camera_id=args.camera_id,
            resolution=resolution_map[args.resolution],
            depth_mode=depth_mode_map[args.depth_mode],
            fps=args.camera_fps,
        )

        # Open camera
        with camera:
            # Get camera information
            info = camera.get_camera_info()
            logger.info(f"Camera Model: {info.get('model', 'Unknown')}")
            logger.info(f"Serial Number: {info.get('serial_number', 'Unknown')}")
            logger.info(f"Firmware: {info.get('firmware', 'Unknown')}")
            logger.info(f"Resolution: {info.get('resolution', {})}")
            logger.info(f"Baseline: {info.get('baseline', 0):.3f}m")

            # Initialize visualizer
            visualizer = ZEDLiveVisualizer(camera, max_depth=args.max_depth)

            logger.info("Starting live visualization... Close the matplotlib window to stop.")
            logger.info("Press Ctrl+C in terminal to force quit.")

            frame_count = 0
            start_time = time.time()

            try:
                while plt.get_fignums():  # Continue while matplotlib window is open
                    loop_start = time.time()

                    # Update display
                    display_start = time.time()
                    success, depth_map = visualizer.update_display()
                    display_time = time.time() - display_start

                    if success:
                        frame_count += 1

                        # Refresh display EVERY frame for true real-time visualization
                        refresh_start = time.time()
                        visualizer.refresh_display()
                        refresh_time = time.time() - refresh_start

                        # Update stats only every 10 frames to maintain performance
                        if frame_count % 10 == 0 and depth_map is not None:
                            visualizer.update_stats(depth_map, frame_count)

                        # Print detailed performance stats every 30 frames
                        if frame_count % 30 == 0:
                            elapsed = time.time() - start_time
                            fps = frame_count / elapsed
                            logger.info(
                                f"Frame {frame_count} | FPS: {fps:.1f} | "
                                f"Display: {display_time * 1000:.1f}ms | "
                                f"Refresh: {refresh_time * 1000:.1f}ms"
                            )

                    # Very minimal frame rate limiting to prevent CPU overload
                    elapsed = time.time() - loop_start
                    min_frame_time = 1.0 / 120.0  # Cap at 120 FPS max for smooth real-time
                    if elapsed < min_frame_time:
                        time.sleep(min_frame_time - elapsed)

            except KeyboardInterrupt:
                logger.info("Stopped by user")

            # Final stats
            total_time = time.time() - start_time
            if total_time > 0:
                avg_fps = frame_count / total_time
                logger.info(
                    f"Final stats: {frame_count} frames in {total_time:.1f}s (avg {avg_fps:.1f} FPS)"
                )

    except Exception as e:
        logger.error(f"Error during execution: {e}")
        raise
    finally:
        plt.close("all")
        logger.info("Demo completed")


if __name__ == "__main__":
    main()
