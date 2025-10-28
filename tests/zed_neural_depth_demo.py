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
ZED Camera Neural Depth Demo - OpenCV Live Visualization with Data Saving

This script demonstrates live visualization of ZED camera RGB and depth data using OpenCV.
Press SPACE to save RGB and depth images to rgbd_data2 folder.
Press ESC or 'q' to quit.
"""

import argparse
from datetime import datetime
import logging
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import open3d as o3d
import yaml

# Add the project root to Python path
sys.path.append(str(Path(__file__).parent.parent))

try:
    import pyzed.sl as sl
except ImportError:
    print("ERROR: ZED SDK not found. Please install the ZED SDK and pyzed Python package.")
    print("Download from: https://www.stereolabs.com/developers/release/")
    sys.exit(1)

from dimos.hardware.zed_camera import ZEDCamera
from dimos.perception.pointcloud.utils import visualize_pcd

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class ZEDLiveVisualizer:
    """Live OpenCV visualization for ZED camera data with saving functionality."""

    def __init__(self, camera, max_depth=10.0, output_dir="assets/rgbd_data2"):
        self.camera = camera
        self.max_depth = max_depth
        self.output_dir = Path(output_dir)
        self.save_counter = 0

        # Store captured pointclouds for later visualization
        self.captured_pointclouds = []

        # Display settings for 480p
        self.display_width = 640
        self.display_height = 480

        # Create output directory structure
        self.setup_output_directory()

        # Get camera info for saving
        self.camera_info = camera.get_camera_info()

        # Save camera info files once
        self.save_camera_info()

        # OpenCV window name (single window)
        self.window_name = "ZED Camera - RGB + Depth"

        # Create window
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

    def setup_output_directory(self):
        """Create the output directory structure."""
        self.output_dir.mkdir(exist_ok=True)
        (self.output_dir / "color").mkdir(exist_ok=True)
        (self.output_dir / "depth").mkdir(exist_ok=True)
        (self.output_dir / "pointclouds").mkdir(exist_ok=True)
        logger.info(f"Created output directory: {self.output_dir}")

    def save_camera_info(self):
        """Save camera info YAML files with ZED camera parameters."""
        # Get current timestamp
        now = datetime.now()
        timestamp_sec = int(now.timestamp())
        timestamp_nanosec = int((now.timestamp() % 1) * 1e9)

        # Get camera resolution
        resolution = self.camera_info.get("resolution", {})
        width = int(resolution.get("width", 1280))
        height = int(resolution.get("height", 720))

        # Extract left camera parameters (for RGB) from already available camera_info
        left_cam = self.camera_info.get("left_cam", {})
        # Convert numpy values to Python floats
        fx = float(left_cam.get("fx", 749.341552734375))
        fy = float(left_cam.get("fy", 748.5587768554688))
        cx = float(left_cam.get("cx", 639.4312744140625))
        cy = float(left_cam.get("cy", 357.2478942871094))

        # Build distortion coefficients from ZED format
        # ZED provides k1, k2, p1, p2, k3 - convert to rational_polynomial format
        k1 = float(left_cam.get("k1", 0.0))
        k2 = float(left_cam.get("k2", 0.0))
        p1 = float(left_cam.get("p1", 0.0))
        p2 = float(left_cam.get("p2", 0.0))
        k3 = float(left_cam.get("k3", 0.0))
        distortion = [k1, k2, p1, p2, k3, 0.0, 0.0, 0.0]

        # Create camera info structure with plain Python types
        camera_info = {
            "D": distortion,
            "K": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "P": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "binning_x": 0,
            "binning_y": 0,
            "distortion_model": "rational_polynomial",
            "header": {
                "frame_id": "camera_color_optical_frame",
                "stamp": {"nanosec": timestamp_nanosec, "sec": timestamp_sec},
            },
            "height": height,
            "roi": {"do_rectify": False, "height": 0, "width": 0, "x_offset": 0, "y_offset": 0},
            "width": width,
        }

        # Save color camera info
        color_info_path = self.output_dir / "color_camera_info.yaml"
        with open(color_info_path, "w") as f:
            yaml.dump(camera_info, f, default_flow_style=False)

        # Save depth camera info (same as color for ZED)
        depth_info_path = self.output_dir / "depth_camera_info.yaml"
        with open(depth_info_path, "w") as f:
            yaml.dump(camera_info, f, default_flow_style=False)

        logger.info(f"Saved camera info files to {self.output_dir}")

    def normalize_depth_for_display(self, depth_map):
        """Normalize depth map for OpenCV visualization."""
        # Handle invalid values
        valid_mask = (depth_map > 0) & np.isfinite(depth_map)

        if not np.any(valid_mask):
            return np.zeros_like(depth_map, dtype=np.uint8)

        # Normalize to 0-255 for display
        depth_norm = np.zeros_like(depth_map, dtype=np.float32)
        depth_clipped = np.clip(depth_map[valid_mask], 0, self.max_depth)
        depth_norm[valid_mask] = depth_clipped / self.max_depth

        # Convert to 8-bit and apply colormap
        depth_8bit = (depth_norm * 255).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

        return depth_colored

    def save_frame(self, rgb_img, depth_map):
        """Save RGB, depth images, and pointcloud with proper naming convention."""
        # Generate filename with 5-digit zero-padding
        filename = f"{self.save_counter:05d}.png"
        pcd_filename = f"{self.save_counter:05d}.ply"

        # Save RGB image
        rgb_path = self.output_dir / "color" / filename
        cv2.imwrite(str(rgb_path), rgb_img)

        # Save depth image (convert to 16-bit for proper depth storage)
        depth_path = self.output_dir / "depth" / filename
        # Convert meters to millimeters and save as 16-bit
        depth_mm = (depth_map * 1000).astype(np.uint16)
        cv2.imwrite(str(depth_path), depth_mm)

        # Capture and save pointcloud
        pcd = self.camera.capture_pointcloud()
        if pcd is not None and len(np.asarray(pcd.points)) > 0:
            pcd_path = self.output_dir / "pointclouds" / pcd_filename
            o3d.io.write_point_cloud(str(pcd_path), pcd)

            # Store pointcloud for later visualization
            self.captured_pointclouds.append(pcd)

            logger.info(
                f"Saved frame {self.save_counter}: {rgb_path}, {depth_path}, and {pcd_path}"
            )
        else:
            logger.warning(f"Failed to capture pointcloud for frame {self.save_counter}")
            logger.info(f"Saved frame {self.save_counter}: {rgb_path} and {depth_path}")

        self.save_counter += 1

    def visualize_captured_pointclouds(self):
        """Visualize all captured pointclouds using Open3D, one by one."""
        if not self.captured_pointclouds:
            logger.info("No pointclouds captured to visualize")
            return

        logger.info(
            f"Visualizing {len(self.captured_pointclouds)} captured pointclouds one by one..."
        )
        logger.info("Close each pointcloud window to proceed to the next one")

        for i, pcd in enumerate(self.captured_pointclouds):
            if len(np.asarray(pcd.points)) > 0:
                logger.info(f"Displaying pointcloud {i + 1}/{len(self.captured_pointclouds)}")
                visualize_pcd(pcd, window_name=f"ZED Pointcloud {i + 1:05d}", point_size=2.0)
            else:
                logger.warning(f"Pointcloud {i + 1} is empty, skipping...")

        logger.info("Finished displaying all pointclouds")

    def update_display(self):
        """Update the live display with new frames."""
        # Capture frame
        left_img, _right_img, depth_map = self.camera.capture_frame()

        if left_img is None or depth_map is None:
            return False, None, None

        # Resize RGB to 480p
        rgb_resized = cv2.resize(left_img, (self.display_width, self.display_height))

        # Create depth visualization
        depth_colored = self.normalize_depth_for_display(depth_map)

        # Resize depth to 480p
        depth_resized = cv2.resize(depth_colored, (self.display_width, self.display_height))

        # Add text overlays
        text_color = (255, 255, 255)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2

        # Add title and instructions to RGB
        cv2.putText(
            rgb_resized, "RGB Camera Feed", (10, 25), font, font_scale, text_color, thickness
        )
        cv2.putText(
            rgb_resized,
            "SPACE: Save | ESC/Q: Quit",
            (10, 50),
            font,
            font_scale - 0.1,
            text_color,
            thickness,
        )

        # Add title and stats to depth
        cv2.putText(
            depth_resized,
            f"Depth Map (0-{self.max_depth}m)",
            (10, 25),
            font,
            font_scale,
            text_color,
            thickness,
        )
        cv2.putText(
            depth_resized,
            f"Saved: {self.save_counter} frames",
            (10, 50),
            font,
            font_scale - 0.1,
            text_color,
            thickness,
        )

        # Stack images horizontally
        combined_display = np.hstack((rgb_resized, depth_resized))

        # Display combined image
        cv2.imshow(self.window_name, combined_display)

        return True, left_img, depth_map

    def handle_key_events(self, rgb_img, depth_map):
        """Handle keyboard input."""
        key = cv2.waitKey(1) & 0xFF

        if key == ord(" "):  # Space key - save frame
            if rgb_img is not None and depth_map is not None:
                self.save_frame(rgb_img, depth_map)
                return "save"
        elif key == 27 or key == ord("q"):  # ESC or 'q' - quit
            return "quit"

        return "continue"

    def cleanup(self):
        """Clean up OpenCV windows."""
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="ZED Camera Neural Depth Demo - OpenCV with Data Saving"
    )
    parser.add_argument("--camera-id", type=int, default=0, help="ZED camera ID (default: 0)")
    parser.add_argument(
        "--resolution",
        type=str,
        default="HD1080",
        choices=["HD2K", "HD1080", "HD720", "VGA"],
        help="Camera resolution (default: HD1080)",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=10.0,
        help="Maximum depth for visualization in meters (default: 10.0)",
    )
    parser.add_argument(
        "--camera-fps", type=int, default=15, help="Camera capture FPS (default: 30)"
    )
    parser.add_argument(
        "--depth-mode",
        type=str,
        default="NEURAL",
        choices=["NEURAL", "NEURAL_PLUS"],
        help="Depth mode (NEURAL=faster, NEURAL_PLUS=more accurate)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="assets/rgbd_data2",
        help="Output directory for saved data (default: rgbd_data2)",
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
            visualizer = ZEDLiveVisualizer(
                camera, max_depth=args.max_depth, output_dir=args.output_dir
            )

            logger.info("Starting live visualization...")
            logger.info("Controls:")
            logger.info("  SPACE - Save current RGB and depth frame")
            logger.info("  ESC/Q - Quit")

            frame_count = 0
            start_time = time.time()

            try:
                while True:
                    loop_start = time.time()

                    # Update display
                    success, rgb_img, depth_map = visualizer.update_display()

                    if success:
                        frame_count += 1

                        # Handle keyboard events
                        action = visualizer.handle_key_events(rgb_img, depth_map)

                        if action == "quit":
                            break
                        elif action == "save":
                            # Frame was saved, no additional action needed
                            pass

                        # Print performance stats every 60 frames
                        if frame_count % 60 == 0:
                            elapsed = time.time() - start_time
                            fps = frame_count / elapsed
                            logger.info(
                                f"Frame {frame_count} | FPS: {fps:.1f} | Saved: {visualizer.save_counter}"
                            )

                    # Small delay to prevent CPU overload
                    elapsed = time.time() - loop_start
                    min_frame_time = 1.0 / 60.0  # Cap at 60 FPS
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
                logger.info(f"Total saved frames: {visualizer.save_counter}")

            # Visualize captured pointclouds
            visualizer.visualize_captured_pointclouds()

    except Exception as e:
        logger.error(f"Error during execution: {e}")
        raise
    finally:
        if "visualizer" in locals():
            visualizer.cleanup()
        logger.info("Demo completed")


if __name__ == "__main__":
    main()
