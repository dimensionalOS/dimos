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

import sys
import time
import threading
from reactivex import operators as ops

import tests.test_header

from pyzed import sl
from dimos.stream.stereo_camera_streams.zed import ZEDCameraStream
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.utils.logging_config import logger
from dimos.manipulation.manip_aio_pipeline import ManipulationPipeline


def main():
    """Test point cloud filtering using the concurrent stream-based ManipulationPipeline."""
    print("Testing point cloud filtering with ManipulationPipeline...")

    # Configuration
    min_confidence = 0.6
    web_port = 5555

    try:
        # Initialize ZED camera stream
        zed_stream = ZEDCameraStream(resolution=sl.RESOLUTION.HD1080, fps=10)

        # Get camera intrinsics
        camera_intrinsics_dict = zed_stream.get_camera_info()
        camera_intrinsics = [
            camera_intrinsics_dict["fx"],
            camera_intrinsics_dict["fy"],
            camera_intrinsics_dict["cx"],
            camera_intrinsics_dict["cy"],
        ]

        # Create the concurrent manipulation pipeline
        pipeline = ManipulationPipeline(
            camera_intrinsics=camera_intrinsics,
            min_confidence=min_confidence,
            max_objects=10,
        )

        # Create ZED stream
        zed_frame_stream = zed_stream.create_stream().pipe(ops.share())

        # Create concurrent processing streams
        streams = pipeline.create_streams(zed_frame_stream)
        detection_viz_stream = streams["detection_viz"]
        pointcloud_viz_stream = streams["pointcloud_viz"]

    except ImportError:
        print("Error: ZED SDK not installed. Please install pyzed package.")
        sys.exit(1)
    except RuntimeError as e:
        print(f"Error: Failed to open ZED camera: {e}")
        sys.exit(1)

    try:
        # Set up web interface with concurrent visualization streams
        print("Initializing web interface...")
        web_interface = RobotWebInterface(
            port=web_port,
            object_detection=detection_viz_stream,
            pointcloud_stream=pointcloud_viz_stream,
        )

        print(f"\nPoint Cloud Filtering Test Running:")
        print(f"Web Interface: http://localhost:{web_port}")
        print(f"Object Detection View: RGB with bounding boxes")
        print(f"Point Cloud View: Depth with colored point clouds and 3D bounding boxes")
        print(f"Confidence threshold: {min_confidence}")
        print("\nPress Ctrl+C to stop the test\n")

        # Start web server (blocking call)
        web_interface.run()

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        print("Cleaning up resources...")
        if "zed_stream" in locals():
            zed_stream.cleanup()
        if "pipeline" in locals():
            pipeline.cleanup()
        print("Test completed")


if __name__ == "__main__":
    main()
