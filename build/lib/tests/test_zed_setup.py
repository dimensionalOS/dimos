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
Simple test script to verify ZED camera setup and basic functionality.
"""

import sys
from pathlib import Path


def test_imports():
    """Test that all required modules can be imported."""
    print("Testing imports...")

    try:
        import numpy as np

        print("✓ NumPy imported successfully")
    except ImportError as e:
        print(f"✗ NumPy import failed: {e}")
        return False

    try:
        import cv2

        print("✓ OpenCV imported successfully")
    except ImportError as e:
        print(f"✗ OpenCV import failed: {e}")
        return False

    try:
        from PIL import Image, ImageDraw, ImageFont

        print("✓ PIL imported successfully")
    except ImportError as e:
        print(f"✗ PIL import failed: {e}")
        return False

    try:
        import pyzed.sl as sl

        print("✓ ZED SDK (pyzed) imported successfully")
        # Note: SDK version method varies between versions
    except ImportError as e:
        print(f"✗ ZED SDK import failed: {e}")
        print("  Please install ZED SDK and pyzed package")
        return False

    try:
        from dimos.hardware.zed_camera import ZEDCamera

        print("✓ ZEDCamera class imported successfully")
    except ImportError as e:
        print(f"✗ ZEDCamera import failed: {e}")
        return False

    try:
        from dimos.perception.zed_visualizer import ZEDVisualizer

        print("✓ ZEDVisualizer class imported successfully")
    except ImportError as e:
        print(f"✗ ZEDVisualizer import failed: {e}")
        return False

    return True


def test_camera_detection():
    """Test if ZED cameras are detected."""
    print("\nTesting camera detection...")

    try:
        import pyzed.sl as sl

        # List available cameras
        cameras = sl.Camera.get_device_list()
        print(f"Found {len(cameras)} ZED camera(s):")

        for i, camera_info in enumerate(cameras):
            print(f"  Camera {i}:")
            print(f"    Model: {camera_info.camera_model}")
            print(f"    Serial: {camera_info.serial_number}")
            print(f"    State: {camera_info.camera_state}")

        return len(cameras) > 0

    except Exception as e:
        print(f"Error detecting cameras: {e}")
        return False


def test_basic_functionality():
    """Test basic ZED camera functionality without actually opening the camera."""
    print("\nTesting basic functionality...")

    try:
        import pyzed.sl as sl
        from dimos.hardware.zed_camera import ZEDCamera
        from dimos.perception.zed_visualizer import ZEDVisualizer

        # Test camera initialization (without opening)
        camera = ZEDCamera(
            camera_id=0,
            resolution=sl.RESOLUTION.HD720,
            depth_mode=sl.DEPTH_MODE.NEURAL,
        )
        print("✓ ZEDCamera instance created successfully")

        # Test visualizer initialization
        visualizer = ZEDVisualizer(max_depth=10.0)
        print("✓ ZEDVisualizer instance created successfully")

        # Test creating a dummy visualization
        dummy_rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        dummy_depth = np.ones((480, 640), dtype=np.float32) * 2.0

        vis = visualizer.create_side_by_side_image(dummy_rgb, dummy_depth)
        print("✓ Dummy visualization created successfully")

        return True

    except Exception as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False


def main():
    """Run all tests."""
    print("ZED Camera Setup Test")
    print("=" * 50)

    # Test imports
    if not test_imports():
        print("\n❌ Import tests failed. Please install missing dependencies.")
        return False

    # Test camera detection
    cameras_found = test_camera_detection()
    if not cameras_found:
        print(
            "\n⚠️  No ZED cameras detected. Please connect a ZED camera to test capture functionality."
        )

    # Test basic functionality
    if not test_basic_functionality():
        print("\n❌ Basic functionality tests failed.")
        return False

    print("\n" + "=" * 50)
    if cameras_found:
        print("✅ All tests passed! You can now run the ZED demo:")
        print("   python examples/zed_neural_depth_demo.py --display-time 10")
    else:
        print("✅ Setup is ready, but no camera detected.")
        print("   Connect a ZED camera and run:")
        print("   python examples/zed_neural_depth_demo.py --display-time 10")

    return True


if __name__ == "__main__":
    # Add the project root to Python path
    sys.path.append(str(Path(__file__).parent))

    # Import numpy after path setup
    import numpy as np

    success = main()
    sys.exit(0 if success else 1)
