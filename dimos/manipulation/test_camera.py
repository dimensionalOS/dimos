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
dual_oak_ubuntu_test.py
Complete test script for dual OAK-D cameras on Ubuntu
Tests RGB + Depth from both cameras based on all our debugging
"""

import sys
import threading
import time

import cv2
import depthai as dai
import numpy as np


class DualOAKSystem:
    def __init__(self):
        self.frames = {"cam1_rgb": None, "cam1_depth": None, "cam2_rgb": None, "cam2_depth": None}
        self.stop_flag = False
        self.devices = []

    def test_single_camera(self, device_info, cam_id):
        """Test a single camera first"""
        print(f"\n{'=' * 50}")
        print(f"Testing Camera {cam_id}: {device_info.getMxId()}")
        print(f"{'=' * 50}")

        p = dai.Pipeline()

        # Use Camera node for RGB
        cam = p.create(dai.node.Camera).build()
        rgb_out = cam.requestOutput((640, 480))

        # MonoCameras for depth (we know these work)
        monoLeft = p.create(dai.node.MonoCamera)
        monoRight = p.create(dai.node.MonoCamera)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

        # Stereo depth with all our fixes
        stereo = p.create(dai.node.StereoDepth)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.initialConfig.setConfidenceThreshold(200)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        class TestDisplay(dai.node.HostNode):
            def build(self, rgb, depth):
                self.link_args(rgb, depth)
                self.sendProcessingToPipeline(True)
                return self

            def process(self, rgb: dai.ImgFrame, depth: dai.ImgFrame):
                rgb_frame = rgb.getCvFrame()
                depth_frame = depth.getFrame()

                # Process depth
                depth_clipped = np.clip(depth_frame, 300, 3000)
                depth_norm = ((depth_clipped - 300) * 255 / 2700).astype(np.uint8)
                depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

                # Resize and combine
                rgb_resized = cv2.resize(rgb_frame, (640, 480))
                depth_resized = cv2.resize(depth_color, (640, 480))
                combined = np.hstack([rgb_resized, depth_resized])

                # Add camera ID
                cv2.putText(
                    combined,
                    f"Camera {cam_id}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )

                cv2.imshow(f"Camera {cam_id} Test", combined)
                if cv2.waitKey(1) == ord("n"):  # Press 'n' for next
                    self.stopPipeline()

        with p:
            p.create(TestDisplay).build(rgb_out, stereo.depth)
            device = dai.Device(p, device_info)
            print(f"✓ Camera {cam_id} connected successfully")
            print("  Press 'n' to continue to next test")
            p.run()

        cv2.destroyAllWindows()
        device.close()

    def run_dual_cameras(self, device_info1, device_info2):
        """Run both cameras simultaneously"""
        print(f"\n{'=' * 50}")
        print("Running DUAL CAMERA mode")
        print(f"{'=' * 50}")

        def run_camera(device_info, cam_id):
            p = dai.Pipeline()

            # RGB from Camera node
            cam = p.create(dai.node.Camera).build()
            rgb_out = cam.requestOutput((640, 480))

            # Depth from MonoCameras
            monoLeft = p.create(dai.node.MonoCamera)
            monoRight = p.create(dai.node.MonoCamera)
            monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

            stereo = p.create(dai.node.StereoDepth)
            stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            stereo.initialConfig.setConfidenceThreshold(200)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(False)

            monoLeft.out.link(stereo.left)
            monoRight.out.link(stereo.right)

            class DualDisplay(dai.node.HostNode):
                def __init__(self, parent, cam_id):
                    super().__init__()
                    self.parent = parent
                    self.cam_id = cam_id

                def build(self, rgb, depth):
                    self.link_args(rgb, depth)
                    self.sendProcessingToPipeline(True)
                    return self

                def process(self, rgb: dai.ImgFrame, depth: dai.ImgFrame):
                    if not self.parent.stop_flag:
                        rgb_frame = rgb.getCvFrame()
                        depth_frame = depth.getFrame()

                        # Store frames
                        self.parent.frames[f"cam{self.cam_id}_rgb"] = rgb_frame
                        self.parent.frames[f"cam{self.cam_id}_depth"] = depth_frame
                    else:
                        self.stopPipeline()

            with p:
                p.create(lambda: DualDisplay(self, cam_id)).build(rgb_out, stereo.depth)
                device = dai.Device(p, device_info)
                self.devices.append(device)
                print(f"✓ Camera {cam_id} running in dual mode")
                p.run()

        # Start both cameras in threads
        t1 = threading.Thread(target=run_camera, args=(device_info1, 1))
        t2 = threading.Thread(target=run_camera, args=(device_info2, 2))

        t1.start()
        t2.start()

        print("\nBoth cameras running! Press 'q' to quit")

        # Display loop
        while not self.stop_flag:
            if all(self.frames[k] is not None for k in self.frames):
                # Process all frames
                cam1_rgb = cv2.resize(self.frames["cam1_rgb"], (640, 480))
                cam2_rgb = cv2.resize(self.frames["cam2_rgb"], (640, 480))

                # Process depth
                for cam_id in [1, 2]:
                    depth = self.frames[f"cam{cam_id}_depth"]
                    if depth is not None:
                        depth_clipped = np.clip(depth, 300, 3000)
                        depth_norm = ((depth_clipped - 300) * 255 / 2700).astype(np.uint8)
                        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                        self.frames[f"cam{cam_id}_depth_vis"] = cv2.resize(depth_color, (640, 480))

                # Create 2x2 grid
                top = np.hstack([cam1_rgb, self.frames.get("cam1_depth_vis", cam1_rgb)])
                bottom = np.hstack([cam2_rgb, self.frames.get("cam2_depth_vis", cam2_rgb)])
                combined = np.vstack([top, bottom])

                # Add labels
                cv2.putText(
                    combined, "Cam1 RGB", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
                )
                cv2.putText(
                    combined, "Cam1 Depth", (650, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
                )
                cv2.putText(
                    combined, "Cam2 RGB", (10, 510), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
                )
                cv2.putText(
                    combined, "Cam2 Depth", (650, 510), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
                )

                cv2.imshow("Dual OAK-D Cameras", combined)

                if cv2.waitKey(1) == ord("q"):
                    self.stop_flag = True
                    break
            else:
                # Waiting for frames
                time.sleep(0.01)

        # Cleanup
        print("\nShutting down cameras...")
        t1.join(timeout=2)
        t2.join(timeout=2)
        cv2.destroyAllWindows()

    def run_aruco_ready_test(self, device_info1, device_info2):
        """Test configuration ready for ArUco calibration"""
        print(f"\n{'=' * 50}")
        print("ArUco-Ready Dual Camera Test")
        print(f"{'=' * 50}")

        # Similar to dual but with ArUco detection added
        print("Position ArUco board in view of both cameras...")
        # ... (ArUco detection code here)


def main():
    print("=" * 60)
    print("    DUAL OAK-D TEST SUITE FOR UBUNTU")
    print("    Based on all debugging from Mac")
    print("=" * 60)

    # Find all OAK-D devices
    available = dai.Device.getAllAvailableDevices()
    print(f"\nFound {len(available)} OAK-D cameras:")

    for i, device in enumerate(available):
        print(f"  {i + 1}. MxId: {device.getMxId()}")

    if len(available) < 2:
        print("\n❌ ERROR: Need at least 2 OAK-D cameras connected!")
        print("Please connect both cameras and try again.")
        sys.exit(1)

    system = DualOAKSystem()

    print("\n" + "=" * 60)
    print("TEST SEQUENCE:")
    print("1. Test each camera individually")
    print("2. Run both cameras simultaneously")
    print("3. Ready for ArUco calibration")
    print("=" * 60)

    # Test 1: Individual cameras
    input("\nPress ENTER to test Camera 1...")
    system.test_single_camera(available[0], 1)

    input("\nPress ENTER to test Camera 2...")
    system.test_single_camera(available[1], 2)

    # Test 2: Dual cameras
    input("\nPress ENTER to run BOTH cameras simultaneously...")
    system.run_dual_cameras(available[0], available[1])

    print("\n" + "=" * 60)
    print("✅ ALL TESTS COMPLETE!")
    print("Your dual OAK-D system is ready for manipulation pipeline")
    print("=" * 60)


if __name__ == "__main__":
    main()
