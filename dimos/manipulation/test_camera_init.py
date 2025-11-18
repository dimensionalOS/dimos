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

# dual_oak_s2_fixed.py - Correct configuration for OAK-D S2
import contextlib

import cv2
import depthai as dai
import numpy as np

print(f"DepthAI version: {dai.__version__}")
print("OAK-D S2 Configuration (Fixed)\n")


def create_pipeline():
    """Create pipeline for OAK-D S2 using ColorCamera ISP output"""
    pipeline = dai.Pipeline()

    # Use ColorCamera nodes for both cameras
    cam_left = pipeline.create(dai.node.ColorCamera)
    cam_right = pipeline.create(dai.node.ColorCamera)

    cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    # Set resolution
    cam_left.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    cam_right.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

    # Configure left camera for color preview
    cam_left.setPreviewSize(640, 480)
    cam_left.setInterleaved(False)
    cam_left.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Output color preview from left camera
    xout_color = pipeline.create(dai.node.XLinkOut)
    xout_color.setStreamName("color")
    cam_left.preview.link(xout_color.input)

    # Stereo depth using ISP output (grayscale) from ColorCamera nodes
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(False)

    # Link ISP outputs to stereo (ISP outputs grayscale for stereo processing)
    cam_left.isp.link(stereo.left)
    cam_right.isp.link(stereo.right)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    return pipeline


# Main execution
with contextlib.ExitStack() as stack:
    device_infos = dai.Device.getAllAvailableDevices()

    print(f"Found {len(device_infos)} devices:")
    for i, info in enumerate(device_infos):
        print(f"  Device {i}: {info.getMxId()}")

    if len(device_infos) < 2:
        print(f"\nERROR: Need 2 devices, found {len(device_infos)}")
        exit(1)

    print("\nConnecting devices...")
    devices = []
    queues = []

    for i, device_info in enumerate(device_infos[:2]):
        try:
            pipeline = create_pipeline()

            device = stack.enter_context(dai.Device(pipeline, device_info, dai.UsbSpeed.SUPER))
            devices.append(device)

            # Get output queues
            q_color = device.getOutputQueue(name="color", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            queues.append((q_color, q_depth))

            print(f"✓ Camera {i + 1} connected: {device_info.getMxId()}")
        except Exception as e:
            print(f"✗ Camera {i + 1} failed: {e}")
            import traceback

            traceback.print_exc()
            exit(1)

    print("\n✅ Both OAK-D S2 cameras connected! Press 'q' to quit\n")
    print("📷 Color from left camera + Stereo depth from both\n")

    frame_count = 0

    # Main loop
    while True:
        frames_ready = True
        color_frames = []
        depth_frames = []

        # Get frames from both cameras
        for q_color, q_depth in queues:
            color = q_color.tryGet()
            depth = q_depth.tryGet()

            if color and depth:
                color_frames.append(color.getCvFrame())
                depth_frames.append(depth.getFrame())
            else:
                frames_ready = False
                break

        if frames_ready and len(color_frames) == 2:
            # Visualize depth
            depth1_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_frames[0], alpha=0.03), cv2.COLORMAP_JET
            )
            depth2_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_frames[1], alpha=0.03), cv2.COLORMAP_JET
            )

            # Resize for display
            color1_display = cv2.resize(color_frames[0], (640, 480))
            depth1_display = cv2.resize(depth1_vis, (640, 480))

            color2_display = cv2.resize(color_frames[1], (640, 480))
            depth2_display = cv2.resize(depth2_vis, (640, 480))

            # Add labels
            cv2.putText(
                color1_display,
                f"Cam1 Color [{device_infos[0].getMxId()[:8]}]",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                depth1_display,
                "Cam1 Depth",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                color2_display,
                f"Cam2 Color [{device_infos[1].getMxId()[:8]}]",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                depth2_display,
                "Cam2 Depth",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Create grid layout: [Color1 | Depth1]
            #                      [Color2 | Depth2]
            top = np.hstack([color1_display, depth1_display])
            bottom = np.hstack([color2_display, depth2_display])
            combined = np.vstack([top, bottom])

            cv2.imshow("Dual OAK-D S2 Cameras", combined)

            frame_count += 1
            if frame_count % 30 == 0:
                print(f"Frames: {frame_count}")

        key = cv2.waitKey(1)
        if key == ord("q") or key == 27:
            break

cv2.destroyAllWindows()
print(f"\nCameras closed successfully! Total frames: {frame_count}")
