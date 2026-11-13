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

"""The R1 Pro's two wrist cameras (RealSense D405), colour read straight off V4L2.

Both wrists share one USB 3.0 controller (a Renesas uPD720201 PCIe card). It
cannot carry colour and depth from both cameras at 1280x720, the size Galaxea's
RealSense driver opens, so that driver stalls every stream and
``vendor_setup.sh`` does not start it. Colour alone fits on both wrists up to
1280x720 at 30 fps.

Galaxea's wrist driver (``start_realsense_camera_r1pro.sh`` in the ``hdas``
tmux session) must not be holding the cameras; while it is, these modules log
and retry.
"""

from __future__ import annotations

from dimos.hardware.sensors.camera.v4l2.module import V4L2Camera, V4L2CameraConfig

# A D405 has six video nodes; video-index4 is colour (YUYV). by-path pins each
# wrist to its USB port (port 2 left, port 1 right), so a re-plug cannot swap them.
_WRIST_V4L2 = (
    "/dev/v4l/by-path/platform-14160000.pcie-pci-0004:01:00.0-usb-0:{port}:1.0-video-index4"
)
WRIST_LEFT_V4L2 = _WRIST_V4L2.format(port=2)
WRIST_RIGHT_V4L2 = _WRIST_V4L2.format(port=1)


class WristLeftCameraConfig(V4L2CameraConfig):
    device: str = WRIST_LEFT_V4L2
    frame_id: str = "left_d405_link"
    # The Jetson encoder path hands USB frames buffers whose rows may be padded wider than the
    # 848 px frame, which shears the image without an error; libjpeg-turbo on the CPU is exact.
    hardware: bool = False


class WristRightCameraConfig(WristLeftCameraConfig):
    device: str = WRIST_RIGHT_V4L2
    frame_id: str = "right_d405_link"


# Distinct classes only because blueprints can't yet run two instances of one module.
class WristLeftCamera(V4L2Camera):
    config: WristLeftCameraConfig


class WristRightCamera(V4L2Camera):
    config: WristRightCameraConfig
