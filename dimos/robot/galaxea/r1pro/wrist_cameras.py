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

"""The R1 Pro's wrist D405s, colour and depth read straight off V4L2.

Both wrists hang off one PCIe USB 3.0 controller (a Renesas uPD720201), and it
cannot carry colour and depth from both cameras at the vendor's 1280x720: all
four streams stall, whether librealsense or raw V4L2 opens them. Measured
limits, both wrists at 30 fps on the stock uvcvideo driver:

    colour only, up to 1280x720                   runs
    colour 640x480 + depth 848x480                runs (the depth default)
    colour 848x480 + depth 640x480                colour stalls
    colour 640x480 + depth 1280x720               depth stalls

So the blueprints read the cameras themselves at sizes that fit: colour alone
through ``V4L2CameraModule``, or colour and depth together through
``V4L2ColorDepthModule``, which a D405 needs to open its two streams in the
right order. The vendor wrist pane (``hdas``, ``start_realsense_camera_r1pro.sh``)
must not be holding the cameras; while it is, these modules log and retry.
"""

from __future__ import annotations

from dimos.hardware.sensors.camera.v4l2_camera import V4L2CameraModule
from dimos.hardware.sensors.camera.v4l2_color_depth import V4L2ColorDepthModule

# Each D405 exposes six video nodes: index0 is depth (Z16), index4 colour
# (YUYV), the others infrared and per-stream metadata. by-path pins the node to
# the USB port (port 1 right, port 2 left) so a re-plug cannot swap the wrists.
_WRIST_V4L2 = (
    "/dev/v4l/by-path/platform-14160000.pcie-pci-0004:01:00.0-usb-0:{port}:1.0-video-index{index}"
)
WRIST_LEFT_COLOR_V4L2 = _WRIST_V4L2.format(port=2, index=4)
WRIST_RIGHT_COLOR_V4L2 = _WRIST_V4L2.format(port=1, index=4)
WRIST_LEFT_DEPTH_V4L2 = _WRIST_V4L2.format(port=2, index=0)
WRIST_RIGHT_DEPTH_V4L2 = _WRIST_V4L2.format(port=1, index=0)


# Distinct classes only because blueprints can't yet run two instances of one
# module (same reason the hosted xArm blueprints declare Front/WristCamera).
class WristLeftCamera(V4L2CameraModule):
    pass


class WristRightCamera(V4L2CameraModule):
    pass


class WristLeftColorDepth(V4L2ColorDepthModule):
    pass


class WristRightColorDepth(V4L2ColorDepthModule):
    pass
