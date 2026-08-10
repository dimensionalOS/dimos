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

"""Constants and pure address/credential helpers shared by the Spot modules.

Intentionally free of any `bosdyn` import so it stays importable — and blueprint
discovery keeps working — on hosts without the SDK.
"""

from __future__ import annotations

from pathlib import Path

# Spot's fixed default addresses: 192.168.80.3 when it hosts its own WiFi AP,
# 10.0.0.3 on the rear Ethernet port. Probed in order when no ip is given.
SPOT_WIFI_AP_IP = "192.168.80.3"
SPOT_ETHERNET_IP = "10.0.0.3"
IP_LABELS = {SPOT_WIFI_AP_IP: "WiFi", SPOT_ETHERNET_IP: "Ethernet"}

# Spot's gRPC API listens on HTTPS/443; a TCP connect confirms real reachability
# (and matches what the SDK does) better than an ICMP ping.
SPOT_API_PORT = 443
REACHABILITY_PROBE_TIMEOUT_S = 2.0

# not configutable, these are limits set by the spot API
MAX_LINEAR_VELOCITY = 1.599
MAX_ANGULAR_VELOCITY = 1.599

# Motor power / posture command timeouts (seconds).
POWER_ON_TIMEOUT_S = 20.0
POWER_OFF_TIMEOUT_S = 20.0
STAND_TIMEOUT_S = 10.0
SIT_TIMEOUT_S = 10.0

SPOT_URDF_PATH = Path(__file__).parent / "spot.urdf"

CAMERA_STREAM_SUFFIXES = ["front_left", "front_right", "left", "right", "back"]

# Spot mounts the two front body cameras rotated ~90° clockwise, so their raw
# fisheye/depth frames arrive sideways. Rotate them back one quarter turn (CW,
# hence -1 for np.rot90) — together with their intrinsics — before publishing so
# each image lines up with its optical frame. Side/back cameras arrive upright.
FRONT_CAMERA_ROTATE_UPRIGHT = -1

# The two front cameras mount mirror-imaged, so frontright's optical frame sits a
# half turn (2 quarter turns) past frontleft's when rolling frames upright in 3D.
FRONT_CAMERA_MIRROR_HALF_TURN = 2

# Spot's right body camera arrives upside down; a half turn (180°) rights it. Its
# intrinsics are unchanged in width/height, only the principal point flips.
RIGHT_CAMERA_ROTATE_UPRIGHT = 2

CAMERA_MAX_HZ = 15.0
