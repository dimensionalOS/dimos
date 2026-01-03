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

"""Demo blueprint for Zenoh image transport with webcam and Canny edge detection.

This demo shows:
1. Webcam capture and publishing via Zenoh transport
2. Subscribing to images via Zenoh
3. Canny edge detection processing
4. Visualization with Rerun

Usage:
    # First, install rerun if not already installed:
    pip install rerun-sdk

    # Run the demo:
    dimos run demo-zenoh-image

    # Or run rerun viewer separately for better control:
    rerun &
    dimos run demo-zenoh-image
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import (
    CongestionControl,
    Reliability,
    ZenohQoS,
    ZenohTransport,
)
from dimos.msgs.sensor_msgs import Image
from dimos.test_zenoh.canny_subscriber import canny_subscriber
from dimos.test_zenoh.webcam_publisher import webcam_publisher

# QoS for high-bandwidth image streaming
image_qos = ZenohQoS(
    reliability=Reliability.BEST_EFFORT,
    congestion_control=CongestionControl.DROP,
)

# Create blueprint with Zenoh transport using native Image encoding
demo_zenoh_image_transport = autoconnect(
    webcam_publisher(),
    canny_subscriber(),
).transports(
    {
        # ZenohTransport uses zenoh_encode/zenoh_decode (10x faster decode)
        ("color_image", Image): ZenohTransport("camera/color", Image, qos=image_qos),
    }
)

__all__ = ["demo_zenoh_image_transport"]
