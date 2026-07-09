#!/usr/bin/env python3

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

from dimos.core.transport import CompressedCodec, LCMTransport
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

# go2 basic with jpeg on the wire instead of 2.76 MB raw frames:
# one encode at publish, decode per subscriber, ~20x less bandwidth.
unitree_go2_compressed_image = unitree_go2_basic.transports(
    {
        ("color_image", Image): CompressedCodec(
            LCMTransport("/color_image", CompressedImage), quality=80
        ),
    }
)
