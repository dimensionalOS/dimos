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

from dimos.core.transport import JpegLcmTransport
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

_with_jpeglcm = unitree_go2.transports(
    {
        ("color_image", Image): JpegLcmTransport("/color_image", Image),
    }
)

__all__ = ["_with_jpeglcm"]
