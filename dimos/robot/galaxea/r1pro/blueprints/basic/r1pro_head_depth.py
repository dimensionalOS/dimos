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

"""The R1 Pro with head depth, placed by Point-LIO: ``dimos run r1pro-head-depth``."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_visualization,
)
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_pointlio import r1pro_lidar_odometry
from dimos.robot.galaxea.r1pro.head_depth import r1pro_head_depth as head_depth
from dimos.robot.galaxea.r1pro.vendor_stack import R1ProVendorStack

r1pro_head_depth = autoconnect(
    R1ProVendorStack.blueprint(stop_vendor_lidar=True),
    r1pro_visualization(),
    # Nothing here reads the wrists, and each costs a JPEG decode per frame.
    r1pro_control(publish_odom=False, enable_wrist_color=False),
    r1pro_lidar_odometry(),
    head_depth(),
).global_config(n_workers=5)
