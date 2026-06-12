# Copyright 2026 Dimensional Inc.
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

"""go2/Telemetry — app-level JSON status packet (topic ``telemetry``)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from dimos.robot.unitree.go2.dds.msgs.base import PrettyMsg


@dataclass(repr=False)
class Telemetry(PrettyMsg):
    type: str
    battery: float  # fraction 0..1
    body_h: float
    current_a: float
    imu_hz: float
    lidar: bool
    lidar_hz: float
    lowstate_hz: float
    mode: int
    obstacle: bool
    odom_hz: float
    points_per_s: float
    rage: bool
    recording: dict[str, Any]  # {active, bytes, duration_s, file}
    rss_mb: float
    sportmode_hz: float
    vel: list[float]  # [vx, vy]
    yaw: float
