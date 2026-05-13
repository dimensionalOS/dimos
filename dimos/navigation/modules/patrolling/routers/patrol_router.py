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

from typing import Protocol

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


class PatrolRouter(Protocol):
    def __init__(self, clearance_radius_m: float) -> None: ...
    def handle_occupancy_grid(self, msg: OccupancyGrid) -> None: ...
    def handle_odom(self, msg: PoseStamped) -> None: ...
    def next_goal(self) -> PoseStamped | None: ...
    def get_saturation(self) -> float: ...
    def reset(self) -> None: ...
