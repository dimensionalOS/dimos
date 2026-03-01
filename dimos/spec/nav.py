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

from typing import Protocol

from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import Path


class Nav(Protocol):
    goal_req: In[PoseStamped]
    goal_active: Out[PoseStamped]
    path_active: Out[Path]
    cmd_vel: Out[Twist]
