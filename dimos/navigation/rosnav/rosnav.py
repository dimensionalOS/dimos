# Copyright 2025 Dimensional Inc.
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

from dimos.core import In, Module, Out
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs import Bool


class ROSNav(Module):
    goal_req: In[PoseStamped] = None  # type: ignore
    goal_active: Out[PoseStamped] = None  # type: ignore
    path_active: Out[Path] = None  # type: ignore
    cancel_goal: In[Bool] = None  # type: ignore
    cmd_vel: Out[Twist] = None  # type: ignore

    # PointcloudPerception attributes
    pointcloud: Out[PointCloud2] = None  # type: ignore

    # Global3DMapSpec attributes
    global_pointcloud: Out[PointCloud2] = None  # type: ignore

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def navigate_to(self, target: PoseStamped) -> None:
        # TODO: Implement navigation logic
        pass

    def stop_navigation(self) -> None:
        # TODO: Implement stop logic
        pass
