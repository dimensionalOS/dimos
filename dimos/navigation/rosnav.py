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
from dimos.mapping.spec import Global3DMapSpec
from dimos.msgs.geometry_msgs import Path, PoseStamped, Twist
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.navigation.spec import NavSpec
from dimos.perception.pointcloud.spec import PointcloudPerception


class RosNav(Module, PointcloudPerception, Global3DMapSpec, NavSpec):
    goal_req: In[PoseStamped] = None  # type: ignore
    goal_active: Out[PoseStamped] = None  # type: ignore
    path_active: Out[Path] = None  # type: ignore

    ctrl: Out[Twist] = None  # type: ignore

    pointcloud: Out[PointCloud2] = None  # type: ignore
    global_pointcloud: Out[PointCloud2] = None  # type: ignore
