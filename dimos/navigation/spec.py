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

"""The navigation stack, as ports: a stream in, a stream out, and tf.

A local planner is blind or map-aware; both hand the follower the same ``path``.

A ``path`` with one pose means "hold, no safe route"; with none, "stop".

tf is ``IO`` where the implementers are dimos-module natives (``#[tf]`` both
subscribes and publishes) and their python twins; ``In`` on the global planner,
whose native only reads it.
"""

from typing import Protocol

from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class GlobalPlanner(Protocol):
    tf: In[TFMessage]
    goal: In[PointStamped]

    path: Out[Path]


class BlindLocalPlanner(Protocol):
    """Shapes the global route without looking: no map."""

    planner_path: In[Path]
    tf: In[TFMessage]

    path: Out[Path]


class MapLocalPlanner(Protocol):
    """Routes around what the local map says is there."""

    planner_path: In[Path]
    local_map: In[PointCloud2]
    tf: IO[TFMessage]

    path: Out[Path]


class TrajectoryFollower(Protocol):
    tf: IO[TFMessage]
    path: In[Path]

    nav_cmd_vel: Out[Twist]
