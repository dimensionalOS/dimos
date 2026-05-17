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

"""Spec protocols for nav-stack producer/consumer pairs."""

from typing import TYPE_CHECKING, Any, Protocol

from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint


class LoopClosure(Protocol):
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    # Per-keyframe SE(3) correction applied by the optimizer when a loop
    # closure fires. Encoded as a NavPath where position = translation delta
    # and orientation = rotation delta quaternion. The Nth pose corresponds
    # to the Nth keyframe — same length and same node-index order as the
    # nodes in `pose_graph` published in the same cycle.
    loop_correction_delta: Out[NavPath]
    pose_graph: Out[Graph3D]

    @classmethod
    def blueprint(cls, **kwargs: Any) -> "Blueprint": ...
