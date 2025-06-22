#!/usr/bin/env python3


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

from dimos.multiprocess.actors.meta import In, Out, module, rpc
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.path import Path
from dimos.types.vector import Vector


@module
class Navigation:
    target_path: Out[Path]

    @rpc
    def navigate_to(self, target: Vector) -> bool: ...

    def __init__(
        self,
        target_position: In[Vector],
        map_stream: In[Map],
        odometry: In[Odometry],
    ):
        self.target_position = target_position
        self.map_stream = map_stream
        self.odometry = odometry


def test_introspect():
    """Test introspection of the Navigation module."""
    assert hasattr(Navigation, "inputs")
    assert hasattr(Navigation, "outputs")
    assert hasattr(Navigation, "rpcs")

    print("\n\n\n", Navigation.io(), "\n\n")
