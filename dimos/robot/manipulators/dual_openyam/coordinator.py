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

"""The dual OpenYAM coordinator, in a worker process of its own."""

from typing import ClassVar

from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator


class DualOpenYamCoordinator(DamiaoCoordinator):
    """A Damiao coordinator whose 100 Hz tick loop never shares a process.

    Packed into a worker with a camera, the voxel map and a grasp generator, the
    tick loop misses ticks, the joint state gaps past the map's 20 ms state
    tolerance and half the overhead clouds are dropped as "robot state
    unavailable".
    """

    dedicated_worker: ClassVar[bool] = True
