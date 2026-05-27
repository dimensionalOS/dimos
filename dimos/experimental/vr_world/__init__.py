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

"""VR World — live "Rerun in VR" for a single robot.

Subscribes to a live robot's odom / lidar / camera streams, accumulates a voxel
map on the fly, and renders the robot + growing map in a Quest 3 headset from a
god view. The same headset drives the robot via the left thumbstick (Twist).

See PLAN.md for the full design. This is the live counterpart to
:mod:`dimos.teleop.memory_world` (which replays a recorded SQLite store).
"""

from dimos.experimental.vr_world.module import VrWorldConfig, VrWorldModule

__all__ = ["VrWorldConfig", "VrWorldModule"]
