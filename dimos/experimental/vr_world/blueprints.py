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

"""Blueprints wiring the live VR World to a single robot.

``autoconnect`` matches the module's ``In`` ports (odom / lidar / color_image)
to the robot's ``Out`` streams by (name, type), and the module's ``Out[Twist]
cmd_vel`` to the robot's ``In[Twist] cmd_vel`` — so the headset both observes
and drives.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.experimental.vr_world.module import VrWorldModule
from dimos.robot.unitree.go2.connection import GO2Connection

# Single Go2 + live VR world, launched directly via `dimos run vr-world-go2`.
# Override module config at launch with the standard flag syntax, e.g.
#   dimos run vr-world-go2 -o vrworldmodule.lidar_world_frame=true
#   dimos run vr-world-go2 -o vrworldmodule.voxel_size=0.05
vr_world_go2 = autoconnect(
    VrWorldModule.blueprint(),
    GO2Connection.blueprint(),
).global_config(n_workers=4, robot_model="unitree_go2")


__all__ = ["vr_world_go2"]
