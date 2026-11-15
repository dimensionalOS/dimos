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

"""M20 navigation driving the simulated walking M20 to a clicked goal."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.deeprobotics.m20.blueprints._nav import m20_nav_stack
from dimos.robot.deeprobotics.m20.model import M20_MODEL
from dimos.robot.deeprobotics.m20.rerun import m20_sim_rerun_config
from dimos.robot.urdf_manager import UrdfManager
from dimos.simulation.m20_sim.world import SIM_M20, SimM20World
from dimos.visualization.vis_module import vis_module

m20_sim_nav = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=m20_sim_rerun_config()),
    SimM20World.blueprint().remappings(
        [
            (SimM20World, "lidar", "slam_body_points"),
            (SimM20World, "odometry", "slam_odom"),
            # the sim's measured joint angles pose the same URDF the real robot shows
            (SimM20World, "joint_state", "coordinator_joint_state"),
        ]
    ),
    UrdfManager.blueprint(model=M20_MODEL),
    m20_nav_stack(SIM_M20),
    # gossip off: with it on, zenoh's pending-connection bug keeps native modules spawned together from linking
).global_config(zenoh_gossip=False, n_workers=9)
