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

"""The Go2 motion host as `dimos bake --deployment` embeds it.

Every module block is its python config class at defaults plus the few values
this deployment tunes; the rust side has no defaults of its own, so this is the
only place they come from.
"""

from __future__ import annotations

from dataclasses import replace

from dimos.cli.bake.deployment import Deployment
from dimos.navigation.motion.adapter.follower_native import TrajectoryFollowerNativeConfig
from dimos.navigation.motion.adapter.planner_native import MotionPlannerNativeConfig
from dimos.navigation.motion.embodiment.go2 import GO2
from dimos.protocol.service.zenohservice import ZenohConfig
from dimos.robot.unitree.go2.tf.go2_tf import Go2TfConfig
from dimos.robot.unitree.go2.zenoh.blueprints import MOTION_BODY_DILATE_M, MOTION_MID360_MOUNT
from dimos.robot.unitree.go2.zenoh.zenohconnection import GO2ZenohConfig

# The deployment's ceiling over GO2's measured cruise; dial here, not in the law.
MAX_SPEED = 0.7
BODY = replace(GO2, max_speed=MAX_SPEED)

# The mount the baked go2_tf publishes. Resolved through GO2ZenohConfig from the
# same constant the laptop half declares, because the two publish the SAME tf
# edges: disagree and base_link jumps between two mounts at their combined
# publish rate. Stated here rather than left to Go2TfConfig's default, so the
# deployment says what the robot runs (test_motion_host.py asserts the pair).
MID360_MOUNT_RPY_DEG = [*GO2ZenohConfig(mid360_mount=MOTION_MID360_MOUNT).mid360_mount]

# go2web is the zenoh ROUTER on 7447; the host is its CLIENT over loopback and
# listens on nothing. Without this block the host opens zenoh's defaults -- a
# peer with multicast scouting -- which a router does not forward to.
SESSION = ZenohConfig(
    mode="client",
    connect=["tcp/127.0.0.1:7447"],
    listen=[],
    multicast=False,
    scouting_interface="lo",
    gossip=True,
    connect_timeout=3.0,
)

GO2_MOTION_HOST = Deployment(
    modules=("motion_planner", "trajectory_follower", "cmd_vel_mux", "go2_tf"),
    configs={
        "motion_planner": MotionPlannerNativeConfig(
            embodiment=BODY, body_dilate_m=MOTION_BODY_DILATE_M
        ).to_config_dict(),
        "trajectory_follower": TrajectoryFollowerNativeConfig(embodiment=BODY).to_config_dict(),
        "go2_tf": Go2TfConfig(mid360_mount_rpy_deg=MID360_MOUNT_RPY_DEG).to_config_dict(),
    },
    session=SESSION.to_wire(),
)
