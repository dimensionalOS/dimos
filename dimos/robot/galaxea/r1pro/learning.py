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

"""Initial R1Pro ACT deployment contract: 18 upper-body joints, one RGB view.

The base and vendor zero-travel grippers stay fixed. This profile is deliberately
separate from any future mobile grasping profile and from OpenYAM checkpoints.
"""

from dimos.imitation.dataprep.core import QualityConfig, SyncConfig
from dimos.imitation.profile import (
    ImageSource,
    JointPositionAction,
    JointPositionSource,
    PolicyIOProfile,
)
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name

R1PRO_SIM_ACT_JOINTS = tuple(coordinator_name(joint) for joint in UPPER_BODY_JOINTS)
R1PRO_SIM_ACT_IO = PolicyIOProfile(
    name="r1pro-sim-upper-body-v1",
    robot_type="r1pro_sim_upper_body",
    observations={
        "observation.images.overview": ImageSource(stream="color_image", shape=(240, 320, 3)),
        "observation.state": JointPositionSource(
            stream="coordinator_joint_state",
            joints=R1PRO_SIM_ACT_JOINTS,
        ),
    },
    action=JointPositionAction(
        key="action",
        demonstration=JointPositionSource(
            stream="applied_joint_position_command",
            joints=R1PRO_SIM_ACT_JOINTS,
        ),
    ),
    sync=SyncConfig(anchor="observation.images.overview", rate_hz=15.0, tolerance_ms=20.0),
    quality=QualityConfig(
        mode="fill",
        max_filled_frame_ratio=0.03,
        min_source_rate_ratio=0.95,
        max_camera_gap_ms=100.0,
        max_alignment_error_ms=20.0,
    ),
)
