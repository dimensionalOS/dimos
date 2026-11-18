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

"""Record dual OpenYAM demonstrations: real leaders and Joy-Cons drive the MuJoCo rig.

The collection stack of ``learning_collect_joycon_leader_dual_openyam`` with the
robot connection and the RealSense cameras replaced by the simulated rig, whose
cameras feed the recorder under the names the real cameras use.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.dual_openyam.blueprints.leader_collection import (
    DUAL_OPENYAM_CAMERAS,
    collection_recorder_parts,
)
from dimos.robot.manipulators.dual_openyam.blueprints.leader_teleop import (
    dual_openyam_joycon_leader_teleop,
)
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import (
    _ROBOT_CONNECTIONS,
    dual_openyam_sim_connection,
)
from dimos.robot.manipulators.dual_openyam.sim_connection import DualOpenYamSimConnection

# The sim's overhead color stream is named as the real overhead camera's; its tf
# stays unremapped for the viewer, and its overhead point cloud is published for
# it too (the recorder records only the streams of its profile).
learning_collect_joycon_leader_dual_openyam_sim = autoconnect(
    *collection_recorder_parts(DUAL_OPENYAM_CAMERAS),
    dual_openyam_joycon_leader_teleop.disabled_modules(*_ROBOT_CONNECTIONS),
    dual_openyam_sim_connection(enable_pointcloud=True).remappings(
        [(DualOpenYamSimConnection, "color_image", "overhead_color_image")]
    ),
)
