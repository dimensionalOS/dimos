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

"""Allowlisted agent-visible observation memory for local navigation evaluation."""

from pathlib import Path
import tempfile

from dimos_lcm.std_msgs import Bool

from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

DEFAULT_AGENT_EVAL_RECORDING_PATH = str(
    Path(tempfile.gettempdir()) / "dimos-agent-eval-recording.db"
)


class AgentEvalObservationRecorderConfig(RecorderConfig):
    db_path: str | Path = DEFAULT_AGENT_EVAL_RECORDING_PATH


class AgentEvalObservationRecorder(Recorder):
    """Record only streams available to the evaluated policy."""

    color_image: In[Image]
    odom: In[PoseStamped]
    global_map: In[PointCloud2]
    goal_reached: In[Bool]
    config: AgentEvalObservationRecorderConfig

    _last_odom_pose: Pose | None = None

    @pose_setter_for("odom")
    async def _odom_pose(self, message: PoseStamped) -> Pose:
        self._last_odom_pose = message
        return message

    @pose_setter_for("global_map", "goal_reached")
    async def _robot_pose(self, _message: object) -> Pose | None:
        return self._last_odom_pose
