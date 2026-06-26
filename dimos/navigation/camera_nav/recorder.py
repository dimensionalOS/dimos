# Copyright 2025-2026 Dimensional Inc.
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

"""Records frame_cloud observations with robot body pose for post-traversal PGO correction."""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.stream import In
from dimos.memory2.module import Recorder, pose_setter_for
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

logger = setup_logger()


class CameraNavRecorder(Recorder):
    """Stores per-frame clouds with world←base_link pose for offline PGO correction."""

    frame_cloud: In[PointCloud2]

    @pose_setter_for("frame_cloud")
    def _body_pose(self, cloud: PointCloud2) -> PoseStamped | None:
        tf = self.tf.get(
            self.config.root_frame,
            "base_link",
            time_point=cloud.ts,
            time_tolerance=self.config.tf_tolerance,
        )
        if tf is None:
            logger.debug("CameraNavRecorder: no TF for ts=%.3f, storing without pose", cloud.ts)
            return None
        return tf.to_pose()
