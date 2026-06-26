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

"""Records ``frame_cloud`` observations with robot body pose for post-traversal PGO correction.

Drop into any camera-nav pipeline to capture data for offline loop closure::

    from dimos.navigation.camera_nav.blueprint import camera_nav_stack
    from dimos.navigation.camera_nav.recorder import CameraNavRecorder
    from dimos.core.coordination.blueprints import autoconnect

    pipeline = autoconnect(
        my_robot_blueprint,
        camera_nav_stack,
        CameraNavRecorder.blueprint(db_path="traversal.db"),
    )

After a traversal run ``correct_map.py`` to produce a drift-corrected map::

    python -m dimos.navigation.camera_nav.correct_map traversal.db --output corrected.pcd
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.stream import In
from dimos.memory2.module import Recorder, pose_setter_for
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

logger = setup_logger()


class CameraNavRecorder(Recorder):
    """Records per-frame clouds with robot body pose attached.

    Each ``frame_cloud`` observation is stored alongside the ``world <- base_link``
    pose from the robot's TF tree.  After a traversal, pass the resulting
    SQLite database to ``correct_map.py`` which runs PGO loop closure and
    re-accumulates the clouds using drift-corrected poses.

    Ports
    -----
    Inputs
        frame_cloud : Per-frame coloured cloud from ``MonocularDepthModule``.
    """

    frame_cloud: In[PointCloud2]

    @pose_setter_for("frame_cloud")
    def _body_pose(self, cloud: PointCloud2) -> PoseStamped | None:
        """Return ``world <- base_link`` pose at the cloud's timestamp.

        Using ``base_link`` (not the cloud's own ``frame_id``) gives PGO the
        robot's odom trajectory, regardless of whether the cloud is already
        in world frame.
        """
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
