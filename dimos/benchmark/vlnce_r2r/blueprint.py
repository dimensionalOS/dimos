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

"""Case-bound public DimOS stack for one VLN-CE R2R episode."""

from pathlib import Path
import tempfile

from dimos_lcm.std_msgs import Bool

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.memory2.module import OnExisting, Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.visualization.vis_module import vis_module

from .connection import VlnceConnection

DEFAULT_RECORDING_PATH = str(Path(tempfile.gettempdir()) / "dimos-vlnce-recording.db")


class VlnceObservationRecorderConfig(RecorderConfig):
    db_path: str | Path = DEFAULT_RECORDING_PATH


class VlnceObservationRecorder(Recorder):
    """Record only observations exposed through the public benchmark boundary."""

    color_image: In[Image]
    depth_image: In[Image]
    odom: In[PoseStamped]
    depth_pointcloud: In[PointCloud2]
    global_costmap: In[OccupancyGrid]
    goal_reached: In[Bool]
    config: VlnceObservationRecorderConfig
    _last_odom_pose: Pose | None = None

    @pose_setter_for("odom")
    async def _odom_pose(self, message: PoseStamped) -> Pose:
        self._last_odom_pose = message
        return message

    @pose_setter_for(
        "color_image",
        "depth_image",
        "depth_pointcloud",
        "global_costmap",
        "goal_reached",
    )
    async def _robot_pose(self, _message: object) -> Pose | None:
        return self._last_odom_pose


def vlnce_r2r_eval_blueprint(
    *,
    socket_path: str | Path,
    attempt_id: str,
    case_id: str,
    episode_id: str,
    protocol_revision: str = "vlnce-public.v1",
    recording_path: str | Path = DEFAULT_RECORDING_PATH,
) -> Blueprint:
    """Compose a public-only navigation stack bound to one benchmark attempt."""

    memory_root = Path(recording_path).parent / "spatial-memory"

    return (
        autoconnect(
            vis_module(viewer_backend=global_config.viewer),
            VlnceConnection.blueprint(
                socket_path=str(socket_path),
                attempt_id=attempt_id,
                case_id=case_id,
                episode_id=episode_id,
                protocol_revision=protocol_revision,
            ),
            # Habitat's navmesh is already eroded by the simulated agent's
            # radius. Inflating its traversable-center projection again closes
            # valid doors and can isolate the episode's starting position.
            ReplanningAStarPlanner.blueprint(
                robot_width=0.0,
                robot_rotation_diameter=0.0,
            ),
            SpatialMemory.blueprint(
                db_path=str(memory_root / "chromadb"),
                visual_memory_path=str(memory_root / "visual-memory.pkl"),
                output_dir=str(memory_root),
                new_memory=True,
            ),
            NavigationSkillContainer.blueprint(),
            VlnceObservationRecorder.blueprint(
                db_path=recording_path,
                on_existing=OnExisting.OVERWRITE,
                stream_codecs={"depth_image": "pickle"},
            ),
        )
        .remappings(
            [
                (ReplanningAStarPlanner, "nav_cmd_vel", "cmd_vel"),
                (ReplanningAStarPlanner, "odometry", "unused_benchmark_odometry"),
                (VlnceObservationRecorder, "depth_pointcloud", "pointcloud"),
            ]
        )
        .global_config(
            configure_system=False,
            n_workers=7,
            robot_model="vlnce_habitat_cylinder",
            transport="zenoh",
        )
    )
