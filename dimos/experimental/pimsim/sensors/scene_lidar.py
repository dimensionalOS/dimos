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

"""Native scene lidar backed by a cooked browser collision mesh."""

from __future__ import annotations

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.experimental.pimsim.entity import EntityStateBatch
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class SceneLidarConfig(NativeModuleConfig):
    cwd: str | None = "rust/scene_lidar"
    executable: str = "target/release/scene_lidar"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    scene_metadata_path: str
    collision_path: str | None = None
    scan_model: str = "uniform"
    frame_id: str = "lidar_link"
    publish_sensor_frame: bool = False
    hz: float = 10.0
    point_rate: int = 200_000
    horizontal_samples: int = 720
    vertical_samples: int = 16
    elevation_min_deg: float = -22.5
    elevation_max_deg: float = 22.5
    min_range: float = 0.0
    max_range: float = 10.0
    sensor_x: float = 0.0
    sensor_y: float = 0.0
    sensor_z: float = 1.0
    sensor_roll_deg: float = 0.0
    sensor_pitch_deg: float = 0.0
    sensor_yaw_deg: float = 0.0
    yaw_offset_deg: float = 0.0
    output_voxel_size: float = 0.03
    support_floor: bool = False
    support_floor_z: float = 0.0
    support_floor_size: float = 0.0


class SceneLidarModule(NativeModule):
    """Raycast lidar from the cooked browser collision scene.

    Optionally subscribes to ``entity_states`` for dynamic obstacle
    coverage — see ``dimos.experimental.pimsim`` for the publisher.
    """

    config: SceneLidarConfig

    pose: In[PoseStamped]
    entity_states: In[EntityStateBatch]
    lidar: Out[PointCloud2]


__all__ = ["SceneLidarConfig", "SceneLidarModule"]
