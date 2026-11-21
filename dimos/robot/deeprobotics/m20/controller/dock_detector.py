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

"""The M20 dock detector: the charging dock's two reflector posts in the front Airy's sweeps -> ``dock_pose``.

The vendor's reflective_column_node, in Rust (``rust/src/dock/detector.rs``, tunables in ``rust/m20_controller.toml``
``[dock.detector]``). It runs next to the lidars (NOS) so full sweeps never reach the controller's board; the
controller's docking sequence (``M20Controller.dock()``) reads the ~10 Hz pose it publishes.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.deeprobotics.m20 import constants as c


class M20DockDetectorConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/deeprobotics/m20/controller/rust"
    executable: str = "result/bin/m20_dock_detector_module"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})

    # the sweeps searched (the lidar stream carries both Airys, each in its own frame) and that lidar's mount
    lidar_frame_id: str = c.FRONT_LIDAR_FRAME
    mount_xyz: tuple[float, float, float] = c.FRONT_LIDAR_XYZ_M
    mount_rpy: tuple[float, float, float] = c.FRONT_LIDAR_RPY
    # an m20_controller.toml path for [dock.detector] ("" = the one built in)
    controller_config: str = ""
    # dock_pose's frame
    frame_id: str = "base_link"

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # JSON arrays, as the rust [f64; 3] reads them
        for key in ("mount_xyz", "mount_rpy"):
            config[key] = list(getattr(self, key))
        return config


class M20DockDetector(NativeModule):
    """The dock's pose in base_link (posts' midpoint, outward axis as yaw) for every front-lidar sweep that sees it."""

    config: M20DockDetectorConfig

    lidar: In[PointCloud2]
    dock_pose: Out[PoseStamped]


if TYPE_CHECKING:
    M20DockDetector()
