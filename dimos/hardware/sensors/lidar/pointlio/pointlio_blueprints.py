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


import os
from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio, PointLioConfig
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.visualization.vis_module import vis_module

voxel_size = 0.05


def mid360_for_pointlio(**kwargs: Any) -> Blueprint:
    """Mid360 driver wired into PointLio: raw streams renamed, stamped in the LIO's sensor frame.

    The full point format carries the tag byte Point-LIO's Livox noise gate reads.
    """
    kwargs.setdefault("frame_id", PointLioConfig.model_fields["sensor_frame_id"].default)
    kwargs.setdefault("point_format", "full")
    return Mid360.blueprint(**kwargs).remappings(
        [(Mid360, "lidar", "lidar_raw"), (Mid360, "imu", "imu_raw")]
    )


mid360_pointlio = autoconnect(
    mid360_for_pointlio(),
    PointLio.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="mid360_pointlio")

mid360_pointlio_voxels = autoconnect(
    mid360_for_pointlio(),
    PointLio.blueprint(),
    VoxelGridMapper.blueprint(voxel_size=voxel_size, carve_columns=False),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
                "world/lidar_raw": None,
            },
        },
    ),
).global_config(n_workers=4, robot_model="mid360_pointlio_voxels")

# Replays the capture named by DIMOS_MID360_PCAP (required) at capture speed.
mid360_pointlio_replay = autoconnect(
    mid360_for_pointlio(pcap=os.environ.get("DIMOS_MID360_PCAP", "")),
    PointLio.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="mid360_pointlio_replay")
