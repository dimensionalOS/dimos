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
from dimos.hardware.sensors.lidar.pointlio.module import PointLio, PointLioRust
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.visualization.vis_module import vis_module

voxel_size = 0.05


mid360_pointlio = autoconnect(
    PointLio.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="mid360_pointlio")

mid360_pointlio_voxels = autoconnect(
    PointLio.blueprint(),
    VoxelGridMapper.blueprint(voxel_size=voxel_size, carve_columns=False),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=3, robot_model="mid360_pointlio_voxels")


def _mid360_for_pointlio(**kwargs: Any) -> Blueprint:
    """Rust driver wired into PointLioRust: raw cloud renamed, stamped in the LIO's sensor frame."""
    return Mid360.blueprint(frame_id="mid360_link", **kwargs).remappings(
        [(Mid360, "lidar", "lidar_raw")]
    )


pointlio_rust = autoconnect(
    _mid360_for_pointlio(),
    PointLioRust.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="mid360_pointlio_rust")

# Replays the capture named by DIMOS_MID360_PCAP (required) at capture speed.
pointlio_rust_replay = autoconnect(
    _mid360_for_pointlio(pcap=os.environ.get("DIMOS_MID360_PCAP", "")),
    PointLioRust.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="mid360_pointlio_rust_replay")
