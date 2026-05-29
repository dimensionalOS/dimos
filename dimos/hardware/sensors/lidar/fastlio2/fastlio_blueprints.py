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
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.voxels import VoxelGridMapper
from dimos.visualization.vis_module import vis_module

voxel_size = 0.05


def _replay_pcap_path() -> Path | None:
    value = os.getenv("REPLAY_PCAP")
    return Path(value).expanduser().resolve() if value else None


def _replay_skip_until_ns(pcap: Path | None) -> int | None:
    if pcap is None:
        return None
    sidecar = pcap.with_suffix(pcap.suffix + ".first.ns")
    if not sidecar.exists():
        return None
    return int(sidecar.read_text().strip())


_replay_pcap = _replay_pcap_path()


mid360_fastlio = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="mid360_fastlio2")

# Live + Rerun + raw-pcap capture. Writes a pcap of every Mid-360 UDP
# packet to fastlio2_pcap/mid360_<timestamp>.pcap (relative to CWD)
# alongside the normal SLAM pipeline. Needs tcpdump capture capability;
# the module prints the one-time `sudo setcap` if missing.
# Override the lidar IP via `LIDAR_IP=192.168.1.157 dimos run ...`.
# `deterministic_clock=True` makes scan boundaries + publish ts come from
# packet sensor ts (instead of wall clock) so this recording can be
# replayed bit-for-bit by demo_replay.
mid360_fastlio_record = autoconnect(
    FastLio2.blueprint(
        voxel_size=voxel_size,
        map_voxel_size=voxel_size,
        map_freq=-1,
        lidar_ip=os.getenv("LIDAR_IP", "192.168.1.155"),
        record_pcap=True,
        deterministic_clock=True,
    ),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="mid360_fastlio2_record")

# Replay a recorded Mid-360 pcap through FAST-LIO offline and visualize the
# output in Rerun. No SDK, no network. Pass the pcap path via env var:
#   REPLAY_PCAP=path/to/mid360.pcap dimos run mid360-fastlio-replay
# If a `<pcap>.first.ns` sidecar (written by the record blueprint when
# deterministic_clock=True) sits next to the pcap, we auto-pass it as
# replay_skip_until_ns so the algorithm starts from the same first packet
# the live SDK saw.
mid360_fastlio_replay = autoconnect(
    FastLio2.blueprint(
        voxel_size=voxel_size,
        map_voxel_size=voxel_size,
        map_freq=-1,
        replay_pcap=_replay_pcap,
        replay_skip_until_ns=_replay_skip_until_ns(_replay_pcap),
        deterministic_clock=True,
        single_threaded=True,
    ),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="mid360_fastlio2_replay")

mid360_fastlio_voxels = autoconnect(
    FastLio2.blueprint(),
    VoxelGridMapper.blueprint(voxel_size=voxel_size, carve_columns=False),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=3, robot_model="mid360_fastlio2_voxels")

mid360_fastlio_voxels_native = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=3.0),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=2, robot_model="mid360_fastlio2")


mid360_fastlio_ray_trace = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    RayTracingVoxelMap.blueprint(voxel_size=voxel_size),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=5, robot_model="mid360_fastlio2_ray_trace")
