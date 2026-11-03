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

"""Recording rrd helper: build a comparison rrd and open it in rerun."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys

from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts import make_rrd


def build_and_open_rrd(
    db_path: Path,
    lidar_stream: str,
    odom_stream: str,
    tag_stream: str,
    world_frame: str,
    camera_stream: str = "color_image",
    camera_info_stream: str = "",
) -> None:
    print("building comparison rrd...", flush=True)
    rrd_path = make_rrd.build(
        db_path,
        lidar_stream=lidar_stream,
        odom_stream=odom_stream,
        tag_stream=tag_stream,
        world_frame=world_frame,
        camera_stream=camera_stream,
        camera_info_stream=camera_info_stream,
    )
    rerun_bin = Path(sys.executable).parent / "rerun"
    if rerun_bin.exists():
        subprocess.Popen([str(rerun_bin), str(rrd_path)])
        print(f"opened {rrd_path}", flush=True)
    else:
        print(f"rerun not found at {rerun_bin}; open manually: rerun {rrd_path}", flush=True)
