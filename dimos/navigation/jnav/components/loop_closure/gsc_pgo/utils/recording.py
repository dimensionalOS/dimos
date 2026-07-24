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

"""Recording layout + rrd helpers: locate the db, load camera extrinsics, open a comparison rrd."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
from typing import Any

from gtsam import Pose3
import numpy as np

from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts import make_rrd
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts.make_rrd import (
    pose3_from_xyzquat,
)


def resolve_recording(rec_arg: str | Path) -> tuple[Path, Path]:
    """--rec is a recording dir (mem2.db + sidecar) or a bare .db; return (dir, db)."""
    rec_path = Path(rec_arg).expanduser()
    if rec_path.is_dir():
        return rec_path, rec_path / "mem2.db"
    return rec_path.parent, rec_path


def load_optical_transform(rec_dir: Path) -> tuple[Pose3, dict[str, Any] | None]:
    """Base<-optical camera transform + parsed intrinsics, or (identity, None) if absent."""
    intrinsics_path = rec_dir / "camera_intrinsics.json"
    if not intrinsics_path.exists():
        return Pose3(), None
    intrinsics = json.loads(intrinsics_path.read_text())
    return pose3_from_xyzquat(np.array(intrinsics["optical_in_base"], float)), intrinsics


def build_and_open_rrd(db_path: Path, lidar_stream: str, odom_stream: str, tag_stream: str) -> None:
    print("building comparison rrd...", flush=True)
    rrd_path = make_rrd.build(
        db_path, lidar_stream=lidar_stream, odom_stream=odom_stream, tag_stream=tag_stream
    )
    rerun_bin = Path(sys.executable).parent / "rerun"
    if rerun_bin.exists():
        subprocess.Popen([str(rerun_bin), str(rrd_path)])
        print(f"opened {rrd_path}", flush=True)
    else:
        print(f"rerun not found at {rerun_bin}; open manually: rerun {rrd_path}", flush=True)
