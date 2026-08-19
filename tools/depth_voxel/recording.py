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

"""Where each recording keeps its depth, lidar, odometry and mount transforms."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

RESULTS_ROOT = Path.home() / ".local/share/cbg/long-tasks/SureFelidae/results"


@dataclass(frozen=True)
class Recording:
    name: str
    db_path: str
    tf_stream: str
    odometry_stream: str
    lidar_stream: str
    depth_stream: str
    depth_info_stream: str
    base_frame: str
    lidar_frame: str
    depth_frame: str

    @property
    def results_dir(self) -> Path:
        return RESULTS_ROOT / self.name


ALFRED_DRIVE = Recording(
    name="alfred_drive",
    db_path=str(Path.home() / "datasets/alfred/drive_2026-08-16_23-46-03.db"),
    tf_stream="tf_corrected",
    odometry_stream="odometry",
    lidar_stream="lidar",
    depth_stream="depth_image",
    depth_info_stream="depth_camera_info",
    base_frame="base_link",
    lidar_frame="mid360_link",
    depth_frame="camera_depth_optical_frame",
)


def d455_handheld(name: str) -> Recording:
    """One of the handheld D455 + Mid-360 recordings under ``~/datasets/d455``."""
    return Recording(
        name=name,
        db_path=str(Path.home() / f"datasets/d455/{name}/main.db"),
        tf_stream="tf",
        odometry_stream="pointlio_odometry",
        lidar_stream="pointlio_lidar",
        depth_stream="realsense_depth_image",
        depth_info_stream="realsense_depth_image_camera_info",
        base_frame="base_link",
        lidar_frame="mid360_link",
        depth_frame="d455_depth_optical_frame",
    )


RECORDINGS = {
    ALFRED_DRIVE.name: ALFRED_DRIVE,
    **{name: d455_handheld(name) for name in ("sf_office1", "sf_office1_2", "sf_office1_3")},
}


def get(name: str) -> Recording:
    if name not in RECORDINGS:
        raise SystemExit(f"unknown recording {name!r}; have {sorted(RECORDINGS)}")
    return RECORDINGS[name]
