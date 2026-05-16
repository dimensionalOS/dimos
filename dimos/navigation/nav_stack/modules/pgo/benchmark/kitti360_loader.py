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

"""Read a KITTI-360 sequence from disk.

Layout (from cvlibs.net/datasets/kitti-360):

    <root>/
        data_3d_raw/2013_05_28_drive_<seq>_sync/velodyne_points/
            data/<frame_id>.bin
            timestamps.txt
        data_poses/2013_05_28_drive_<seq>_sync/poses.txt
        calibration/calib_cam_to_velo.txt

poses.txt rows are cam0→world; we left-multiply by inv(cam0→lidar) to get
lidar→world.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

KITTI360_DRIVE_TEMPLATE = "2013_05_28_drive_{seq:04d}_sync"


@dataclass(frozen=True)
class Kitti360Frame:
    frame_id: int
    timestamp: float
    pose_world: np.ndarray
    scan_path: Path


def _parse_kitti_calib_matrix(path: Path, key_prefix: str = "") -> np.ndarray:
    """Parse a KITTI calibration matrix file.

    Two on-disk formats are accepted: a labelled ``<key>: f1 f2 ... f12``
    line (picks the first row matching ``key_prefix``, or the first row
    if no prefix) or a bare 16-float matrix block.
    """
    text = path.read_text().strip()
    if not text:
        raise ValueError(f"Empty calibration file: {path}")

    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if ":" in line:
            key, _, rest = line.partition(":")
            if key_prefix and not key.strip().startswith(key_prefix):
                continue
            values = [float(token) for token in rest.split()]
        else:
            values = [float(token) for token in line.split()]
        if len(values) == 12:
            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :4] = np.array(values, dtype=np.float64).reshape(3, 4)
            return matrix
        if len(values) == 16:
            return np.array(values, dtype=np.float64).reshape(4, 4)

    raise ValueError(f"No 12/16-float row found in {path}")


def _load_poses_file(path: Path) -> dict[int, np.ndarray]:
    poses: dict[int, np.ndarray] = {}
    for line in path.read_text().splitlines():
        tokens = line.split()
        if len(tokens) < 13:
            continue
        frame_id = int(tokens[0])
        values = np.array([float(token) for token in tokens[1:13]], dtype=np.float64)
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :4] = values.reshape(3, 4)
        poses[frame_id] = matrix
    return poses


def _load_timestamps_file(path: Path) -> dict[int, float]:
    """Read ``timestamps.txt`` as line_index → seconds-since-first-sample.

    Returned dict is line-keyed (not frame-keyed) so the caller can decide
    how to align with the actual frame ids in the split — see the
    rekeying in ``load_kitti360_sequence``.
    """
    timestamps: dict[int, float] = {}
    base: float | None = None
    for index, line in enumerate(path.read_text().splitlines()):
        line = line.strip()
        if not line:
            continue
        try:
            parsed = datetime.fromisoformat(line)
        except ValueError:
            # Some KITTI-360 files use a space instead of 'T'.
            parsed = datetime.fromisoformat(line.replace(" ", "T", 1))
        unix_seconds = parsed.timestamp()
        if base is None:
            base = unix_seconds
        timestamps[index] = unix_seconds - base
    return timestamps


@dataclass
class Kitti360Sequence:
    sequence_id: int
    velodyne_dir: Path
    timestamps: dict[int, float]
    poses_world: dict[int, np.ndarray]
    velo_to_cam: np.ndarray

    @property
    def frame_ids(self) -> list[int]:
        scan_ids = {int(scan.stem) for scan in self.velodyne_dir.glob("*.bin")}
        return sorted(scan_ids & self.poses_world.keys())

    def lidar_pose(self, frame_id: int) -> np.ndarray:
        cam0_to_world = self.poses_world[frame_id]
        return cam0_to_world @ np.linalg.inv(self.velo_to_cam)

    def scan_xyz(self, frame_id: int) -> np.ndarray:
        scan_path = self.velodyne_dir / f"{frame_id:010d}.bin"
        data = np.fromfile(str(scan_path), dtype=np.float32)
        if data.size % 4 != 0:
            raise ValueError(f"Scan {scan_path} has unexpected length {data.size}")
        return data.reshape(-1, 4)

    def frames(self, frame_ids: list[int] | None = None) -> Iterator[Kitti360Frame]:
        selected = frame_ids if frame_ids is not None else self.frame_ids
        for frame_id in selected:
            yield Kitti360Frame(
                frame_id=frame_id,
                timestamp=self.timestamps.get(frame_id, float(frame_id)),
                pose_world=self.lidar_pose(frame_id),
                scan_path=self.velodyne_dir / f"{frame_id:010d}.bin",
            )


def load_kitti360_sequence(root: Path, sequence_id: int) -> Kitti360Sequence:
    drive = KITTI360_DRIVE_TEMPLATE.format(seq=sequence_id)
    velodyne_dir = root / "data_3d_raw" / drive / "velodyne_points" / "data"
    poses_path = root / "data_poses" / drive / "poses.txt"
    timestamps_path = root / "data_3d_raw" / drive / "velodyne_points" / "timestamps.txt"
    calib_path = root / "calibration" / "calib_cam_to_velo.txt"

    for required in (velodyne_dir, poses_path, calib_path):
        if not required.exists():
            raise FileNotFoundError(f"KITTI-360 layout missing under {root}: {required}")

    velo_to_cam = _parse_kitti_calib_matrix(calib_path)
    poses_world = _load_poses_file(poses_path)

    # Rekey timestamps by actual frame_id (the on-disk file is line-indexed
    # but the Test SLAM split's frame_ids don't start at 0)
    timestamps: dict[int, float] = {}
    if timestamps_path.exists():
        sorted_scan_ids = sorted(int(scan.stem) for scan in velodyne_dir.glob("*.bin"))
        line_indexed = _load_timestamps_file(timestamps_path)
        if len(sorted_scan_ids) != len(line_indexed):
            raise ValueError(
                f"KITTI-360 timestamp count mismatch under {root}: "
                f"{len(sorted_scan_ids)} .bin files in {velodyne_dir} but "
                f"{len(line_indexed)} lines in {timestamps_path}."
            )
        timestamps = {
            frame_id: line_indexed[line_index]
            for line_index, frame_id in enumerate(sorted_scan_ids)
        }

    return Kitti360Sequence(
        sequence_id=sequence_id,
        velodyne_dir=velodyne_dir,
        timestamps=timestamps,
        poses_world=poses_world,
        velo_to_cam=velo_to_cam,
    )
