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

"""Read a KITTI-360 sequence from disk: poses + Velodyne scans.

KITTI-360 directory convention (from http://www.cvlibs.net/datasets/kitti-360/):

    <root>/
        data_3d_raw/2013_05_28_drive_<seq>_sync/velodyne_points/
            data/<frame_id>.bin           # float32 [x, y, z, intensity]
            timestamps.txt                # one ISO ts per frame
        data_poses/2013_05_28_drive_<seq>_sync/
            poses.txt                     # "<frame_id> <16 floats>" per row
            cam0_to_world.txt             # alternative pose format
        calibration/
            calib_cam_to_velo.txt
            calib_cam_to_pose.txt

The poses in ``poses.txt`` are the camera (cam0) pose in world coordinates as
a flattened 4x4 matrix. To get lidar pose in world we apply
``cam0_to_velo`` (so velo_world = cam0_world @ cam0_to_velo^-1).
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np

KITTI360_DRIVE_TEMPLATE = "2013_05_28_drive_{seq:04d}_sync"


@dataclass(frozen=True)
class Kitti360Frame:
    """One synchronized (scan, pose, timestamp) triple."""

    frame_id: int
    timestamp: float
    pose_world: np.ndarray  # (4, 4) — lidar-frame pose in world
    scan_path: Path


def _parse_kitti_calib_matrix(path: Path, key_prefix: str = "") -> np.ndarray:
    """Read a KITTI calibration matrix file.

    Supports two formats:
    * "<key>: f1 f2 ... f12" lines (camera-to-X style) — picks the first
      one matching ``key_prefix``, or the first row if prefix is empty.
    * Whitespace-only 16-float matrix written as a single block.
    """
    text = path.read_text().strip()
    if not text:
        raise ValueError(f"Empty calibration file: {path}")

    # Try labelled-row format.
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
            mat = np.eye(4, dtype=np.float64)
            mat[:3, :4] = np.array(values, dtype=np.float64).reshape(3, 4)
            return mat
        if len(values) == 16:
            return np.array(values, dtype=np.float64).reshape(4, 4)
        # otherwise keep scanning

    raise ValueError(f"No 12/16-float row found in {path}")


def _load_poses_file(path: Path) -> dict[int, np.ndarray]:
    """Parse a KITTI-360 ``poses.txt`` (per-frame 4x4 in world).

    Each row: ``frame_id f00 f01 f02 f03 f10 ... f23`` (12 numbers,
    interpreted as a 3x4 matrix; the bottom row is implicitly
    [0, 0, 0, 1]). Returns dict mapping ``frame_id`` → 4x4 pose.
    """
    poses: dict[int, np.ndarray] = {}
    for line in path.read_text().splitlines():
        tokens = line.split()
        if len(tokens) < 13:
            continue
        frame_id = int(tokens[0])
        values = np.array([float(token) for token in tokens[1:13]], dtype=np.float64)
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :4] = values.reshape(3, 4)
        poses[frame_id] = mat
    return poses


def _load_timestamps_file(path: Path) -> dict[int, float]:
    """Read Velodyne ``timestamps.txt``; map (line index → unix-style float).

    KITTI-360 uses ISO 8601 with nanoseconds. We convert to a float
    second offset relative to the first sample for stable monotonic
    timestamps regardless of timezone.
    """
    from datetime import datetime

    timestamps: dict[int, float] = {}
    base: float | None = None
    for index, line in enumerate(path.read_text().splitlines()):
        line = line.strip()
        if not line:
            continue
        try:
            parsed = datetime.fromisoformat(line)
        except ValueError:
            # Some KITTI-360 files use ' ' instead of 'T'; replace and retry.
            parsed = datetime.fromisoformat(line.replace(" ", "T", 1))
        unix = parsed.timestamp()
        if base is None:
            base = unix
        timestamps[index] = unix - base
    return timestamps


@dataclass
class Kitti360Sequence:
    """A loaded KITTI-360 sequence — poses + an indexable scan list."""

    sequence_id: int
    velodyne_dir: Path
    timestamps: dict[int, float]
    poses_world: dict[int, np.ndarray]  # cam0 → world per frame
    velo_to_cam: np.ndarray  # (4, 4) lidar → cam0

    @property
    def frame_ids(self) -> list[int]:
        """Frame ids that have BOTH a scan AND a pose."""
        scan_ids = {int(p.stem) for p in self.velodyne_dir.glob("*.bin")}
        return sorted(scan_ids & self.poses_world.keys())

    def lidar_pose(self, frame_id: int) -> np.ndarray:
        """4x4 lidar pose in world frame."""
        cam0_to_world = self.poses_world[frame_id]
        # lidar → world = cam0 → world  ⊕  cam0 → lidar^-1
        return cam0_to_world @ np.linalg.inv(self.velo_to_cam)

    def scan_xyz(self, frame_id: int) -> np.ndarray:
        """Load the (N, 4) Velodyne scan for ``frame_id`` as float32."""
        scan_path = self.velodyne_dir / f"{frame_id:010d}.bin"
        data = np.fromfile(str(scan_path), dtype=np.float32)
        if data.size % 4 != 0:
            raise ValueError(f"Scan {scan_path} has unexpected length {data.size}")
        return data.reshape(-1, 4)

    def frames(self, frame_ids: list[int] | None = None) -> Iterator[Kitti360Frame]:
        """Yield ``Kitti360Frame`` for the given (or all valid) frames."""
        ids = frame_ids if frame_ids is not None else self.frame_ids
        for frame_id in ids:
            yield Kitti360Frame(
                frame_id=frame_id,
                timestamp=self.timestamps.get(frame_id, float(frame_id)),
                pose_world=self.lidar_pose(frame_id),
                scan_path=self.velodyne_dir / f"{frame_id:010d}.bin",
            )


def load_kitti360_sequence(root: Path, sequence_id: int) -> Kitti360Sequence:
    """Load poses + scan index for one KITTI-360 sequence.

    Raises ``FileNotFoundError`` if the expected directory layout is
    missing; the caller is responsible for falling back / skipping.
    """
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
    # timestamps.txt has one line per .bin file in this split's velodyne_points/data,
    # in sorted-by-filename order. Rekey by the actual frame_id so callers can do
    # ``timestamps[frame_id]`` instead of "line index" lookups (which silently miss
    # in the Test SLAM split because frame_ids don't start at 0).
    timestamps: dict[int, float] = {}
    if timestamps_path.exists():
        sorted_scan_ids = sorted(int(p.stem) for p in velodyne_dir.glob("*.bin"))
        line_indexed = _load_timestamps_file(timestamps_path)
        if len(sorted_scan_ids) == len(line_indexed):
            timestamps = {fid: line_indexed[i] for i, fid in enumerate(sorted_scan_ids)}

    return Kitti360Sequence(
        sequence_id=sequence_id,
        velodyne_dir=velodyne_dir,
        timestamps=timestamps,
        poses_world=poses_world,
        velo_to_cam=velo_to_cam,
    )
