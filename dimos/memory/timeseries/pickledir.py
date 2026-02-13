# Copyright 2025-2026 Dimensional Inc.
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
"""Pickle directory backend for TimeSeriesStore."""

import bisect
from collections.abc import Iterator
import glob
import os
from pathlib import Path
import pickle

from dimos.memory.timeseries.base import T, TimeSeriesStore
from dimos.utils.data import get_data, get_data_dir


class PickleDirStore(TimeSeriesStore[T]):
    """Pickle directory backend. Files named by timestamp.

    Directory structure:
        {name}/
            1704067200.123.pickle
            1704067200.456.pickle
            ...

    Usage:
        # Load existing recording (auto-downloads from LFS if needed)
        store = PickleDirStore("unitree_go2_bigoffice/lidar")
        data = store.find_closest_seek(10.0)

        # Create new recording (directory created on first save)
        store = PickleDirStore("my_recording/images")
        store.save(image)  # saves using image.ts
    """

    def __init__(self, name: str) -> None:
        """
        Args:
            name: Data directory name (e.g. "unitree_go2_bigoffice/lidar")
        """
        self._name = name
        self._root_dir: Path | None = None

        # Cached sorted timestamps for find_closest
        self._timestamps: list[float] | None = None

    def _get_root_dir(self, for_write: bool = False) -> Path:
        """Get root directory, creating on first write if needed."""
        if self._root_dir is not None:
            return self._root_dir

        # If absolute path, use directly
        if Path(self._name).is_absolute():
            self._root_dir = Path(self._name)
            if for_write:
                self._root_dir.mkdir(parents=True, exist_ok=True)
        elif for_write:
            # For writing: use get_data_dir and create if needed
            self._root_dir = get_data_dir(self._name)
            self._root_dir.mkdir(parents=True, exist_ok=True)
        else:
            # For reading: use get_data (handles LFS download)
            self._root_dir = get_data(self._name)

        return self._root_dir

    def _save(self, timestamp: float, data: T) -> None:
        root_dir = self._get_root_dir(for_write=True)
        full_path = root_dir / f"{timestamp}.pickle"

        if full_path.exists():
            raise RuntimeError(f"File {full_path} already exists")

        with open(full_path, "wb") as f:
            pickle.dump(data, f)

        self._timestamps = None  # Invalidate cache

    def _load(self, timestamp: float) -> T | None:
        filepath = self._get_root_dir() / f"{timestamp}.pickle"
        if filepath.exists():
            return self._load_file(filepath)
        return None

    def _delete(self, timestamp: float) -> T | None:
        filepath = self._get_root_dir() / f"{timestamp}.pickle"
        if filepath.exists():
            data = self._load_file(filepath)
            filepath.unlink()
            self._timestamps = None  # Invalidate cache
            return data
        return None

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        for ts in self._get_timestamps():
            if start is not None and ts < start:
                continue
            if end is not None and ts >= end:
                break
            data = self._load(ts)
            if data is not None:
                yield (ts, data)

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        timestamps = self._get_timestamps()
        if not timestamps:
            return None

        pos = bisect.bisect_left(timestamps, timestamp)

        # Check neighbors
        candidates = []
        if pos > 0:
            candidates.append(timestamps[pos - 1])
        if pos < len(timestamps):
            candidates.append(timestamps[pos])

        if not candidates:
            return None

        closest = min(candidates, key=lambda ts: abs(ts - timestamp))

        if tolerance is not None and abs(closest - timestamp) > tolerance:
            return None

        return closest

    def _get_timestamps(self) -> list[float]:
        """Get sorted list of all timestamps."""
        if self._timestamps is not None:
            return self._timestamps

        timestamps: list[float] = []
        root_dir = self._get_root_dir()
        for filepath in glob.glob(os.path.join(root_dir, "*.pickle")):
            try:
                ts = float(Path(filepath).stem)
                timestamps.append(ts)
            except ValueError:
                continue

        timestamps.sort()
        self._timestamps = timestamps
        return timestamps

    def _count(self) -> int:
        return len(self._get_timestamps())

    def _last_timestamp(self) -> float | None:
        timestamps = self._get_timestamps()
        return timestamps[-1] if timestamps else None

    def _find_before(self, timestamp: float) -> tuple[float, T] | None:
        timestamps = self._get_timestamps()
        if not timestamps:
            return None
        pos = bisect.bisect_left(timestamps, timestamp)
        if pos > 0:
            ts = timestamps[pos - 1]
            data = self._load(ts)
            if data is not None:
                return (ts, data)
        return None

    def _find_after(self, timestamp: float) -> tuple[float, T] | None:
        timestamps = self._get_timestamps()
        if not timestamps:
            return None
        pos = bisect.bisect_right(timestamps, timestamp)
        if pos < len(timestamps):
            ts = timestamps[pos]
            data = self._load(ts)
            if data is not None:
                return (ts, data)
        return None

    def _load_file(self, filepath: Path) -> T | None:
        """Load data from a pickle file (LRU cached)."""
        try:
            with open(filepath, "rb") as f:
                data: T = pickle.load(f)
                return data
        except Exception:
            return None
