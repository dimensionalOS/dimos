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
"""Legacy pickle directory backend for TimeSeriesStore.

Compatible with TimedSensorReplay/TimedSensorStorage file format.
"""

from collections.abc import Callable, Iterator
import glob
import os
from pathlib import Path
import pickle
import re
import time
from typing import Any, cast

import reactivex as rx
from reactivex.disposable import CompositeDisposable, Disposable
from reactivex.observable import Observable
from reactivex.scheduler import TimeoutScheduler

from dimos.memory.timeseries.base import T, TimeSeriesStore
from dimos.utils.data import get_data, get_data_dir


class LegacyPickleStore(TimeSeriesStore[T]):
    """Legacy pickle backend compatible with TimedSensorReplay/TimedSensorStorage.

    File format:
        {name}/
            000.pickle  # contains (timestamp, data) tuple
            001.pickle
            ...

    Files are assumed to be in chronological order (timestamps increase with file number).
    No index is built - iteration is lazy and memory-efficient for large datasets.

    Usage:
        # Load existing recording (auto-downloads from LFS if needed)
        store = LegacyPickleStore("unitree_go2_bigoffice/lidar")
        data = store.find_closest_seek(10.0)

        # Create new recording (directory created on first save)
        store = LegacyPickleStore("my_recording/images")
        store.save_ts(image)  # uses image.ts for timestamp

    Backward compatibility:
        This class also supports the old TimedSensorReplay/SensorReplay API:
        - iterate_ts() - iterate returning (timestamp, data) tuples
        - files - property returning list of file paths
        - load_one() - load a single pickle file
    """

    def __init__(self, name: str | Path, autocast: Callable[[Any], T] | None = None) -> None:
        """
        Args:
            name: Data directory name (e.g. "unitree_go2_bigoffice/lidar") or absolute path.
            autocast: Optional function to transform data after loading (for replay) or
                      before saving (for storage). E.g., `Odometry.from_msg`.
        """
        self._name = str(name)
        self._root_dir: Path | None = None
        self._counter: int = 0
        self._autocast = autocast

    def _get_root_dir(self, for_write: bool = False) -> Path:
        """Get root directory, creating on first write if needed."""
        if self._root_dir is not None:
            # Ensure directory exists if writing
            if for_write:
                self._root_dir.mkdir(parents=True, exist_ok=True)
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

    def _iter_files(self) -> Iterator[Path]:
        """Iterate pickle files in sorted order (by number in filename)."""

        def extract_number(filepath: str) -> int:
            basename = os.path.basename(filepath)
            match = re.search(r"(\d+)\.pickle$", basename)
            return int(match.group(1)) if match else 0

        root_dir = self._get_root_dir()
        files = sorted(
            glob.glob(os.path.join(root_dir, "*.pickle")),
            key=extract_number,
        )
        for f in files:
            yield Path(f)

    def _save(self, timestamp: float, data: T) -> None:
        root_dir = self._get_root_dir(for_write=True)

        # Initialize counter from existing files if needed
        if self._counter == 0:
            existing = list(root_dir.glob("*.pickle"))
            if existing:
                # Find highest existing counter
                max_num = 0
                for filepath in existing:
                    match = re.search(r"(\d+)\.pickle$", filepath.name)
                    if match:
                        max_num = max(max_num, int(match.group(1)))
                self._counter = max_num + 1

        full_path = root_dir / f"{self._counter:03d}.pickle"

        if full_path.exists():
            raise RuntimeError(f"File {full_path} already exists")

        # Save as (timestamp, data) tuple for legacy compatibility
        with open(full_path, "wb") as f:
            pickle.dump((timestamp, data), f)

        self._counter += 1

    def _load(self, timestamp: float) -> T | None:
        """Load data at exact timestamp (linear scan)."""
        for ts, data in self._iter_items():
            if ts == timestamp:
                return data
        return None

    def _delete(self, timestamp: float) -> T | None:
        """Delete not supported for legacy pickle format."""
        raise NotImplementedError("LegacyPickleStore does not support deletion")

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        """Lazy iteration - loads one file at a time.

        Handles both timed format (timestamp, data) and non-timed format (just data).
        For non-timed data, uses file index as synthetic timestamp.
        """
        for idx, filepath in enumerate(self._iter_files()):
            try:
                with open(filepath, "rb") as f:
                    raw = pickle.load(f)

                # Handle both timed (timestamp, data) and non-timed (just data) formats
                if isinstance(raw, tuple) and len(raw) == 2:
                    ts, data = raw
                    ts = float(ts)
                else:
                    # Non-timed format: use index as synthetic timestamp
                    ts = float(idx)
                    data = raw
            except Exception:
                continue

            if start is not None and ts < start:
                continue
            if end is not None and ts >= end:
                break

            if self._autocast is not None:
                data = self._autocast(data)
            yield (ts, cast("T", data))

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        """Linear scan with early exit (assumes timestamps are monotonically increasing)."""
        closest_ts: float | None = None
        closest_diff = float("inf")

        for ts, _ in self._iter_items():
            diff = abs(ts - timestamp)

            if diff < closest_diff:
                closest_diff = diff
                closest_ts = ts
            elif diff > closest_diff:
                # Moving away from target, can stop
                break

        if closest_ts is None:
            return None

        if tolerance is not None and closest_diff > tolerance:
            return None

        return closest_ts

    def _count(self) -> int:
        return sum(1 for _ in self._iter_files())

    def _last_timestamp(self) -> float | None:
        last_ts: float | None = None
        for ts, _ in self._iter_items():
            last_ts = ts
        return last_ts

    def _find_before(self, timestamp: float) -> tuple[float, T] | None:
        result: tuple[float, T] | None = None
        for ts, data in self._iter_items():
            if ts < timestamp:
                result = (ts, data)
            else:
                break
        return result

    def _find_after(self, timestamp: float) -> tuple[float, T] | None:
        for ts, data in self._iter_items():
            if ts > timestamp:
                return (ts, data)
        return None

    # === Backward-compatible API (TimedSensorReplay/SensorReplay) ===

    @property
    def files(self) -> list[Path]:
        """Return list of pickle files (backward compatibility with SensorReplay)."""
        return list(self._iter_files())

    def load_one(self, name: int | str | Path) -> T | Any:
        """Load a single pickle file (backward compatibility with SensorReplay).

        Args:
            name: File index (int), filename without extension (str), or full path (Path)

        Returns:
            For TimedSensorReplay: (timestamp, data) tuple
            For SensorReplay: just the data
        """
        root_dir = self._get_root_dir()

        if isinstance(name, int):
            full_path = root_dir / f"{name:03d}.pickle"
        elif isinstance(name, Path):
            full_path = name
        else:
            full_path = root_dir / Path(f"{name}.pickle")

        with open(full_path, "rb") as f:
            data = pickle.load(f)

        # Legacy format: (timestamp, data) tuple
        if isinstance(data, tuple) and len(data) == 2:
            ts, payload = data
            if self._autocast is not None:
                payload = self._autocast(payload)
            return (ts, payload)

        # Non-timed format: just data
        if self._autocast is not None:
            data = self._autocast(data)
        return data

    def iterate_ts(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[tuple[float, T]]:
        """Iterate with timestamps (backward compatibility with TimedSensorReplay).

        Args:
            seek: Relative seconds from start
            duration: Duration window in seconds
            from_timestamp: Absolute timestamp to start from
            loop: Whether to loop the data

        Yields:
            (timestamp, data) tuples
        """
        first = self.first_timestamp()
        if first is None:
            return

        # Calculate start timestamp
        start: float | None = None
        if from_timestamp is not None:
            start = from_timestamp
        elif seek is not None:
            start = first + seek

        # Calculate end timestamp
        end: float | None = None
        if duration is not None:
            start_ts = start if start is not None else first
            end = start_ts + duration

        while True:
            yield from self._iter_items(start=start, end=end)
            if not loop:
                break

    def stream(
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Observable[T]:
        """Stream data as Observable with timing control.

        Uses stored timestamps from pickle files for timing (not data.ts).
        """

        def subscribe(
            observer: rx.abc.ObserverBase[T],
            scheduler: rx.abc.SchedulerBase | None = None,
        ) -> rx.abc.DisposableBase:
            sched = scheduler or TimeoutScheduler()
            disp = CompositeDisposable()
            is_disposed = False

            iterator = self.iterate_ts(
                seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
            )

            try:
                first_ts, first_data = next(iterator)
            except StopIteration:
                observer.on_completed()
                return Disposable()

            start_local_time = time.time()
            start_replay_time = first_ts

            observer.on_next(first_data)

            try:
                next_message: tuple[float, T] | None = next(iterator)
            except StopIteration:
                observer.on_completed()
                return disp

            def schedule_emission(message: tuple[float, T]) -> None:
                nonlocal next_message, is_disposed

                if is_disposed:
                    return

                ts, data = message

                try:
                    next_message = next(iterator)
                except StopIteration:
                    next_message = None

                target_time = start_local_time + (ts - start_replay_time) / speed
                delay = max(0.0, target_time - time.time())

                def emit(
                    _scheduler: rx.abc.SchedulerBase, _state: object
                ) -> rx.abc.DisposableBase | None:
                    if is_disposed:
                        return None
                    observer.on_next(data)
                    if next_message is not None:
                        schedule_emission(next_message)
                    else:
                        observer.on_completed()
                    return None

                sched.schedule_relative(delay, emit)

            if next_message is not None:
                schedule_emission(next_message)

            def dispose() -> None:
                nonlocal is_disposed
                is_disposed = True
                disp.dispose()

            return Disposable(dispose)

        return rx.create(subscribe)
