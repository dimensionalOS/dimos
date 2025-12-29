# Copyright 2025 Dimensional Inc.
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
from collections.abc import Callable, Iterator
import functools
import glob
import os
from pathlib import Path
import pickle
import re
import time
from typing import Any, Generic, TypeVar

from reactivex import (
    from_iterable,
    interval,
    operators as ops,
)
from reactivex.observable import Observable
from reactivex.scheduler import TimeoutScheduler

from dimos.utils.data import _get_data_dir, get_data

T = TypeVar("T")


class SensorReplay(Generic[T]):
    """Generic sensor data replay utility.

    Args:
        name: The name of the test dataset
        autocast: Optional function that takes unpickled data and returns a processed result.
                  For example: lambda data: LidarMessage.from_msg(data)
    """

    def __init__(self, name: str, autocast: Callable[[Any], T] | None = None) -> None:
        self.root_dir = get_data(name)
        self.autocast = autocast

    def load(self, *names: int | str) -> T | Any | list[T] | list[Any]:
        if len(names) == 1:
            return self.load_one(names[0])
        return list(map(lambda name: self.load_one(name), names))

    def load_one(self, name: int | str | Path) -> T | Any:
        if isinstance(name, int):
            full_path = self.root_dir / f"/{name:03d}.pickle"
        elif isinstance(name, Path):
            full_path = name
        else:
            full_path = self.root_dir / Path(f"{name}.pickle")

        with open(full_path, "rb") as f:
            data = pickle.load(f)
            if self.autocast:
                return self.autocast(data)
            return data

    def first(self) -> T | Any | None:
        try:
            return next(self.iterate())
        except StopIteration:
            return None

    @functools.cached_property
    def files(self) -> list[Path]:
        def extract_number(filepath):  # type: ignore[no-untyped-def]
            """Extract last digits before .pickle extension"""
            basename = os.path.basename(filepath)
            match = re.search(r"(\d+)\.pickle$", basename)
            return int(match.group(1)) if match else 0

        return sorted(
            glob.glob(os.path.join(self.root_dir, "*")),  # type: ignore[arg-type]
            key=extract_number,
        )

    def iterate(self, loop: bool = False) -> Iterator[T | Any]:
        while True:
            for file_path in self.files:
                yield self.load_one(Path(file_path))
            if not loop:
                break

    def stream(self, rate_hz: float | None = None, loop: bool = False) -> Observable[T | Any]:
        if rate_hz is None:
            return from_iterable(self.iterate(loop=loop))

        sleep_time = 1.0 / rate_hz

        return from_iterable(self.iterate(loop=loop)).pipe(
            ops.zip(interval(sleep_time)),
            ops.map(lambda x: x[0] if isinstance(x, tuple) else x),
        )


class SensorStorage(Generic[T]):
    """Generic sensor data storage utility
    .
        Creates a directory in the test data directory and stores pickled sensor data.

        Args:
            name: The name of the storage directory
            autocast: Optional function that takes data and returns a processed result before storage.
    """

    def __init__(self, name: str, autocast: Callable[[T], Any] | None = None) -> None:
        self.name = name
        self.autocast = autocast
        self.cnt = 0

        # Create storage directory in the data dir
        self.root_dir = _get_data_dir() / name

        # Check if directory exists and is not empty
        if self.root_dir.exists():
            existing_files = list(self.root_dir.glob("*.pickle"))
            if existing_files:
                raise RuntimeError(
                    f"Storage directory '{name}' already exists and contains {len(existing_files)} files. "
                    f"Please use a different name or clean the directory first."
                )
        else:
            # Create the directory
            self.root_dir.mkdir(parents=True, exist_ok=True)

    def consume_stream(self, observable: Observable[T | Any]) -> None:
        """Consume an observable stream of sensor data without saving."""
        return observable.subscribe(self.save_one)  # type: ignore[arg-type, return-value]

    def save_stream(self, observable: Observable[T | Any]) -> Observable[int]:
        """Save an observable stream of sensor data to pickle files."""
        return observable.pipe(ops.map(lambda frame: self.save_one(frame)))

    def save(self, *frames) -> int:  # type: ignore[no-untyped-def]
        """Save one or more frames to pickle files."""
        for frame in frames:
            self.save_one(frame)
        return self.cnt

    def save_one(self, frame) -> int:  # type: ignore[no-untyped-def]
        """Save a single frame to a pickle file."""
        file_name = f"{self.cnt:03d}.pickle"
        full_path = self.root_dir / file_name

        if full_path.exists():
            raise RuntimeError(f"File {full_path} already exists")

        # Apply autocast if provided
        data_to_save = frame
        if self.autocast:
            data_to_save = self.autocast(frame)
        # Convert to raw message if frame has a raw_msg attribute
        elif hasattr(frame, "raw_msg"):
            data_to_save = frame.raw_msg

        with open(full_path, "wb") as f:
            pickle.dump(data_to_save, f)

        self.cnt += 1
        return self.cnt


class TimedSensorStorage(SensorStorage[T]):
    def save_one(self, frame: T) -> int:
        return super().save_one((time.time(), frame))


class TimedSensorReplay(SensorReplay[T]):
    def load_one(self, name: int | str | Path) -> T | Any:
        if isinstance(name, int):
            full_path = self.root_dir / f"/{name:03d}.pickle"
        elif isinstance(name, Path):
            full_path = name
        else:
            full_path = self.root_dir / Path(f"{name}.pickle")

        with open(full_path, "rb") as f:
            data = pickle.load(f)
            if self.autocast:
                return (data[0], self.autocast(data[1]))
            return data

    def find_closest(self, timestamp: float, tolerance: float | None = None) -> T | Any | None:
        """Find the frame closest to the given timestamp.

        Args:
            timestamp: The target timestamp to search for
            tolerance: Optional maximum time difference allowed

        Returns:
            The data frame closest to the timestamp, or None if no match within tolerance
        """
        closest_data = None
        closest_diff = float("inf")

        # Check frames before and after the timestamp
        for ts, data in self.iterate_ts():
            diff = abs(ts - timestamp)

            if diff < closest_diff:
                closest_diff = diff
                closest_data = data
            elif diff > closest_diff:
                # We're moving away from the target, can stop
                break

        if tolerance is not None and closest_diff > tolerance:
            return None

        return closest_data

    def find_closest_seek(
        self, relative_seconds: float, tolerance: float | None = None
    ) -> T | Any | None:
        """Find the frame closest to a time relative to the start.

        Args:
            relative_seconds: Seconds from the start of the dataset
            tolerance: Optional maximum time difference allowed

        Returns:
            The data frame closest to the relative timestamp, or None if no match within tolerance
        """
        # Get the first timestamp
        first_ts = self.first_timestamp()
        if first_ts is None:
            return None

        # Calculate absolute timestamp and use find_closest
        target_timestamp = first_ts + relative_seconds
        return self.find_closest(target_timestamp, tolerance)

    def first_timestamp(self) -> float | None:
        """Get the timestamp of the first item in the dataset.

        Returns:
            The first timestamp, or None if dataset is empty
        """
        try:
            ts, _ = next(self.iterate_ts())
            return ts
        except StopIteration:
            return None

    def iterate(self, loop: bool = False) -> Iterator[T | Any]:
        return (x[1] for x in super().iterate(loop=loop))  # type: ignore[index]

    def iterate_duration(self, **kwargs: Any) -> Iterator[tuple[float, T] | Any]:
        """Iterate with timestamps relative to the start of the dataset."""
        first_ts = self.first_timestamp()
        if first_ts is None:
            return
        for ts, data in self.iterate_ts(**kwargs):
            yield (ts - first_ts, data)

    def iterate_realtime(self, speed: float = 1.0, **kwargs: Any) -> Iterator[T | Any]:
        """Iterate data, sleeping to match original timing.

        Args:
            speed: Playback speed multiplier (1.0 = realtime, 2.0 = 2x speed)
            **kwargs: Passed to iterate_ts (seek, duration, from_timestamp, loop)
        """
        iterator = self.iterate_ts(**kwargs)

        try:
            first_ts, first_data = next(iterator)
        except StopIteration:
            return

        start_time = time.time()
        start_ts = first_ts
        yield first_data

        for ts, data in iterator:
            target_time = start_time + (ts - start_ts) / speed
            sleep_duration = target_time - time.time()
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            yield data

    def iterate_ts(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[tuple[float, T] | Any]:
        """Iterate with absolute timestamps, with optional seek and duration."""
        first_ts = None
        if (seek is not None) or (duration is not None):
            first_ts = self.first_timestamp()
            if first_ts is None:
                return

        if seek is not None:
            from_timestamp = first_ts + seek  # type: ignore[operator]

        end_timestamp = None
        if duration is not None:
            end_timestamp = (from_timestamp if from_timestamp else first_ts) + duration  # type: ignore[operator]

        while True:
            for ts, data in super().iterate():  # type: ignore[misc]
                if from_timestamp is None or ts >= from_timestamp:
                    if end_timestamp is not None and ts >= end_timestamp:
                        break
                    yield (ts, data)
            if not loop:
                break

    def stream(  # type: ignore[override]
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Observable[T | Any]:
        def _subscribe(observer, scheduler=None):  # type: ignore[no-untyped-def]
            from reactivex.disposable import CompositeDisposable, Disposable

            scheduler = scheduler or TimeoutScheduler()
            disp = CompositeDisposable()
            is_disposed = False

            iterator = self.iterate_ts(
                seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
            )

            # Get first message
            try:
                first_ts, first_data = next(iterator)
            except StopIteration:
                observer.on_completed()
                return Disposable()

            # Establish timing reference
            start_local_time = time.time()
            start_replay_time = first_ts

            # Emit first sample immediately
            observer.on_next(first_data)

            # Pre-load next message
            try:
                next_message = next(iterator)
            except StopIteration:
                observer.on_completed()
                return disp

            def schedule_emission(message) -> None:  # type: ignore[no-untyped-def]
                nonlocal next_message, is_disposed

                if is_disposed:
                    return

                ts, data = message

                # Pre-load the following message while we have time
                try:
                    next_message = next(iterator)
                except StopIteration:
                    next_message = None

                # Calculate absolute emission time
                target_time = start_local_time + (ts - start_replay_time) / speed
                delay = max(0.0, target_time - time.time())

                def emit() -> None:
                    if is_disposed:
                        return
                    observer.on_next(data)
                    if next_message is not None:
                        schedule_emission(next_message)
                    else:
                        observer.on_completed()
                        # Dispose of the scheduler to clean up threads
                        if hasattr(scheduler, "dispose"):
                            scheduler.dispose()

                scheduler.schedule_relative(delay, lambda sc, _: emit())

            schedule_emission(next_message)

            # Create a custom disposable that properly cleans up
            def dispose() -> None:
                nonlocal is_disposed
                is_disposed = True
                disp.dispose()
                # Ensure scheduler is disposed to clean up any threads
                if hasattr(scheduler, "dispose"):
                    scheduler.dispose()

            return Disposable(dispose)

        from reactivex import create

        return create(_subscribe)
