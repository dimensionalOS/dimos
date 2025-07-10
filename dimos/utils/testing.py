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

import glob
import os
import pickle
import time
from pathlib import Path
from typing import Any, Callable, Generic, Iterator, Optional, Tuple, TypeVar, Union

from reactivex import concat, empty, from_iterable, interval, just, merge, timer
from reactivex import operators as ops
from reactivex import timer as rx_timer
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

    def __init__(self, name: str, autocast: Optional[Callable[[Any], T]] = None):
        self.root_dir = get_data(name)
        self.autocast = autocast

    def load(self, *names: Union[int, str]) -> Union[T, Any, list[T], list[Any]]:
        if len(names) == 1:
            return self.load_one(names[0])
        return list(map(lambda name: self.load_one(name), names))

    def load_one(self, name: Union[int, str, Path]) -> Union[T, Any]:
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

    def iterate(self) -> Iterator[Union[T, Any]]:
        pattern = os.path.join(self.root_dir, "*")
        for file_path in sorted(glob.glob(pattern)):
            yield self.load_one(Path(file_path))

    def stream(self, rate_hz: Optional[float] = None) -> Observable[Union[T, Any]]:
        if rate_hz is None:
            return from_iterable(self.iterate())

        sleep_time = 1.0 / rate_hz

        return from_iterable(self.iterate()).pipe(
            ops.zip(interval(sleep_time)),
            ops.map(lambda x: x[0] if isinstance(x, tuple) else x),
        )


class SensorStorage(Generic[T]):
    """Generic sensor data storage utility.

    Creates a directory in the test data directory and stores pickled sensor data.

    Args:
        name: The name of the storage directory
        autocast: Optional function that takes data and returns a processed result before storage.
    """

    def __init__(self, name: str, autocast: Optional[Callable[[T], Any]] = None):
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

    def save_stream(self, observable: Observable[Union[T, Any]]) -> Observable[int]:
        """Save an observable stream of sensor data to pickle files."""
        return observable.pipe(ops.map(lambda frame: self.save_one(frame)))

    def save(self, *frames) -> int:
        """Save one or more frames to pickle files."""
        for frame in frames:
            self.save_one(frame)
        return self.cnt

    def save_one(self, frame) -> int:
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
    def load_one(self, name: Union[int, str, Path]) -> Union[T, Any]:
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

    def iterate(self) -> Iterator[Union[T, Any]]:
        return (x[1] for x in super().iterate())

    def iterate_ts(self) -> Iterator[Union[Tuple[float, T], Any]]:
        return super().iterate()

    def stream(self) -> Observable[Union[T, Any]]:
        """Stream sensor data with original timing preserved (non-blocking)."""

        def create_timed_stream():
            iterator = self.iterate_ts()

            try:
                prev_timestamp, first_data = next(iterator)

                yield just(first_data)

                for timestamp, data in iterator:
                    time_diff = timestamp - prev_timestamp

                    if time_diff > 0:
                        yield rx_timer(time_diff).pipe(ops.map(lambda _: data))
                    else:
                        yield just(data)

                    prev_timestamp = timestamp

            except StopIteration:
                yield empty()

        return concat(*create_timed_stream())
