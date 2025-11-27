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

from collections.abc import Iterator
import glob
import os
import pickle
from typing import cast, overload

from reactivex import from_iterable, interval, operators as ops
from reactivex.observable import Observable

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage, RawLidarMsg


class Mock:
    def __init__(self, root: str = "office", autocast: bool = True) -> None:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.root = os.path.join(current_dir, f"mockdata/{root}")
        self.autocast = autocast
        self.cnt = 0

    @overload
    def load(self, name: int | str, /) -> LidarMessage: ...
    @overload
    def load(self, *names: int | str) -> list[LidarMessage]: ...

    def load(self, *names: int | str) -> LidarMessage | list[LidarMessage]:
        if len(names) == 1:
            return self.load_one(names[0])
        return list(map(lambda name: self.load_one(name), names))

    def load_one(self, name: int | str) -> LidarMessage:
        if isinstance(name, int):
            file_name = f"/lidar_data_{name:03d}.pickle"
        else:
            file_name = f"/{name}.pickle"

        full_path = self.root + file_name
        with open(full_path, "rb") as f:
            return LidarMessage.from_msg(cast("RawLidarMsg", pickle.load(f)))

    def iterate(self) -> Iterator[LidarMessage]:
        pattern = os.path.join(self.root, "lidar_data_*.pickle")
        print("loading data", pattern)
        for file_path in sorted(glob.glob(pattern)):
            basename = os.path.basename(file_path)
            filename = os.path.splitext(basename)[0]
            yield self.load_one(filename)

    def stream(self, rate_hz: float = 10.0):  # type: ignore[no-untyped-def]
        sleep_time = 1.0 / rate_hz

        return from_iterable(self.iterate()).pipe(
            ops.zip(interval(sleep_time)),
            ops.map(lambda x: x[0] if isinstance(x, tuple) else x),
        )

    def save_stream(self, observable: Observable[LidarMessage]):  # type: ignore[no-untyped-def]
        return observable.pipe(ops.map(lambda frame: self.save_one(frame)))  # type: ignore[no-untyped-call]

    def save(self, *frames):  # type: ignore[no-untyped-def]
        [self.save_one(frame) for frame in frames]  # type: ignore[no-untyped-call]
        return self.cnt

    def save_one(self, frame):  # type: ignore[no-untyped-def]
        file_name = f"/lidar_data_{self.cnt:03d}.pickle"
        full_path = self.root + file_name

        self.cnt += 1

        if os.path.isfile(full_path):
            raise Exception(f"file {full_path} exists")

        if frame.__class__ == LidarMessage:
            frame = frame.raw_msg

        with open(full_path, "wb") as f:
            pickle.dump(frame, f)

        return self.cnt
