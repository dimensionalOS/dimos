import os
import pickle
import glob
from typing import Union, Iterator, cast, overload
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage, RawLidarMsg

from reactivex import operators as ops
from reactivex import interval, from_iterable
from reactivex.observable import Observable


class Mock:
    def __init__(self, root="office", autocast: bool = True):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.root = os.path.join(current_dir, f"mockdata/{root}")
        self.autocast = autocast
        self.cnt = 0

    @overload
    def load(self, name: Union[int, str], /) -> LidarMessage: ...
    @overload
    def load(self, *names: Union[int, str]) -> list[LidarMessage]: ...

    def load(self, *names: Union[int, str]) -> Union[LidarMessage, list[LidarMessage]]:
        if len(names) == 1:
            return self.load_one(names[0])
        return list(map(lambda name: self.load_one(name), names))

    def load_one(self, name: Union[int, str]) -> LidarMessage:
        if isinstance(name, int):
            file_name = f"/lidar_data_{name:03d}.pickle"
        else:
            file_name = f"/{name}.pickle"

        full_path = self.root + file_name
        with open(full_path, "rb") as f:
            return LidarMessage.from_msg(cast(RawLidarMsg, pickle.load(f)))

    def iterate(self) -> Iterator[LidarMessage]:
        pattern = os.path.join(self.root, "lidar_data_*.pickle")
        print("loading data", pattern)
        for file_path in sorted(glob.glob(pattern)):
            basename = os.path.basename(file_path)
            filename = os.path.splitext(basename)[0]
            yield self.load_one(filename)

    def stream(self, rate_hz=10.0):
        sleep_time = 1.0 / rate_hz

        return from_iterable(self.iterate()).pipe(
            ops.zip(interval(sleep_time)),
            ops.map(lambda x: x[0] if isinstance(x, tuple) else x),
        )

    def save_stream(self, observable: Observable[LidarMessage]):
        return observable.pipe(ops.map(lambda frame: self.save_one(frame)))

    def save(self, *frames):
        [self.save_one(frame) for frame in frames]
        return self.cnt

    def save_one(self, frame):
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
