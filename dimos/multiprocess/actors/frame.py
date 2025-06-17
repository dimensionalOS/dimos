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

import datetime

from streamz import Stream

from dimos.multiprocess.types import Frame


class TimedFrame(Frame):
    latency: float


class LatencyActor:
    avg_latency: float = 0
    frame_count: int = 0

    def __init__(self, name, verbose=False):
        self.name = name
        self.verbose = verbose
        self.stream = Stream(asynchronous=True)
        self.stream.map(self._measure_latency).map(self._update_avg_latency).sink(
            lambda frame: print(
                f"{self.name}: {frame.get('frame_number')} - {frame.get('latency')}"
            )
            if self.verbose
            else None
        )
        # self.stream.sink(lambda frame: print(f"{self.name}: {frame}") if self.verbose else None)

    def _measure_latency(self, frame: Frame) -> TimedFrame:
        time_diff = (
            datetime.datetime.now() - datetime.datetime.fromtimestamp(frame["timestamp"])
        ).total_seconds() * 1_000

        timed_frame: TimedFrame = {
            "frame": frame["frame"],
            "timestamp": frame["timestamp"],
            "frame_number": frame["frame_number"],
            "latency": time_diff,
        }
        return timed_frame

    def _update_avg_latency(self, timed_frame: TimedFrame) -> TimedFrame:
        time_diff = timed_frame["latency"]

        self.frame_count += 1
        self.avg_latency = (
            self.avg_latency * (self.frame_count - 1) + time_diff
        ) / self.frame_count

        return timed_frame

    async def get_latency(self) -> float:
        return self.avg_latency

    async def receive_frame(self, frame: Frame) -> None:
        # print("LatencyActor received frame", frame)
        self.stream.emit(frame)
