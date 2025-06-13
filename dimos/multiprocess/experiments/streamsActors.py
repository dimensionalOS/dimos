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

import time
from datetime import datetime

from dask.distributed import get_client
from streamz import Stream


class FrameProcessor:
    latency: float = 0
    frame_count: int = 0

    def __init__(self, name, verbose=False):
        self.client = get_client()
        self.name = name
        self.verbose = verbose
        self.stream = Stream(asynchronous=True)
        self.stream.map(self._measure_latency).map(self._update_latency).sink(
            lambda frame: print(f"{self.name}: {frame}") if self.verbose else None
        )

    def _measure_latency(self, frame):
        (timestamp, n) = frame
        time_diff = (datetime.now() - datetime.fromtimestamp(timestamp)).total_seconds() * 1_000
        return (timestamp, n, time_diff)

    def _update_latency(self, frame):
        (timestamp, n, time_diff) = frame
        # Update running average
        self.frame_count += 1
        self.latency = (self.latency * (self.frame_count - 1) + time_diff) / self.frame_count
        return frame

    async def get_latency(self) -> float:
        print(f"{self.name}: GET LATENCY CALLED Latency: {self.latency}")
        return self.latency

    async def receive_frame(self, frame) -> float:
        self.stream.emit(frame)


def camera_loop(total=100, *processors):
    n = 0
    while True:
        for proc in processors:
            proc.receive_frame((time.time(), n))
        n += 1
        if n >= total:
            break
        time.sleep(1.0 / 60)
