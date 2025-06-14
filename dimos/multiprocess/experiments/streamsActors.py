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

import asyncio
import time
from datetime import datetime

from dask.distributed import get_client
from streamz import Stream


class FrameProcessor:
    avg_latency: float = 0
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
        self.avg_latency = (
            self.avg_latency * (self.frame_count - 1) + time_diff
        ) / self.frame_count
        return frame

    async def get_latency(self) -> float:
        return self.avg_latency

    async def receive_frame(self, frame) -> float:
        """Legacy method for backwards compatibility"""
        self.stream.emit(frame)


class CameraLoop:
    stream: Stream = Stream(asynchronous=True)

    def __init__(self, fps=60):
        self.client = get_client()
        self.fps = fps
        self.frame_interval = 1.0 / fps

    def add_processor(self, processor):
        self.stream.sink(processor.receive_frame)

    def add_processors(self, *processors):
        for processor in processors:
            self.add_processor(processor)

    async def run(self, total=100):
        n = 0
        while True:
            frame = (time.time(), n)
            self.stream.emit(frame)
            n += 1
            if n >= total:
                break
            await asyncio.sleep(self.frame_interval)
