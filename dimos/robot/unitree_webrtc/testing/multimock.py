#!/usr/bin/env python3
"""Multimock – lightweight persistence & replay helper built on RxPy.

A directory of pickle files acts as a tiny append-only log of (timestamp, data)
pairs.  You can:
  • save() / consume(): append new frames
  • iterate():          read them back lazily
  • interval_stream():  emit at a fixed cadence
  • stream():           replay with original timing (optionally scaled)

The implementation keeps memory usage constant by relying on reactive
operators instead of pre-materialising lists.  Timing is reproduced via
`rx.timer`, and drift is avoided with `concat_map`.
"""

from __future__ import annotations

import glob
import os
import pickle
import time
from typing import Any, Generic, Iterator, List, Tuple, TypeVar, Union, Optional
from reactivex.scheduler import ThreadPoolScheduler

from reactivex import from_iterable, interval, operators as ops
from reactivex.observable import Observable
from dimos.utils.threadpool import get_scheduler
from dimos.robot.unitree_webrtc.type.timeseries import TEvent, Timeseries

T = TypeVar("T")


class Multimock(Generic[T], Timeseries[TEvent[T]]):
    """Persist frames as pickle files and replay them with RxPy."""

    def __init__(self, root: str = "office", file_prefix: str = "msg") -> None:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.root = os.path.join(current_dir, f"multimockdata/{root}")
        self.file_prefix = file_prefix

        os.makedirs(self.root, exist_ok=True)
        self.cnt: int = 0

    def save(self, *frames: Any) -> int:
        """Persist one or more frames; returns the new counter value."""
        for frame in frames:
            self.save_one(frame)
        return self.cnt

    def save_one(self, frame: Any) -> int:
        """Persist a single frame and return the running count."""
        file_name = f"/{self.file_prefix}_{self.cnt:03d}.pickle"
        full_path = os.path.join(self.root, file_name.lstrip("/"))
        self.cnt += 1

        if os.path.isfile(full_path):
            raise FileExistsError(f"file {full_path} exists")

        # Optional convinience magic to extract raw messages from advanced types
        # trying to deprecate for now
        # if hasattr(frame, "raw_msg"):
        #    frame = frame.raw_msg  # type: ignore[attr-defined]

        with open(full_path, "wb") as f:
            pickle.dump([time.time(), frame], f)

        return self.cnt

    def load(self, *names: Union[int, str]) -> List[Tuple[float, T]]:
        """Load multiple items by name or index."""
        return list(map(self.load_one, names))

    def load_one(self, name: Union[int, str]) -> TEvent[T]:
        """Load a single item by name or index."""
        if isinstance(name, int):
            file_name = f"/{self.file_prefix}_{name:03d}.pickle"
        else:
            file_name = f"/{name}.pickle"

        full_path = os.path.join(self.root, file_name.lstrip("/"))

        with open(full_path, "rb") as f:
            timestamp, data = pickle.load(f)

        return TEvent(timestamp, data)

    def iterate(self) -> Iterator[TEvent[T]]:
        """Yield all persisted TEvent(timestamp, data) pairs lazily in order."""
        pattern = os.path.join(self.root, f"{self.file_prefix}_*.pickle")
        for file_path in sorted(glob.glob(pattern)):
            with open(file_path, "rb") as f:
                timestamp, data = pickle.load(f)
                yield TEvent(timestamp, data)

    def list(self) -> List[TEvent[T]]:
        return list(self.iterate())

    def interval_stream(self, rate_hz: float = 10.0) -> Observable[T]:
        """Emit frames at a fixed rate, ignoring recorded timing."""
        sleep_time = 1.0 / rate_hz
        return from_iterable(self.iterate()).pipe(
            ops.zip(interval(sleep_time)),
            ops.map(lambda pair: pair[1]),  # keep only the frame
        )

    def stream(
        self,
        replay_speed: float = 1.0,
        scheduler: Optional[ThreadPoolScheduler] = None,
    ) -> Observable[T]:
        def _generator():
            prev_ts: float | None = None
            for event in self.iterate():
                if prev_ts is not None:
                    delay = (event.ts - prev_ts).total_seconds() / replay_speed
                    time.sleep(delay)
                prev_ts = event.ts
                yield event.data

        return from_iterable(_generator(), scheduler=scheduler or get_scheduler())

    def consume(self, observable: Observable[Any]) -> Observable[int]:
        """Side-effect: save every frame that passes through."""
        return observable.pipe(ops.map(self.save_one))

    def __iter__(self) -> Iterator[TEvent[T]]:
        """Allow iteration over the Multimock instance to yield TEvent(timestamp, data) pairs."""
        return self.iterate()
