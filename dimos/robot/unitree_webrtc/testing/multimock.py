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

T = TypeVar("T")


class Multimock(Generic[T]):
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

    def load_one(self, name: Union[int, str]) -> Tuple[float, T]:
        """Load a single item by name or index."""
        if isinstance(name, int):
            file_name = f"/{self.file_prefix}_{name:03d}.pickle"
        else:
            file_name = f"/{name}.pickle"

        full_path = os.path.join(self.root, file_name.lstrip("/"))

        with open(full_path, "rb") as f:
            timestamp, data = pickle.load(f)

        return timestamp, data

    def iterate(self) -> Iterator[Tuple[float, T]]:
        """Yield all persisted (timestamp, data) pairs lazily in order."""
        pattern = os.path.join(self.root, f"{self.file_prefix}_*.pickle")
        for file_path in sorted(glob.glob(pattern)):
            with open(file_path, "rb") as f:
                timestamp, data = pickle.load(f)
            yield timestamp, data

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
            for ts, data in self.iterate():
                if prev_ts is not None:
                    delay = (ts - prev_ts) / replay_speed
                    time.sleep(delay)
                prev_ts = ts
                yield data

        return from_iterable(_generator(), scheduler=scheduler or get_scheduler())

    def consume(self, observable: Observable[Any]) -> Observable[int]:
        """Side-effect: save every frame that passes through."""
        return observable.pipe(ops.map(self.save_one))
