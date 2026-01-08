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

# multirate.py
import time, threading, logging
from typing import Callable, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

# Handler signature: handler(img, meta) where
#   img  = np.ndarray (CPU) or cupy.ndarray (CUDA)
#   meta = {"seq": int, "ts_ns": int, "device": "cpu"|"cuda"}
Handler = Callable[[object, dict], None]


class MultiRateProcessor:
    """
    Per-model FPS workers that read the freshest frame from a FrameChannel.
    - Works with CPU or CUDA backends via the channel interface.
    - You supply a handler per model name.
    """

    def __init__(
        self,
        channel,  # FrameChannel
        target_fps_by_model: Dict[str, float],
        handlers: Dict[str, Handler],
        max_age_s: Optional[float] = None,  # drop stale frames if set
    ):
        self.slot = channel
        self._target = dict(target_fps_by_model)
        self._handlers = dict(handlers)
        self._stop = threading.Event()
        self._threads: Dict[str, threading.Thread] = {}
        self._max_age_s = max_age_s

        # quick sanity
        missing = set(self._target) - set(self._handlers)
        if missing:
            raise ValueError(f"Missing handlers for models: {sorted(missing)}")

    def start(self):
        for name, fps in self._target.items():
            th = threading.Thread(target=self._worker_loop, args=(name, fps), daemon=True)
            self._threads[name] = th
            th.start()

    def stop(self):
        self._stop.set()
        for th in self._threads.values():
            th.join(timeout=0.5)
        self._threads.clear()

    def _worker_loop(self, name: str, fps: float):
        min_interval = 1.0 / max(fps, 1e-6)
        next_t = time.monotonic()
        last_seq = -1
        handler = self._handlers[name]

        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(next_t - now, 0.005))
                continue

            seq, ts_ns, view = self.slot.read(last_seq=last_seq, require_new=True)
            if view is None:
                # no new frame since last read; still maintain cadence
                next_t = now + min_interval
                continue

            # Optional staleness guard
            if self._max_age_s is not None and ts_ns:
                age = (time.time_ns() - ts_ns) * 1e-9
                if age > self._max_age_s:
                    last_seq = seq
                    next_t = time.monotonic() + min_interval
                    continue

            try:
                handler(view, {"seq": seq, "ts_ns": ts_ns, "device": self.slot.device})
            except Exception as e:
                logger.error("Handler '%s' failed: %s", name, e, exc_info=True)
            finally:
                last_seq = seq
                next_t = time.monotonic() + min_interval
