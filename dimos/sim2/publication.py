# Copyright 2026 Dimensional Inc.
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

"""Wall-clock publication warnings; independent of simulation-time acquisition scheduling."""

import math
from threading import Lock
from time import monotonic
from typing import TYPE_CHECKING

from reactivex import interval

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.module import Module

logger = setup_logger()
WARNING_INTERVAL_S = 5.0
MIN_WINDOW_SAMPLES = 10
MIN_RATE_FRACTION = 0.9


class PublicationRate:
    """Count completed publications and warn on sustained throughput below the requested Hz.

    Call begin when ready to process the first input, then published after each
    successful output. The module owns the periodic check, including when no
    further output arrives. A camera counts one complete RGB/depth/info batch.
    """

    def __init__(self, module: "Module", output: str, hz: float) -> None:
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("publication rate must be finite and positive")
        self._module = module
        self._output = output
        self._hz = hz
        self._window_s = max(WARNING_INTERVAL_S, MIN_WINDOW_SAMPLES / hz)
        self._lock = Lock()
        self._since: float | None = None
        self._published = 0

    def begin(self) -> None:
        """Arm once input is available; loading assets does not count as lost throughput."""
        with self._lock:
            if self._since is not None:
                return
            self._since = monotonic()
            self._module.register_disposable(
                interval(self._window_s).subscribe(lambda _: self._check())
            )

    def published(self) -> None:
        with self._lock:
            self._published += 1

    def _check(self) -> None:
        with self._lock:
            now = monotonic()
            if self._since is None or now <= self._since:
                return
            elapsed = now - self._since
            count = self._published
            self._since, self._published = now, 0
        actual_hz = count / elapsed
        expected = self._hz * elapsed
        if count < expected * MIN_RATE_FRACTION:
            logger.warning(
                "Sim2 output is below its requested publication rate",
                module=self._module.config.instance_name or type(self._module).__name__,
                output=self._output,
                requested_hz=self._hz,
                actual_hz=round(actual_hz, 2),
                published=count,
                # Shortfall against the configured rate, not a measured transport loss.
                publication_shortfall=max(0, round(expected) - count),
                window_s=round(elapsed, 2),
            )
