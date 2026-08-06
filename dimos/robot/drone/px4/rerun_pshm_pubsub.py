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

"""Fixed-topic pSHM adapter for the PX4 Rerun bridge."""

from collections.abc import Callable
import time
from typing import Protocol, final

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.shmpubsub import PickleSharedMemory
from dimos.utils.logging_config import setup_logger

_COLOR_IMAGE_TOPIC = "/color_image"
_DEFAULT_ERROR_INTERVAL_S = 5.0
logger = setup_logger()


class ExceptionLogger(Protocol):
    """Structured logger surface required at the bridge callback boundary."""

    def exception(self, event: str, **kwargs: str) -> None:
        """Record an exception with structured fields."""


class PickleSharedMemorySubscriber(Protocol):
    """Lifecycle and typed subscription surface used by the local adapter."""

    def start(self) -> None:
        """Start shared-memory delivery."""

    def stop(self) -> None:
        """Stop shared-memory delivery."""

    def subscribe(self, topic: str, callback: Callable[[Image, str], None]) -> Callable[[], None]:
        """Subscribe to one typed shared-memory topic."""
        ...


@final
class Px4FixedTopicPshm:
    """Expose PX4's raw camera pSHM topic through the bridge pubsub protocol."""

    def __init__(
        self,
        *,
        shared_memory: PickleSharedMemorySubscriber | None = None,
        clock: Callable[[], float] = time.monotonic,
        logger: ExceptionLogger = logger,
        error_interval_s: float = _DEFAULT_ERROR_INTERVAL_S,
    ) -> None:
        self._shared_memory: PickleSharedMemorySubscriber = (
            PickleSharedMemory(default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE)
            if shared_memory is None
            else shared_memory
        )
        self._clock: Callable[[], float] = clock
        self._logger: ExceptionLogger = logger
        self._error_interval_s: float = error_interval_s
        self._last_error_at: float | None = None

    def start(self) -> None:
        """Start the local shared-memory subscription backend."""
        self._shared_memory.start()

    def stop(self) -> None:
        """Stop the local shared-memory subscription backend."""
        self._shared_memory.stop()

    def subscribe_all(self, callback: Callable[[Image, str], None]) -> Callable[[], None]:
        """Subscribe the bridge callback to the sole PX4 raw-camera topic."""
        return self._shared_memory.subscribe(_COLOR_IMAGE_TOPIC, self._forward(callback))

    def _forward(self, callback: Callable[[Image, str], None]) -> Callable[[Image, str], None]:
        def forward(message: Image, topic: str) -> None:
            try:
                callback(message, topic)
            except Exception:
                # BROAD_EXCEPT_OK: bridge callback boundary must isolate Rerun failures.
                now = self._clock()
                last_error_at = self._last_error_at
                if last_error_at is None or now - last_error_at >= self._error_interval_s:
                    self._last_error_at = now
                    self._logger.exception("PX4 pSHM bridge callback failed", topic=topic)

        return forward
