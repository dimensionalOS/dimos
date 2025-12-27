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

from threading import RLock

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ReplanLimiter:
    """
    This class limits replanning too many times in the same area. But if we exit
    the area, the number of attempts is reset.
    """

    _max_attempts: int = 6
    _reset_distance: float = 2.0
    _attempt_pos: Vector3 | None = None
    _lock: RLock

    _attempt: int

    def __init__(self) -> None:
        self._lock = RLock()
        self._attempt = 0

    def can_retry(self, position: Vector3) -> bool:
        with self._lock:
            if self._attempt == 0:
                self._attempt_pos = position

            if self._attempt >= 1 and self._attempt_pos:
                distance = self._attempt_pos.distance(position)
                if distance >= self._reset_distance:
                    logger.info(
                        "Traveled enough to reset attempts",
                        attempts=self._attempt,
                        distance=distance,
                    )
                    self._attempt = 0
                    self._attempt_pos = position

            return self._attempt + 1 <= self._max_attempts

    def will_retry(self) -> None:
        with self._lock:
            self._attempt += 1

    def reset(self) -> None:
        with self._lock:
            self._attempt = 0

    def get_attempt(self) -> int:
        with self._lock:
            return self._attempt
