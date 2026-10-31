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

"""Coordinator-tick timing shared by joint and EEF trajectory playback."""

from dataclasses import dataclass
import math
import time


@dataclass
class TrajectoryClock:
    """Start on the first execution tick, independently of RPC submission time.

    It also tracks an optional caller heartbeat in the same time domain.
    Each task owns completion, cancellation and tracking policies.
    Read-only sampling never starts the clock or refreshes the heartbeat.
    """

    start_time: float | None = None
    elapsed: float = 0.0
    deadman_timeout: float | None = None
    last_keepalive: float | None = None

    def __post_init__(self) -> None:
        if self.deadman_timeout is not None and (
            not math.isfinite(self.deadman_timeout) or self.deadman_timeout <= 0.0
        ):
            raise ValueError("deadman_timeout must be finite and positive")

    def update(self, t_now: float) -> float:
        """Advance from coordinator time, retaining the latest tick's elapsed time."""
        if self.start_time is None:
            self.start_time = t_now
        if self.deadman_timeout is not None and self.last_keepalive is None:
            self.last_keepalive = t_now
        self.elapsed = max(0.0, t_now - self.start_time)
        return self.elapsed

    def keepalive(self) -> None:
        """Stamp heartbeat receipt with the coordinator process's monotonic clock."""
        if self.deadman_timeout is not None:
            self.last_keepalive = time.perf_counter()

    def expired(self, t_now: float) -> bool:
        """Return whether a supervised run exceeded its heartbeat interval."""
        return (
            self.deadman_timeout is not None
            and self.last_keepalive is not None
            and t_now - self.last_keepalive > self.deadman_timeout
        )

    def sample(self, duration: float, t_now: float | None = None) -> tuple[float, float, float]:
        """Return nominal progress, elapsed and remaining time without mutation.

        Omitting ``t_now`` reports the last tick. Passing it projects timing
        from the same start, preserving the joint task's status-query contract.
        """
        elapsed = self.elapsed
        if t_now is not None:
            elapsed = 0.0 if self.start_time is None else max(0.0, t_now - self.start_time)
        progress = min(1.0, elapsed / duration) if duration > 0.0 else 0.0
        return progress, elapsed, max(0.0, duration - elapsed)
