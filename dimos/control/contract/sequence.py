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

"""Ordering and freshness, with no clock of their own.

Every function here takes the current time as an argument. Nothing calls
``time.monotonic()`` inside this package, which is what makes staleness
testable without sleeping and portable to Rust without a runtime.

Sequences are only comparable inside one epoch. A new epoch restarts them, so a
frame from a newer epoch is newer whatever its sequence says.
"""

from __future__ import annotations

from dataclasses import dataclass


def is_newer(epoch: int, sequence: int, last: tuple[int, int] | None) -> bool:
    """Whether ``(epoch, sequence)`` supersedes ``last``.

    A higher epoch always wins and a lower one never does; within one epoch the
    sequence must be strictly greater, so a replayed or duplicated frame loses.

    Args:
        epoch: Epoch of the frame being judged.
        sequence: Sequence of the frame being judged.
        last: The last accepted ``(epoch, sequence)``, or ``None`` if none yet.
    """
    if last is None:
        return True
    last_epoch, last_sequence = last
    if epoch != last_epoch:
        return epoch > last_epoch
    return sequence > last_sequence


@dataclass(slots=True)
class Freshness:
    """When a source was last heard from.

    Deliberately the one mutable type in this package: it is updated on every
    state sample, at up to 500 Hz, and making the caller rebind a frozen value
    that often buys nothing. It still owns no clock -- receipts come from the
    caller's ``time.monotonic()``.
    """

    last_receipt: float | None = None

    def mark(self, now: float) -> None:
        """Record that a frame arrived at monotonic time ``now``."""
        self.last_receipt = now

    def is_stale(self, now: float, timeout_s: float) -> bool:
        """Whether nothing has arrived within ``timeout_s`` of ``now``.

        A source that has never reported is stale. Landing exactly on the
        timeout is not: the comparison is strictly greater, so a source running
        precisely at its declared rate does not flap.
        """
        if self.last_receipt is None:
            return True
        return (now - self.last_receipt) > timeout_s

    def age(self, now: float) -> float | None:
        """Seconds since the last receipt, or ``None`` if nothing has arrived."""
        if self.last_receipt is None:
            return None
        return now - self.last_receipt
